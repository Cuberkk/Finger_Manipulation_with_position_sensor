#!/usr/bin/env python3

"""
object_axis_rviz_visualizer.py

Live RViz2 visualization for object rotation-axis training.

What RViz displays
------------------
1. A transparent reference cylinder at the initial object pose.
2. A solid current cylinder that follows sensor4 in real time.
3. Initial and current object-frame x/y/z axes.
4. A double tolerance cone around the desired task axis.
5. A measured incremental rotation-axis arrow.
6. A translation tolerance sphere centered at the initial pose.
7. A line from the initial object center to the current center.
8. Live text containing x/y/z axis errors and translation error.

Task-axis convention
--------------------
    roll  -> object x-axis
    pitch -> object y-axis
    yaw   -> object z-axis

Main rotation calculation
-------------------------
TF provides:

    R_base_from_previous
    R_base_from_current

The current orientation relative to the previous object frame is:

    R_previous_from_current
        = R_base_from_previous.T @ R_base_from_current

The relative rotation vector is:

    rotation_vector = measured_axis_previous * rotation_angle

The measured axis in the base frame is:

    measured_axis_base
        = R_base_from_previous @ measured_axis_previous

Axis errors
-----------
For the normalized measured axis in the previous object frame:

    x_error_deg = degrees(acos(abs(axis_x)))
    y_error_deg = degrees(acos(abs(axis_y)))
    z_error_deg = degrees(acos(abs(axis_z)))

The absolute value treats +axis and -axis as the same physical axis line.

Status colors
-------------
    Green  -> within axis and translation tolerances
    Red    -> outside at least one tolerance
    Blue   -> object is nearly stationary, so the rotation axis is undefined

Run example
-----------
    python3 object_axis_rviz_visualizer.py --task roll

For an 80 mm diameter cylinder:
    python3 object_axis_rviz_visualizer.py \
        --task roll \
        --diameter-mm 80 \
        --height-mm 120 \
        --axis-tolerance-deg 15 \
        --translation-tolerance-mm 20

Reset the reference pose:
    ros2 service call \
        /ERIE_Manipulation/reset_axis_visualization_reference \
        std_srvs/srv/Trigger "{}"

RViz setup
----------
1. Start:
       rviz2
2. Set Fixed Frame:
       polhemus_base
3. Add:
       MarkerArray
4. Select topic:
       /ERIE_Manipulation/object_axis_visualization
"""

import argparse
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class ObjectAxisRvizVisualizer(Node):
    """Publish live object and rotation-axis guidance as RViz markers."""

    def __init__(
        self,
        task: str,
        publish_rate_hz: float,
        diameter_mm: float,
        height_mm: float,
        axis_tolerance_deg: float,
        translation_tolerance_mm: float,
        guide_length_mm: float,
        sensor_at_bottom: bool,
    ) -> None:
        super().__init__("object_axis_rviz_visualizer")

        if task not in ("roll", "pitch", "yaw"):
            raise ValueError("task must be roll, pitch, or yaw")

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        if not 0.0 < axis_tolerance_deg < 89.0:
            raise ValueError(
                "axis_tolerance_deg must be between 0 and 89 degrees"
            )

        self.task = task
        self.publish_rate_hz = float(publish_rate_hz)

        # TF frames.
        self.base_frame = "polhemus_base"
        self.object_frame = "sensor4"

        # Object geometry in meters.
        self.object_diameter = float(diameter_mm) / 1000.0
        self.object_height = float(height_mm) / 1000.0
        self.guide_length = float(guide_length_mm) / 1000.0

        # Allowed errors.
        self.axis_tolerance_deg = float(axis_tolerance_deg)
        self.translation_tolerance = (
            float(translation_tolerance_mm) / 1000.0
        )

        self.sensor_at_bottom = bool(sensor_at_bottom)

        # Ignore tiny incremental rotations because their axis is dominated
        # by sensor noise and is mathematically undefined at zero rotation.
        self.minimum_rotation_deg = 0.05
        self.minimum_rotation_rad = math.radians(
            self.minimum_rotation_deg
        )

        self.maximum_tf_age_sec = 1.5

        self.local_axes = {
            "roll": np.array([1.0, 0.0, 0.0], dtype=np.float64),
            "pitch": np.array([0.0, 1.0, 0.0], dtype=np.float64),
            "yaw": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        }

        self.task_axis_index = {
            "roll": 0,
            "pitch": 1,
            "yaw": 2,
        }[self.task]

        # TF.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # Marker output.
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/ERIE_Manipulation/object_axis_visualization",
            10,
        )

        # Reference pose reset service.
        self.reset_service = self.create_service(
            Trigger,
            "/ERIE_Manipulation/reset_axis_visualization_reference",
            self.reset_reference_callback,
        )

        # Reference and previous TF values.
        self.initial_position: Optional[np.ndarray] = None
        self.initial_rotation: Optional[np.ndarray] = None
        self.initial_center: Optional[np.ndarray] = None

        self.previous_rotation: Optional[np.ndarray] = None

        self.reset_requested = False

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "RViz object-axis visualizer started."
        )
        self.get_logger().info(
            f"Task: {self.task}; axis tolerance: "
            f"{self.axis_tolerance_deg:.1f} deg; translation tolerance: "
            f"{translation_tolerance_mm:.1f} mm"
        )
        self.get_logger().info(
            "RViz MarkerArray topic: "
            "/ERIE_Manipulation/object_axis_visualization"
        )

    # ------------------------------------------------------------------
    # TF and geometry helpers
    # ------------------------------------------------------------------

    def lookup_object_pose(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Return the latest sensor4 position and orientation.

        Returns:
            position_base: shape (3,)
            R_base_from_object: shape (3, 3)
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.object_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            self.get_logger().warning(
                f"Could not look up {self.base_frame} <- "
                f"{self.object_frame}: {error}",
                throttle_duration_sec=1.0,
            )
            return None

        transform_time = Time.from_msg(transform.header.stamp)
        age_sec = (
            self.get_clock().now() - transform_time
        ).nanoseconds * 1e-9

        if age_sec > self.maximum_tf_age_sec:
            self.get_logger().warning(
                f"TF is too old: {age_sec:.3f} s",
                throttle_duration_sec=1.0,
            )
            return None

        translation = transform.transform.translation
        position = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

        q = transform.transform.rotation
        quaternion = np.array(
            [q.x, q.y, q.z, q.w],
            dtype=np.float64,
        )

        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-12:
            return None

        quaternion /= norm
        rotation = Rotation.from_quat(quaternion).as_matrix()

        return position, rotation

    def object_center(
        self,
        sensor_position: np.ndarray,
        object_rotation: np.ndarray,
    ) -> np.ndarray:
        """
        Convert the sensor4 position to the cylinder center.

        sensor4 is mounted at the top center of the cylinder.
        Therefore, the cylinder center is half the cylinder height
        along the object's local -z direction.
        """
        local_offset = np.array(
            [0.0, 0.0, -self.object_height / 2.0],
            dtype=np.float64,
        )

        return sensor_position + object_rotation @ local_offset

    @staticmethod
    def point_message(vector: Sequence[float]) -> Point:
        point = Point()
        point.x = float(vector[0])
        point.y = float(vector[1])
        point.z = float(vector[2])
        return point

    @staticmethod
    def quaternion_message(rotation_matrix: np.ndarray) -> Quaternion:
        qx, qy, qz, qw = Rotation.from_matrix(
            rotation_matrix
        ).as_quat()

        quaternion = Quaternion()
        quaternion.x = float(qx)
        quaternion.y = float(qy)
        quaternion.z = float(qz)
        quaternion.w = float(qw)
        return quaternion

    @staticmethod
    def orthogonal_basis(
        axis: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return two unit vectors perpendicular to axis."""
        axis = axis / np.linalg.norm(axis)

        if abs(axis[2]) < 0.9:
            helper = np.array([0.0, 0.0, 1.0])
        else:
            helper = np.array([1.0, 0.0, 0.0])

        basis_1 = np.cross(axis, helper)
        basis_1 /= np.linalg.norm(basis_1)

        basis_2 = np.cross(axis, basis_1)
        basis_2 /= np.linalg.norm(basis_2)

        return basis_1, basis_2

    @staticmethod
    def axis_error_deg(axis_component: float) -> float:
        component = float(
            np.clip(abs(axis_component), 0.0, 1.0)
        )
        return math.degrees(math.acos(component))

    # ------------------------------------------------------------------
    # Marker helper functions
    # ------------------------------------------------------------------

    def base_marker(
        self,
        marker_id: int,
        marker_type: int,
        namespace: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Marker disappears if this node stops publishing.
        marker.lifetime = Duration(seconds=0.25).to_msg()
        return marker

    def cylinder_marker(
        self,
        marker_id: int,
        namespace: str,
        center: np.ndarray,
        rotation: np.ndarray,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.CYLINDER,
            namespace,
        )

        marker.pose.position = self.point_message(center)
        marker.pose.orientation = self.quaternion_message(rotation)

        marker.scale.x = self.object_diameter
        marker.scale.y = self.object_diameter
        marker.scale.z = self.object_height

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def arrow_marker(
        self,
        marker_id: int,
        namespace: str,
        start: np.ndarray,
        end: np.ndarray,
        rgba: Tuple[float, float, float, float],
        shaft_diameter: float = 0.008,
        head_diameter: float = 0.016,
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.ARROW,
            namespace,
        )

        marker.points = [
            self.point_message(start),
            self.point_message(end),
        ]

        marker.scale.x = shaft_diameter
        marker.scale.y = head_diameter
        marker.scale.z = head_diameter * 1.25

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def line_marker(
        self,
        marker_id: int,
        namespace: str,
        start: np.ndarray,
        end: np.ndarray,
        rgba: Tuple[float, float, float, float],
        width: float = 0.004,
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.LINE_STRIP,
            namespace,
        )

        marker.points = [
            self.point_message(start),
            self.point_message(end),
        ]

        marker.scale.x = width

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        center: np.ndarray,
        diameter: float,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.SPHERE,
            namespace,
        )

        marker.pose.position = self.point_message(center)

        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def text_marker(
        self,
        marker_id: int,
        position: np.ndarray,
        text: str,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.TEXT_VIEW_FACING,
            "status_text",
        )

        marker.pose.position = self.point_message(position)
        marker.scale.z = 0.025

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        marker.text = text
        return marker

    def cone_marker(
        self,
        marker_id: int,
        namespace: str,
        apex: np.ndarray,
        axis: np.ndarray,
        length: float,
        half_angle_deg: float,
        rgba: Tuple[float, float, float, float],
        segments: int = 48,
    ) -> Marker:
        """
        Construct a cone using TRIANGLE_LIST.

        The apex is at the reference cylinder center. The cone opens in the
        supplied axis direction. Its half-angle is the allowed axis error.
        """
        marker = self.base_marker(
            marker_id,
            Marker.TRIANGLE_LIST,
            namespace,
        )

        axis = axis / np.linalg.norm(axis)
        basis_1, basis_2 = self.orthogonal_basis(axis)

        base_center = apex + axis * length
        radius = length * math.tan(math.radians(half_angle_deg))

        circle_points: List[np.ndarray] = []

        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            circle_point = (
                base_center
                + radius * math.cos(angle) * basis_1
                + radius * math.sin(angle) * basis_2
            )
            circle_points.append(circle_point)

        for index in range(segments):
            current_point = circle_points[index]
            next_point = circle_points[(index + 1) % segments]

            # Side triangle.
            marker.points.append(self.point_message(apex))
            marker.points.append(self.point_message(current_point))
            marker.points.append(self.point_message(next_point))

            # Base cap triangle.
            marker.points.append(self.point_message(base_center))
            marker.points.append(self.point_message(next_point))
            marker.points.append(self.point_message(current_point))

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def delete_marker(
        self,
        marker_id: int,
        namespace: str,
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.ARROW,
            namespace,
        )
        marker.action = Marker.DELETE
        return marker

    def frame_axis_markers(
        self,
        start_id: int,
        namespace: str,
        center: np.ndarray,
        rotation: np.ndarray,
        length: float,
        alpha: float,
    ) -> List[Marker]:
        """Create x, y, and z arrows for an object frame."""
        colors = [
            (1.0, 0.0, 0.0, alpha),  # x
            (0.0, 1.0, 0.0, alpha),  # y
            (0.0, 0.4, 1.0, alpha),  # z
        ]

        markers: List[Marker] = []

        for index in range(3):
            local_axis = np.zeros(3)
            local_axis[index] = 1.0

            axis_base = rotation @ local_axis
            end = center + length * axis_base

            markers.append(
                self.arrow_marker(
                    marker_id=start_id + index,
                    namespace=namespace,
                    start=center,
                    end=end,
                    rgba=colors[index],
                    shaft_diameter=0.004,
                    head_diameter=0.010,
                )
            )

        return markers

    # ------------------------------------------------------------------
    # Reference reset
    # ------------------------------------------------------------------

    def reset_reference_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self.reset_requested = True
        response.success = True
        response.message = (
            "The reference pose will reset on the next valid TF sample."
        )
        return response

    def set_reference(
        self,
        sensor_position: np.ndarray,
        object_rotation: np.ndarray,
    ) -> None:
        self.initial_position = sensor_position.copy()
        self.initial_rotation = object_rotation.copy()
        self.initial_center = self.object_center(
            sensor_position,
            object_rotation,
        )
        self.previous_rotation = object_rotation.copy()
        self.reset_requested = False

        self.get_logger().info(
            "Stored a new object reference pose."
        )

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def timer_callback(self) -> None:
        pose = self.lookup_object_pose()

        if pose is None:
            return

        sensor_position, current_rotation = pose
        current_center = self.object_center(
            sensor_position,
            current_rotation,
        )

        if (
            self.initial_rotation is None
            or self.initial_center is None
            or self.previous_rotation is None
            or self.reset_requested
        ):
            self.set_reference(
                sensor_position,
                current_rotation,
            )
            return

        # --------------------------------------------------------------
        # Incremental relative rotation
        # --------------------------------------------------------------
        relative_matrix = (
            self.previous_rotation.T @ current_rotation
        )

        relative_rotation = Rotation.from_matrix(relative_matrix)
        rotation_vector_previous = relative_rotation.as_rotvec()
        rotation_angle_rad = float(
            np.linalg.norm(rotation_vector_previous)
        )

        measured_axis_previous: Optional[np.ndarray] = None
        measured_axis_base: Optional[np.ndarray] = None

        x_error_deg = math.nan
        y_error_deg = math.nan
        z_error_deg = math.nan
        task_error_deg = math.nan

        if rotation_angle_rad >= self.minimum_rotation_rad:
            measured_axis_previous = (
                rotation_vector_previous / rotation_angle_rad
            )

            x_error_deg = self.axis_error_deg(
                measured_axis_previous[0]
            )
            y_error_deg = self.axis_error_deg(
                measured_axis_previous[1]
            )
            z_error_deg = self.axis_error_deg(
                measured_axis_previous[2]
            )

            all_errors = [
                x_error_deg,
                y_error_deg,
                z_error_deg,
            ]
            task_error_deg = all_errors[self.task_axis_index]

            # Convert the axis from the previous object frame into base.
            measured_axis_base = (
                self.previous_rotation @ measured_axis_previous
            )
            measured_axis_base /= np.linalg.norm(
                measured_axis_base
            )

        # Desired task axis is fixed from the initial object orientation.
        desired_axis_base = (
            self.initial_rotation @ self.local_axes[self.task]
        )
        desired_axis_base /= np.linalg.norm(desired_axis_base)

        # Flip the displayed measured arrow toward the nearest side of the
        # desired axis line. The underlying error remains directionless.
        if (
            measured_axis_base is not None
            and np.dot(measured_axis_base, desired_axis_base) < 0.0
        ):
            measured_axis_base = -measured_axis_base

        # --------------------------------------------------------------
        # Translation deviation
        # --------------------------------------------------------------
        translation_vector = current_center - self.initial_center
        translation_error_m = float(
            np.linalg.norm(translation_vector)
        )
        translation_error_mm = 1000.0 * translation_error_m

        position_ok = (
            translation_error_m <= self.translation_tolerance
        )

        axis_available = measured_axis_base is not None
        axis_ok = (
            axis_available
            and task_error_deg <= self.axis_tolerance_deg
        )

        if axis_available:
            overall_ok = axis_ok and position_ok
            status_color = (
                (0.0, 0.85, 0.15, 0.90)
                if overall_ok
                else (1.0, 0.05, 0.05, 0.90)
            )
            status_word = "GOOD" if overall_ok else "OUTSIDE GUIDE"
        else:
            # A stationary object has no defined incremental rotation axis.
            overall_ok = position_ok
            status_color = (0.1, 0.45, 1.0, 0.90)
            status_word = "WAITING FOR ROTATION"

        markers: List[Marker] = []

        # --------------------------------------------------------------
        # Reference and current object cylinders
        # --------------------------------------------------------------
        markers.append(
            self.cylinder_marker(
                marker_id=0,
                namespace="reference_object",
                center=self.initial_center,
                rotation=self.initial_rotation,
                rgba=(0.65, 0.65, 0.65, 0.20),
            )
        )

        markers.append(
            self.cylinder_marker(
                marker_id=1,
                namespace="current_object",
                center=current_center,
                rotation=current_rotation,
                rgba=status_color,
            )
        )

        # Initial frame axes and current frame axes.
        frame_axis_length = max(
            self.object_diameter * 0.9,
            0.06,
        )

        markers.extend(
            self.frame_axis_markers(
                start_id=10,
                namespace="initial_frame",
                center=self.initial_center,
                rotation=self.initial_rotation,
                length=frame_axis_length,
                alpha=0.30,
            )
        )

        markers.extend(
            self.frame_axis_markers(
                start_id=20,
                namespace="current_frame",
                center=current_center,
                rotation=current_rotation,
                length=frame_axis_length,
                alpha=0.95,
            )
        )

        # --------------------------------------------------------------
        # Desired axis tolerance double-cone
        # --------------------------------------------------------------
        cone_color = (0.1, 0.9, 0.2, 0.16)

        markers.append(
            self.cone_marker(
                marker_id=30,
                namespace="axis_tolerance_cone_positive",
                apex=self.initial_center,
                axis=desired_axis_base,
                length=self.guide_length,
                half_angle_deg=self.axis_tolerance_deg,
                rgba=cone_color,
            )
        )

        markers.append(
            self.cone_marker(
                marker_id=31,
                namespace="axis_tolerance_cone_negative",
                apex=self.initial_center,
                axis=-desired_axis_base,
                length=self.guide_length,
                half_angle_deg=self.axis_tolerance_deg,
                rgba=cone_color,
            )
        )

        # Desired axis centerline.
        markers.append(
            self.arrow_marker(
                marker_id=32,
                namespace="desired_task_axis",
                start=self.initial_center,
                end=(
                    self.initial_center
                    + desired_axis_base * self.guide_length
                ),
                rgba=(1.0, 1.0, 1.0, 0.95),
                shaft_diameter=0.006,
                head_diameter=0.014,
            )
        )

        # --------------------------------------------------------------
        # Measured incremental rotation axis
        # --------------------------------------------------------------
        if measured_axis_base is not None:
            markers.append(
                self.arrow_marker(
                    marker_id=40,
                    namespace="measured_rotation_axis",
                    start=current_center,
                    end=(
                        current_center
                        + measured_axis_base * self.guide_length
                    ),
                    rgba=status_color,
                    shaft_diameter=0.008,
                    head_diameter=0.018,
                )
            )
        else:
            markers.append(
                self.delete_marker(
                    marker_id=40,
                    namespace="measured_rotation_axis",
                )
            )

        # --------------------------------------------------------------
        # Translation boundary and error line
        # --------------------------------------------------------------
        markers.append(
            self.sphere_marker(
                marker_id=50,
                namespace="translation_tolerance",
                center=self.initial_center,
                diameter=2.0 * self.translation_tolerance,
                rgba=(1.0, 0.8, 0.1, 0.10),
            )
        )

        translation_line_color = (
            (1.0, 0.85, 0.0, 0.9)
            if position_ok
            else (1.0, 0.0, 0.0, 0.95)
        )

        markers.append(
            self.line_marker(
                marker_id=51,
                namespace="translation_error",
                start=self.initial_center,
                end=current_center,
                rgba=translation_line_color,
            )
        )

        # --------------------------------------------------------------
        # Live text
        # --------------------------------------------------------------
        text_position = (
            current_center
            + np.array(
                [
                    0.0,
                    0.0,
                    self.object_height * 0.85 + 0.06,
                ]
            )
        )

        if axis_available:
            error_text = (
                f"{status_word}\n"
                f"X axis error: {x_error_deg:5.1f} deg\n"
                f"Y axis error: {y_error_deg:5.1f} deg\n"
                f"Z axis error: {z_error_deg:5.1f} deg\n"
                f"{self.task.upper()} task error: "
                f"{task_error_deg:5.1f} deg\n"
                f"Translation: {translation_error_mm:5.1f} mm"
            )
        else:
            error_text = (
                f"{status_word}\n"
                "X axis error: undefined\n"
                "Y axis error: undefined\n"
                "Z axis error: undefined\n"
                f"Translation: {translation_error_mm:5.1f} mm"
            )

        markers.append(
            self.text_marker(
                marker_id=60,
                position=text_position,
                text=error_text,
                rgba=status_color,
            )
        )

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_publisher.publish(marker_array)

        # Current orientation becomes previous for the next incremental
        # rotation calculation.
        self.previous_rotation = current_rotation.copy()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Display live object rotation-axis and translation guidance "
            "in RViz2."
        )
    )

    parser.add_argument(
        "--task",
        choices=["roll", "pitch", "yaw"],
        default="roll",
        help="Desired task axis. Default: roll",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Marker update rate in Hz. Default: 30",
    )

    parser.add_argument(
        "--diameter-mm",
        type=float,
        default=80.0,
        help="Cylinder diameter in millimeters. Default: 80",
    )

    parser.add_argument(
        "--height-mm",
        type=float,
        default=120.0,
        help="Cylinder height in millimeters. Default: 120",
    )

    parser.add_argument(
        "--axis-tolerance-deg",
        type=float,
        default=15.0,
        help="Allowed half-angle around desired axis. Default: 15",
    )

    parser.add_argument(
        "--translation-tolerance-mm",
        type=float,
        default=20.0,
        help="Allowed movement from initial center. Default: 20",
    )

    parser.add_argument(
        "--guide-length-mm",
        type=float,
        default=180.0,
        help="Desired cone and axis-arrow length. Default: 180",
    )

    parser.add_argument(
        "--sensor-at-center",
        action="store_true",
        help=(
            "Use this only if sensor4 is at the cylinder center. "
            "By default sensor4 is assumed to be at the bottom center."
        ),
    )

    arguments, _ = parser.parse_known_args()
    return arguments


def main(args=None) -> None:
    command_line_arguments = parse_arguments()

    rclpy.init(args=args)

    node = ObjectAxisRvizVisualizer(
        task=command_line_arguments.task,
        publish_rate_hz=command_line_arguments.rate,
        diameter_mm=command_line_arguments.diameter_mm,
        height_mm=command_line_arguments.height_mm,
        axis_tolerance_deg=(
            command_line_arguments.axis_tolerance_deg
        ),
        translation_tolerance_mm=(
            command_line_arguments.translation_tolerance_mm
        ),
        guide_length_mm=command_line_arguments.guide_length_mm,
        sensor_at_bottom=not command_line_arguments.sensor_at_center,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
