#!/usr/bin/env python3

"""
object_axis_rviz_incremental.py

Standalone RViz2 visualizer for object rotation-axis training.

This version does NOT compare the object's orientation or position with the
initial set-down pose. Instead, it calculates the relative rotation between
the previous TF sample and the current TF sample:

    R_previous_from_current
        = R_base_from_previous.T @ R_base_from_current

The relative rotation is converted to a rotation vector:

    rotation_vector_previous
        = measured_axis_previous * rotation_angle

The measured rotation axis is compared with the x, y, and z axes of the
PREVIOUS object frame:

    x_error_deg = degrees(acos(abs(axis_x)))
    y_error_deg = degrees(acos(abs(axis_y)))
    z_error_deg = degrees(acos(abs(axis_z)))

Task convention:

    roll  -> previous object x-axis
    pitch -> previous object y-axis
    yaw   -> previous object z-axis

The tolerance cone follows the current cylinder position and is oriented using
the previous object frame. Translation from the initial position is allowed and
does not affect the status.

RViz displays:

1. The current cylinder pose.
2. The current object-frame x/y/z axes.
3. A double tolerance cone around the desired incremental rotation axis.
4. A desired-axis arrow.
5. A measured incremental rotation-axis arrow.
6. Live x/y/z axis errors and task status.

Run example:

    python3 object_axis_rviz_incremental.py --task roll

For an 80 mm diameter cylinder:

    python3 object_axis_rviz_incremental.py \
        --task roll \
        --diameter-mm 80 \
        --height-mm 120 \
        --axis-tolerance-deg 15 \
        --guide-length-mm 180 \
        --rate 30

Reset the previous-sample reference:

    ros2 service call \
        /ERIE_Manipulation/reset_axis_visualization_reference \
        std_srvs/srv/Trigger "{}"

RViz setup:

1. Start RViz:
       rviz2
2. Set Fixed Frame:
       polhemus_base
3. Add a MarkerArray display.
4. Select:
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


class ObjectAxisRvizIncremental(Node):
    """Publish live incremental object rotation-axis guidance in RViz2."""

    def __init__(
        self,
        task: str,
        publish_rate_hz: float,
        diameter_mm: float,
        height_mm: float,
        axis_tolerance_deg: float,
        guide_length_mm: float,
        minimum_rotation_deg: float,
        sensor_at_bottom: bool,
    ) -> None:
        super().__init__("object_axis_rviz_incremental")

        if task not in ("roll", "pitch", "yaw"):
            raise ValueError("task must be roll, pitch, or yaw")

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        if diameter_mm <= 0.0:
            raise ValueError("diameter_mm must be greater than zero")

        if height_mm <= 0.0:
            raise ValueError("height_mm must be greater than zero")

        if guide_length_mm <= 0.0:
            raise ValueError("guide_length_mm must be greater than zero")

        if not 0.0 < axis_tolerance_deg < 89.0:
            raise ValueError(
                "axis_tolerance_deg must be between 0 and 89 degrees"
            )

        if minimum_rotation_deg <= 0.0:
            raise ValueError("minimum_rotation_deg must be greater than zero")

        self.task = task
        self.publish_rate_hz = float(publish_rate_hz)

        # TF frames.
        self.base_frame = "polhemus_base"
        self.object_frame = "sensor4"

        # Cylinder geometry in meters.
        self.object_diameter = float(diameter_mm) / 1000.0
        self.object_height = float(height_mm) / 1000.0
        self.guide_length = float(guide_length_mm) / 1000.0

        self.axis_tolerance_deg = float(axis_tolerance_deg)

        # Below this incremental angle, the measured rotation axis is too
        # sensitive to noise and is treated as undefined.
        self.minimum_rotation_deg = float(minimum_rotation_deg)
        self.minimum_rotation_rad = math.radians(
            self.minimum_rotation_deg
        )

        # Reject stale TF data.
        self.maximum_tf_age_sec = 1.5

        # True: sensor4 is mounted at the cylinder's bottom center and its
        # local +z axis points from the sensor toward the cylinder center.
        # False: sensor4 is located directly at the cylinder center.
        self.sensor_at_bottom = bool(sensor_at_bottom)

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

        # TF listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # RViz marker publisher.
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/ERIE_Manipulation/object_axis_visualization",
            10,
        )

        # This service discards the saved previous orientation. The next valid
        # TF sample becomes the new previous sample.
        self.reset_service = self.create_service(
            Trigger,
            "/ERIE_Manipulation/reset_axis_visualization_reference",
            self.reset_reference_callback,
        )

        # R_base_from_object_previous.
        self.previous_rotation: Optional[np.ndarray] = None
        self.reset_requested = False

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Incremental RViz object-axis visualizer started."
        )
        self.get_logger().info(
            f"Task: {self.task}; axis tolerance: "
            f"{self.axis_tolerance_deg:.1f} deg; minimum rotation: "
            f"{self.minimum_rotation_deg:.3f} deg"
        )
        self.get_logger().info(
            "Rotation error is calculated from the previous and current "
            "TF samples. Initial translation and orientation are not used."
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
            sensor_position_base: shape (3,)
            R_base_from_object_current: shape (3, 3)
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
        sensor_position = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

        q = transform.transform.rotation
        quaternion = np.array(
            [q.x, q.y, q.z, q.w],
            dtype=np.float64,
        )

        quaternion_norm = float(np.linalg.norm(quaternion))

        if quaternion_norm < 1e-12:
            self.get_logger().warning(
                "Received a zero-length TF quaternion.",
                throttle_duration_sec=1.0,
            )
            return None

        quaternion /= quaternion_norm
        current_rotation = Rotation.from_quat(quaternion).as_matrix()

        if not (
            np.all(np.isfinite(sensor_position))
            and np.all(np.isfinite(current_rotation))
        ):
            self.get_logger().warning(
                "Received non-finite sensor4 TF values.",
                throttle_duration_sec=1.0,
            )
            return None

        return sensor_position, current_rotation

    def object_center(
        self,
        sensor_position: np.ndarray,
        object_rotation: np.ndarray,
    ) -> np.ndarray:
        """Return the displayed cylinder center in the base frame."""
        if self.sensor_at_bottom:
            # sensor4 is at the bottom center. Move half the cylinder height
            # along the object's local +z axis to reach the cylinder center.
            local_offset = np.array(
                [0.0, 0.0, self.object_height / 2.0],
                dtype=np.float64,
            )
        else:
            # sensor4 is already at the cylinder center.
            local_offset = np.zeros(3, dtype=np.float64)

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
        """Return the line-angle error from one normalized axis component."""
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

        # The marker disappears shortly after this node stops publishing.
        marker.lifetime = Duration(seconds=0.25).to_msg()
        return marker

    def cylinder_marker(
        self,
        marker_id: int,
        center: np.ndarray,
        rotation: np.ndarray,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = self.base_marker(
            marker_id,
            Marker.CYLINDER,
            "current_object",
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
        """Construct a closed cone using a TRIANGLE_LIST marker."""
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

            # Base-cap triangle.
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
        center: np.ndarray,
        rotation: np.ndarray,
        length: float,
    ) -> List[Marker]:
        """Create x, y, and z arrows for the current object frame."""
        colors = [
            (1.0, 0.0, 0.0, 0.95),  # x
            (0.0, 1.0, 0.0, 0.95),  # y
            (0.0, 0.4, 1.0, 0.95),  # z
        ]

        markers: List[Marker] = []

        for index in range(3):
            local_axis = np.zeros(3, dtype=np.float64)
            local_axis[index] = 1.0

            axis_base = rotation @ local_axis
            end = center + length * axis_base

            markers.append(
                self.arrow_marker(
                    marker_id=start_id + index,
                    namespace="current_frame",
                    start=center,
                    end=end,
                    rgba=colors[index],
                    shaft_diameter=0.004,
                    head_diameter=0.010,
                )
            )

        return markers

    # ------------------------------------------------------------------
    # Reset service
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
            "The previous-sample reference will reset on the next valid TF "
            "sample."
        )
        return response

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

        # The first valid sample, or the first sample after a reset, becomes
        # the previous orientation for the next calculation.
        if self.previous_rotation is None or self.reset_requested:
            self.previous_rotation = current_rotation.copy()
            self.reset_requested = False
            self.get_logger().info(
                "Stored a new previous object orientation."
            )
            return

        # Keep a local copy because self.previous_rotation is replaced at the
        # end of this callback.
        previous_rotation = self.previous_rotation.copy()

        # --------------------------------------------------------------
        # Relative rotation between the last two orientation samples
        # --------------------------------------------------------------
        #
        # R_previous_from_current
        #     = R_base_from_previous.T @ R_base_from_current
        #
        relative_matrix = previous_rotation.T @ current_rotation
        relative_rotation = Rotation.from_matrix(relative_matrix)

        # rotation_vector_previous = measured_axis_previous * angle_rad
        rotation_vector_previous = relative_rotation.as_rotvec()
        rotation_angle_rad = float(
            np.linalg.norm(rotation_vector_previous)
        )
        rotation_angle_deg = math.degrees(rotation_angle_rad)

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

            # Convert the measured axis from the previous object frame to the
            # fixed polhemus_base frame for RViz display.
            measured_axis_base = (
                previous_rotation @ measured_axis_previous
            )
            measured_axis_base /= np.linalg.norm(
                measured_axis_base
            )

        # The desired axis is taken from the PREVIOUS object orientation.
        # It is not tied to the initial set-down orientation.
        desired_axis_base = (
            previous_rotation @ self.local_axes[self.task]
        )
        desired_axis_base /= np.linalg.norm(desired_axis_base)

        # The error treats +axis and -axis as the same physical axis line.
        # Flip only the displayed measured arrow toward the closest side of
        # the desired axis line.
        if (
            measured_axis_base is not None
            and np.dot(measured_axis_base, desired_axis_base) < 0.0
        ):
            measured_axis_base = -measured_axis_base

        axis_available = measured_axis_base is not None
        axis_ok = (
            axis_available
            and task_error_deg <= self.axis_tolerance_deg
        )

        if axis_available:
            status_color = (
                (0.0, 0.85, 0.15, 0.90)
                if axis_ok
                else (1.0, 0.05, 0.05, 0.90)
            )
            status_word = "GOOD" if axis_ok else "OUTSIDE GUIDE"
        else:
            # With essentially zero incremental rotation, there is no unique
            # measured rotation axis.
            status_color = (0.1, 0.45, 1.0, 0.90)
            status_word = "WAITING FOR ROTATION"

        markers: List[Marker] = []

        # --------------------------------------------------------------
        # Current cylinder and current object frame
        # --------------------------------------------------------------
        markers.append(
            self.cylinder_marker(
                marker_id=0,
                center=current_center,
                rotation=current_rotation,
                rgba=status_color,
            )
        )

        frame_axis_length = max(
            self.object_diameter * 0.9,
            0.06,
        )

        markers.extend(
            self.frame_axis_markers(
                start_id=10,
                center=current_center,
                rotation=current_rotation,
                length=frame_axis_length,
            )
        )

        # --------------------------------------------------------------
        # Desired previous-frame task-axis tolerance cone
        # --------------------------------------------------------------
        # The cone follows the current cylinder center, but its orientation
        # comes from the previous sample's object frame.
        cone_color = (0.1, 0.9, 0.2, 0.16)

        markers.append(
            self.cone_marker(
                marker_id=30,
                namespace="axis_tolerance_cone_positive",
                apex=current_center,
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
                apex=current_center,
                axis=-desired_axis_base,
                length=self.guide_length,
                half_angle_deg=self.axis_tolerance_deg,
                rgba=cone_color,
            )
        )

        markers.append(
            self.arrow_marker(
                marker_id=32,
                namespace="desired_task_axis",
                start=current_center,
                end=(
                    current_center
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
        # Live status text
        # --------------------------------------------------------------
        text_position = (
            current_center
            + np.array(
                [
                    0.0,
                    0.0,
                    self.object_height * 0.85 + 0.06,
                ],
                dtype=np.float64,
            )
        )

        # Keep the RViz text simple: show only the three axis errors and
        # the selected task-axis error. The marker color still indicates
        # whether the selected task error is inside or outside the tolerance.
        if axis_available:
            error_text = (
                f"X axis error: {x_error_deg:5.1f} deg\n"
                f"Y axis error: {y_error_deg:5.1f} deg\n"
                f"Z axis error: {z_error_deg:5.1f} deg\n"
                f"{self.task.upper()} task error: "
                f"{task_error_deg:5.1f} deg"
            )
        else:
            error_text = (
                "X axis error: undefined\n"
                "Y axis error: undefined\n"
                "Z axis error: undefined\n"
                f"{self.task.upper()} task error: undefined"
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

        # The current orientation becomes the previous orientation for the
        # next timer callback.
        self.previous_rotation = current_rotation.copy()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Display incremental object rotation-axis guidance in RViz2 "
            "using the previous and current sensor4 TF samples."
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
        help="Calculation and marker update rate in Hz. Default: 30",
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
        help="Allowed half-angle around the desired axis. Default: 15",
    )

    parser.add_argument(
        "--guide-length-mm",
        type=float,
        default=180.0,
        help="Tolerance cone and axis-arrow length. Default: 180",
    )

    parser.add_argument(
        "--minimum-rotation-deg",
        type=float,
        default=0.05,
        help=(
            "Minimum incremental rotation required to calculate an axis. "
            "Default: 0.05"
        ),
    )

    parser.add_argument(
        "--sensor-at-center",
        action="store_true",
        help=(
            "Use this only when sensor4 is located at the cylinder center. "
            "By default sensor4 is assumed to be at the bottom center."
        ),
    )

    arguments, _ = parser.parse_known_args()
    return arguments


def main(args=None) -> None:
    command_line_arguments = parse_arguments()

    rclpy.init(args=args)

    node = ObjectAxisRvizIncremental(
        task=command_line_arguments.task,
        publish_rate_hz=command_line_arguments.rate,
        diameter_mm=command_line_arguments.diameter_mm,
        height_mm=command_line_arguments.height_mm,
        axis_tolerance_deg=(
            command_line_arguments.axis_tolerance_deg
        ),
        guide_length_mm=command_line_arguments.guide_length_mm,
        minimum_rotation_deg=(
            command_line_arguments.minimum_rotation_deg
        ),
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
