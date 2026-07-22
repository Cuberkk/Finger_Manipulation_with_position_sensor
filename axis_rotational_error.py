#!/usr/bin/env python3

#ros2 topic echo /ERIE_Manipulation/axis_rotational_error
#python3 object_axis_rotational_error_realtime.py --rate 50
"""
object_axis_rotational_error_realtime.py

Calculates the object's incremental rotation from the previous TF sample to
the current TF sample:

    R_previous_from_current
        = R_base_from_previous.T @ R_base_from_current

The relative rotation is converted to a measured rotation axis:

    measured_axis = [axis_x, axis_y, axis_z]

The angular error between the measured rotation axis and each object axis is:

    x_error_deg = degrees(acos(abs(axis_x)))
    y_error_deg = degrees(acos(abs(axis_y)))
    z_error_deg = degrees(acos(abs(axis_z)))

The absolute value treats +x and -x as the same physical axis line, and does
the same for y and z. Clockwise and counterclockwise rotations therefore have
the same axis-alignment error.

Published topics
----------------

1. Combined axis errors:

    /ERIE_Manipulation/axis_rotational_error

    Float64MultiArray order:

        [x_axis_error_deg, y_axis_error_deg, z_axis_error_deg]

2. Individual error topics:

    /ERIE_Manipulation/axis_rotational_error/x
    /ERIE_Manipulation/axis_rotational_error/y
    /ERIE_Manipulation/axis_rotational_error/z

3. Relative quaternion:

    /ERIE_Manipulation/object_relative_quaternion

    Float64MultiArray order:

        [qx, qy, qz, qw]

The quaternion w value is included on the quaternion topic, but w is not a
rotation axis and therefore does not have an axis-error value.

4. Signed rotation-vector components:

    /ERIE_Manipulation/object_rotation_components_deg

    Float64MultiArray order:

        [rotation_x_deg, rotation_y_deg, rotation_z_deg]

These are the signed incremental rotation-vector components, not the
axis-alignment errors.

Run:

    python3 object_axis_rotational_error_realtime.py

Optional rate:

    python3 object_axis_rotational_error_realtime.py --rate 100

Echo the combined errors:

    ros2 topic echo /ERIE_Manipulation/axis_rotational_error
"""

import argparse
import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class ObjectAxisRotationalErrorNode(Node):
    """Publish live object rotation-axis error for x, y, and z."""

    def __init__(self, publish_rate_hz: float = 100.0) -> None:
        super().__init__("object_axis_rotational_error_node")

        # --------------------------------------------------------------
        # TF frames
        # --------------------------------------------------------------
        self.base_frame = "polhemus_base"
        self.object_frame = "sensor4"

        # --------------------------------------------------------------
        # Timing and numerical settings
        # --------------------------------------------------------------
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero.")

        self.publish_rate_hz = float(publish_rate_hz)

        # Reject TF data older than this value.
        self.maximum_tf_age_sec = 1.5

        # Below this angle, the measured rotation axis is not reliable.
        # When this happens, the code publishes NaN for each axis error
        # instead of incorrectly publishing zero error.
        self.minimum_rotation_deg = 0.01
        self.minimum_rotation_rad = math.radians(
            self.minimum_rotation_deg
        )

        # --------------------------------------------------------------
        # TF listener
        # --------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # --------------------------------------------------------------
        # Publishers
        # --------------------------------------------------------------

        # Combined message:
        # [x_axis_error_deg, y_axis_error_deg, z_axis_error_deg]
        self.axis_error_publisher = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/axis_rotational_error",
            10,
        )

        # Individual axis-error topics.
        self.x_error_publisher = self.create_publisher(
            Float64,
            "/ERIE_Manipulation/axis_rotational_error/x",
            10,
        )
        self.y_error_publisher = self.create_publisher(
            Float64,
            "/ERIE_Manipulation/axis_rotational_error/y",
            10,
        )
        self.z_error_publisher = self.create_publisher(
            Float64,
            "/ERIE_Manipulation/axis_rotational_error/z",
            10,
        )

        # Relative quaternion:
        # [qx, qy, qz, qw]
        self.quaternion_publisher = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/object_relative_quaternion",
            10,
        )

        # Signed incremental rotation-vector components in degrees:
        # [rotation_x_deg, rotation_y_deg, rotation_z_deg]
        self.rotation_components_publisher = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/object_rotation_components_deg",
            10,
        )

        # R_base_from_object_previous.
        # The first valid TF sample initializes this matrix.
        self.R_previous: Optional[np.ndarray] = None

        # --------------------------------------------------------------
        # Timer
        # --------------------------------------------------------------
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Object rotation-axis error node started."
        )
        self.get_logger().info(
            "Combined error order: "
            "[x_axis_error_deg, y_axis_error_deg, z_axis_error_deg]"
        )
        self.get_logger().info(
            f"Publishing at {self.publish_rate_hz:.1f} Hz."
        )

    def lookup_current_object_rotation(self) -> Optional[np.ndarray]:
        """
        Return R_base_from_object_current from the latest sensor4 TF.
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
        transform_age_sec = (
            self.get_clock().now() - transform_time
        ).nanoseconds * 1e-9

        if transform_age_sec > self.maximum_tf_age_sec:
            self.get_logger().warning(
                f"TF is too old: {transform_age_sec:.3f} s",
                throttle_duration_sec=1.0,
            )
            return None

        q = transform.transform.rotation

        # SciPy quaternion order is [x, y, z, w].
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

        # TF lookup polhemus_base <- sensor4 gives:
        #
        #     R_base_from_object_current
        #
        return Rotation.from_quat(quaternion).as_matrix()

    @staticmethod
    def axis_error_degrees(axis_component: float) -> float:
        """
        Calculate angular error relative to one coordinate-axis line.

        For a normalized measured axis u:

            x error = acos(abs(u_x))
            y error = acos(abs(u_y))
            z error = acos(abs(u_z))
        """
        component = float(
            np.clip(abs(axis_component), 0.0, 1.0)
        )
        return math.degrees(math.acos(component))

    def publish_axis_errors(
        self,
        x_error_deg: float,
        y_error_deg: float,
        z_error_deg: float,
    ) -> None:
        """Publish combined and individual x/y/z axis errors."""

        combined_message = Float64MultiArray()
        combined_message.data = [
            float(x_error_deg),
            float(y_error_deg),
            float(z_error_deg),
        ]
        self.axis_error_publisher.publish(combined_message)

        x_message = Float64()
        x_message.data = float(x_error_deg)
        self.x_error_publisher.publish(x_message)

        y_message = Float64()
        y_message.data = float(y_error_deg)
        self.y_error_publisher.publish(y_message)

        z_message = Float64()
        z_message.data = float(z_error_deg)
        self.z_error_publisher.publish(z_message)

    def timer_callback(self) -> None:
        """Calculate and publish the latest incremental rotation errors."""

        R_current = self.lookup_current_object_rotation()

        if R_current is None:
            return

        # The first sample provides the previous orientation.
        if self.R_previous is None:
            self.R_previous = R_current.copy()
            self.get_logger().info(
                "Stored the first object orientation."
            )
            return

        # --------------------------------------------------------------
        # Relative rotation
        # --------------------------------------------------------------
        #
        # R_previous_from_current
        #     = R_previous_from_base @ R_base_from_current
        #
        # R_previous_from_base
        #     = R_base_from_previous.T
        #
        # Therefore:
        #
        # R_previous_from_current
        #     = R_base_from_previous.T @ R_base_from_current
        #
        R_previous_from_current = (
            self.R_previous.T @ R_current
        )

        relative_rotation = Rotation.from_matrix(
            R_previous_from_current
        )

        # --------------------------------------------------------------
        # Relative quaternion [qx, qy, qz, qw]
        # --------------------------------------------------------------
        qx, qy, qz, qw = relative_rotation.as_quat()

        # q and -q represent the same rotation. This keeps the output
        # representation consistent.
        if qw < 0.0:
            qx = -qx
            qy = -qy
            qz = -qz
            qw = -qw

        quaternion_message = Float64MultiArray()
        quaternion_message.data = [
            float(qx),
            float(qy),
            float(qz),
            float(qw),
        ]
        self.quaternion_publisher.publish(quaternion_message)

        # --------------------------------------------------------------
        # Rotation-vector decomposition
        # --------------------------------------------------------------
        #
        # rotation_vector = measured_axis * angle_rad
        #
        rotation_vector_rad = relative_rotation.as_rotvec()
        rotation_angle_rad = float(
            np.linalg.norm(rotation_vector_rad)
        )

        # Signed x/y/z rotation-vector components in degrees.
        rotation_vector_deg = np.degrees(rotation_vector_rad)

        rotation_components_message = Float64MultiArray()
        rotation_components_message.data = [
            float(rotation_vector_deg[0]),
            float(rotation_vector_deg[1]),
            float(rotation_vector_deg[2]),
        ]
        self.rotation_components_publisher.publish(
            rotation_components_message
        )

        # --------------------------------------------------------------
        # Axis errors
        # --------------------------------------------------------------
        if rotation_angle_rad >= self.minimum_rotation_rad:
            measured_axis = (
                rotation_vector_rad / rotation_angle_rad
            )

            axis_x = float(measured_axis[0])
            axis_y = float(measured_axis[1])
            axis_z = float(measured_axis[2])

            x_axis_error_deg = self.axis_error_degrees(axis_x)
            y_axis_error_deg = self.axis_error_degrees(axis_y)
            z_axis_error_deg = self.axis_error_degrees(axis_z)
        else:
            # When there is essentially no rotation, there is no uniquely
            # defined rotation axis. NaN accurately indicates that the
            # three axis-error values are unavailable for this sample.
            x_axis_error_deg = math.nan
            y_axis_error_deg = math.nan
            z_axis_error_deg = math.nan

        self.publish_axis_errors(
            x_axis_error_deg,
            y_axis_error_deg,
            z_axis_error_deg,
        )

        # The current sample becomes the previous sample for the next cycle.
        self.R_previous = R_current.copy()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish live x/y/z object rotation-axis errors from sensor4 TF."
        )
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="Calculation and publication rate in Hz. Default: 100",
    )

    arguments, _ = parser.parse_known_args()
    return arguments


def main(args=None) -> None:
    command_line_arguments = parse_arguments()

    rclpy.init(args=args)

    node = ObjectAxisRotationalErrorNode(
        publish_rate_hz=command_line_arguments.rate
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