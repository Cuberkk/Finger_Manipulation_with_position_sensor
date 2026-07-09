#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R


class ObjectAxisErrorNode(Node):
    """
    Estimate object rotation-axis error using the object position sensor.

    Main idea:
        1. Read object orientation from TF:
              polhemus_base <- sensor4

        2. Compute incremental rotation:
              R_inc = R_current @ R_previous.T

        3. Convert incremental rotation into a rotation vector.
           The direction of this vector is the measured axis of rotation.

        4. Compare the measured axis to the desired task axis:
              roll  -> object x-axis
              pitch -> object y-axis
              yaw   -> object z-axis

        5. Publish axis error in degrees.
    """

    def __init__(self):
        super().__init__("object_axis_error_node")

        # -----------------------------
        # Frames
        # -----------------------------
        self.base_frame = "polhemus_base"
        self.object_frame = "sensor4"

        # -----------------------------
        # Choose task axis
        # -----------------------------
        # Change this to "roll", "pitch", or "yaw"
        self.task = "yaw"

        self.task_axes_object = {
            "roll": np.array([1.0, 0.0, 0.0]),   # object x-axis
            "pitch": np.array([0.0, 1.0, 0.0]),  # object y-axis
            "yaw": np.array([0.0, 0.0, 1.0]),    # object z-axis
        }

        if self.task not in self.task_axes_object:
            raise ValueError(f"Invalid task: {self.task}")

        self.desired_axis_object = self.task_axes_object[self.task]
        self.desired_axis_object = self.desired_axis_object / np.linalg.norm(
            self.desired_axis_object
        )

        # -----------------------------
        # Settings
        # -----------------------------
        self.sample_rate = 100.0

        # Ignore very tiny rotations because noise can make the axis unstable.
        self.min_rotation_deg = 0.25
        self.min_rotation_rad = np.deg2rad(self.min_rotation_deg)

        # If True, +axis and -axis count as the same axis.
        # This is useful because rotating clockwise vs counterclockwise
        # should usually still count as the same intended axis.
        self.sign_insensitive_axis = True

        # -----------------------------
        # TF listener
        # -----------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Publisher
        # -----------------------------
        self.error_pub = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/object_axis_error",
            1000,
        )

        # -----------------------------
        # Stored rotations
        # -----------------------------
        self.R_base_object_initial = None
        self.R_base_object_previous = None

        self.timer = self.create_timer(
            1.0 / self.sample_rate,
            self.timer_callback,
        )

        self.get_logger().info(
            f"Object axis error node started. Task = {self.task}, "
            f"desired object-frame axis = {self.desired_axis_object}"
        )

    def get_object_rotation(self):
        """
        Returns R_base_object.

        This rotation matrix maps a vector from the object frame into
        the polhemus_base frame.

        Example:
            v_base = R_base_object @ v_object
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.object_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )

        except Exception as exc:
            self.get_logger().warn(
                f"Could not get TF {self.base_frame} <- {self.object_frame}: {exc}"
            )
            return None

        q = transform.transform.rotation

        quat_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

        R_base_object = R.from_quat(quat_xyzw).as_matrix()

        return R_base_object

    def timer_callback(self):
        R_base_object_current = self.get_object_rotation()

        if R_base_object_current is None:
            return

        # Save the initial object orientation.
        # This defines the reference object frame for the task.
        if self.R_base_object_initial is None:
            self.R_base_object_initial = R_base_object_current
            self.R_base_object_previous = R_base_object_current

            self.get_logger().info(
                "Saved initial object orientation as reference frame."
            )
            return

        # -------------------------------------------------
        # 1. Compute incremental rotation in the base frame
        # -------------------------------------------------
        # This is the rotation from the previous object orientation
        # to the current object orientation.
        R_increment_base = (
            R_base_object_current @ self.R_base_object_previous.T
        )

        rotvec_base = R.from_matrix(R_increment_base).as_rotvec()
        rotation_amount_rad = np.linalg.norm(rotvec_base)

        # Ignore tiny changes because the axis is unreliable when rotation is near zero.
        if rotation_amount_rad < self.min_rotation_rad:
            self.R_base_object_previous = R_base_object_current
            return

        # -------------------------------------------------
        # 2. Express rotation vector in the initial object frame
        # -------------------------------------------------
        # This makes the error easier to interpret:
        # x = roll-like motion
        # y = pitch-like motion
        # z = yaw-like motion
        rotvec_object = self.R_base_object_initial.T @ rotvec_base

        # -------------------------------------------------
        # 3. Separate desired-axis motion from off-axis motion
        # -------------------------------------------------
        desired_component_rad = np.dot(
            rotvec_object,
            self.desired_axis_object,
        )

        desired_rotation_vec = (
            desired_component_rad * self.desired_axis_object
        )

        off_axis_vec = rotvec_object - desired_rotation_vec
        off_axis_amount_rad = np.linalg.norm(off_axis_vec)

        # -------------------------------------------------
        # 4. Compute axis error angle
        # -------------------------------------------------
        if self.sign_insensitive_axis:
            desired_amount_rad = abs(desired_component_rad)
        else:
            desired_amount_rad = desired_component_rad

        axis_error_rad = np.arctan2(
            off_axis_amount_rad,
            abs(desired_amount_rad),
        )

        axis_error_deg = np.rad2deg(axis_error_rad)
        rotation_amount_deg = np.rad2deg(rotation_amount_rad)
        desired_component_deg = np.rad2deg(desired_component_rad)
        off_axis_amount_deg = np.rad2deg(off_axis_amount_rad)

        # Actual measured axis in object reference frame
        actual_axis_object = rotvec_object / rotation_amount_rad

        # -------------------------------------------------
        # 5. Publish result
        # -------------------------------------------------
        timestamp_sec = self.get_clock().now().nanoseconds * 1e-9

        msg = Float64MultiArray()
        msg.data = [
            timestamp_sec,

            axis_error_deg,
            rotation_amount_deg,

            desired_component_deg,
            off_axis_amount_deg,

            actual_axis_object[0],
            actual_axis_object[1],
            actual_axis_object[2],
        ]

        self.error_pub.publish(msg)

        self.get_logger().info(
            f"Axis error: {axis_error_deg:.2f} deg | "
            f"total rot: {rotation_amount_deg:.2f} deg | "
            f"desired: {desired_component_deg:.2f} deg | "
            f"off-axis: {off_axis_amount_deg:.2f} deg | "
            f"axis obj: [{actual_axis_object[0]:.2f}, "
            f"{actual_axis_object[1]:.2f}, "
            f"{actual_axis_object[2]:.2f}]"
        )

        self.R_base_object_previous = R_base_object_current


def main(args=None):
    rclpy.init(args=args)

    node = ObjectAxisErrorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()