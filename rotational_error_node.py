import rclpy
from rclpy.time import Time
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
import argparse

class RotationalError(Node):

    def __init__(self, sensor_number = 1):
        super().__init__('rotational_calibration_node')

        self.polhemus_base_frame = 'polhemus_base'
        self.finger_position_frame = f'sensor{sensor_number}'

        # Maximum allowed TF transform age in seconds.
        self.trans_delay = 1.5

        self.rotational_gt = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0,-1.0]
        ])

        # ─────────────────────────────────────────────────────────────────────
        # ROS publisher and timer
        # ─────────────────────────────────────────────────────────────────────

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            f'ERIE_Manipulation/rotational_error/sensor{sensor_number}',
            1
        )

        self.timer = self.create_timer(
            1./100.,
            self.timer_callback
        )

        # ─────────────────────────────────────────────────────────────────────
        # TF setup
        # ─────────────────────────────────────────────────────────────────────

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Rotational error node initialized. Calibration sensor {sensor_number}"
        )

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            t_base_finger = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.finger_position_frame,
                rclpy.time.Time()
            )

            # Check transform age.
            if not self.transform_is_recent(t_base_finger, now):
                self.get_logger().warn("Transform is too old.")
                return

            rot_curr = self.transform_rot_generator(t_base_finger)
            # print(f"Current rotation matrix:\n{rot_curr}")
            err = np.trace(self.rotational_gt.T @ rot_curr) - 3

            data_msg = Float64MultiArray(
                data=np.array([err], dtype=np.float64).flatten().tolist()
            )

            self.error_pub.publish(data_msg)
            self.get_logger().info(f"Current rotational error: {err:.4f}")

        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"Transform not received: {exc}. "
            )

    def transform_is_recent(self, trans_msg, now):
        """
        Check whether a TF transform is recent enough to use.
        """

        tf_time = trans_msg.header.stamp
        tf_stamp = Time(
            seconds=tf_time.sec,
            nanoseconds=tf_time.nanosec
        )

        dt = abs(now.nanoseconds - tf_stamp.nanoseconds) * 1e-9
        print(f"Transform age: {dt:.3f} seconds")

        return dt < self.trans_delay

    def transform_rot_generator(self, trans_msg):
        """
        Extract quaternion from a TF transform and convert it to a 3x3
        rotation matrix.
        """

        q = trans_msg.transform.rotation

        rotation = R.from_quat([
            q.x,
            q.y,
            q.z,
            q.w
        ]).as_matrix()

        return rotation

def main(sensor_number):
    rclpy.init()

    node = RotationalError(sensor_number=sensor_number)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Shutting down the rotational error node...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Init the Rotational Error Node")
    # Add argument which takes path to a bag file as an input
    parser.add_argument("-s", "--sensor_number", type=int, default=1, help="Sensor number")
    args = parser.parse_args()
    main(args.sensor_number)