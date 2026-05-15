import rclpy
from rclpy.time import Time
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

# Make sure this filename matches the file that contains your NIDAQReaderDual class.
# Example: if your reader file is named NIDAQReader.py, keep this line.
from utils.NIDAQReaderDual import NIDAQReaderDual


class NIDAQATINode(Node):
    """
    ROS 2 publisher node for two NI-DAQ ATI force/torque sensors.

    Reader output from NIDAQReaderDual.read():
        Sensor 1: indices 0:6  -> [Fx1, Fy1, Fz1, Tx1, Ty1, Tz1]
        Sensor 2: indices 6:12 -> [Fx2, Fy2, Fz2, Tx2, Ty2, Tz2]

    Published Float64MultiArray by default:
        [Fx1_rot, Fy1_rot, Fz1_rot, Fx2_rot, Fy2_rot, Fz2_rot, timestamp_sec]

    Topic:
        nidaq_ati_data
    """

    def __init__(self):
        super().__init__('nidaq_ati_node')

        # ─────────────────────────────────────────────────────────────────────
        # User settings
        # ─────────────────────────────────────────────────────────────────────
        self.cal1_path = 'calibration_files/FT44298.cal'
        self.cal2_path = 'calibration_files/FT45281.cal'

        # This should match your NI hardware channel setup.
        # Your NIDAQReaderDual default is:
        # "Dev1/ai0:7,Dev1/ai16:19"
        self.phys_channels = 'Dev1/ai0:7,Dev1/ai16:19'

        # Publish/read rate in Hz.
        self.publish_rate = 60.0

        # Bias settings.
        self.bias_time = 5
        self.compute_bias_on_start = True

        # TF frame names.
        # Change these to match your actual TF tree.
        self.gripper_base_frame = 'KWGripperBase'
        self.sensor1_frame = 'KWATI1'
        self.sensor2_frame = 'KWATI2'

        # If both NI sensors use the same ATI frame in TF, set both frames equal.
        # Example:
        # self.sensor1_frame = 'KWATI'
        # self.sensor2_frame = 'KWATI'

        # Rotation from gripper base frame to final/global desired frame.
        # This is copied from your ATI_publisher.py.
        self.rot_gl_gbase = np.array([
            [1., 0., 0.],
            [0., -0.74895572, 0.66262005],
            [0., -0.66262005, -0.74895572]
        ])

        # Maximum allowed transform age in seconds.
        self.trans_delay = 1.5

        # ─────────────────────────────────────────────────────────────────────
        # NI-DAQ reader
        # ─────────────────────────────────────────────────────────────────────
        self.nidaq_reader = NIDAQReaderDual(
            cal1_path=self.cal1_path,
            cal2_path=self.cal2_path,
            aq_rate=self.publish_rate,
            phys=self.phys_channels,
            bias_time=self.bias_time,
            bias_switch=self.compute_bias_on_start,
        )

        # ─────────────────────────────────────────────────────────────────────
        # ROS publisher and timer
        # ─────────────────────────────────────────────────────────────────────
        self.nidaq_pub = self.create_publisher(Float64MultiArray, 'nidaq_ati_data', 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # ─────────────────────────────────────────────────────────────────────
        # TF setup
        # ─────────────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.miss_itr = 0
        self.total_itr = 0

        self.get_logger().info("NI-DAQ ATI Node Initialized")

    def timer_callback(self):
        """
        Read two NI sensors, rotate each force vector into the final desired frame,
        append a timestamp, and publish as Float64MultiArray.
        """
        self.total_itr += 1

        # Reader returns 12 values:
        # [Fx1,Fy1,Fz1,Tx1,Ty1,Tz1,Fx2,Fy2,Fz2,Tx2,Ty2,Tz2]
        ft_arr = self.nidaq_reader.read()

        # Extract force vectors only.
        force_s1 = np.asarray(ft_arr[0:3], dtype=np.float64)
        force_s2 = np.asarray(ft_arr[6:9], dtype=np.float64)

        timestamp = self.get_clock().now().to_msg()
        timestamp_sec = np.float64(timestamp.sec + 1e-9 * timestamp.nanosec)

        try:
            now = self.get_clock().now()

            # Look up current orientation from each ATI sensor frame to gripper base frame.
            t_gbase_s1 = self.tf_buffer.lookup_transform(
                self.gripper_base_frame,
                self.sensor1_frame,
                rclpy.time.Time()
            )

            t_gbase_s2 = self.tf_buffer.lookup_transform(
                self.gripper_base_frame,
                self.sensor2_frame,
                rclpy.time.Time()
            )

            # Check transform age.
            if not self.transform_is_recent(t_gbase_s1, now):
                self.miss_itr += 1
                self.get_logger().warn(
                    f"Sensor 1 transform too old. Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
                )
                return

            if not self.transform_is_recent(t_gbase_s2, now):
                self.miss_itr += 1
                self.get_logger().warn(
                    f"Sensor 2 transform too old. Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
                )
                return

            # Convert TF quaternions to rotation matrices.
            rot_gbase_s1 = self.transform_rot_generator(t_gbase_s1)
            rot_gbase_s2 = self.transform_rot_generator(t_gbase_s2)

            # Rotate each force vector:
            # sensor frame -> gripper base frame -> final/global desired frame
            force_s1_rot = (self.rot_gl_gbase @ rot_gbase_s1 @ force_s1.T).T
            force_s2_rot = (self.rot_gl_gbase @ rot_gbase_s2 @ force_s2.T).T

            # Publish [Fx1,Fy1,Fz1,Fx2,Fy2,Fz2,timestamp]
            data_arr = np.concatenate([
                force_s1_rot.astype(np.float64),
                force_s2_rot.astype(np.float64),
                np.array([timestamp_sec], dtype=np.float64)
            ])

            data_msg = Float64MultiArray(data=data_arr.flatten().tolist())
            self.nidaq_pub.publish(data_msg)

        except tf2_ros.TransformException as exc:
            self.miss_itr += 1
            self.get_logger().warn(
                f"Transform not received: {exc}. Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
            )

    def transform_is_recent(self, trans_msg, now):
        """
        Check whether a TF transform is recent enough to use.
        """
        tf_time = trans_msg.header.stamp
        tf_stamp = Time(seconds=tf_time.sec, nanoseconds=tf_time.nanosec)
        dt = abs(now.nanoseconds - tf_stamp.nanoseconds) * 1e-9
        return dt < self.trans_delay

    def transform_rot_generator(self, trans_msg):
        """
        Extract quaternion from a TF transform and convert it to a 3x3 rotation matrix.
        """
        q = trans_msg.transform.rotation
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return rotation

    def destroy_node(self):
        """
        Close NI task before shutting down ROS node.
        """
        try:
            self.nidaq_reader.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NIDAQATINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down NI-DAQ ATI node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
