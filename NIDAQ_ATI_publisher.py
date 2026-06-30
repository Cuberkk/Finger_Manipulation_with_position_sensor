import rclpy
from rclpy.time import Time
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

# Make sure this filename matches the file that contains your NIDAQReaderDual class.
from utils.NIDAQReaderDual import NIDAQReaderDual


class NIDAQATINode(Node):

    def __init__(self):
        super().__init__('nidaq_ati_node')

        # ─────────────────────────────────────────────────────────────────────
        # NI-DAQ / ATI settings
        # ─────────────────────────────────────────────────────────────────────

        self.cal1_path = 'calibration_files/FT44298.cal'
        self.cal2_path = 'calibration_files/FT45281.cal'

        # Your NIDAQReaderDual default is usually:
        # "Dev1/ai0:7,Dev1/ai16:19"
        self.phys_channels = 'Dev1/ai0:7,Dev1/ai16:19'

        # Publish/read rate in Hz.
        self.publish_rate = 600.0

        # Bias settings.
        self.bias_time = 5
        self.compute_bias_on_start = True

        # ─────────────────────────────────────────────────────────────────────
        # TF / Polhemus Viper frame names
        # ─────────────────────────────────────────────────────────────────────

        # Shared Polhemus tracking/world base.
        self.polhemus_base_frame = 'polhemus_base'

        # NI-DAQ ATI force sensor frames.
        self.sensor1_frame = 'sensor1'
        self.sensor2_frame = 'sensor2'

        # Object position sensor frame.
        # This is the Polhemus Viper sensor attached to the object/cylinder.
        self.object_position_frame = 'sensor4'

        # Individual finger position sensor frames.
        # These now match the Polhemus TF frame names directly.
        self.finger1_position_frame = 'sensor1'
        self.finger2_position_frame = 'sensor2'

        # Physical sensor spacing around the object.
        # sensor1 uses -120 degrees.
        # sensor2 uses 0 degrees.
        self.sensor_angles_deg = {
            'sensor1': -120.0,
            'sensor2': 0.0,
        }

        # Extra ATI sensor z-axis alignment rotation.
        # This is Rz(48 degrees), same idea as the LabJack ATI publisher.
        z_rot_48 = -48.
        z_rot_rad = np.deg2rad(z_rot_48)
        self.sensor_rot_z_48 = np.array([
            [np.cos(z_rot_rad), -np.sin(z_rot_rad), 0.0],
            [np.sin(z_rot_rad),  np.cos(z_rot_rad), 0.0],
            [0.0,                0.0,               1.0]
        ])

        # Precompute manual ATI force-sensor-to-object rotations.
        # sensor1: R_object_force1 = Rz(-120) @ rot_base @ Rz(48)
        # sensor2: R_object_force2 = Rz(+120) @ rot_base @ Rz(48)
        self.rot_obj_force_s1 = self.ati_force_to_object_rotation('sensor1')
        self.rot_obj_force_s2 = self.ati_force_to_object_rotation('sensor2')

        # Maximum allowed TF transform age in seconds.
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

        self.nidaq_pub_s1 = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s1_finger1',
            1
        )

        self.nidaq_pub_s2 = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s2_finger2',
            1
        )
        
        self.nidaq_pub_raw_s1 = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s1_raw',
            1
        )
        
        self.nidaq_pub_raw_s2  = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s2_raw',
            1
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        # ─────────────────────────────────────────────────────────────────────
        # TF setup
        # ─────────────────────────────────────────────────────────────────────

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.miss_itr = 0
        self.total_itr = 0

        self.get_logger().info(
            "NI-DAQ ATI + Polhemus Finger Position Node Initialized"
        )


    def timer_callback(self):
        """
        Main loop.

        1. Stream force vectors from NI-DAQ ATI sensors 1 and 2.
        2. Read Polhemus Viper transforms from TF.
        3. Compute:
              force sensor 1 -> finger 1 position sensor frame
              force sensor 2 -> finger 2 position sensor frame
        4. Publish:
              force_s1_finger1, force_s2_finger2, timestamp
        """

        self.total_itr += 1

        # Reader returns 12 values:
        # [Fx1,Fy1,Fz1,Tx1,Ty1,Tz1,
        #  Fx2,Fy2,Fz2,Tx2,Ty2,Tz2]
        ft_arr = self.nidaq_reader.read()
        if ft_arr is False:
            return
        
        # Reader returns 12 values:
        # [Fx1,Fy1,Fz1,Tx1,Ty1,Tz1,
        #  Fx2,Fy2,Fz2,Tx2,Ty2,Tz2]
        ft_s1 = np.asarray(ft_arr[0:6], dtype=np.float64)
        ft_s2 = np.asarray(ft_arr[6:12], dtype=np.float64)

        # Keep force vectors for the transformed force publishing below.
        force_s1 = ft_s1[0:3]
        force_s2 = ft_s2[0:3]

        timestamp = self.get_clock().now().to_msg()
        timestamp_sec = np.float64(
            timestamp.sec + 1e-9 * timestamp.nanosec
        )

        raw_data_msg_f1 = Float64MultiArray(
            data=np.concatenate([
                ft_s1,
                np.array([timestamp_sec], dtype=np.float64)
            ]).flatten().tolist()
        )
        self.nidaq_pub_raw_s1.publish(raw_data_msg_f1)

        raw_data_msg_f2 = Float64MultiArray(
            data=np.concatenate([
                ft_s2,
                np.array([timestamp_sec], dtype=np.float64)
            ]).flatten().tolist()
        )
        self.nidaq_pub_raw_s2.publish(raw_data_msg_f2)

        try:
            now = self.get_clock().now()

            # ─────────────────────────────────────────────────────────────────
            # Look up Polhemus Viper transforms relative to polhemus_base.
            #
            # lookup_transform(target_frame, source_frame, time)
            #
            # lookup_transform(polhemus_base, sensor4, ...)
            # gives:
            #       sensor4/object frame -> polhemus_base frame
            #
            # lookup_transform(polhemus_base, sensor1, ...)
            # gives:
            #       finger 1 position frame -> polhemus_base frame
            #
            # lookup_transform(polhemus_base, sensor2, ...)
            # gives:
            #       finger 2 position frame -> polhemus_base frame
            # ─────────────────────────────────────────────────────────────────

            t_base_obj = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.object_position_frame,
                rclpy.time.Time()
            )

            t_base_finger1 = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.finger1_position_frame,
                rclpy.time.Time()
            )

            t_base_finger2 = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.finger2_position_frame,
                rclpy.time.Time()
            )

            # Check transform age.
            for name, trans in [
                ("object sensor4", t_base_obj),
                ("finger1 position sensor", t_base_finger1),
                ("finger2 position sensor", t_base_finger2),
            ]:
                if not self.transform_is_recent(trans, now):
                    self.handle_old_transform(name)
                    return

            # ─────────────────────────────────────────────────────────────────
            # Convert Polhemus TF transforms into rotation matrices.
            #
            # These rotations map:
            #       object frame -> polhemus_base frame
            #       finger position frame -> polhemus_base frame
            # ─────────────────────────────────────────────────────────────────

            rot_base_obj = self.transform_rot_generator(t_base_obj)
            rot_base_finger1 = self.transform_rot_generator(t_base_finger1)
            rot_base_finger2 = self.transform_rot_generator(t_base_finger2)

            # ─────────────────────────────────────────────────────────────────
            # Compute finger position sensor rotations into object frame.
            #
            # Given:
            #       R_base_obj      = object frame -> polhemus_base frame
            #       R_base_finger   = finger position frame -> polhemus_base frame
            #
            # We want:
            #       R_object_finger = finger position frame -> object frame
            #
            # Since:
            #       R_object_base = R_base_obj.T
            #
            # Therefore:
            #       R_object_finger = R_base_obj.T @ R_base_finger
            # ─────────────────────────────────────────────────────────────────

            rot_obj_finger1 = rot_base_obj.T @ rot_base_finger1
            rot_obj_finger2 = rot_base_obj.T @ rot_base_finger2

            # ─────────────────────────────────────────────────────────────────
            # Compute ATI force sensor rotations into individual finger
            # position sensor frames.
            #
            # sensor1:
            #       R_object_force1 = Rz(-120) @ rot_base @ Rz(48)
            #       R_finger1_force1 = R_object_finger1.T @ R_object_force1
            #
            # sensor2:
            #       R_object_force2 = Rz(+120) @ rot_base @ Rz(48)
            #       R_finger2_force2 = R_object_finger2.T @ R_object_force2
            # ─────────────────────────────────────────────────────────────────

            rot_finger1_force_s1 = rot_obj_finger1.T @ self.rot_obj_force_s1
            rot_finger2_force_s2 = rot_obj_finger2.T @ self.rot_obj_force_s2

            # Rotate force vectors into matching finger position sensor frames.
            force_s1_finger1 = rot_finger1_force_s1 @ force_s1
            force_s2_finger2 = rot_finger2_force_s2 @ force_s2
            
            # ─────────────────────────────────────────────────────────────────
            # Publish data.
            # ─────────────────────────────────────────────────────────────────

            data_s1 = np.concatenate([
                # Force sensor 1 in finger 1 position sensor frame
                force_s1_finger1.astype(np.float64),

                # Timestamp
                np.array([timestamp_sec], dtype=np.float64)
            ])

            data_s2 = np.concatenate([
                # Force sensor 2 in finger 2 position sensor frame
                force_s2_finger2.astype(np.float64),

                # Timestamp
                np.array([timestamp_sec], dtype=np.float64)
            ])

            msg_s1 = Float64MultiArray(data=data_s1.flatten().tolist())
            msg_s2 = Float64MultiArray(data=data_s2.flatten().tolist())

            self.nidaq_pub_s1.publish(msg_s1)
            self.nidaq_pub_s2.publish(msg_s2)

        except tf2_ros.TransformException as exc:
            self.miss_itr += 1
            self.get_logger().warn(
                f"Transform not received: {exc}. "
                f"Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
            )

    def ati_force_to_object_rotation(self, sensor_name):
        """
        Manually define ATI force sensor rotation into the object frame.

        This represents:

            R_object_force

        meaning:

            force sensor frame -> object frame

        For this file:

            sensor1: R_object_force1 = Rz(-120) @ rot_base @ Rz(48)
            sensor2: R_object_force2 = Rz(+120) @ rot_base @ Rz(48)
        """

        theta_deg = self.sensor_angles_deg[sensor_name]
        theta_rad = np.deg2rad(theta_deg)

        rot_z = np.array([
            [np.cos(theta_rad), -np.sin(theta_rad), 0.0],
            [np.sin(theta_rad),  np.cos(theta_rad), 0.0],
            [0.0,                0.0,               1.0]
        ])

        # Base ATI force sensor to object rotation.
        #
        # This maps the ATI local force axes into the object axes.
        # Edit this matrix only if the physical ATI axis directions are different.
        #
        # Matrix:
        # [ 0  0  1]
        # [ 0  1  0]
        # [-1  0  0]
        rot_base = np.array([
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0]
        ])

        rot_obj_force = rot_z.T @ rot_base @ self.sensor_rot_z_48.T

        return rot_obj_force

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

    def transform_translation_generator(self, trans_msg):
        """
        Extract translation from a TF transform and convert it to a 3-element
        numpy vector.
        """

        t = trans_msg.transform.translation

        translation = np.array([
            t.x,
            t.y,
            t.z
        ], dtype=np.float64)

        return translation

    def handle_old_transform(self, sensor_name):
        """
        Handle a stale TF transform.
        """

        self.miss_itr += 1

        self.get_logger().warn(
            f"{sensor_name} transform too old. "
            f"Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
        )

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
        print("Shutting down NI-DAQ ATI + Polhemus Finger Position Node...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
