from datetime import time

import rclpy
from rclpy.time import Time
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from utils.LabjackReader import LabjackATIReader
from scipy.spatial.transform import Rotation as R


class LabjackATINode(Node):
    """
    ROS 2 publisher node for LabJack A
    TI force sensor 3
    plus Polhemus Viper position sensor transforms.

    Force sensor:
        sensor3 = LabJack ATI force sensor for finger 3

    Polhemus Viper position sensors:
        sensor4 = object position sensor
        finger3_position_sensor = finger 3 position sensor

    Main rotations computed:

        1. ATI force sensor 3 frame -> object frame

            R_object_force3

        2. Finger 3 position sensor frame -> object frame

            R_object_finger3pos

        3. ATI force sensor 3 frame -> finger 3 position sensor frame

            R_finger3pos_force3 = R_object_finger3pos.T @ R_object_force3

    Published Float64MultiArray:

        [
            Fx3_obj, Fy3_obj, Fz3_obj,
            Fx3_fingerpos3, Fy3_fingerpos3, Fz3_fingerpos3,
            finger3_x_obj, finger3_y_obj, finger3_z_obj,
            timestamp_sec
        ]

    Topic:
        ati_data
    """

    def __init__(self):
        super().__init__('ati_node')

        # ─────────────────────────────────────────────────────────────────────
        # LabJack ATI settings
        # ─────────────────────────────────────────────────────────────────────

        self.cal_path = 'calibration_files/FT44297.cal'
        self.publish_rate = 600.0

        self.ati_reader = LabjackATIReader(
            cal_path=self.cal_path,
            aq_rate=self.publish_rate,
            bias_frames=30,
            bias_switch=True
        )

        # ─────────────────────────────────────────────────────────────────────
        # TF / Polhemus Viper frame names
        # ─────────────────────────────────────────────────────────────────────

        # Shared Polhemus tracking/world base.
        self.polhemus_base_frame = 'polhemus_base'

        # Object position sensor frame.
        # This is the Polhemus Viper sensor attached to the object/cylinder.
        self.object_position_frame = 'sensor4'

        # Individual finger 3 position sensor frame.
        # Rename this string if your actual TF frame has a different name.
        self.finger3_position_frame = 'sensor3'

        # Sensor 3 is spaced 120 degrees around the object from the reference.
        # If your physical layout is reversed, change this to -120.0.
        self.sensor_angles_deg = {
            'sensor3': 120.0,
        }

        # Maximum allowed TF transform age in seconds.
        self.trans_delay = 1.5

        z_rot_48 = -48.
        z_rot_rad = np.deg2rad(z_rot_48)
        self.sensor_rot_z_48 = np.array([
            [np.cos(z_rot_rad), -np.sin(z_rot_rad), 0.0],
            [np.sin(z_rot_rad),  np.cos(z_rot_rad), 0.0],
            [0.0,                0.0,               1.0]
        ])

        self.rot_obj_force_s3 = self.ati_force_to_object_rotation('sensor3')

        # ─────────────────────────────────────────────────────────────────────
        # ROS publisher and timer
        # ─────────────────────────────────────────────────────────────────────

        self.ati_pub = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/finger3',
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
            "LabJack ATI Sensor 3 + Polhemus Finger 3 Position Node Initialized"
        )

        # ─────────────────────────────────────────────────────────────────────
        # Gravity compensation settings
        # ─────────────────────────────────────────────────────────────────────

        #self.gravity_m_s2 = 9.80665

        # Set this to the effective mass assigned to sensor/finger 3.
        # Do NOT leave this at 0.0 during real data collection.
        #self.mass_s3_kg = 0.0

    def timer_callback(self):
        """
        Main loop.

        1. Read force vector from LabJack ATI sensor 3.
        2. Read object sensor4 and finger3 position sensor transforms from TF.
        3. Compute:
              force sensor 3 -> object frame
              finger 3 position sensor -> object frame
              force sensor 3 -> finger 3 position sensor frame
        4. Publish force and finger 3 position.
        """

        self.total_itr += 1

        # LabJack reader returns:
        # [Fx3, Fy3, Fz3, Tx3, Ty3, Tz3]
        ft_arr = self.ati_reader.stream_read()

        # Extract force vector only.
        force_s3 = np.asarray(ft_arr[:3], dtype=np.float64)

        timestamp = self.get_clock().now().to_msg()
        timestamp_sec = np.float64(
            timestamp.sec + 1e-9 * timestamp.nanosec
        )

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
            # lookup_transform(polhemus_base, finger3_position_sensor, ...)
            # gives:
            #       finger3 position frame -> polhemus_base frame
            # ─────────────────────────────────────────────────────────────────

            t_base_obj = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.object_position_frame,
                rclpy.time.Time()
            )

            t_base_finger3 = self.tf_buffer.lookup_transform(
                self.polhemus_base_frame,
                self.finger3_position_frame,
                rclpy.time.Time()
            )

            # Check transform age.
            for name, trans in [
                ("object sensor4", t_base_obj),
                ("finger3 position sensor", t_base_finger3),
            ]:
                if not self.transform_is_recent(trans, now):
                    self.handle_old_transform(name)
                    return

            # ─────────────────────────────────────────────────────────────────
            # Convert Polhemus TF transforms into rotation matrices.
            #
            # These rotations map:
            #       object frame -> polhemus_base frame
            #       finger3 position frame -> polhemus_base frame
            # ─────────────────────────────────────────────────────────────────

            rot_base_obj = self.transform_rot_generator(t_base_obj)
            rot_base_finger3 = self.transform_rot_generator(t_base_finger3)

            # ─────────────────────────────────────────────────────────────────
            # Compute finger 3 position sensor rotation into object frame.
            #
            # Given:
            #       R_base_obj      = object frame -> polhemus_base frame
            #       R_base_finger3  = finger3 position frame -> polhemus_base frame
            #
            # We want:
            #       R_object_finger3 = finger3 position frame -> object frame
            #
            # Since:
            #       R_object_base = R_base_obj.T
            #
            # Therefore:
            #       R_object_finger3 = R_base_obj.T @ R_base_finger3
            # ─────────────────────────────────────────────────────────────────

            rot_finger3_obj = rot_base_finger3.T @ rot_base_obj
            print(f"Rotation from finger 3 position sensor frame to object frame:\n{rot_finger3_obj}")

            # ─────────────────────────────────────────────────────────────────
            # Compute finger 3 position sensor location in object frame.
            #
            # Given:
            #       p_base_obj
            #       p_base_finger3
            #
            # Position of finger3 sensor relative to object, expressed in object:
            #
            #       p_object_finger3 = R_base_obj.T @ (p_base_finger3 - p_base_obj)
            # ─────────────────────────────────────────────────────────────────

            rot_finger3_force_s3 = rot_finger3_obj @ self.rot_obj_force_s3

            # Rotate force sensor 3 force vector into finger 3 position sensor frame.
            force_s3_finger3 = rot_finger3_force_s3 @ force_s3
            print(f"Force sensor 3 in finger 3 position sensor frame: {force_s3_finger3}")

            # ─────────────────────────────────────────────────────────────────
            # Gravity compensation
            #
            # Polhemus base z-axis points down, so:
            #   F_g,B = [0, 0, m*g]
            #
            # Rotate gravity:
            #   base frame -> object frame -> finger 3 position sensor frame
            #
            # Then subtract it from force_s3_finger3.
            # ─────────────────────────────────────────────────────────────────

            gravity_base_s3 = np.array([
                0.0,
                0.0,
                .04
                #self.mass_s3_kg * self.gravity_m_s2
            ], dtype=np.float64)

            # Base frame -> object frame
            gravity_obj_s3 = rot_base_obj.T @ gravity_base_s3

            # Object frame -> finger 3 position sensor frame
            gravity_s3_finger3 = rot_finger3_obj.T @ gravity_obj_s3

            # Final gravity-compensated force
            force_s3_finger3_gc = force_s3_finger3 

            # ─────────────────────────────────────────────────────────────────
            # Publish data.
            # ─────────────────────────────────────────────────────────────────

            data_arr = np.concatenate([

                # Force sensor 3 in finger 3 position sensor frame
                force_s3_finger3.astype(np.float64),

                # # Finger 3 position sensor location expressed in object frame
                # pos_obj_finger3.astype(np.float64),

                # Timestamp
                np.array([timestamp_sec], dtype=np.float64)
            ])

            data_msg = Float64MultiArray(
                data=data_arr.flatten().tolist()
            )

            self.ati_pub.publish(data_msg)

        except tf2_ros.TransformException as exc:
            self.miss_itr += 1
            self.get_logger().warn(
                f"Transform not received: {exc}. "
                f"Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
            )

    def ati_force_to_object_rotation(self, sensor_name):
        """
        Manually define ATI force sensor 3 rotation into the object frame.

        This represents:

            R_object_force3

        meaning:

            force sensor 3 frame -> object frame

        Sensor 3 is treated as being 120 degrees around the object
        from the reference sensor.

        If the direction is wrong, change:

            self.sensor_angles_deg['sensor3'] = -120.0

        to:

            self.sensor_angles_deg['sensor3'] = 120.0
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

        rot_obj_force = rot_z @ rot_base @ self.sensor_rot_z_48

        return rot_obj_force

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


def main(args=None):
    rclpy.init(args=args)

    node = LabjackATINode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Shutting down LabJack ATI Sensor 3 node...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()