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

        # self.sensor_rot_x_n180 = np.array([
        #     [1., 0., 0.0],
        #     [0.,  -1., 0.0],
        #     [0.0, 0.0, -1.0]
        # ])

        self.rot_obj_force_s3 = self.ati_force_to_object_rotation('sensor3')
        
        
        # #(OBJECT ORGIN FRAME CHANGES)
        # sensor_origin_42 = 42.
        # sensor_origin_rad = np.deg2rad(sensor_origin_42)
        # self.sensor_rot_x_42 = np.array([
        #     [np.cos(sensor_origin_rad), -np.sin(sensor_origin_rad), 0.0],
        #     [np.sin(sensor_origin_rad),  np.cos(sensor_origin_rad), 0.0],
        #     [0.0,                0.0,               1.0]
        # ])
        
        # self.finger_l = 7.3       # offset length sensoor to object origin (mm)
        # self.finger_H = 10.996    # sensor origin height (mm)
        # self.finger_R = 25.0      # object radius (mm)
        

        # ─────────────────────────────────────────────────────────────────────
        # ROS publisher and timer
        # ─────────────────────────────────────────────────────────────────────

        self.ati_pub = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s3_finger3',
            1
        )
        
        self.ati_pub_raw = self.create_publisher(
            Float64MultiArray,
            'ERIE_Manipulation/force/force_s3_raw',
            1
        )
        
        # self.ati_pub_object_raw = self.create_publisher(
        #     Float64MultiArray,
        #     'ERIE_Manipulation/force/object_force_s3_raw',
        #     1
        # )

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
        try:
            ft_arr = self.ati_reader.stream_read()

            if ft_arr is None:
                self.get_logger().warn("No force data received from sensor 3. Skipping this sample.")
                return
            
            if ft_arr.size < 6:
                self.get_logger().warn(
                    f"Bad force/torque data from sensor 3: expected 6 values, got {ft_arr.size}. Skipping this sample."
                )
                return

            ft_s3 = np.asarray(ft_arr[:6], dtype=np.float64)
            force_s3 = ft_s3[:3]
            
            # #(OBJECT ORGIN FRAME CHANGES)
            # torque_s3 = ft_s3[3:6]

            timestamp = self.get_clock().now().to_msg()
            timestamp_sec = np.float64(
                timestamp.sec + 1e-9 * timestamp.nanosec
            )

            raw_data_msg_f3 = Float64MultiArray(
                data=np.concatenate([
                    ft_s3,
                    np.array([timestamp_sec], dtype=np.float64)
                ]).flatten().tolist()
            )

            self.ati_pub_raw.publish(raw_data_msg_f3)
            
            

        except Exception as exc:
            self.get_logger().warn(f"Could not read force sensor 3: {exc}. Skipping this sample.")
            return

        try:
            now = self.get_clock().now()

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

            rot_base_obj = self.transform_rot_generator(t_base_obj)
            rot_base_finger3 = self.transform_rot_generator(t_base_finger3)

            rot_finger3_obj = rot_base_finger3.T @ rot_base_obj

            rot_finger3_force_s3 = rot_finger3_obj @ self.rot_obj_force_s3

            # Rotate force sensor 3 force vector into finger 3 position sensor frame.
            force_s3_finger3 = rot_finger3_force_s3 @ force_s3

            # force_s3_finger3 = self.sensor_rot_x_n180 @ force_s3_finger3
            
            # print(f"Force sensor 3 in finger 3 position sensor frame: {force_s3_finger3[0]:.2f}, {force_s3_finger3[1]:.2f}, {force_s3_finger3[2]:.2f}")
            
            # # After the existing force rotation, rotate force+torque into the
            # # object origin frame and solve for (theta, h) (CHANGE FOR OBJECT ORIGIN FRAME)
            # force3_obj, torque3_obj = self.rotate_to_object_origin_frame(force_s3, torque_s3)

            # contact = self.solve_contact_position(force3_obj, torque3_obj)

            # if contact is not None:
            #     contact_arr = np.array([
            #         contact['fx'],
            #         contact['fy'],
            #         contact['fz'],
            #         timestamp_sec
            #     ], dtype=np.float64)



            # ─────────────────────────────────────────────────────────────────
            # Publish data.
            # ─────────────────────────────────────────────────────────────────

            data_arr = np.concatenate([

                # Force sensor 3 in finger 3 position sensor frame
                force_s3_finger3.astype(np.float64),

                # Timestamp
                np.array([timestamp_sec], dtype=np.float64)
            ])

            data_msg = Float64MultiArray(
                data=data_arr.flatten().tolist()
            )

            self.ati_pub.publish(data_msg)
            
            #(OBJECT ORGIN FRAME CHANGES)
            # contact_msg = Float64MultiArray(data=contact_arr.flatten().tolist())
            # self.ati_pub_object_raw.publish(contact_msg)

        except tf2_ros.TransformException as exc:
            self.miss_itr += 1
            self.get_logger().warn(
                f"Transform not received: {exc}. "
                f"Miss rate: {self.miss_itr / self.total_itr * 100:.2f}%"
            )

    # #(OBJECT ORGIN FRAME CHANGES)
    # def rotate_to_object_origin_frame(self, force_s3, torque_s3):
    #     force3_obj = self.sensor_rot_x_42 @ force_s3
    #     torque3_obj = self.sensor_rot_x_42 @ torque_s3
    #     return force3_obj, torque3_obj
    
    # #(OBJECT ORGIN FRAME CHANGES)
    # def solve_contact_position(self, force3_obj, torque3_obj):
    #     """
    #     Solve for contact position (theta, h) and finger-frame forces.

    #     Inputs (all in object frame):
    #         f_obj   : [fx', fy', fz'] forces  (N)
    #         tau_obj : [zx', zy', zz'] torques (N·mm)

    #     Returns dict with:
    #         theta_deg  : angular contact position around finger (degrees)
    #         h_mm       : axial contact position along finger (mm)
    #         fx, fy, fz : forces in finger frame (N)
    #     or None if the system is degenerate (zero force).
    #     """
    #     fx_p, fy_p, fz_p = force3_obj
    #     zx_p, zy_p, zz_p = torque3_obj

    #     l = self.finger_l
    #     H = self.finger_H
    #     Rf = self.finger_R

    #     # Solve for theta
    #     magnitude = Rf * np.sqrt(fx_p**2 + fz_p**2)
    #     if magnitude < 1e-9:
    #         self.get_logger().warn("fx' and fz' near zero — cannot solve for theta. Skipping.")
    #         return None

    #     phi     = np.arctan2(fx_p, fz_p)
    #     sin_arg = np.clip(-(fx_p * l + zy_p) / magnitude, -1.0, 1.0)
    #     theta   = np.arcsin(sin_arg) + phi

    #     # Solve for h
    #     if abs(fx_p) < 1e-9:
    #         self.get_logger().warn("fx' near zero — cannot solve for h. Skipping.")
    #         return None

    #     h = H + (zz_p - fy_p * Rf * np.sin(theta)) / fx_p

    #     # Recover finger-frame forces
    #     fx =  fy_p
    #     fy = -fx_p * np.cos(theta) + fz_p * np.sin(theta)
    #     fz =  fx_p * np.sin(theta) + fz_p * np.cos(theta)

    #     return {
    #         'fx': fx,
    #         'fy': fy,
    #         'fz': fz
    #     }
        
    
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

        rot_obj_force = rot_z.T @ rot_base @ self.sensor_rot_z_48.T

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
        if dt > self.trans_delay:
            print(f"Transform age: {dt:.3f} seconds (too old), Data time: {tf_stamp.nanoseconds}")
        # print(f"Transform age: {dt:.3f} seconds")

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