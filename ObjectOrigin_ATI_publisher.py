
## ros2 run your_package object_force_geometry_node --ros-args -p diameter_mm:=50 
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


GEOMETRY_BY_DIAMETER_MM = {
    50: {
        "finger_l": 21.778,
        "finger_H": 10.996,
        "finger_R": 26.18,
    },
    80: {
        # Change these to your real 80 mm object geometry.
        "finger_l": 23.684,
        "finger_H": 10.996,
        "finger_R": 41.18,
    },
}


class ObjectForceGeometryNode(Node):
    def __init__(self):
        super().__init__("object_force_geometry_node")

        self.declare_parameter("diameter_mm", 50)
        self.diameter_mm = int(self.get_parameter("diameter_mm").value)

        if self.diameter_mm not in GEOMETRY_BY_DIAMETER_MM:
            raise ValueError(
                f"diameter_mm must be 50 or 80, got {self.diameter_mm}"
            )

        geometry = GEOMETRY_BY_DIAMETER_MM[self.diameter_mm]
        self.finger_l = geometry["finger_l"]
        self.finger_H = geometry["finger_H"]
        self.finger_R = geometry["finger_R"]

        sensor_origin_42 = 42.0
        sensor_origin_rad = np.deg2rad(sensor_origin_42)

        # This is a Z rotation, despite the old variable name saying x.
        self.sensor_rot_42 = np.array([
            [np.cos(sensor_origin_rad), -np.sin(sensor_origin_rad), 0.0],
            [np.sin(sensor_origin_rad),  np.cos(sensor_origin_rad), 0.0],
            [0.0,                        0.0,                       1.0],
        ])

        self.pub_s1 = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/force/object_force_s1_raw",
            10,
        )
        self.pub_s2 = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/force/object_force_s2_raw",
            10,
        )
        self.pub_s3 = self.create_publisher(
            Float64MultiArray,
            "/ERIE_Manipulation/force/object_force_s3_raw",
            10,
        )

        self.sub_s1 = self.create_subscription(
            Float64MultiArray,
            "/ERIE_Manipulation/force/force_s1_raw",
            lambda msg: self.raw_force_callback(msg, "sensor1", self.pub_s1),
            100,
        )
        self.sub_s2 = self.create_subscription(
            Float64MultiArray,
            "/ERIE_Manipulation/force/force_s2_raw",
            lambda msg: self.raw_force_callback(msg, "sensor2", self.pub_s2),
            100,
        )
        self.sub_s3 = self.create_subscription(
            Float64MultiArray,
            "/ERIE_Manipulation/force/force_s3_raw",
            lambda msg: self.raw_force_callback(msg, "sensor3", self.pub_s3),
            100,
        )

        self.get_logger().info(
            f"Object force geometry node using {self.diameter_mm} mm object: "
            f"l={self.finger_l}, H={self.finger_H}, R={self.finger_R}"
        )

    def raw_force_callback(self, msg, sensor_name, publisher):
        data = np.asarray(msg.data, dtype=np.float64)

        # Expected from your raw publishers:
        # [Fx, Fy, Fz, Tx, Ty, Tz, timestamp_sec]
        if data.size < 7:
            self.get_logger().warn(
                f"[{sensor_name}] Expected [Fx,Fy,Fz,Tx,Ty,Tz,timestamp], "
                f"got {data.size} values"
            )
            return

        force_raw = data[0:3]
        torque_raw = data[3:6]
        timestamp_sec = data[-1]

        force_obj, torque_obj = self.rotate_to_object_origin_frame(
            force_raw,
            torque_raw,
        )

        contact = self.solve_contact_position(
            force_obj,
            torque_obj,
            sensor_name,
        )

        if contact is None:
            return

        out_msg = Float64MultiArray()
        out_msg.data = [
            float(contact["fx"]),
            float(contact["fy"]),
            float(contact["fz"]),
            float(timestamp_sec),
        ]

        publisher.publish(out_msg)

    def rotate_to_object_origin_frame(self, force_raw, torque_raw):
        force_obj = self.sensor_rot_42 @ force_raw
        torque_obj = self.sensor_rot_42 @ torque_raw
        return force_obj, torque_obj

    def solve_contact_position(self, force_obj, torque_obj, sensor_name):
        fx_p, fy_p, fz_p = force_obj
        _, zy_p, zz_p = torque_obj

        l = self.finger_l
        H = self.finger_H
        Rf = self.finger_R

        magnitude = Rf * np.sqrt(fx_p**2 + fz_p**2)
        if magnitude < 1e-9:
            self.get_logger().warn(
                f"[{sensor_name}] fx' and fz' near zero. Skipping."
            )
            return None

        phi = np.arctan2(fx_p, fz_p)
        sin_arg = np.clip(-(fx_p * l + zy_p) / magnitude, -1.0, 1.0)
        theta = np.arcsin(sin_arg) + phi

        if abs(fx_p) < 1e-9:
            self.get_logger().warn(
                f"[{sensor_name}] fx' near zero. Cannot solve h. Skipping."
            )
            return None

        h = H + (zz_p - fy_p * Rf * np.sin(theta)) / fx_p

        fx = fy_p
        fy = -fx_p * np.cos(theta) + fz_p * np.sin(theta)
        fz = fx_p * np.sin(theta) + fz_p * np.cos(theta)

        return {
            "fx": fx,
            "fy": fy,
            "fz": fz,
        }


def main(args=None):
    rclpy.init(args=args)
    node = ObjectForceGeometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down object force geometry node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()