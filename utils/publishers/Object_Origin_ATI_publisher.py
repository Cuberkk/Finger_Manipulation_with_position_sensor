"""Compute contact-frame force from raw ATI F/T and confirmed central offsets."""

from collections import deque
from pathlib import Path
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from utils.gui.rest import load_study_settings
from utils.gui.study_io import ROSBridge, FORCE_BIAS_STATUS_TOPIC, FORCE_BIAS_STATUS_MAX_AGE
from utils.paths import GUI_CONFIG_FILE


GEOMETRY_BY_DIAMETER_MM = {
    50: {
        "finger_l": 21.778,
        "finger_H": 10.996,
        "finger_R": 26.18,
        # Includes 1.18 mm for the rubber grip thickness.
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
        try:
            self._initialize()
        except BaseException:
            self.destroy_node()
            raise

    def _initialize(self):
        settings = load_study_settings(GUI_CONFIG_FILE)
        self.max_source_age_sec = settings.tf_wait_tolerance_ms / 1000.0
        self._bias_status = None
        self._bias_arrival = None
        self._bias_valid = False
        self._retired_processors = deque(maxlen=16)
        self._last_source_stamps = {}

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
            "/ERIE_Manipulation/acquisition/wrench_s1_uncorrected",
            lambda msg: self.raw_force_callback(msg, "sensor1", self.pub_s1),
            100,
        )
        self.sub_s2 = self.create_subscription(
            Float64MultiArray,
            "/ERIE_Manipulation/acquisition/wrench_s2_uncorrected",
            lambda msg: self.raw_force_callback(msg, "sensor2", self.pub_s2),
            100,
        )
        self.sub_s3 = self.create_subscription(
            Float64MultiArray,
            "/ERIE_Manipulation/acquisition/wrench_s3_uncorrected",
            lambda msg: self.raw_force_callback(msg, "sensor3", self.pub_s3),
            100,
        )
        self.status_subscription = self.create_subscription(
            String, FORCE_BIAS_STATUS_TOPIC, self._on_processing_status, 10,
        )

        self.get_logger().info(
            f"Object force geometry node using {self.diameter_mm} mm object: "
            f"l={self.finger_l}, H={self.finger_H}, R={self.finger_R}"
        )

    def _on_processing_status(self, message):
        """Accept fresh, ordered central bias identities without requesting a tare."""
        try:
            status = ROSBridge._parse_force_bias_status(message.data)
        except (ValueError, TypeError, OverflowError):
            self._bias_valid = False
            return False
        previous = self._bias_status
        if status["processor_id"] in self._retired_processors:
            return False
        if previous and previous["processor_id"] == status["processor_id"]:
            if (status["status_seq"] <= previous["status_seq"]
                    or status["request_id"] < previous["request_id"]
                    or status["generation"] < previous["generation"]):
                return False
        elif previous:
            self._retired_processors.append(previous["processor_id"])
        if previous is None or any(status[field] != previous[field]
                                   for field in ("processor_id", "request_id", "generation")):
            self._last_source_stamps.clear()
        self._bias_status = status
        self._bias_arrival = time.monotonic()
        self._bias_valid = True
        return True

    def _active_bias(self, sensor_name):
        status = self._bias_status
        if (not self._bias_valid or status is None or self._bias_arrival is None
                or not 0 <= time.monotonic() - self._bias_arrival <= FORCE_BIAS_STATUS_MAX_AGE
                or not status["ready"] or status["busy"] or status["generation"] < 1
                or status["biases"] is None):
            return None
        return np.asarray(status["biases"][sensor_name[len("sensor"):]], dtype=np.float64)

    def raw_force_callback(self, msg, sensor_name, publisher):
        try:
            data = np.asarray(msg.data, dtype=np.float64)
        except (ValueError, TypeError, OverflowError):
            data = np.array([])
        if data.shape != (7,) or not np.isfinite(data).all():
            self.get_logger().warn(
                f"[{sensor_name}] Expected six finite raw F/T components and a timestamp.",
                throttle_duration_sec=5.0,
            )
            return
        timestamp_sec = float(data[6])
        if timestamp_sec <= 0 or abs(time.time() - timestamp_sec) > self.max_source_age_sec:
            return
        previous_stamp = self._last_source_stamps.get(sensor_name)
        if previous_stamp is not None and timestamp_sec <= previous_stamp:
            return
        bias = self._active_bias(sensor_name)
        if bias is None:
            self.get_logger().warn(
                "Waiting for fresh, completed central Force bias before contact-force processing.",
                throttle_duration_sec=5.0,
            )
            return
        metadata = self._bias_status["bias_metadata"] or {}
        window_end = metadata.get("window_end_unix_sec")
        if (isinstance(window_end, (int, float)) and not isinstance(window_end, bool)
                and timestamp_sec <= window_end):
            return
        self._last_source_stamps[sensor_name] = timestamp_sec
        with np.errstate(over="ignore", invalid="ignore"):
            corrected = data[:6] - bias
        if not np.isfinite(corrected).all():
            return

        force_obj, torque_obj = self.rotate_to_object_origin_frame(
            corrected[:3], corrected[3:],
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
