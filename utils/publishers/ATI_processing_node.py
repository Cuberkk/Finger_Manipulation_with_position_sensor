"""Central ATI zeroing and finger-frame transforms; no device drivers are imported.

Hardware publishers send factory-calibrated, uncorrected six-component wrenches.
All three offsets live on this host. The central node publishes only corrected
three-axis finger forces, preserving the acquisition host's read timestamp.
"""

from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import uuid

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
import tf2_ros

from utils.acquisition.bias import save_bias
from utils.acquisition.central_bias import CentralBiasManager
from utils.gui.rest import load_study_settings
from utils.paths import BIAS_DIR, GUI_CONFIG_FILE


SENSORS = (1, 2, 3)
INPUT_TOPICS = {
    sensor: f"/ERIE_Manipulation/acquisition/wrench_s{sensor}_uncorrected"
    for sensor in SENSORS
}
REBIAS_SERVICE = "/ERIE_Manipulation/force/rebias"
STATUS_TOPIC = "/ERIE_Manipulation/force/processing_status"


def ati_to_object_rotation(sensor):
    """Preserve the installed ATI geometry, including both transposes."""
    angle = {1: -120.0, 2: 0.0, 3: 120.0}[sensor]
    rot_z = Rotation.from_euler("z", angle, degrees=True).as_matrix()
    alignment = Rotation.from_euler("z", -48.0, degrees=True).as_matrix()
    rot_base = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    return rot_z.T @ rot_base @ alignment.T


class ATIProcessingNode(Node):
    def __init__(self):
        super().__init__("ati_processing_node")
        try:
            self._initialize()
        except BaseException:
            self.destroy_node()
            raise

    def _initialize(self):
        self.processor_id = uuid.uuid4().hex
        self._status_seq = 0
        settings = load_study_settings(GUI_CONFIG_FILE)
        self.max_age_sec = float(self.declare_parameter(
            "max_sample_age_sec", settings.tf_wait_tolerance_ms / 1000.0).value)
        self.max_tf_age_sec = float(self.declare_parameter("max_tf_age_sec", 1.5).value)
        if not math.isfinite(self.max_tf_age_sec) or self.max_tf_age_sec <= 0:
            raise ValueError("max_tf_age_sec must be finite and positive")
        self.bias_manager = CentralBiasManager(
            bias_time_sec=self.declare_parameter("bias_time_sec", 5.0).value,
            warmup_sec=self.declare_parameter("bias_warmup_sec", 0.5).value,
            timeout_sec=self.declare_parameter("bias_timeout_sec", 15.0).value,
            max_age_sec=self.max_age_sec,
            max_gap_sec=self.declare_parameter("bias_max_gap_sec", 0.5).value,
            min_samples=self.declare_parameter("bias_min_samples", 30).value,
            save_callback=self._save_bias,
        )
        self._generation = self.bias_manager.generation
        self.polhemus_base_frame = "polhemus_base"
        self.object_position_frame = "sensor4"
        self.rot_obj_force = {sensor: ati_to_object_rotation(sensor) for sensor in SENSORS}
        # A delayed TF can become available on the next timer tick. Bounded queues
        # keep exact source timestamps without blocking a subscription callback.
        self.pending = {sensor: deque(maxlen=1024) for sensor in SENSORS}
        self.force_publishers = {}
        self.input_subscriptions = []
        for sensor in SENSORS:
            prefix = "/ERIE_Manipulation/force"
            self.force_publishers[sensor] = self.create_publisher(
                Float64MultiArray, f"{prefix}/force_s{sensor}_finger{sensor}", 10)
            self.input_subscriptions.append(self.create_subscription(
                Float64MultiArray, INPUT_TOPICS[sensor],
                lambda message, sensor=sensor: self._on_sample(sensor, message),
                qos_profile_sensor_data,
            ))
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self.rebias_service = self.create_service(Trigger, REBIAS_SERVICE, self._on_rebias)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._status_due = 0.0
        self.timer = self.create_timer(0.01, self._on_timer)
        if self.declare_parameter("bias_on_start", True).value:
            self._request_bias()
        else:
            self._publish_status(time.monotonic())
            self.get_logger().info(
                f"Waiting for a bias request on {REBIAS_SERVICE}; output is paused.")
        self.get_logger().info("Central ATI processing initialized; sensor1=thumb, sensor2=middle, sensor3=index.")

    def _save_bias(self, samples, biases, metadata):
        rows = np.concatenate([
            np.column_stack((np.full(len(samples[sensor]), sensor), samples[sensor]))
            for sensor in SENSORS
        ])
        payload = {
            "NIDAQ_S1": np.asarray(biases[1], dtype=float).tolist(),
            "NIDAQ_S2": np.asarray(biases[2], dtype=float).tolist(),
            "Labjack": np.asarray(biases[3], dtype=float).tolist(),
            "_metadata": {
                **metadata,
                "csv_columns": ["sensor_id", "source_timestamp_sec", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
                "input": "factory-calibrated uncorrected force/torque",
                "force_units": "N",
                "torque_units": "N-mm",
                "timestamp_kind": "acquisition-host Unix read timestamp",
            },
        }
        save_bias(
            BIAS_DIR / "bias_csv" / "Central_ATI_Bias.csv",
            BIAS_DIR / "bias_json" / "Central_ATI_Bias.json",
            rows, payload,
        )

    def _request_bias(self):
        self.bias_manager.request_bias(time.time(), time.monotonic())
        for pending in self.pending.values():
            pending.clear()
        self._publish_status(time.monotonic())
        self.get_logger().info(
            "Bias requested for all three ATI sensors. Keep all sensors unloaded and stationary.")

    def _on_rebias(self, request, response):
        try:
            self._request_bias()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = json.dumps({
                "message": "Bias request accepted for all three sensors; keep them unloaded.",
                "processor_id": self.processor_id,
                "request_id": self.bias_manager.request_id,
                "status_seq": self._status_seq,
                "status_topic": STATUS_TOPIC,
            }, sort_keys=True)
        return response

    def _on_sample(self, sensor, message):
        try:
            data = np.asarray(message.data, dtype=float)
        except (TypeError, ValueError):
            return
        if data.shape != (7,) or not np.isfinite(data).all():
            self.get_logger().warn(
                f"Sensor {sensor}: expected six finite FT components and a Unix timestamp.",
                throttle_duration_sec=5.0)
            return
        stamp = float(data[6])
        corrected = self.bias_manager.ingest(
            sensor, data[:6], stamp, time.time(), time.monotonic())
        if corrected is None:
            if self.bias_manager.last_rejection:
                self.get_logger().warn(self.bias_manager.last_rejection, throttle_duration_sec=5.0)
            return
        # Acquisition already publishes all six raw components. Only force is
        # needed for the finger-frame output and the source-time TF queue.
        self.pending[sensor].append((stamp, corrected[:3].copy()))
        self._drain_sensor(sensor, time.time())

    def _publish_status(self, now_monotonic):
        self._status_seq += 1
        self.status_publisher.publish(String(data=json.dumps(
            {**self.bias_manager.status(), "processor_id": self.processor_id,
             "status_seq": self._status_seq}, sort_keys=True, allow_nan=False)))
        self._status_due = now_monotonic + 0.5

    def _on_timer(self):
        now_wall, now_monotonic = time.time(), time.monotonic()
        event = self.bias_manager.tick(now_wall, now_monotonic)
        if event in ("completed", "failed") or self.bias_manager.generation != self._generation:
            # A queued vector must never cross a calibration transaction.
            for pending in self.pending.values():
                pending.clear()
            self._generation = self.bias_manager.generation
        if event == "completed":
            self.get_logger().info("All three ATI biases saved and activated together.")
        elif event == "failed":
            self.get_logger().error(
                f"Central ATI bias failed: {self.bias_manager.last_error}")
        if event is not None or now_monotonic >= self._status_due:
            self._publish_status(now_monotonic)
        if self.bias_manager.busy or not self.bias_manager.ready:
            for pending in self.pending.values():
                pending.clear()
            return
        for sensor in SENSORS:
            self._drain_sensor(sensor, now_wall)

    def _drain_sensor(self, sensor, now_wall):
        pending = self.pending[sensor]
        while pending:
            stamp, corrected_force = pending[0]
            age_sec = now_wall - stamp
            if abs(age_sec) > self.max_age_sec:
                pending.popleft()
                self.get_logger().warn(
                    f"Sensor {sensor}: dropping ATI sample outside the "
                    f"{self.max_age_sec * 1000:g} ms age limit while waiting for "
                    f"source-time Polhemus TF (sample age: {age_sec * 1000:.1f} ms).",
                    throttle_duration_sec=5.0)
                continue
            when = Time(seconds=stamp, clock_type=self.get_clock().clock_type)
            try:
                obj = self.tf_buffer.lookup_transform(
                    self.polhemus_base_frame, self.object_position_frame, when)
                finger = self.tf_buffer.lookup_transform(
                    self.polhemus_base_frame, f"sensor{sensor}", when)
                if not all(self._transform_is_recent(transform, now_wall)
                           for transform in (obj, finger)):
                    pending.popleft()
                    self.get_logger().warn(
                        "Dropping ATI sample because its Polhemus TF is stale.",
                        throttle_duration_sec=5.0)
                    continue
                rot_base_obj = self._transform_rotation(obj)
                rot_base_finger = self._transform_rotation(finger)
                rotation = rot_base_finger.T @ rot_base_obj @ self.rot_obj_force[sensor]
                force = rotation @ corrected_force
            except tf2_ros.TransformException:
                # A force sample can arrive before its matching tracking data.
                # Retry silently until its source-age deadline (500 ms by
                # default), preserving the requested measurement timestamp.
                break
            except ValueError as exc:
                pending.popleft()
                self.get_logger().warn(f"Invalid Polhemus rotation: {exc}", throttle_duration_sec=5.0)
                continue
            pending.popleft()
            self.force_publishers[sensor].publish(Float64MultiArray(
                data=np.append(force, stamp).tolist()))

    def _transform_is_recent(self, transform, now_wall):
        stamp = transform.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        # Static transforms use stamp zero and are valid at every query time.
        return timestamp == 0.0 or abs(now_wall - timestamp) <= self.max_tf_age_sec

    @staticmethod
    def _transform_rotation(transform):
        q = transform.transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ATIProcessingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
