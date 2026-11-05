"""Publish calibrated, uncorrected NI-DAQ force/torque in ATI sensor axes."""

from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from utils.paths import CALIBRATION_DIR
from utils.acquisition.NIDAQReaderDual import NIDAQReaderDual
from utils.acquisition.source_clock import source_time


class NIDAQATINode(Node):
    def __init__(self):
        super().__init__('nidaq_ati_node')
        self.nidaq_reader = None
        try:
            self._initialize()
        except BaseException:
            # Constructor failure leaves the caller without a node to close.
            self.destroy_node()
            raise

    def _initialize(self):
        self.cal1_path = CALIBRATION_DIR / 'FT44298.cal'
        self.cal2_path = CALIBRATION_DIR / 'FT45281.cal'
        self.phys_channels = 'Dev1/ai0:7,Dev1/ai16:19'
        self.publish_rate = 350.0
        self.nidaq_reader = NIDAQReaderDual(
            cal1_path=self.cal1_path,
            cal2_path=self.cal2_path,
            aq_rate=self.publish_rate,
            phys=self.phys_channels,
        )

        self.acquisition_pub_s1 = self.create_publisher(
            Float64MultiArray,
            '/ERIE_Manipulation/acquisition/wrench_s1_uncorrected',
            1,
        )
        self.acquisition_pub_s2 = self.create_publisher(
            Float64MultiArray,
            '/ERIE_Manipulation/acquisition/wrench_s2_uncorrected',
            1,
        )
        # Incoming TF messages provide another executor wakeup source.
        self.tf_wakeup_subscription = self.create_subscription(
            TFMessage, '/tf', self._on_tf_wakeup, qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(
            f"NI-DAQ targets {self.publish_rate:g} Hz and publishes ATI force/torque without bias subtraction; "
            "start the central force processor for bias and coordinate transforms. "
            "A /tf no-op subscription is enabled for executor wakeups."
        )

    def _on_tf_wakeup(self, _message):
        """Wake the executor on TF arrival without processing transforms."""
        pass

    def timer_callback(self):
        """Publish [Fx, Fy, Fz, Tx, Ty, Tz, source_host_unix_time]."""
        try:
            ft_arr = self.nidaq_reader.read()
            # Wall time at read completion, independent of ROS /clock.
            # This is not a hardware acquisition timestamp.
            timestamp_sec = source_time()
            if ft_arr is False:
                return
            values = np.asarray(ft_arr, dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(
                f"Could not read NI-DAQ force/torque: {exc}. Skipping sample.",
                throttle_duration_sec=5.0,
            )
            return

        if values.shape != (12,) or not np.isfinite(values).all():
            self.get_logger().warn(
                "Invalid NI-DAQ sample; expected 12 finite force/torque values.",
                throttle_duration_sec=5.0,
            )
            return

        self.acquisition_pub_s1.publish(Float64MultiArray(
            data=[*values[0:6].tolist(), timestamp_sec]
        ))
        self.acquisition_pub_s2.publish(Float64MultiArray(
            data=[*values[6:12].tolist(), timestamp_sec]
        ))

    def destroy_node(self):
        """Release the acquisition device before destroying ROS callbacks."""
        reader = getattr(self, 'nidaq_reader', None)
        self.nidaq_reader = None
        try:
            if reader is not None:
                reader.close()
        except Exception as exc:
            self.get_logger().warn(f"NI-DAQ cleanup failed: {exc}")
        finally:
            super().destroy_node()


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = NIDAQATINode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutting down NI-DAQ ATI acquisition node...")
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
