"""Publish calibrated, uncorrected LabJack force/torque in ATI sensor axes."""

from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from utils.paths import CALIBRATION_DIR
from utils.acquisition.LabjackReader import LabjackATIReader
from utils.acquisition.source_clock import source_time


class LabjackATINode(Node):
    def __init__(self):
        super().__init__('ati_node')
        self.ati_reader = None
        try:
            self._initialize()
        except BaseException:
            # Constructor failure leaves the caller without a node to close.
            self.destroy_node()
            raise

    def _initialize(self):
        self.cal_path = CALIBRATION_DIR / 'FT44297.cal'
        self.publish_rate = 350.0
        self.ati_reader = LabjackATIReader(
            cal_path=self.cal_path,
            aq_rate=self.publish_rate,
        )

        self.acquisition_pub_s3 = self.create_publisher(
            Float64MultiArray,
            '/ERIE_Manipulation/acquisition/wrench_s3_uncorrected',
            1,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(
            f"LabJack targets {self.publish_rate:g} Hz and publishes ATI force/torque without bias subtraction; "
            "start the central force processor for bias and coordinate transforms."
        )

    def timer_callback(self):
        """Publish [Fx, Fy, Fz, Tx, Ty, Tz, source_host_unix_time]."""
        try:
            ft_arr = self.ati_reader.stream_read()
            # Wall time at read completion, independent of ROS /clock.
            # This is not a hardware acquisition timestamp.
            timestamp_sec = source_time()
            if ft_arr is None:
                return
            values = np.asarray(ft_arr, dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(
                f"Could not read LabJack force/torque: {exc}. Skipping sample.",
                throttle_duration_sec=5.0,
            )
            return

        if values.shape != (6,) or not np.isfinite(values).all():
            self.get_logger().warn(
                "Invalid LabJack sample; expected 6 finite force/torque values.",
                throttle_duration_sec=5.0,
            )
            return

        self.acquisition_pub_s3.publish(Float64MultiArray(
            data=[*values[0:6].tolist(), timestamp_sec]
        ))

    def destroy_node(self):
        """Release the acquisition device before destroying ROS callbacks."""
        reader = getattr(self, 'ati_reader', None)
        self.ati_reader = None
        try:
            if reader is not None:
                reader.close()
        except Exception as exc:
            self.get_logger().warn(f"LabJack cleanup failed: {exc}")
        finally:
            super().destroy_node()


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = LabjackATINode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutting down LabJack ATI acquisition node...")
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
