"""Measure the NI-DAQ publisher on its acquisition host without changing samples.

Stop the normal NI-DAQ publisher, then run::

    python -m utils.diagnostics.nidaq_rate --interval 5

This runs the original publisher with counters around its existing callbacks,
DAQ reads and publish calls. It requires the same ROS and NI-DAQ environment.
"""

import argparse
from contextlib import nullcontext
import math
import sys
import time


class RateStatistics:
    """Constant-memory window counters; elapsed time uses a performance clock."""

    def __init__(self, clock=time.perf_counter):
        self.clock = clock
        self.reset()

    def reset(self):
        self.started = self.clock()
        self.callbacks = 0
        self.callback_seconds = 0.0
        self.tf_messages = 0
        self.reader_calls = 0
        self.reader_seconds = 0.0
        self.reader_errors = 0
        self.empty_or_invalid_reads = 0
        self.read_calls = 0
        self.read_seconds = 0.0
        self.read_errors = 0
        self.batches = 0
        self.scans = 0
        self.publish_calls = 0
        self.publish_seconds = 0.0
        self.publish_errors = 0
        self.published = {1: 0, 2: 0}

    def summary(self, sample_clock_hz):
        elapsed = self.clock() - self.started
        if elapsed <= 0:
            return None
        return {
            "elapsed_sec": elapsed,
            "sample_clock_hz": sample_clock_hz,
            "callbacks_hz": self.callbacks / elapsed,
            "tf_messages_hz": self.tf_messages / elapsed,
            "batches_hz": self.batches / elapsed,
            "drained_scans_hz": self.scans / elapsed,
            "mean_batch": self.scans / self.batches if self.batches else 0.0,
            "skipped_old_scans_hz": (self.scans - self.batches) / elapsed,
            "publish_s1_hz": self.published[1] / elapsed,
            "publish_s2_hz": self.published[2] / elapsed,
            "reader_mean_ms": 1000 * self.reader_seconds / self.reader_calls if self.reader_calls else 0.0,
            "read_mean_ms": 1000 * self.read_seconds / self.read_calls if self.read_calls else 0.0,
            "publish_mean_ms": 1000 * self.publish_seconds / self.publish_calls if self.publish_calls else 0.0,
            "callback_mean_ms": 1000 * self.callback_seconds / self.callbacks if self.callbacks else 0.0,
            "read_errors": self.read_errors,
            "reader_errors": self.reader_errors,
            "empty_or_invalid_reads": self.empty_or_invalid_reads,
            "publish_errors": self.publish_errors,
        }


class MeasuredReader:
    """Time the whole read, including availability queries and calibration."""

    def __init__(self, reader, statistics):
        self._reader = reader
        self._statistics = statistics

    def __getattr__(self, name):
        return getattr(self._reader, name)

    def read(self):
        statistics = self._statistics
        started = statistics.clock()
        statistics.reader_calls += 1
        try:
            result = self._reader.read()
            if result is False:
                statistics.empty_or_invalid_reads += 1
            return result
        except Exception:
            statistics.reader_errors += 1
            raise
        finally:
            statistics.reader_seconds += statistics.clock() - started


class MeasuredTask:
    """Count the scans the original reader actually drains, with no extra read."""

    def __init__(self, task, statistics):
        self._task = task
        self._statistics = statistics

    def __getattr__(self, name):
        return getattr(self._task, name)

    def read(self, *args, **kwargs):
        statistics = self._statistics
        started = statistics.clock()
        statistics.read_calls += 1
        try:
            result = self._task.read(*args, **kwargs)
            statistics.batches += 1
            statistics.scans += kwargs["number_of_samples_per_channel"]
            return result
        except Exception:
            statistics.read_errors += 1
            raise
        finally:
            statistics.read_seconds += statistics.clock() - started


class MeasuredPublisher:
    """Count successful publish calls; this does not count DDS delivery."""

    def __init__(self, publisher, sensor, statistics):
        self._publisher = publisher
        self._sensor = sensor
        self._statistics = statistics

    def __getattr__(self, name):
        return getattr(self._publisher, name)

    def publish(self, message):
        statistics = self._statistics
        started = statistics.clock()
        statistics.publish_calls += 1
        try:
            result = self._publisher.publish(message)
            statistics.published[self._sensor] += 1
            return result
        except Exception:
            statistics.publish_errors += 1
            raise
        finally:
            statistics.publish_seconds += statistics.clock() - started


def diagnostic_node_type(base_class, interval):
    """Reuse the original node's acquisition, messages and cleanup unchanged."""

    class DiagnosticNIDAQNode(base_class):
        def __init__(self):
            self.rate_statistics = RateStatistics()
            super().__init__()
            try:
                statistics = self.rate_statistics
                self.nidaq_reader.task = MeasuredTask(self.nidaq_reader.task, statistics)
                self.nidaq_reader = MeasuredReader(self.nidaq_reader, statistics)
                self.acquisition_pub_s1 = MeasuredPublisher(self.acquisition_pub_s1, 1, statistics)
                self.acquisition_pub_s2 = MeasuredPublisher(self.acquisition_pub_s2, 2, statistics)
                statistics.reset()  # Exclude hardware and ROS startup time.
                self.get_logger().info(
                    "NI-DAQ rate diagnostics enabled. sample_clock_hz is the device-reported "
                    "clock setting; drained_scans_hz counts scans read, and publish_s*_hz "
                    "counts successful publish calls per topic, not subscriber delivery. "
                    "reader_mean_ms includes availability queries and calibration; "
                    "read_mean_ms times only task.read. tf_messages_hz counts actual "
                    "/tf callbacks. mean_batch > 1 means the original reader discards older scans."
                )
            except BaseException:
                self.destroy_node()
                raise

        def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
            if topic == '/tf':
                original_callback = callback

                def callback(message):
                    self.rate_statistics.tf_messages += 1
                    return original_callback(message)

            return super().create_subscription(msg_type, topic, callback, qos_profile, **kwargs)

        def timer_callback(self):
            statistics = self.rate_statistics
            started = statistics.clock()
            statistics.callbacks += 1
            try:
                super().timer_callback()
            finally:
                statistics.callback_seconds += statistics.clock() - started
                if statistics.clock() - statistics.started >= interval:
                    summary = statistics.summary(self.nidaq_reader.aq_rate)
                    self.get_logger().info("NI-DAQ rates: " + ", ".join(
                        f"{name}={value:.3f}" for name, value in summary.items()
                    ))
                    statistics.reset()

    return DiagnosticNIDAQNode


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=5.0, help="Statistics window in seconds (default: 5)")
    parser.add_argument(
        "--windows-timer-ms", type=int,
        help="Request a Windows timer resolution for this diagnostic run, e.g. 1 ms",
    )
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.interval) or options.interval <= 0:
        parser.error("--interval must be finite and greater than zero")
    if options.windows_timer_ms is not None:
        if sys.platform != "win32":
            parser.error("--windows-timer-ms is only available on Windows")
        if not 1 <= options.windows_timer_ms <= 0xFFFFFFFF:
            parser.error("--windows-timer-ms must be a positive 32-bit integer")

    # Keep the counters importable and testable without ROS or NI hardware.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from utils.publishers.NIDAQ_ATI_publisher import NIDAQATINode

    timer_context = nullcontext()
    if options.windows_timer_ms is not None:
        from utils.diagnostics.windows_timer import windows_timer_resolution
        timer_context = windows_timer_resolution(options.windows_timer_ms)

    with timer_context:
        node = None
        try:
            rclpy.init(args=ros_args)
            node = diagnostic_node_type(NIDAQATINode, options.interval)()
            if options.windows_timer_ms is not None:
                node.get_logger().info(
                    f"Windows timer resolution request accepted: {options.windows_timer_ms} ms. "
                    "Compare the measured rates with the run without this option."
                )
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            print("Shutting down NI-DAQ rate diagnostics...")
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()
