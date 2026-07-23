#!/usr/bin/env python3
import argparse
import csv
import json
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


VALID_TASKS = ["roll", "pitch", "yaw", "finger_gating","single_fg"]
VALID_DIAMETERS_MM = [50, 80]


@dataclass(frozen=True)
class SensorConfig:
    sensor_number: int
    finger_name: str
    topic: str
    csv_name: str


class ForceTrialCSVRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("object_raw_csv_recorder")

        self.user_number = str(args.user_number)
        self.diameter_mm = int(args.diameter_mm)
        self.task_performed = str(args.task_performed)
        self.trial_number = str(args.trial_number)
        self.flush_every = max(1, int(args.flush_every))
        self.sensor_subfolders = bool(args.sensor_subfolders)

        self.sensors: List[SensorConfig] = [
            SensorConfig(
                sensor_number=1,
                finger_name="thumb",
                topic=args.thumb_topic,
                csv_name="object_origin_thumb_force.csv",
            ),
            SensorConfig(
                sensor_number=2,
                finger_name="middle",
                topic=args.middle_topic,
                csv_name="object_origin_middle_force.csv",
            ),
            SensorConfig(
                sensor_number=3,
                finger_name="index",
                topic=args.index_topic,
                csv_name="object_origin_index_force.csv",
            ),
        ]

        self.trial_dir = self._make_trial_dir(args.data_root, args.overwrite)
        self.get_logger().info(f"Recording trial into: {self.trial_dir}")

        self.files: Dict[str, object] = {}
        self.writers: Dict[str, csv.DictWriter] = {}
        self.counts: Dict[str, int] = {s.finger_name: 0 for s in self.sensors}
        self._open_csv_files()

        self.subscribers = []
        for sensor in self.sensors:
            sub = self.create_subscription(
                Float64MultiArray,
                sensor.topic,
                lambda msg, sensor=sensor: self.force_callback(msg, sensor),
                1000,
            )
            self.subscribers.append(sub)
            # self.get_logger().info(
            #     f"Subscribed to {sensor.topic} -> sensor {sensor.sensor_number} ({sensor.finger_name})"
            # )

        self.first_frame = True
        if args.duration_sec is not None and float(args.duration_sec) > 0.0:
            self.duration_sec = float(args.duration_sec)
            self.stop_timer = self.create_timer(self.duration_sec, self.stop_after_duration)
        else:
            self.duration_sec = None
            self.stop_timer = None

    def _make_trial_dir(self, data_root: str, overwrite: bool) -> Path:
        trial_dir = (
            Path(data_root)
            / f"user_{self.user_number}"
            / f"{self.diameter_mm}mm"
            / self.task_performed
            / f"trial_{self.trial_number}"
        )

        if overwrite and trial_dir.exists():
            shutil.rmtree(trial_dir)

        trial_dir.mkdir(parents=True, exist_ok=True)
        return trial_dir

    def _csv_path_for_sensor(self, sensor: SensorConfig) -> Path:
        if self.sensor_subfolders:
            sensor_dir = self.trial_dir / f"sensor{sensor.sensor_number}_{sensor.finger_name}"
            sensor_dir.mkdir(parents=True, exist_ok=True)
            return sensor_dir / sensor.csv_name

        return self.trial_dir / sensor.csv_name

    def _open_csv_files(self) -> None:
        header = [
            "source_timestamp_sec",
            "Fx",
            "Fy",
            "Fz",
        ]

        for sensor in self.sensors:
            csv_path = self._csv_path_for_sensor(sensor)
            file_exists_and_has_data = csv_path.exists() and csv_path.stat().st_size > 0

            f = open(csv_path, "a", newline="")
            writer = csv.DictWriter(f, fieldnames=header)

            if not file_exists_and_has_data:
                writer.writeheader()
                f.flush()

            self.files[sensor.finger_name] = f
            self.writers[sensor.finger_name] = writer


    def force_callback(self, msg: Float64MultiArray, sensor: SensorConfig) -> None:
        if self.first_frame:
            self.start_time = time.time()
            self.first_frame = False
        
        # print(f"Callback fired for {sensor.finger_name}: {list(msg.data)}")
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn(
                f"Skipping {sensor.finger_name}: expected at least [Fx, Fy, Fz], got {len(data)} values"
            )
            return
        
        fx = float(data[0])
        fy  = float(data[1])
        fz = float(data[2])

        # Your publishers currently send [Fx, Fy, Fz, timestamp_sec].
        # If a future publisher sends extra values, this keeps using the last
        # value as the source timestamp.
        source_timestamp_sec: Optional[float]
        source_timestamp_sec = float(data[-1]) if len(data) >= 4 else None

        received_timestamp_sec = self.get_clock().now().nanoseconds * 1e-9

        self.counts[sensor.finger_name] += 1
        sample_index = self.counts[sensor.finger_name]

        self.writers[sensor.finger_name].writerow(
            {
            "Fx": f"{fx:.9f}",
            "Fy": f"{fy:.9f}",
            "Fz": f"{fz:.9f}",
            "source_timestamp_sec": "" if source_timestamp_sec is None else f"{source_timestamp_sec:.9f}"
            }
        )

        if sample_index % self.flush_every == 0:
            self.files[sensor.finger_name].flush()
        curr_time = time.time()
        elapsed_time = curr_time - self.start_time

        print(f"Elapsed time: {elapsed_time:.2f} seconds", end='\r')

    def stop_after_duration(self) -> None:
        self.get_logger().info("Requested duration reached. Closing CSV files and stopping recorder.")
        self.close_files()
        rclpy.shutdown()

    def close_files(self) -> None:
        for f in self.files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass

    def destroy_node(self) -> None:
        self.close_files()
        super().destroy_node()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record three ROS 2 force topics into trial-organized CSV files."
    )

    parser.add_argument("-u", "--user-number", required=True, help="User/participant number, e.g. 1 or 001")
    parser.add_argument("-s", "--diameter-mm", required=True, type=int, choices=VALID_DIAMETERS_MM)
    parser.add_argument("-t", "--task-performed", required=True, choices=VALID_TASKS)
    parser.add_argument("-tn", "--trial-number", required=True, help="Trial number, e.g. 1 or 001")

    parser.add_argument("--data-root", default="data", help="Root folder where data/ will be created")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Stop automatically after this many seconds. 0 means run until Ctrl+C")
    parser.add_argument("--flush-every", type=int, default=25, help="Flush each CSV after this many samples")
    parser.add_argument("-ow","--overwrite", action="store_true", help="Delete and recreate the trial folder before recording")
    parser.add_argument("--sensor-subfolders", action="store_true", help="Put each CSV inside its own sensor folder inside the trial folder")

    parser.add_argument(
        "--thumb-topic",
        default="/ERIE_Manipulation/force/object_force_s1_raw",
        help="Topic for sensor 1 / thumb",
    )
    parser.add_argument(
        "--middle-topic",
        default="/ERIE_Manipulation/force/object_force_s2_raw",
        help="Topic for sensor 2 / middle",
    )
    parser.add_argument(
        "--index-topic",
        default="/ERIE_Manipulation/force/object_force_s3_raw",
        help="Topic for sensor 3 / index",
    )

    return parser


def main(cli_args=None) -> None:
    parser = build_arg_parser()
    parsed_args, ros_args = parser.parse_known_args(cli_args)

    rclpy.init(args=ros_args)
    node = ForceTrialCSVRecorder(parsed_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping force trial CSV recorder...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
