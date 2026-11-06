#!/usr/bin/env python3
"""
force_raw_csv_recorder.py

ROS 2 node that records ATI-calibrated, uncorrected sensor-axis force and torque
from the three acquisition topics into one CSV per finger for a single trial.

Expected topic message format from your publishers:
    Float64MultiArray([Fx, Fy, Fz, Tx, Ty, Tz, timestamp_sec])

Default topic mapping:
    sensor 1 -> thumb -> /ERIE_Manipulation/acquisition/wrench_s1_uncorrected
    sensor 2 -> middle -> /ERIE_Manipulation/acquisition/wrench_s2_uncorrected
    sensor 3 -> index -> /ERIE_Manipulation/acquisition/wrench_s3_uncorrected
"""

import argparse
import csv
import json
import math
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List
import time

# Allow direct execution as well as python -m utils.<package>.<module>.
if __package__ in (None, ""):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.paths import DATA_DIR

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


VALID_TASKS = ["roll", "pitch", "yaw", "finger_gating", "single_fg"]
VALID_DIAMETERS_MM = [50, 80]


@dataclass(frozen=True)
class SensorConfig:
    sensor_number: int
    finger_name: str
    topic: str
    csv_name: str


class ForceTrialCSVRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("force_raw_csv_recorder")
        self.files: Dict[str, object] = {}
        self.writers: Dict[str, csv.DictWriter] = {}
        try:
            self._initialize(args)
        except BaseException:
            self.destroy_node()
            raise

    def _initialize(self, args: argparse.Namespace):
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
                csv_name="thumb_raw_force.csv",
            ),
            SensorConfig(
                sensor_number=2,
                finger_name="middle",
                topic=args.middle_topic,
                csv_name="middle_raw_force.csv",
            ),
            SensorConfig(
                sensor_number=3,
                finger_name="index",
                topic=args.index_topic,
                csv_name="index_raw_force.csv",
            ),
        ]

        self.trial_dir = self._make_trial_dir(args.data_root, args.overwrite)
        self.get_logger().info(f"Recording trial into: {self.trial_dir}")

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
            Path(data_root).expanduser()
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
            return sensor_dir / sensor.csv_name

        return self.trial_dir / sensor.csv_name

    def _open_csv_files(self) -> None:
        header = [
            "source_timestamp_sec",
            "Fx",
            "Fy",
            "Fz",
            "Tx",
            "Ty",
            "Tz",
        ]
        paths = {sensor.finger_name: self._csv_path_for_sensor(sensor) for sensor in self.sensors}
        topics = {sensor.finger_name: sensor.topic for sensor in self.sensors}
        canonical = all(sensor.topic == (
            f"/ERIE_Manipulation/acquisition/wrench_s{sensor.sensor_number}_uncorrected")
                        for sensor in self.sensors)
        schema = {
            "schema_version": 2,
            "fields": header,
            "source_topics": topics,
            "bias_applied": False if canonical else None,
            "coordinate_frame": "ati_sensor" if canonical else None,
            "csv_files": {finger: path.relative_to(self.trial_dir).as_posix()
                          for finger, path in paths.items()},
        }
        self._ensure_raw_schema(schema, paths)

        for sensor in self.sensors:
            csv_path = paths[sensor.finger_name]
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            file_exists_and_has_data = csv_path.exists() and csv_path.stat().st_size > 0

            f = open(csv_path, "a", newline="")
            self.files[sensor.finger_name] = f
            writer = csv.DictWriter(f, fieldnames=header)
            self.writers[sensor.finger_name] = writer

            if not file_exists_and_has_data:
                writer.writeheader()
                f.flush()

    def _ensure_raw_schema(self, schema, paths) -> None:
        """Append only when provenance, paths, and seven-column headers match."""
        schema_path = self.trial_dir / "raw_force_schema.json"
        existing = {path for path in self.trial_dir.rglob("*_raw_force.csv")
                    if path.is_file() and path.stat().st_size > 0}
        message = "Choose a new trial folder to keep raw recording schemas separate."
        if schema_path.exists():
            try:
                recorded_schema = json.loads(schema_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise ValueError(f"Invalid raw_force_schema.json. {message}") from exc
            if recorded_schema != schema:
                raise ValueError(f"Raw recording schema or source topics do not match. {message}")
        elif existing:
            raise FileExistsError(
                f"Existing raw CSVs have no raw_force_schema.json provenance. {message}")
        expected_paths = set(paths.values())
        if not existing <= expected_paths:
            raise ValueError(f"Existing raw CSV paths do not match this recording layout. {message}")
        for path in existing:
            with path.open(newline="", encoding="utf-8") as stream:
                header = next(csv.reader(stream), None)
            if header != schema["fields"]:
                raise ValueError(f"Raw CSV header does not match: {path.name}. {message}")
        if not schema_path.exists():
            # Exclusive creation also prevents competing first-time recorders
            # from silently claiming the same new trial's provenance.
            with schema_path.open("x", encoding="utf-8") as stream:
                json.dump(schema, stream, indent=2, allow_nan=False)
                stream.write("\n")


    def force_callback(self, msg: Float64MultiArray, sensor: SensorConfig) -> None:
        try:
            data = tuple(float(value) for value in msg.data)
        except (ValueError, TypeError, OverflowError):
            data = ()
        if len(data) != 7 or not all(math.isfinite(value) for value in data):
            self.get_logger().warn(
                f"Skipping {sensor.finger_name}: expected six finite raw F/T components and a timestamp.",
                throttle_duration_sec=5.0,
            )
            return
        if self.first_frame:
            self.start_time = time.time()
            self.first_frame = False

        fx = float(data[0])
        fy = float(data[1])
        fz = float(data[2])
        tx = float(data[3])
        ty = float(data[4])
        tz = float(data[5])
        source_timestamp_sec = float(data[6])

        self.counts[sensor.finger_name] += 1
        sample_index = self.counts[sensor.finger_name]

        self.writers[sensor.finger_name].writerow(
            {
                "Fx": f"{fx:.9f}",
                "Fy": f"{fy:.9f}",
                "Fz": f"{fz:.9f}",
                "Tx": f"{tx:.9f}",
                "Ty": f"{ty:.9f}",
                "Tz": f"{tz:.9f}",
                "source_timestamp_sec": f"{source_timestamp_sec:.9f}"
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
        description="Record three uncorrected ATI force/torque topics into trial-organized CSV files."
    )

    parser.add_argument("-u", "--user-number", required=True, help="User/participant number, e.g. 1 or 001")
    parser.add_argument("-s", "--diameter-mm", required=True, type=int, choices=VALID_DIAMETERS_MM)
    parser.add_argument("-t", "--task-performed", required=True, choices=VALID_TASKS)
    parser.add_argument("-tn", "--trial-number", required=True, help="Trial number, e.g. 1 or 001")

    parser.add_argument("--data-root", default=str(DATA_DIR), help="Output root (defaults to the repository data/ directory)")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Stop automatically after this many seconds. 0 means run until Ctrl+C")
    parser.add_argument("--flush-every", type=int, default=25, help="Flush each CSV after this many samples")
    parser.add_argument("-ow","--overwrite", action="store_true", help="Delete and recreate the trial folder before recording")
    parser.add_argument("--sensor-subfolders", action="store_true", help="Put each CSV inside its own sensor folder inside the trial folder")

    parser.add_argument(
        "--thumb-topic",
        default="/ERIE_Manipulation/acquisition/wrench_s1_uncorrected",
        help="Topic for sensor 1 / thumb",
    )
    parser.add_argument(
        "--middle-topic",
        default="/ERIE_Manipulation/acquisition/wrench_s2_uncorrected",
        help="Topic for sensor 2 / middle",
    )
    parser.add_argument(
        "--index-topic",
        default="/ERIE_Manipulation/acquisition/wrench_s3_uncorrected",
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
