#!/usr/bin/env python3
"""
random_force_matrix_publisher.py

ROS 2 node that publishes random 3x1 force data to three finger topics.

Published message format:
    Float64MultiArray([Fx, Fy, Fz, timestamp_sec])

Default topic mapping:
    sensor 1 -> thumb  -> /ERIE_Manipulation/force/force_s1_finger1
    sensor 2 -> index  -> /ERIE_Manipulation/force/force_s2_finger2
    sensor 3 -> middle -> /ERIE_Manipulation/force/finger3
"""

import argparse
import random
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


@dataclass(frozen=True)
class SensorTopic:
    sensor_number: int
    finger_name: str
    topic: str


class RandomForceMatrixPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("random_force_matrix_publisher")

        self.publish_rate_hz = float(args.publish_rate)
        self.min_force = float(args.min_force)
        self.max_force = float(args.max_force)

        self.sensors: List[SensorTopic] = [
            SensorTopic(1, "thumb", args.thumb_topic),
            SensorTopic(2, "index", args.index_topic),
            SensorTopic(3, "middle", args.middle_topic),
        ]

        self.publishers = {
            sensor.finger_name: self.create_publisher(Float64MultiArray, sensor.topic, 10)
            for sensor in self.sensors
        }

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        for sensor in self.sensors:
            self.get_logger().info(
                f"Publishing random force data for sensor {sensor.sensor_number} "
                f"({sensor.finger_name}) on {sensor.topic}"
            )

    def _build_layout(self) -> MultiArrayLayout:
        return MultiArrayLayout(
            dim=[
                MultiArrayDimension(label="rows", size=3, stride=3),
                MultiArrayDimension(label="cols", size=1, stride=1),
            ],
            data_offset=0,
        )

    def _random_force_vector(self) -> List[float]:
        return [
            random.uniform(self.min_force, self.max_force),
            random.uniform(self.min_force, self.max_force),
            random.uniform(self.min_force, self.max_force),
        ]

    def timer_callback(self) -> None:
        timestamp_sec = self.get_clock().now().nanoseconds * 1e-9
        layout = self._build_layout()

        for sensor in self.sensors:
            fx, fy, fz = self._random_force_vector()

            msg = Float64MultiArray()
            msg.layout = layout
            msg.data = [fx, fy, fz, timestamp_sec]

            self.publishers[sensor.finger_name].publish(msg)

            self.get_logger().debug(
                f"{sensor.finger_name}: "
                f"[{fx:.3f}, {fy:.3f}, {fz:.3f}, {timestamp_sec:.6f}]"
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish random 3x1 force vectors to three ROS 2 finger topics."
    )

    parser.add_argument(
        "--publish-rate",
        type=float,
        default=60.0,
        help="Publishing frequency in Hz",
    )
    parser.add_argument(
        "--min-force",
        type=float,
        default=-5.0,
        help="Minimum random force value",
    )
    parser.add_argument(
        "--max-force",
        type=float,
        default=5.0,
        help="Maximum random force value",
    )

    parser.add_argument(
        "--thumb-topic",
        default="/ERIE_Manipulation/force/force_s1_finger1",
        help="Topic for sensor 1 / thumb",
    )
    parser.add_argument(
        "--index-topic",
        default="/ERIE_Manipulation/force/force_s2_finger2",
        help="Topic for sensor 2 / index",
    )
    parser.add_argument(
        "--middle-topic",
        default="/ERIE_Manipulation/force/finger3",
        help="Topic for sensor 3 / middle",
    )

    return parser


def main(cli_args=None) -> None:
    parser = build_arg_parser()
    parsed_args, ros_args = parser.parse_known_args(cli_args)

    rclpy.init(args=ros_args)
    node = RandomForceMatrixPublisher(parsed_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping random force matrix publisher...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
