#!/usr/bin/env python3
"""
launch_force_trial_recording.py

Launcher-style script that starts:
    1. NI-DAQ force publisher for sensor 1/thumb and sensor 2/index
    2. LabJack force publisher for sensor 3/middle
    3. force_trial_csv_recorder.py for CSV logging
"""

import argparse
import atexit
from email import parser
import os
import shlex
import signal
import subprocess
import time


DEFAULT_ROS_NODES_DIR = "/home/erie_lab/Finger_Manipulation_with_position_sensor/publishing_files"
DEFAULT_CONDA_ACTIVATE_CMD = "activate-dlc"

processes = []


def run_command(command: str, shell_name: str):
    full_command = f"bash -i -c {shlex.quote(command)}"
    terminal_command = f"gnome-terminal --title={shlex.quote(shell_name)} -- bash -c {shlex.quote(full_command)}"

    proc = subprocess.Popen(
        terminal_command,
        shell=True,
        preexec_fn=os.setsid,
    )
    processes.append(proc)
    return proc


def cleanup_processes():
    for proc in processes:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Launch force publishers and CSV recorder for one trial.")

    parser.add_argument("-u", "--user-number", required=True)
    parser.add_argument("-s", "--diameter-mm", required=True, type=int, choices=[50, 80])
    parser.add_argument("-t", "--task-performed", required=True, choices=["roll", "pitch", "yaw", "finger_gating"])
    parser.add_argument("-tn", "--trial-number", required=True)

    parser.add_argument("--duration-sec", type=float, default=0.0, help="0 means record until Ctrl+C in the recorder terminal")
    parser.add_argument("--data-root", default="data")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--sensor-subfolders", action="store_true")

    parser.add_argument("--ros-nodes-dir", default=DEFAULT_ROS_NODES_DIR)
    parser.add_argument("--activate-cmd", default=DEFAULT_CONDA_ACTIVATE_CMD)

    parser.add_argument("--nidaq-file", default="NIDAQ_ATI_publisher.py")
    parser.add_argument("--labjack-file", default="ATI_publisher.py")
    parser.add_argument("--recorder-file", default="force_trial_csv_recorder.py")

    parser.add_argument("--no-publishers", action="store_true", help="Only launch the CSV recorder")

    return parser


def main():
    args = build_arg_parser().parse_args()
    atexit.register(cleanup_processes)

    nodes_dir = args.ros_nodes_dir.rstrip("/")
    activate_prefix = f"source ~/.bashrc && {args.activate_cmd} && cd {shlex.quote(nodes_dir)}"

    nidaq_cmd = f"{activate_prefix} && python {shlex.quote(args.nidaq_file)}"
    labjack_cmd = f"{activate_prefix} && python {shlex.quote(args.labjack_file)}"

    recorder_parts = [
        f"{activate_prefix} && python {shlex.quote(args.recorder_file)}",
        "-u", shlex.quote(str(args.user_number)),
        "-s", shlex.quote(str(args.diameter_mm)),
        "-t", shlex.quote(str(args.task_performed)),
        "-tn", shlex.quote(str(args.trial_number)),
        "--data-root", shlex.quote(str(args.data_root)),
        "--duration-sec", shlex.quote(str(args.duration_sec)),
    ]

    if args.overwrite:
        recorder_parts.append("--overwrite")
    if args.sensor_subfolders:
        recorder_parts.append("--sensor-subfolders")

    recorder_cmd = " ".join(recorder_parts)

    if not args.no_publishers:
        run_command(nidaq_cmd, "Shell 1 (NI-DAQ ATI Publisher: Thumb + Index)")
        time.sleep(2)

        run_command(labjack_cmd, "Shell 2 (LabJack ATI Publisher: Middle)")
        time.sleep(2)

    run_command(recorder_cmd, "Shell 3 (Force Trial CSV Recorder)")

    print("All force recording processes are launched.")
    print("Trial folder will be under:")
    print(f"  {args.data_root}/user_{args.user_number}/{args.diameter_mm}mm/{args.task_performed}/trial_{args.trial_number}")


if __name__ == "__main__":
    main()
