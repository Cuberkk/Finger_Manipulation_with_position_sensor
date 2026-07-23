import subprocess
import time
import argparse
import os

VALID_TASKS = ["roll", "pitch", "yaw", "finger_gating","single_fg"]
VALID_DIAMETERS_MM = [50, 80]

def run_command(command, shell_name):
    full_command = f"bash -i -c '{command}'"
    proc = subprocess.Popen(
        f'gnome-terminal --title="{shell_name}" -- bash -c "{full_command}"',
        shell=True,
        preexec_fn=os.setsid
    )
    return proc

def main(args):

    record_raw = f"source ~/.bashrc && cd /home/erie_lab/Documents/kxz365/Github_repos/Finger_manipulation_with_Polhemus && python force_raw_csv_recorder.py -u {args.user_number} -s {args.diameter_mm} -t {args.task_performed} -tn {args.trial_number} --data-root {args.data_root} --duration-sec {args.duration_sec} --flush-every {args.flush_every} {'--overwrite' if args.overwrite else ''} {'--sensor-subfolders' if args.sensor_subfolders else ''}"
    record_polhemus_transformed = f"source ~/.bashrc && cd /home/erie_lab/Documents/kxz365/Github_repos/Finger_manipulation_with_Polhemus && python force_trial_csv_recorder.py -u {args.user_number} -s {args.diameter_mm} -t {args.task_performed} -tn {args.trial_number} --data-root {args.data_root} --duration-sec {args.duration_sec} --flush-every {args.flush_every} {'--sensor-subfolders' if args.sensor_subfolders else ''}"
    record_object_raw = f"source ~/.bashrc && cd /home/erie_lab/Documents/kxz365/Github_repos/Finger_manipulation_with_Polhemus && python object_origin_csv_recorder.py -u {args.user_number} -s {args.diameter_mm} -t {args.task_performed} -tn {args.trial_number} --data-root {args.data_root} --duration-sec {args.duration_sec} --flush-every {args.flush_every} {'--overwrite' if args.overwrite else ''} {'--sensor-subfolders' if args.sensor_subfolders else ''}"

    process1 = run_command(record_raw, "Raw force recorder")
    time.sleep(0.5)
    process2 = run_command(record_polhemus_transformed, "Polhemus transformed force recorder")
    time.sleep(0.5)
    process3 = run_command(record_object_raw, "Object Origin raw force recorder")
    time.sleep(0.5)

    print("All recorders are launched.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch multiple recorder processes.")
    parser.add_argument("-u", "--user-number", required=True, help="User/participant number, e.g. 1 or 001")
    parser.add_argument("-s", "--diameter-mm", required=True, type=int, choices=VALID_DIAMETERS_MM)
    parser.add_argument("-t", "--task-performed", required=True, choices=VALID_TASKS)
    parser.add_argument("-tn", "--trial-number", required=True, help="Trial number, e.g. 1 or 001")

    parser.add_argument("--data-root", default="data", help="Root folder where data/ will be created")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Stop automatically after this many seconds. 0 means run until Ctrl+C")
    parser.add_argument("--flush-every", type=int, default=25, help="Flush each CSV after this many samples")
    parser.add_argument("-ow","--overwrite", action="store_true", help="Delete and recreate the trial folder before recording")
    parser.add_argument("--sensor-subfolders", action="store_true", help="Put each CSV inside its own sensor folder inside the trial folder")

    args = parser.parse_args()
    main(args)