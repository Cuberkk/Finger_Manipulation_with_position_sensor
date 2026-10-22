import subprocess
import time
import argparse
import shlex
import shutil
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.paths import DATA_DIR, PROJECT_ROOT

VALID_TASKS = ["roll", "pitch", "yaw", "finger_gating","single_fg"]
VALID_DIAMETERS_MM = [50, 80]

def run_command(command, shell_name):
    full_command = f"cd {shlex.quote(str(PROJECT_ROOT))} && exec {shlex.join(command)}"
    proc = subprocess.Popen(
        ["gnome-terminal", f"--title={shell_name}", "--", "bash", "-i", "-c", full_command],
        cwd=PROJECT_ROOT,
        start_new_session=True,
    )
    return proc

def main(args):
    # Resolve before opening terminals, whose working directory is PROJECT_ROOT.
    data_root = Path(args.data_root).expanduser().resolve()
    if args.overwrite:
        trial_dir = (
            data_root / f"user_{args.user_number}" / f"{args.diameter_mm}mm"
            / args.task_performed / f"trial_{args.trial_number}"
        )
        # All recorders share this folder. Clear it once before any file is opened.
        if trial_dir.exists():
            shutil.rmtree(trial_dir)

    common_args = [
        "-u", str(args.user_number), "-s", str(args.diameter_mm),
        "-t", args.task_performed, "-tn", str(args.trial_number),
        "--data-root", str(data_root), "--duration-sec", str(args.duration_sec),
        "--flush-every", str(args.flush_every),
    ]
    if args.sensor_subfolders:
        common_args.append("--sensor-subfolders")

    for module, title in [
        ("force_raw_csv_recorder", "Raw force recorder"),
        ("force_trial_csv_recorder", "Polhemus transformed force recorder"),
        ("object_origin_csv_recorder", "Object Origin raw force recorder"),
    ]:
        run_command(
            [sys.executable, "-m", f"utils.recording.{module}", *common_args],
            title,
        )
        time.sleep(0.5)

    print("All recorders are launched.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch multiple recorder processes.")
    parser.add_argument("-u", "--user-number", required=True, help="User/participant number, e.g. 1 or 001")
    parser.add_argument("-s", "--diameter-mm", required=True, type=int, choices=VALID_DIAMETERS_MM)
    parser.add_argument("-t", "--task-performed", required=True, choices=VALID_TASKS)
    parser.add_argument("-tn", "--trial-number", required=True, help="Trial number, e.g. 1 or 001")

    parser.add_argument("--data-root", default=str(DATA_DIR), help="Output root (defaults to the repository data/ directory)")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Stop automatically after this many seconds. 0 means run until Ctrl+C")
    parser.add_argument("--flush-every", type=int, default=25, help="Flush each CSV after this many samples")
    parser.add_argument("-ow","--overwrite", action="store_true", help="Delete and recreate the trial folder before recording")
    parser.add_argument("--sensor-subfolders", action="store_true", help="Put each CSV inside its own sensor folder inside the trial folder")

    args = parser.parse_args()
    main(args)
