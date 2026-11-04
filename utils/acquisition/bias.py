"""Collect unbiased sensor scans and persist completed zero calibrations."""

import csv
import json
import math
import operator
import os
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np


def _sample_count(value, name, minimum):
    try:
        count = operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be an integer >= {minimum}.") from exc
    if isinstance(value, (bool, np.bool_)) or count < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}.")
    return count


def collect_bias(read_batch, sample_count, channel_count, *, warmup_samples=0, timeout_sec):
    """Return exactly N valid, uncorrected scans after discarding warmup scans.

    ``read_batch`` returns a sample-major (scans, channels) array, or None when
    no scans are available. Readers must remove device-specific dummy scans
    before calibration. Genuine zero scans are valid; nonfinite rows are not.
    The deadline is monotonic, independent of ROS/NTP time changes. A blocking
    native driver call must have its own finite timeout: Python cannot interrupt
    such a call when this deadline expires.
    """
    sample_count = _sample_count(sample_count, "sample_count", 1)
    channel_count = _sample_count(channel_count, "channel_count", 1)
    warmup_samples = _sample_count(warmup_samples, "warmup_samples", 0)
    timeout_sec = float(timeout_sec)
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError("timeout_sec must be finite and positive.")

    deadline = time.monotonic() + timeout_sec
    chunks = []
    collected = 0
    while collected < sample_count:
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Bias calibration timed out: received {collected}/{sample_count} valid scans. "
                "Check the device connection and sample validity."
            )
        batch = read_batch()
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Bias calibration timed out: received {collected}/{sample_count} valid scans."
            )
        accepted = 0
        if batch is not None and batch is not False:
            batch = np.asarray(batch, dtype=np.float64)
            if batch.size:
                if batch.ndim != 2 or batch.shape[1] != channel_count:
                    raise ValueError(f"Bias scans must have shape (N, {channel_count}).")
                discard = min(warmup_samples, len(batch))
                warmup_samples -= discard
                batch = batch[discard:]
                batch = batch[np.isfinite(batch).all(axis=1)]
                batch = batch[:sample_count - collected]
                accepted = len(batch)
                if accepted:
                    chunks.append(batch.copy())
                    collected += accepted
        if not accepted:
            # Do not busy-spin if a device has no samples or only invalid ones.
            time.sleep(max(0.0, min(0.002, deadline - time.monotonic())))
    return np.concatenate(chunks, axis=0)


def save_bias(csv_path, json_path, samples, payload):
    """Stage both outputs and roll back replacements on a handled write failure.

    Collection errors never reach this function. Staging also prevents a failed
    write from replacing an existing calibration with a truncated file. Backups
    allow both previous files to be restored if a replacement fails. Each file
    replacement is atomic; the pair is not a transaction across a process crash.
    CSV has no header, preserving the existing sensor-bias file format.
    """
    samples = np.asarray(samples, dtype=np.float64)
    if samples.ndim != 2 or not samples.shape[0] or not samples.shape[1]:
        raise ValueError("Bias samples must be a nonempty two-dimensional array.")
    if not np.isfinite(samples).all():
        raise ValueError("Bias samples must contain only finite values.")
    json_text = json.dumps(payload, indent=4, allow_nan=False) + "\n"
    csv_path, json_path = Path(csv_path), Path(json_path)
    if csv_path.resolve() == json_path.resolve():
        raise ValueError("Bias CSV and JSON must have different paths.")
    staged = []
    backups = []
    retained_backups = set()
    try:
        for path, is_csv in ((csv_path, True), (json_path, False)):
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", newline="", dir=path.parent,
                prefix=f".{path.name}.", suffix=".tmp", delete=False,
            ) as handle:
                staged.append((Path(handle.name), path))
                if is_csv:
                    csv.writer(handle).writerows(samples)
                else:
                    handle.write(json_text)
        for _, path in staged:
            backup = None
            if path.exists():
                with tempfile.NamedTemporaryFile(
                    dir=path.parent, prefix=f".{path.name}.", suffix=".bak", delete=False,
                ) as handle:
                    backup = Path(handle.name)
                backups.append((backup, path))
                shutil.copyfile(path, backup)
            else:
                backups.append((None, path))
        attempted = []
        try:
            for (temporary, path), (backup, _) in zip(staged, backups):
                attempted.append((backup, path))
                os.replace(temporary, path)
        except BaseException as error:
            restore_errors = []
            for backup, path in reversed(attempted):
                try:
                    if backup is None:
                        path.unlink(missing_ok=True)
                    else:
                        os.replace(backup, path)
                except OSError as restore_error:
                    if backup is not None:
                        retained_backups.add(backup)
                        restore_errors.append(f"{restore_error} (previous data retained at {backup})")
                    else:
                        restore_errors.append(str(restore_error))
            if restore_errors:
                raise RuntimeError(
                    f"Bias save failed ({error}); restoring previous files also failed: "
                    + "; ".join(restore_errors)
                ) from error
            raise
    finally:
        for temporary, _ in staged + backups:
            if temporary is not None and temporary not in retained_backups:
                temporary.unlink(missing_ok=True)
