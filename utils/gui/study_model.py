"""Study lookup, orientation measurements, and cycle counting without GUI or hardware."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
import operator
from pathlib import Path
import re

import numpy as np


_TASKS = {"R": "roll", "P": "pitch", "Y": "yaw", "G": "finger_gating"}
_AXES = {"roll": "x", "pitch": "y", "yaw": "z", "finger_gating": None}


@dataclass(frozen=True)
class TrialSpec:
    reference: str
    code: str
    diameter_mm: int
    task: str
    repetition: int | None
    practice: bool

    @property
    def axis(self) -> str | None:
        """Fixed mesh axis for rotation tasks; gating is manually counted."""
        return _AXES[self.task]

    @property
    def task_position(self) -> int | None:
        return None if self.repetition is None else self.repetition + 1

    @property
    def display_number(self) -> str:
        """Keep the lookup reference, including its leading zero or practice prefix."""
        return self.reference


def load_trials(path: str | Path) -> list[TrialSpec]:
    """Read lookup order and distinguish practice rows from measured repetitions."""
    trials = []
    references = set()
    codes = set()
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream, skipinitialspace=True)
        if reader.fieldnames is None:
            raise ValueError("Trial lookup is empty.")
        reader.fieldnames = [name.strip() for name in reader.fieldnames]
        if not {"Trial_Reference_Number", "Trial_Task"}.issubset(reader.fieldnames):
            raise ValueError("Trial lookup needs Trial_Reference_Number and Trial_Task columns.")
        for row_number, row in enumerate(reader, start=2):
            reference = (row.get("Trial_Reference_Number") or "").strip()
            code = (row.get("Trial_Task") or "").strip().strip("'\"")
            match = re.fullmatch(r"(\d+)([RPYG])(Practice|\d+)", code)
            if match is None:
                raise ValueError(f"Invalid trial code on row {row_number}: {code!r}")
            diameter, task_code, suffix = match.groups()
            practice = suffix == "Practice"
            repetition = None if practice else int(suffix)
            if repetition is not None and not 0 <= repetition < 5:
                raise ValueError(f"Trial repetition must be 0 through 4 on row {row_number}.")
            reference_pattern = r"P\d+" if practice else r"\d+"
            if re.fullmatch(reference_pattern, reference) is None:
                raise ValueError(f"Invalid trial reference on row {row_number}: {reference!r}")
            if reference in references or code in codes:
                raise ValueError(f"Duplicate trial on row {row_number}: {reference!r}, {code!r}")
            if int(diameter) <= 0:
                raise ValueError(f"Invalid diameter on row {row_number}: {diameter!r}")
            references.add(reference)
            codes.add(code)
            trials.append(TrialSpec(reference, code, int(diameter), _TASKS[task_code],
                                    repetition, practice))
    if not trials:
        raise ValueError("Trial lookup contains no trials.")
    return trials


def normalize_quaternion(quaternion) -> np.ndarray:
    """Return a finite unit quaternion in ROS order (x, y, z, w)."""
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,) or not np.all(np.isfinite(value)):
        raise ValueError("Quaternion must contain four finite values in xyzw order.")
    scale = np.max(np.abs(value))
    if scale == 0:
        raise ValueError("A zero quaternion has no orientation.")
    scaled = value / scale
    return scaled / np.linalg.norm(scaled)


def quaternion_to_matrix(quaternion) -> np.ndarray:
    """Convert an xyzw quaternion to a proper 3 by 3 rotation matrix."""
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def _rotation_matrix(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-6)):
        raise ValueError("Expected a finite, proper 3 by 3 rotation matrix.")
    return matrix


def relative_rotation(baseline_quat, current_quat, alignment_matrix=None) -> np.ndarray:
    """Measure baseline-relative rotation in fixed mesh axes.

    A maps mesh coordinates to sensor 4 coordinates, so the result is
    A.T @ R0.T @ Rt @ A. No implicit axis permutation is applied.
    """
    alignment = np.eye(3) if alignment_matrix is None else _rotation_matrix(alignment_matrix)
    baseline = quaternion_to_matrix(baseline_quat)
    current = quaternion_to_matrix(current_quat)
    return alignment.T @ baseline.T @ current @ alignment


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Stable xyzw conversion, including rotations close to 180 degrees."""
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1) * 2
        return np.array([(matrix[2, 1] - matrix[1, 2]) / scale,
                         (matrix[0, 2] - matrix[2, 0]) / scale,
                         (matrix[1, 0] - matrix[0, 1]) / scale, scale / 4])
    i = int(np.argmax(np.diag(matrix)))
    j, k = (i + 1) % 3, (i + 2) % 3
    scale = math.sqrt(max(0, 1 + matrix[i, i] - matrix[j, j] - matrix[k, k])) * 2
    quaternion = np.zeros(4)
    quaternion[i] = scale / 4
    quaternion[j] = (matrix[j, i] + matrix[i, j]) / scale
    quaternion[k] = (matrix[k, i] + matrix[i, k]) / scale
    quaternion[3] = (matrix[k, j] - matrix[j, k]) / scale
    return quaternion


def signed_angle_degrees(rotation_matrix, axis: str | np.ndarray) -> float:
    """Return signed twist about a fixed mesh axis in [-180, 180) degrees.

    Swing/twist projection excludes rotation about other axes. A 180-degree
    swing perpendicular to the requested axis has no defined twist and raises
    ValueError; it must not be treated as a valid tracking sample.
    """
    if isinstance(axis, str):
        if axis not in ("x", "y", "z"):
            raise ValueError("Rotation axis must be x, y, or z.")
        direction = np.eye(3)[("x", "y", "z").index(axis)]
    else:
        direction = np.asarray(axis, dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            raise ValueError("Rotation axis must be a finite three-component vector.")
        scale = np.max(np.abs(direction))
        if scale == 0:
            raise ValueError("Rotation axis must be nonzero.")
        direction = direction / scale
        direction = direction / np.linalg.norm(direction)
    quaternion = _matrix_to_quaternion(_rotation_matrix(rotation_matrix))
    projected = float(np.dot(quaternion[:3], direction))
    scalar = float(quaternion[3])
    if math.hypot(projected, scalar) < 1e-10:
        raise ValueError("Twist is undefined for this perpendicular 180-degree swing.")
    angle = math.degrees(2 * math.atan2(projected, scalar))
    return (angle + 180) % 360 - 180


class RotationCounter:
    """Count neutral -> negative -> neutral -> positive -> neutral sequences.

    Every zone must be occupied for dwell_sec using fresh, increasing sample
    timestamps (seconds). Repeated timestamps do not advance a dwell. Invalid
    samples, backwards timestamps, or gaps greater than max_gap_sec discard
    the partial sequence while preserving completed cycles.
    """

    _INSTRUCTIONS = {
        "seek_neutral": "Return to neutral and hold.",
        "negative": "Rotate toward the negative target and hold.",
        "neutral_after_negative": "Return to neutral and hold.",
        "positive": "Rotate toward the positive target and hold.",
        "neutral_after_positive": "Return to neutral to finish the cycle.",
        "complete": "All cycles complete.",
    }

    def __init__(self, target_deg: float = 30, neutral_deg: float = 5,
                 dwell_sec: float = .12, target_cycles: int = 20,
                 max_gap_sec: float = .5):
        self.target_deg = float(target_deg)
        self.neutral_deg = float(neutral_deg)
        self.dwell_sec = float(dwell_sec)
        self.max_gap_sec = float(max_gap_sec)
        self.target_cycles = operator.index(target_cycles)
        if (not all(math.isfinite(value) for value in
                    (self.target_deg, self.neutral_deg, self.dwell_sec, self.max_gap_sec))
                or not 0 <= self.neutral_deg < self.target_deg < 180
                or self.dwell_sec < 0 or self.max_gap_sec <= 0
                or self.target_cycles <= 0):
            raise ValueError("Invalid rotation counter thresholds, timing, or cycle count.")
        self.count = 0
        self.phase = "seek_neutral"
        self._dwell_start = None
        self._last_timestamp = None

    @property
    def complete(self) -> bool:
        return self.count >= self.target_cycles

    @property
    def instruction(self) -> str:
        return self._INSTRUCTIONS[self.phase]

    @property
    def progress(self) -> float:
        return min(1.0, self.count / self.target_cycles)

    def _restart_partial(self):
        self.phase = "complete" if self.complete else "seek_neutral"
        self._dwell_start = None

    def reset_partial(self):
        """Require a new neutral hold, retaining only already completed cycles."""
        self._restart_partial()
        self._last_timestamp = None

    def update(self, angle_deg: float, timestamp: float) -> bool:
        """Return True only on the fresh sample that completes a full cycle."""
        if self.complete:
            return False
        try:
            angle, time = float(angle_deg), float(timestamp)
        except (TypeError, ValueError, OverflowError):
            self._restart_partial()
            return False
        if not math.isfinite(angle) or not math.isfinite(time) or abs(angle) > 180:
            self._restart_partial()
            return False
        if self._last_timestamp is not None:
            if time < self._last_timestamp:
                self._restart_partial()
                return False
            if time == self._last_timestamp:
                return False
            if time - self._last_timestamp > self.max_gap_sec:
                self._restart_partial()
        self._last_timestamp = time

        if self.phase == "negative":
            in_zone = angle <= -self.target_deg
        elif self.phase == "positive":
            in_zone = angle >= self.target_deg
        else:
            in_zone = abs(angle) <= self.neutral_deg
        if not in_zone:
            self._dwell_start = None
            return False
        if self._dwell_start is None:
            self._dwell_start = time
        if time - self._dwell_start + 1e-12 < self.dwell_sec:
            return False
        self._dwell_start = None
        if self.phase == "neutral_after_positive":
            self.count += 1
            self.phase = "complete" if self.complete else "negative"
            return True
        self.phase = {
            "seek_neutral": "negative",
            "negative": "neutral_after_negative",
            "neutral_after_negative": "positive",
            "positive": "neutral_after_positive",
        }[self.phase]
        return False
