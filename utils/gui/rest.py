"""Literal study settings and monotonic countdowns, independent of Tk and ROS."""
from __future__ import annotations

import ast
from dataclasses import dataclass
import math
import operator
from pathlib import Path
from typing import NamedTuple

import numpy as np

from utils.gui.study_model import TrialSpec


_KINDS = ("trials", "tasks", "objects")
_STUDY_SETTINGS = ("PRACTICE_TARGETS_DEG", "SENSOR4_FROM_OBJECT")


def _whole_seconds(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a nonnegative whole number of seconds.")
    try:
        seconds = operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be a nonnegative whole number of seconds.") from exc
    if seconds < 0:
        raise ValueError(f"{name} must be a nonnegative whole number of seconds.")
    return seconds


@dataclass(frozen=True)
class RestSpec:
    kind: str
    total_sec: int

    def __post_init__(self):
        if self.kind not in _KINDS:
            raise ValueError("Rest kind must be trials, tasks, or objects.")
        _whole_seconds(self.total_sec, "Rest duration")


@dataclass(frozen=True)
class RestTimes:
    trials_sec: int
    tasks_sec: int
    objects_sec: int

    def __post_init__(self):
        for kind in _KINDS:
            _whole_seconds(getattr(self, f"{kind}_sec"), kind)

    def between(self, previous: TrialSpec, next: TrialSpec) -> RestSpec:
        """Use one boundary duration: object changes take priority over tasks.

        Practice and measured repetitions within the same task/object use the
        ordinary trial rest. Boundary durations are not added together.
        """
        kind = ("objects" if previous.diameter_mm != next.diameter_mm else
                "tasks" if previous.task != next.task else "trials")
        return RestSpec(kind, getattr(self, f"{kind}_sec"))


@dataclass(frozen=True)
class StudySettings:
    rest_times: RestTimes
    practice_targets_deg: dict[int, float]
    sensor4_from_object: np.ndarray


def _read_literal_settings(path: str | Path) -> dict:
    """Read only whitelisted literal assignments, never importing the file."""
    path = Path(path)
    try:
        module = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    except SyntaxError as exc:
        raise ValueError(f"Invalid study configuration: {exc.msg}") from exc
    values = {}
    for statement in module.body:
        if (isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Constant)
                and isinstance(statement.value.value, str)):
            continue
        if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
                and isinstance(statement.targets[0], ast.Name)):
            name, expression = statement.targets[0].id, statement.value
        elif isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name):
            name, expression = statement.target.id, statement.value
        else:
            raise ValueError("Study configuration must contain only literal assignments to "
                             "trials, tasks, objects, PRACTICE_TARGETS_DEG, and SENSOR4_FROM_OBJECT.")
        if name not in _KINDS + _STUDY_SETTINGS or name in values:
            raise ValueError(f"Unexpected or duplicate study setting: {name}.")
        try:
            values[name] = ast.literal_eval(expression)
        except (TypeError, ValueError, SyntaxError) as exc:
            raise ValueError(f"{name} must be a literal value.") from exc
    return values


def _rest_times_from_values(values: dict) -> RestTimes:
    missing = set(_KINDS) - values.keys()
    if missing:
        raise ValueError("Missing rest time settings: " + ", ".join(sorted(missing)) + ".")
    seconds_by_kind = {}
    for name in _KINDS:
        minutes = values[name]
        if isinstance(minutes, bool) or not isinstance(minutes, (int, float)):
            raise ValueError(f"{name} must be a finite, nonnegative number of minutes.")
        try:
            seconds = minutes * 60
            valid = math.isfinite(seconds) and minutes >= 0
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"{name} must be a finite, nonnegative number of minutes.")
        seconds_by_kind[name] = math.ceil(seconds)
    return RestTimes(**{f"{kind}_sec": seconds_by_kind[kind] for kind in _KINDS})


def load_rest_times(path: str | Path) -> RestTimes:
    """Read literal minute settings without executing the configuration file.

    Rest-only files remain supported. Positive fractional seconds round up so
    the integer-second rest never shortens the requested duration.
    """
    return _rest_times_from_values(_read_literal_settings(path))


def _practice_targets(value) -> dict[int, float]:
    name = "PRACTICE_TARGETS_DEG"
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a diameter-to-angle dictionary.")
    targets = {}
    for diameter, target in value.items():
        if isinstance(diameter, bool) or not isinstance(diameter, int) or diameter <= 0:
            raise ValueError(f"{name} diameter keys must be positive integers in millimetres.")
        if isinstance(target, bool) or not isinstance(target, (int, float)):
            raise ValueError(f"{name}[{diameter}] must be a finite, positive angle in degrees.")
        try:
            target = float(target)
        except (ValueError, OverflowError) as exc:
            raise ValueError(f"{name}[{diameter}] must be a finite, positive angle in degrees.") from exc
        if not math.isfinite(target) or target <= 0:
            raise ValueError(f"{name}[{diameter}] must be a finite, positive angle in degrees.")
        targets[diameter] = target
    missing = {50, 80} - targets.keys()
    if missing:
        raise ValueError(f"{name} is missing object diameters: " + ", ".join(map(str, sorted(missing))) + ".")
    return targets


def _sensor4_from_object(value) -> np.ndarray:
    name = "SENSOR4_FROM_OBJECT"
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(not isinstance(row, (list, tuple)) or len(row) != 3 for row in value)):
        raise ValueError(f"{name} must be a 3 by 3 rotation matrix.")
    if any(isinstance(entry, bool) or not isinstance(entry, (int, float))
           for row in value for entry in row):
        raise ValueError(f"{name} must contain only finite real numbers.")
    try:
        matrix = np.array(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain only finite real numbers.") from exc
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must contain only finite real numbers.")
    with np.errstate(over="ignore", invalid="ignore"):
        orthogonal = np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6, rtol=0)
    if not orthogonal or not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} must be orthogonal with determinant +1.")
    matrix.setflags(write=False)
    return matrix


def load_study_settings(path: str | Path) -> StudySettings:
    """Load required study settings from literal assignments without execution."""
    values = _read_literal_settings(path)
    missing = set(_STUDY_SETTINGS) - values.keys()
    if missing:
        raise ValueError("Missing study settings: " + ", ".join(sorted(missing)) + ".")
    return StudySettings(
        rest_times=_rest_times_from_values(values),
        practice_targets_deg=_practice_targets(values["PRACTICE_TARGETS_DEG"]),
        sensor4_from_object=_sensor4_from_object(values["SENSOR4_FROM_OBJECT"]),
    )


class RestSnapshot(NamedTuple):
    elapsed: int
    remaining: int
    progress: float
    complete: bool


@dataclass(frozen=True)
class RestCountdown:
    spec: RestSpec
    started: float

    def __post_init__(self):
        if not math.isfinite(self.started):
            raise ValueError("Rest start time must be finite.")

    @property
    def deadline(self):
        return self.started + self.spec.total_sec

    def snapshot(self, now: float) -> RestSnapshot:
        if not math.isfinite(now):
            raise ValueError("Rest time must be finite.")
        total = self.spec.total_sec
        if total == 0 or now >= self.deadline:
            return RestSnapshot(total, 0, 1.0, True)
        elapsed = min(total, max(0.0, now - self.started))
        return RestSnapshot(math.floor(elapsed), math.ceil(total - elapsed), elapsed / total, False)
