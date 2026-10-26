"""Rest durations and monotonic countdowns, independent of Tk and ROS."""
from __future__ import annotations

import ast
from dataclasses import dataclass
import math
import operator
from pathlib import Path
from typing import NamedTuple

from utils.gui.study_model import TrialSpec


_KINDS = ("trials", "tasks", "objects")


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


def load_rest_times(path: str | Path) -> RestTimes:
    """Read literal minute settings from Rest_Time.py without executing it.

    Positive fractional seconds round up so the integer-second rest never
    shortens the requested duration. Set a value to zero to omit that rest.
    """
    path = Path(path)
    try:
        module = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    except SyntaxError as exc:
        raise ValueError(f"Invalid rest time configuration: {exc.msg}") from exc
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
            raise ValueError("Rest time configuration must contain literal assignments to trials, tasks, and objects.")
        if name not in _KINDS or name in values:
            raise ValueError(f"Unexpected or duplicate rest time setting: {name}.")
        try:
            minutes = ast.literal_eval(expression)
        except (TypeError, ValueError, SyntaxError) as exc:
            raise ValueError(f"{name} must be a finite, nonnegative number of minutes.") from exc
        if isinstance(minutes, bool) or not isinstance(minutes, (int, float)):
            raise ValueError(f"{name} must be a finite, nonnegative number of minutes.")
        try:
            seconds = minutes * 60
            valid = math.isfinite(seconds) and minutes >= 0
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"{name} must be a finite, nonnegative number of minutes.")
        values[name] = math.ceil(seconds)
    missing = set(_KINDS) - values.keys()
    if missing:
        raise ValueError("Missing rest time settings: " + ", ".join(sorted(missing)) + ".")
    return RestTimes(**{f"{kind}_sec": values[kind] for kind in _KINDS})


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
