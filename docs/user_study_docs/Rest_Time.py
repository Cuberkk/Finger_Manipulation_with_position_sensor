# Study settings: literal assignments only; this file is read without execution.
# Rest time between trials, tasks, and objects, in minutes.
trials = 1.0
tasks = 2.0
objects = 5.0

# Practice rotation targets by object diameter (millimetres), in degrees.
PRACTICE_TARGETS_DEG = {50: 20, 80: 10}

# v_sensor4 = SENSOR4_FROM_OBJECT @ v_object.
# 0.8660254037844386 is sqrt(3) / 2; keep matrix entries as numeric literals.
SENSOR4_FROM_OBJECT = [
    [-0.5, 0.8660254037844386, 0.0],
    [0.8660254037844386, 0.5, 0.0],
    [0.0, 0.0, -1.0],
]
