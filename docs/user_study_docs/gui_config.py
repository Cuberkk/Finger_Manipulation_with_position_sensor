# Shared GUI, acquisition, and central ATI settings: literal assignments only.
# This file is read without execution. Restart the affected process after edits.
# Rest time between trials, tasks, and objects, in seconds.
# Use finite nonnegative numbers; 0 skips rest and fractional seconds round up.
trials = 60
tasks = 120
objects = 300

# Preparation durations in seconds, before every trial and angle test.
# Use finite positive numbers; fractional seconds round up. Restart the GUI.
PICKUP_SEC = 10
HOLD_NEUTRAL_SEC = 5

# Hardware sampling and ROS publication targets, in Hz; finite positive numbers.
# Each publisher reads its local copy at startup. Restart it after rate changes.
NIDAQ_SAMPLE_RATE_HZ = 600.0
LABJACK_SAMPLE_RATE_HZ = 360.0

# Maximum ATI sample age while waiting for matching Polhemus TF, in milliseconds.
# Shorter TF waits are retried silently; expired samples are dropped with a warning.
TF_WAIT_TOLERANCE_MS = 500.0

# Cancel an unfinished rotation cycle when the sum of all three bias-corrected
# finger-force magnitudes is at or below this value in N. Keep the original neutral.
RELEASE_FORCE_THRESHOLD_N = 0.5

# Positive magnitude limits shared by all advisor bars and waveforms.
# Bars show 0-FORCE_LIMIT_N; waveforms share the signed +/-FORCE_LIMIT_N axis.
# Torque is displayed as torque * FORCE_LIMIT_N / TORQUE_LIMIT_NMM on that axis.
# Bar red indicates positive and blue indicates negative values.
# Force is N; torque is N*mm. Each limit must be a finite positive number.
FORCE_LIMIT_N = 10.0
TORQUE_LIMIT_NMM = 100.0

# Practice targets and upper bounds for calibrated trial targets, in degrees.
# Keys are object diameters in millimetres. Measured targets use the smaller of
# both measured directional limits and the configured bound for that diameter.
PRACTICE_TARGETS_DEG = {50: 20, 80: 10}

# v_sensor4 = SENSOR4_FROM_OBJECT @ v_object.
# 0.8660254037844386 is sqrt(3) / 2; keep matrix entries as numeric literals.
SENSOR4_FROM_OBJECT = [
    [-0.5, 0.8660254037844386, 0.0],
    [0.8660254037844386, 0.5, 0.0],
    [0.0, 0.0, -1.0],
]
