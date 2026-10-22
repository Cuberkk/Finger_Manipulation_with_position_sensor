"""Repository paths shared by runtime tools, independent of the working directory."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "data"
CALIBRATION_DIR = PROJECT_ROOT / "utils" / "config" / "calibration_files"
BIAS_DIR = DATA_DIR / "sensor_bias"
