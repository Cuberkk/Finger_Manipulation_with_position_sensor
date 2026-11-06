"""Repository paths shared by runtime tools, independent of the working directory."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
GUI_CONFIG_FILE = PROJECT_ROOT / "docs" / "user_study_docs" / "gui_config.py"
DATA_DIR = PROJECT_ROOT / "data"
STUDY_DATA_DIR = PROJECT_ROOT / "User_Study_Data"
CALIBRATION_DIR = PROJECT_ROOT / "utils" / "config" / "calibration_files"
BIAS_DIR = PROJECT_ROOT / "utils" / "sensor_bias"
