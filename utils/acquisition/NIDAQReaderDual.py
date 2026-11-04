from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.paths import BIAS_DIR, CALIBRATION_DIR
from utils.acquisition.bias import collect_bias, save_bias

import math
import time
import numpy as np
import xml.etree.ElementTree as ET
try:
    import nidaqmx
    from nidaqmx.constants import TerminalConfiguration, AcquisitionType
except Exception:
    nidaqmx = None

class NIDAQReaderDual:
    """
    Read two ATI sensors via NI-DAQ as a single 12-channel vector:
      Sensor1: [Fx,Fy,Fz,Tx,Ty,Tz] indices 0..5
      Sensor2: [Fx,Fy,Fz,Tx,Ty,Tz] indices 6..11
    """

    def __init__(self, cal1_path, cal2_path, aq_rate=60, phys="Dev1/ai0:7,Dev1/ai16:19", bias_time=30, bias_switch=True,
                 *, bias_warmup=0.5, bias_timeout=None):
        # Guard: make failure explicit if NI-DAQmx isn't installed/available
        if nidaqmx is None:
            raise RuntimeError("nidaqmx not available. Install NI-DAQmx driver + Python package.")
        self.aq_rate = float(aq_rate)
        if not math.isfinite(self.aq_rate) or self.aq_rate <= 0:
            raise ValueError("aq_rate must be finite and greater than zero")
        if bias_switch:
            self._bias_parameters(bias_time, bias_warmup, bias_timeout)

        self.sensor_len = 12

        # Parse ATI .cal files (each → 6x6 matrix)
        self.cal1 = self._read_cal(cal1_path)
        self.cal2 = self._read_cal(cal2_path)

        # Build 12x12 block diagonal calibration matrix
        zero6 = np.zeros((6, 6))
        self.matrix = np.vstack((
            np.hstack((self.cal1, zero6)),
            np.hstack((zero6, self.cal2))
        ))
        if not np.isfinite(self.matrix).all():
            raise ValueError("Calibration matrix must contain only finite values")
        
        self.bias_save_path = BIAS_DIR / "bias_csv" / "NIDAQ_Bias.csv"
        self.bias_json_path = BIAS_DIR / "bias_json" / "NIDAQ_Bias.json"

        # Configure an NI Task with differential channels
        self.task = None
        self.nidaq_bias = np.zeros(self.sensor_len)
        try:
            self.task = nidaqmx.Task()
            phys_clean = phys.replace(", ", ",")  # NI prefers no spaces
            self.task.ai_channels.add_ai_voltage_chan(
                physical_channel=phys_clean,
                terminal_config=TerminalConfiguration.DIFF,
                min_val=-10.0, max_val=10.0
            )
            self.task.timing.cfg_samp_clk_timing(
                rate=self.aq_rate,
                sample_mode=AcquisitionType.CONTINUOUS
            )
            self.task.start()
            # NI hardware can coerce the requested rate to a supported clock.
            self.aq_rate = float(self.task.timing.samp_clk_rate)
            if not math.isfinite(self.aq_rate) or self.aq_rate <= 0:
                raise ValueError("NI sample clock rate must be finite and greater than zero")
            if bias_switch:
                print("Computing bias on initialization...")
                self.compute_bias(time_period=bias_time, warmup_sec=bias_warmup,
                                  timeout_sec=bias_timeout)
        except BaseException:
            # Includes Ctrl+C while waiting for the unloaded bias window.
            try:
                self.close()
            except BaseException:
                pass  # Preserve the original setup/acquisition exception.
            raise

    def _read_cal(self, path):
        """Parse an ATI .cal XML file and return a robust 6x6 matrix in Fx,Fy,Fz,Tx,Ty,Tz row order."""
        tree = ET.parse(path)
        root = tree.getroot()
        order = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
        axes = {e.get('Name'): e for e in root.findall('.//UserAxis')}
        rows = []
        for n in order:
            if n not in axes:
                raise KeyError(f'UserAxis "{n}" not found in {path}')
            vals = [float(v) for v in axes[n].get('values').split()]
            if len(vals) != 6:
                raise ValueError(f'Axis {n} has {len(vals)} values (expect 6)')
            rows.append(vals)
        return np.array(rows).reshape(6, 6)
        
    def _read_calibrated_batch(self):
        """Drain available scans, calibrated but without subtracting any bias."""
        samples_available = self.task.in_stream.avail_samp_per_chan
        if samples_available <= 0:
            return None
        # Explicit sample count returns (channels, samples), even for one scan.
        # timeout=0 keeps no-data handling under the bias collector's deadline.
        raw_data = self.task.read(number_of_samples_per_channel=samples_available, timeout=0.0)
        volts = np.asarray(raw_data, dtype=float)
        if volts.shape != (self.sensor_len, samples_available):
            raise ValueError(f"Expected {self.sensor_len} NI channels and {samples_available} scans; got {volts.shape}")
        return volts.T @ self.matrix.T

    def read(self):
        """
        Get one calibrated 12-element vector for the two NI sensors with bias subtracted.
        Returns: np.ndarray shape (12,), or False if no scan is available.
        """
        batch = self._read_calibrated_batch()
        if batch is not None:
            # Live reads keep the newest scan, while bias uses the entire batch.
            FT_arr = batch[-1]
            if abs(FT_arr[0]) >= 24 or abs(FT_arr[1]) >= 24 or abs(FT_arr[2]) >= 24:
                print("Warning: Sensor 1 Force approaching Limit!")
            elif abs(FT_arr[3]) >= 240 or abs(FT_arr[4]) >= 240 or abs(FT_arr[5]) >= 240:
                print("Warning: Sensor 1 Torque approaching Limit!")

            if abs(FT_arr[6]) >= 45 or abs(FT_arr[7]) >= 45 or abs(FT_arr[8]) >= 265:
                print("Warning: Sensor 2 Force approaching Limit!")
            elif abs(FT_arr[9]) >= 450 or abs(FT_arr[10]) >= 450 or abs(FT_arr[11]) >= 450:
                print("Warning: Sensor 2 Torque approaching Limit!")

            FT_biased = FT_arr - self.nidaq_bias
            return FT_biased
        else:
            return False
        
    def _bias_parameters(self, time_period, warmup_sec, timeout_sec):
        duration = float(time_period)
        warmup = float(warmup_sec)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("time_period must be finite and greater than zero")
        if not math.isfinite(warmup) or warmup < 0:
            raise ValueError("warmup_sec must be finite and nonnegative")
        timeout = (warmup + duration + max(5.0, duration)
                   if timeout_sec is None else float(timeout_sec))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout_sec must be finite and greater than zero")
        return duration, warmup, timeout

    def compute_bias(self, time_period=30, *, warmup_sec=0.5, timeout_sec=None):
        """Average a fresh unloaded window; retain the old bias if acquisition/save fails."""
        duration, warmup, timeout = self._bias_parameters(time_period, warmup_sec, timeout_sec)
        total_frames = math.ceil(self.aq_rate * duration)
        warmup_frames = math.ceil(self.aq_rate * warmup)
        print(f"Computing bias over {duration:g} seconds ({total_frames} scans at {self.aq_rate:g} Hz); "
              f"keep both sensors unloaded (warmup {warmup:g} s).")
        # Samples acquired before this request must never contribute to a new zero.
        self._read_calibrated_batch()
        started = time.monotonic()
        samples = collect_bias(self._read_calibrated_batch, total_frames, self.sensor_len,
                               warmup_samples=warmup_frames, timeout_sec=timeout)
        bias = samples.mean(axis=0)
        bias_json = {
            "NIDAQ_S1": bias[:6].tolist(), "NIDAQ_S2": bias[6:].tolist(),
            "_metadata": {
                "sample_count": total_frames,
                "sample_rate_hz": self.aq_rate,
                "warmup_sec": warmup,
                "elapsed_sec": time.monotonic() - started,
            },
        }
        save_bias(self.bias_save_path, self.bias_json_path, samples, bias_json)
        self.nidaq_bias = bias
        return bias.copy()

    def close(self):
        """Release NI resources once, including after partial initialization."""
        task = getattr(self, "task", None)
        if task is None:
            return
        self.task = None
        task.close()

if __name__ == "__main__":
    ni = NIDAQReaderDual(CALIBRATION_DIR / "FT44298.cal", CALIBRATION_DIR / "FT45281.cal", aq_rate=60, bias_time=5,)
    itr = 0
    try:
        while True:
            FT_lis = ni.read()
            if itr % 5 == 0:
                print(f"Sensor 1:\nFx: {FT_lis[0]:.2f}, Fy: {FT_lis[1]:.2f}, Fz: {FT_lis[2]:.2f},\nTx: {FT_lis[3]:.2f}, Ty: {FT_lis[4]:.2f}, Tz: {FT_lis[5]:.2f}\nSensor 2:\nFx: {FT_lis[6]:.2f}, Fy: {FT_lis[7]:.2f}, Fz: {FT_lis[8]:.2f},\nTx: {FT_lis[9]:.2f}, Ty: {FT_lis[10]:.2f}, Tz: {FT_lis[11]:.2f}\n")
            itr += 1
            time.sleep(1 / ni.aq_rate)
    except KeyboardInterrupt:
        ni.close()
