from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.paths import BIAS_DIR, CALIBRATION_DIR
from utils.acquisition.bias import collect_bias, save_bias

import numpy as np
import xml.etree.ElementTree as ET
import time
try:
    from labjack import ljm
except Exception:
    ljm = None

# ─────────────────────────────────────────────────────────────────────────────
# LabJack Reader: returns 6 calibrated channels for one ATI sensor
# ─────────────────────────────────────────────────────────────────────────────
class LabjackATIReader:
    """
    Read one ATI sensor through a LabJack T7 as a 6-element vector:
      [Fx,Fy,Fz,Tx,Ty,Tz]
    """

    def __init__(self, cal_path, aq_rate=60, bias_frames=30, bias_switch=True,
                 *, bias_time=None, bias_warmup=0.5, bias_timeout=None):
        # Guard: make failure explicit if LJM isn't installed/available
        if ljm is None:
            raise RuntimeError("labjack.ljm not available. Install LabJack LJM.")
        
        self.sensor_len = 6
        self.handle = None
        self.stream_start = False
        self._initializing_bias = False
        try:
            self.handle = ljm.openS("T7", "ANY", "ANY")
            self._configure(
                cal_path, aq_rate, bias_frames, bias_switch,
                bias_time, bias_warmup, bias_timeout,
            )
        except BaseException:
            # Ctrl+C during bias acquisition must release the handle too.
            try:
                self.close()
            except Exception as exc:
                print(f"LabJack cleanup failed during initialization: {exc}", file=sys.stderr)
            raise

    def _configure(self, cal_path, aq_rate, bias_frames, bias_switch,
                   bias_time, bias_warmup, bias_timeout):
        self.calibration_matrix = self._read_cal(cal_path)

        # Configure six differential pairs: (0-1),(2-3),(4-5),(6-7),(8-9),(10-11)
        for ch, neg in zip(range(0, 12, 2), range(1, 12, 2)):
            ljm.eWriteName(self.handle, f"AIN{ch}_NEGATIVE_CH", neg)

        self.bias_save_path = BIAS_DIR / "bias_csv" / "Labjack_Bias.csv"
        self.bias_json_path = BIAS_DIR / "bias_json" / "Labjack_Bias.json"

        self.channels = [f"AIN{i}" for i in range(0, 12, 2)]  # AIN0,2,4,6,8,10
        self.num_channels = len(self.channels)
        self._requested_scan_rate = self._positive_number(aq_rate, "aq_rate")
        self.scan_rate = self._requested_scan_rate
        self.labjack_bias = np.zeros(self.sensor_len)
        self.channel_address = ljm.namesToAddresses(self.num_channels, self.channels)[0]
        self.scans_per_read = 1
        self._start_stream()
        if bias_switch:
            self._initializing_bias = True
            try:
                self.compute_bias(
                    total_frames=bias_frames, time_period=bias_time,
                    warmup_sec=bias_warmup, timeout_sec=bias_timeout,
                )
            finally:
                self._initializing_bias = False

    @staticmethod
    def _positive_number(value, name):
        value = float(value)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _start_stream(self):
        """Start a fresh FIFO with finite LJM timeout and normal blocking reads."""
        try:
            ljm.eStreamStop(self.handle)
        except ljm.LJMError as e:
            if e.errorCode != 2620:
                raise
        self.stream_start = False
        # These LJM settings apply at stream start. Restore finite, calculated
        # timeouts even if an earlier user of LJM selected an infinite timeout.
        # ALL=1 retains blocking reads for the publisher (ALL_OR_NONE=2 does not).
        ljm.writeLibraryConfigS("LJM_STREAM_RECEIVE_TIMEOUT_MODE", 1)
        ljm.writeLibraryConfigS("LJM_STREAM_SCANS_RETURN", 1)
        actual_rate = ljm.eStreamStart(
            self.handle, self.scans_per_read, self.num_channels,
            self.channel_address, self._requested_scan_rate,
        )
        self.stream_start = True
        self.scan_rate = self._positive_number(actual_rate, "actual stream scan rate")
        print(f"Stream started at {self.scan_rate:.2f} Hz\n")

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
    
    def _calibrate_voltages(self, voltages):
        """Return valid calibrated scans without subtracting the current bias."""
        voltages = np.asarray(voltages, dtype=float).reshape(-1, self.num_channels)
        # -9999 is LJM's skipped-sample marker, not a physical voltage. Reject
        # its entire scan before matrix multiplication can mix channels.
        valid = np.isfinite(voltages).all(axis=1) & (voltages != -9999).all(axis=1)
        with np.errstate(over="ignore", invalid="ignore"):
            samples = voltages[valid] @ self.calibration_matrix.T
        return samples[np.isfinite(samples).all(axis=1)]

    def _read_unbiased_stream(self):
        data = ljm.eStreamRead(self.handle)
        # LJM returns scan-interleaved channels: [scan0_ch0..5, scan1_ch0..5].
        return self._calibrate_voltages(data[0])

    @staticmethod
    def _warn_limits(values):
        if np.any(np.abs(values[:3]) >= 24):
            print("Warning: Force approaching Limit!")
        elif np.any(np.abs(values[3:]) >= 240):
            print("Warning: Torque approaching Limit!")

    def stream_read(self):
        """Return the latest valid corrected 6-vector, or None for a skipped scan."""
        samples = self._read_unbiased_stream()
        if not len(samples):
            return None
        values = samples[-1]
        self._warn_limits(values)
        return values - self.labjack_bias

    def straight_read(self):
        """Read one command-response scan with the same calibration and correction."""
        samples = self._calibrate_voltages(
            ljm.eReadNames(self.handle, self.num_channels, self.channels)
        )
        if not len(samples):
            return None
        values = samples[-1]
        self._warn_limits(values)
        return values - self.labjack_bias

    def compute_bias(self, total_frames=30, *, time_period=None, warmup_sec=0.5,
                     timeout_sec=None):
        """Replace bias using fresh, uncorrected streaming samples.

        ``time_period`` selects seconds instead of the legacy frame count.
        Invalid/skipped scans do not count, while physical zero scans do. The
        acquisition deadline uses monotonic time; an in-flight LJM read may
        extend it by the driver's finite, calculated receive timeout.
        """
        if self.handle is None or not self.stream_start:
            raise RuntimeError("LabJack stream is not running")
        warmup_sec = float(warmup_sec)
        if not np.isfinite(warmup_sec) or warmup_sec < 0:
            raise ValueError("warmup_sec must be finite and nonnegative")
        if time_period is not None:
            time_period = self._positive_number(time_period, "time_period")
        else:
            total_frames = self._positive_number(total_frames, "total_frames")
            if not total_frames.is_integer():
                raise ValueError("total_frames must be a whole number")
        if timeout_sec is not None:
            timeout_sec = self._positive_number(timeout_sec, "timeout_sec")

        # Restarting also clears old loaded scans queued before a repeated tare.
        if not self._initializing_bias:
            self._start_stream()
        sample_count = (int(np.ceil(self.scan_rate * time_period))
                        if time_period is not None else int(total_frames))
        warmup_samples = int(np.ceil(self.scan_rate * warmup_sec))
        if timeout_sec is None:
            timeout_sec = max(5.0, 2 * (sample_count + warmup_samples) / self.scan_rate + 1)
        print(
            "Computing LabJack bias: keep sensor unloaded and still "
            f"({sample_count} samples at {self.scan_rate:.2f} Hz, "
            f"{warmup_sec:g} s warm-up)."
        )
        started = time.monotonic()
        samples = collect_bias(
            self._read_unbiased_stream, sample_count, self.sensor_len,
            warmup_samples=warmup_samples, timeout_sec=timeout_sec,
        )
        bias = samples.mean(axis=0)
        save_bias(self.bias_save_path, self.bias_json_path, samples, {
            "Labjack": bias.tolist(),
            "_metadata": {
                "sample_count": sample_count,
                "sample_rate_hz": self.scan_rate,
                "warmup_sec": warmup_sec,
                "elapsed_sec": time.monotonic() - started,
            },
        })
        # Commit the valid saved calibration before resuming acquisition, so
        # memory and disk agree even if restarting the device fails.
        self.labjack_bias = bias
        # Disk writes can queue stream samples. Resume from a fresh stream so
        # publishing doesn't send old data with a new ROS timestamp.
        self._start_stream()
        return bias.copy()

    def close(self):
        """Stop an active stream and release the device, including partial startup."""
        handle = getattr(self, "handle", None)
        if handle is None:
            return
        self.handle = None
        errors = []
        try:
            if getattr(self, "stream_start", False):
                ljm.eStreamStop(handle)
        except Exception as exc:
            # A stream already stopped by the driver requires no further action.
            if getattr(exc, "errorCode", None) != 2620:
                errors.append(exc)
        finally:
            self.stream_start = False
            try:
                ljm.close(handle)
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("LabJack cleanup failed: " + "; ".join(str(exc) for exc in errors)) from errors[0]

if __name__ == "__main__":
    lj = None
    try:
        lj = LabjackATIReader(CALIBRATION_DIR / "FT44297.cal", aq_rate=60, bias_frames=5)
        while True:
            FT_lis = lj.stream_read()
            if FT_lis is not None:
                print(f"Labjack Readings: Fx: {FT_lis[0]:.2f}, Fy: {FT_lis[1]:.2f}, Fz: {FT_lis[2]:.2f}, Tx: {FT_lis[3]:.2f}, Ty: {FT_lis[4]:.2f}, Tz: {FT_lis[5]:.2f}")
            time.sleep(1 / (lj.scan_rate*2))
    except KeyboardInterrupt:
        pass
    finally:
        if lj is not None:
            lj.close()
