import sys
import numpy as np
import xml.etree.ElementTree as ET
try:
    from labjack import ljm
except Exception:
    ljm = None

# ─────────────────────────────────────────────────────────────────────────────
# LabJack Reader: returns 6 calibrated channels for one ATI sensor
# ─────────────────────────────────────────────────────────────────────────────
class LabjackATIReader:
    """
    Read one ATI sensor through a LabJack T7 as a calibrated, uncorrected vector:
      [Fx,Fy,Fz,Tx,Ty,Tz]

    Taring and coordinate transforms belong to the external processing node.
    """

    def __init__(self, cal_path, aq_rate=60):
        # Guard: make failure explicit if LJM isn't installed/available
        if ljm is None:
            raise RuntimeError("labjack.ljm not available. Install LabJack LJM.")
        
        self.sensor_len = 6
        self.handle = None
        self.stream_start = False
        self._requested_scan_rate = self._positive_number(aq_rate, "aq_rate")
        try:
            self.handle = ljm.openS("T7", "ANY", "ANY")
            self._configure(cal_path)
        except BaseException:
            # Ctrl+C during device setup must release the handle too.
            try:
                self.close()
            except BaseException as exc:
                print(f"LabJack cleanup failed during initialization: {exc}", file=sys.stderr)
            raise

    def _configure(self, cal_path):
        self.calibration_matrix = self._read_cal(cal_path)
        if not np.isfinite(self.calibration_matrix).all():
            raise ValueError("Calibration matrix must contain only finite values")

        # Configure six differential pairs: (0-1),(2-3),(4-5),(6-7),(8-9),(10-11)
        for ch, neg in zip(range(0, 12, 2), range(1, 12, 2)):
            ljm.eWriteName(self.handle, f"AIN{ch}_NEGATIVE_CH", neg)

        self.channels = [f"AIN{i}" for i in range(0, 12, 2)]  # AIN0,2,4,6,8,10
        self.num_channels = len(self.channels)
        self.scan_rate = self._requested_scan_rate
        self.channel_address = ljm.namesToAddresses(self.num_channels, self.channels)[0]
        self.scans_per_read = 1
        self._start_stream()

    @staticmethod
    def _positive_number(value, name):
        value = float(value)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _start_stream(self):
        """Start acquisition once with finite LJM timeout and blocking reads."""
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
        """Reject invalid voltages and apply only the factory ATI calibration."""
        scans = np.asarray(voltages, dtype=float).reshape(-1, self.num_channels)
        if not len(scans):
            return None
        # Keep only the newest scan so a skipped latest measurement never
        # causes an older scan to be published with a fresh timestamp.
        voltages = scans[-1]
        # -9999 is LJM's skipped-sample marker, not a physical voltage. Reject
        # its entire scan before matrix multiplication can mix channels.
        if not np.isfinite(voltages).all() or np.any(voltages == -9999):
            return None
        with np.errstate(over="ignore", invalid="ignore"):
            sample = self.calibration_matrix @ voltages
        return sample if np.isfinite(sample).all() else None

    def stream_read(self):
        """Return the latest valid uncorrected 6-vector, or None for a skipped scan."""
        data = ljm.eStreamRead(self.handle)
        # LJM returns scan-interleaved channels: [scan0_ch0..5, scan1_ch0..5].
        return self._calibrate_voltages(data[0])

    def straight_read(self):
        """Read one command-response scan with the same factory calibration."""
        return self._calibrate_voltages(
            ljm.eReadNames(self.handle, self.num_channels, self.channels)
        )

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
