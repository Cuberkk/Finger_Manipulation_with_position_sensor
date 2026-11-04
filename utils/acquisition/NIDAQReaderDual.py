import math
import numpy as np
import xml.etree.ElementTree as ET
try:
    import nidaqmx
    from nidaqmx.constants import TerminalConfiguration, AcquisitionType
except Exception:
    nidaqmx = None

class NIDAQReaderDual:
    """
    Read two ATI sensors via NI-DAQ as a calibrated, uncorrected 12-channel vector:
      Sensor1: [Fx,Fy,Fz,Tx,Ty,Tz] indices 0..5
      Sensor2: [Fx,Fy,Fz,Tx,Ty,Tz] indices 6..11

    Taring and coordinate transforms belong to the external processing node.
    """

    def __init__(self, cal1_path, cal2_path, aq_rate=60, phys="Dev1/ai0:7,Dev1/ai16:19"):
        # Guard: make failure explicit if NI-DAQmx isn't installed/available
        if nidaqmx is None:
            raise RuntimeError("nidaqmx not available. Install NI-DAQmx driver + Python package.")
        self.aq_rate = float(aq_rate)
        if not math.isfinite(self.aq_rate) or self.aq_rate <= 0:
            raise ValueError("aq_rate must be finite and greater than zero")
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
        
        # Configure an NI Task with differential channels
        self.task = None
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
        except BaseException:
            # Release the task even when Ctrl+C interrupts device setup.
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
        
    def read(self):
        """Return the latest calibrated, uncorrected vector, or False if unavailable."""
        samples_available = self.task.in_stream.avail_samp_per_chan
        if samples_available <= 0:
            return False
        # Explicit sample count returns (channels, samples), even for one scan.
        # These samples are already buffered, so acquisition need not block.
        raw_data = self.task.read(number_of_samples_per_channel=samples_available, timeout=0.0)
        volts = np.asarray(raw_data, dtype=float)
        if volts.shape != (self.sensor_len, samples_available):
            raise ValueError(f"Expected {self.sensor_len} NI channels and {samples_available} scans; got {volts.shape}")
        # Publish the newest acquisition only. An invalid latest scan must not
        # cause an older buffered scan to be stamped as a fresh measurement.
        latest = volts[:, -1]
        if not np.isfinite(latest).all():
            return False
        with np.errstate(over="ignore", invalid="ignore"):
            calibrated = self.matrix @ latest
        return calibrated if np.isfinite(calibrated).all() else False

    def close(self):
        """Release NI resources once, including after partial initialization."""
        task = getattr(self, "task", None)
        if task is None:
            return
        self.task = None
        task.close()
