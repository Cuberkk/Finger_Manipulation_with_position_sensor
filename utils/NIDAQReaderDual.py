import csv
import json
import pandas as pd
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

    def __init__(self, cal1_path, cal2_path, aq_rate=60, phys="Dev1/ai0:7,Dev1/ai16:19",bias_time=30, bias_switch = True):
        # Guard: make failure explicit if NI-DAQmx isn't installed/available
        if nidaqmx is None:
            raise RuntimeError("nidaqmx not available. Install NI-DAQmx driver + Python package.")
        self.aq_rate = int(aq_rate)

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
        
        self.bias_save_path = f"sensor_bias/bias_csv/NIDAQ_Bias.csv"
        self.bias_json_path = f"sensor_bias/bias_json/NIDAQ_Bias.json"

        # Configure an NI Task with differential channels
        self.task = nidaqmx.Task()
        phys_clean = phys.replace(", ", ",")  # NI prefers no spaces
        self.task.ai_channels.add_ai_voltage_chan(
            physical_channel=phys_clean,
            terminal_config=TerminalConfiguration.DIFF,
            min_val=-10.0, max_val=10.0 # Min and max stands for the voltage range expected from the sensors.
        )

        self.task.timing.cfg_samp_clk_timing(
            rate=self.aq_rate,
            sample_mode=AcquisitionType.CONTINUOUS
        )

        self.task.start()

        self.nidaq_bias = np.zeros(self.sensor_len,)  # Placeholder for bias; can be set by compute_bias()
        if bias_switch:
            print("Computing bias on initialization...")
            self.compute_bias(time_period=bias_time)  # Automatically compute bias on init; adjust time_period as needed
        else:
            pass

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
        """
        Get one calibrated 12-element vector for the two NI sensors with bias subtracted.
        Returns: np.ndarray shape (12,)
        """
        samples_available = self.task.in_stream.avail_samp_per_chan
        # print(f"Samples available: {samples_available}")
        if samples_available > 0:
            # Discard old samples to minimize latency; keep only the most recent
            raw_data = self.task.read(number_of_samples_per_channel=samples_available)
            volts = np.array(raw_data)[:, -1].reshape(self.sensor_len,)
            FT_arr = self.matrix.dot(volts).flatten()
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
            print("No samples available, returning zeros.")
            return np.zeros(self.sensor_len,)
        
    def compute_bias(self, time_period = 30):
        total_frames = int(self.aq_rate * time_period)
        print(f"Computing bias over {time_period} seconds ({total_frames} frames at {self.aq_rate} Hz)...")
        itr = 0
        while itr < total_frames:
            FT_arr = self.read()
            if np.all(FT_arr == 0):
                print("No samples available during bias computation, skipping this frame.")
                continue
            mode = 'w' if itr == 0 else 'a'
            with open(self.bias_save_path, mode, newline='') as csvfile:
                write = csv.writer(csvfile)
                write.writerow(FT_arr)
            itr += 1
            time.sleep(1 / self.aq_rate)
        
        bias = pd.read_csv(self.bias_save_path, delimiter=',', header=None).mean().values
        self.nidaq_bias = np.array(bias).reshape(self.sensor_len,)
        bias_json = {"NIDAQ_S1": bias[0:6].tolist(),
                     "NIDAQ_S2": bias[6:12].tolist()}
        with open(self.bias_json_path, 'w') as json_file:
            json.dump(bias_json, json_file, indent=4)

    def close(self):
        """Release NI resources."""
        try:
            self.task.close()
        except Exception:
            pass

if __name__ == "__main__":
    ni = NIDAQReaderDual("calibration_files/FT44298.cal", "calibration_files/FT45281.cal", aq_rate=60, bias_time=5,)
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