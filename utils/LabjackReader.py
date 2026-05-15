import numpy as np
import xml.etree.ElementTree as ET
import time
import csv
import json
import pandas as pd
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

    def __init__(self, cal_path, aq_rate=60, bias_time=30, bias_switch = True):
        # Guard: make failure explicit if LJM isn't installed/available
        if ljm is None:
            raise RuntimeError("labjack.ljm not available. Install LabJack LJM.")
        
        self.sensor_len = 6
        self.handle = ljm.openS("T7", "ANY", "ANY")
        self.calibration_matrix = self._read_cal(cal_path)

        # Configure six differential pairs: (0-1),(2-3),(4-5),(6-7),(8-9),(10-11)
        for ch, neg in zip(range(0, 12, 2), range(1, 12, 2)):
            ljm.eWriteName(self.handle, f"AIN{ch}_NEGATIVE_CH", neg)

        self.bias_save_path = f"sensor_bias/bias_csv/Labjack_Bias.csv"
        self.bias_json_path = f"sensor_bias/bias_json/Labjack_Bias.json"

        self.channels = [f"AIN{i}" for i in range(0, 12, 2)]  # AIN0,2,4,6,8,10
        self.num_channels = len(self.channels)
        # self.channel_address = ljm.namesToAddresses(self.num_channels, self.channels)[0]

        self.scan_rate = int(aq_rate)  # Hz
        # print(f"Configuring LabJack stream with {self.num_channels} channels at {self.scan_rate} Hz...")

        # self.scans_per_read = 1
        # try:
        #     ljm.eStreamStop(self.handle)
        # except ljm.LJMError as e:
        #     if e.errorCode != 2620:
        #         raise 
        # actual_rate = ljm.eStreamStart(self.handle, self.scans_per_read, self.num_channels, self.channel_address, self.scan_rate)
        # print(f"Stream started at {actual_rate:.2f} Hz\n")

        self.labjack_bias = np.zeros(self.sensor_len,)  # Placeholder for bias; can be set by compute_bias()
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
        # data = ljm.eStreamRead(self.handle)
        data = ljm.eReadNames(self.handle, self.num_channels, self.channels)
        # Read the last scan of voltages for the configured channels
        # voltages = np.array(data[0]).reshape((self.num_channels, self.scans_per_read))
        voltages = np.array(data).reshape((self.num_channels, 1))
        # print(f"Latest voltages: {latest_voltages.shape}")

        # Calculate forces and torques
        forces_torques = np.dot(self.calibration_matrix, voltages)

        # Assign forces and torques
        FT_lis = forces_torques.flatten()
            
        if abs(FT_lis[0]) >= 24 or abs(FT_lis[1]) >= 24 or abs(FT_lis[2]) >= 24:
            print("Warning: Force approaching Limit!")
        elif abs(FT_lis[3]) >= 240 or abs(FT_lis[4]) >= 240 or abs(FT_lis[5]) >= 240:
            print("Warning: Torque approaching Limit!")
            
        FT_biased = FT_lis - self.labjack_bias
        return FT_biased
    
    def compute_bias(self, time_period = 30):
        total_frames = int(self.scan_rate * time_period)
        print(f"Computing bias over {time_period} seconds ({total_frames} frames at {self.scan_rate} Hz)...")
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
            # time.sleep(1 / self.scan_rate)
        
        bias = pd.read_csv(self.bias_save_path, delimiter=',', header=None).mean().values
        self.labjack_bias = np.array(bias).reshape(self.sensor_len,)
        bias_json = {"Labjack": bias.tolist()}
        with open(self.bias_json_path, 'w') as json_file:
            json.dump(bias_json, json_file, indent=4)

    def close(self):
        """Release LabJack resources."""
        try:
            ljm.close(self.handle)
        except Exception:
            pass

if __name__ == "__main__":
    lj = LabjackATIReader("calibration_files/FT44297.cal", aq_rate=60, bias_time=5)
    try:
        while True:
            FT_lis = lj.read()
            print(f"Labjack Readings: Fx: {FT_lis[0]:.2f}, Fy: {FT_lis[1]:.2f}, Fz: {FT_lis[2]:.2f}, Tx: {FT_lis[3]:.2f}, Ty: {FT_lis[4]:.2f}, Tz: {FT_lis[5]:.2f}")
            time.sleep(1 / (lj.scan_rate*2))
    except KeyboardInterrupt:
        lj.close()