"""
CSV logging utilities for IMU measurements.

Provides buffered writing of IMU measurements to CSV files and
automatic generation of timestamped log filenames.
"""

# import struct
# import serial
import datetime as dt
import csv
from pathlib import Path
from enum import Enum
# from typing import List, Dict, Tuple, Optional, Union

from utils.uartcom.base_dataclasses import *


class ImuLogger:
    """
    Buffered CSV logger for IMU measurements.

    Supports logging of scaled IMU measurements, Euler angles,
    and quaternion orientation data.
    """
    def __init__(self, *,filename = None, log_dir = None, header = None):
        if filename is None or log_dir is None:
            self.file_name = self._generate_filename()
        else:
            self.file_name = filename

        self.create_new_csv(self.file_name, header)
        self.max_buf_size = 100
        self.data_buffer: list = []
    
    def save_scaled_data(self, data: ReadImuScaledMeasResponse) -> None:
        record = [f"{data.timestamp:>22}", 
                    *[f"{m:>9.3f}" for m in data.accel_meas],
                    *[f"{m:>9.3f}" for m in data.gyro_meas],
                    *[f"{m:>9.3f}" for m in data.mag_meas]]

        self.data_buffer.append(record)
        
        if len(self.data_buffer) >= self.max_buf_size:
            self.flush_buffer()
            
            
    def save_angle_data(self, data: ReadImuEulerResponse) -> None:
        """Add an Euler-angle measurement to the output buffer."""
        record = [f"{data.timestamp:>22}", f"{data.roll:>9.3f}",
                    f"{data.pitch:>9.3f}",  f"{data.yaw:>9.3f}"]
        self.data_buffer.append(record)
        
        if len(self.data_buffer) >= self.max_buf_size:
            self.flush_buffer()
    
    def save_quaternions(self, data: QuaternionMsgResponse) -> None:
        """Add a quaternion orientation measurement to the output buffer."""
        record = [f"{data.timestamp:>22}", 
                  f"{data.w:>9.3f}",
                    f"{data.x:>9.3f}",  
                    f"{data.y:>9.3f}",
                    f"{data.z:>9.3f}"]
        self.data_buffer.append(record)
        
        if len(self.data_buffer) >= self.max_buf_size:
            self.flush_buffer()
            
            
    def flush_buffer(self) -> None:
        """Write all buffered records to the CSV file."""
        with open(self.file_name, mode = 'a', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.data_buffer)
        self.data_buffer = []
    
    @staticmethod
    def create_new_csv(fname, header):
        """
        Create or overwrite a CSV file and optionally write its header.
        """
        # header = ["time_ms",
        #                 "A_X", "A_Y", "A_Z", 
        #                 "G_X", "G_Y", "G_Z", 
        #                 "M_X", "M_Y", "M_Z"]
        with open(fname, mode = 'w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            if header is not None:
                #header_formatted = [f"{data:>9}" for data in header]
                writer.writerow(header)


    
    @staticmethod
    def _generate_filename():   
        """Generate a timestamped filename inside the log_data directory."""  
        timestamp = dt.datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = Path("log_data")
        log_dir.mkdir(exist_ok=True)
        return log_dir / f"imu_log_{timestamp}.csv"
    
        
# if __name__ == "__main__":
#     filename1 = Path("log_data")/"imu_log_2026-08-28_02-33-36_mag_only.csv"
#     filename2 = Path("log_data")/"imu_log_2026-08-28_02-33-36_mag_mdf.csv"
#     with open(filename1) as f_read, open(filename2, mode='w', newline='') as f_write:
#         reader = csv.reader(f_read)
#         writer = csv.writer(f_write, delimiter='\t',quotechar=' ')
#         for rec in reader:
#             # new_rec = ['(' + ','.join([data for data in rec]) + ')']
#             new_rec = [data for data in rec]
#             writer.writerow(new_rec)
            
    
#print(dt.datetime.today().strftime("%d:%m:%Y %H:%M:%S"))









    
