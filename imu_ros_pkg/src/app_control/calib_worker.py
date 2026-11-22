import sys
from pathlib import Path
import threading
from queue import Queue, Empty, Full
from enum import Enum


sys.path.insert(0, str(Path(__file__).resolve().parents[1])) 

from base_dataclasses import ReadImuScaledMeasResponse, AcknowledgeResponse, Transport
from src.measlog.imu_csv_logger import ImuLogger
from protocol_codes import MessageCode, ErrorCode
from src.calibration.calibrate import MagnetometerCalibrator


class CalibrationHandlerThread(threading.Thread):
    def __init__(self, scaled_msg_queue: Queue, stop_event: threading.Event):
        super().__init__(name="CalibrationHandler", daemon=True)
        self._stop_event = stop_event

        self._calib_obj = MagnetometerCalibrator()
        self._data = None

        header = ["Time_us",
                                "A_X", "A_Y", "A_Z", 
                                "G_X", "G_Y", "G_Z", 
                                "M_X", "M_Y", "M_Z"]
        
        header_formatted = [f'{header[0]:>22}'] + [f"{data:>9}" for data in header[1:]]
        self.scaled_meas_logger = ImuLogger(header = header_formatted)
        self.scaled_msg_queue: Queue = scaled_msg_queue


    def run(self):
        try:
            self.gather_measurements()
            self.process_measurements()
        except Exception as e:
            self.error = e
        finally:
            self.scaled_meas_logger.flush_buffer()

    def gather_measurements(self):
         while not self._stop_event.is_set():
            try:
                data: ReadImuScaledMeasResponse = self.scaled_msg_queue.get(timeout=0.1)
                self.scaled_meas_logger.save_scaled_data(data)
            except Empty:
                print(f"{self.name}: Scaled msg queue queue is empty for too long") # log warn/info"

    def process_measurements(self):
        # elipsoid calibration logic with MagnetometerCalibrator()
        # self._calib_obj.save_calibration(filename)
        self._data = b'\x00' # self._calib_obj.get_params() # TODO: temp mock
        

    def get_serialized_params(self):
        if self._data is None:
            filename = "my_default_params.json"
            self._calib_obj.load_calibration(filename)
            return b'\x00' # TODO: temp mock
                 
        return self._data

