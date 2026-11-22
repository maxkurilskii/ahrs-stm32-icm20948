import sys
from pathlib import Path
import threading
from queue import Queue, Empty, Full
from enum import Enum
import serial
import struct
import time
from typing import Union


sys.path.insert(0, str(Path(__file__).resolve().parents[1])) 

from base_dataclasses import    Transport, \
                                ParsedFrame, \
                                QuaternionMsgResponse, \
                                ReadImuScaledMeasResponse, \
                                AcknowledgeResponse

# from rx_workers import RxMsgHandlerThread
# from tx_worker import TransmitHandler
# from calib_worker import CalibrationHandler

from src.uartcom.serial_worker import UartCommunicator, TransportReadTimeOutError
from src.measlog.imu_csv_logger import ImuLogger
from protocol_codes import MessageCode, ErrorCode
from src.calibration.calibrate import MagnetometerCalibrator
from keyboard_cmd_source import ControlCommmands


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


class RxMsgHandlerThread(threading.Thread):
    def __init__(self, 
                transport: Transport,
                ack_queue: Queue,
                scaled_meas_queue: Queue,
                stop_event: threading.Event):

        super().__init__(name="RxMsgHandler", daemon=True)

        self.transport = transport
        self.ack_queue = ack_queue
        self.scaled_meas_queue = scaled_meas_queue
        
        time_str = 'Time_us'
        quat_header = [f"{time_str:>22}"] + [f"{data:>9}" for data in ["W", "X", "Y", "Z"]]
        self.quat_meas_logger = ImuLogger(header = quat_header)
       
        self._stop_event = stop_event
        # self.ack_msg_buffer = ack_msg_buffer
    
    def run(self):
        
        while not self._stop_event.is_set():
            try:
                msg: bytes = self.transport.read() 
                frame = ParsedFrame(MessageCode(msg[1]), msg[3:-2])

                if frame.msg_code == MessageCode.QUAT_MEAS:
                    # LOG QUAT measurements to special file
                    # TODO: change to memoryview
                    w, x, y, z, timestamp, _ = struct.unpack_from('<4f1Q1H', frame.payload, offset = 0)
                    self.quat_meas_logger.save_quaternions(QuaternionMsgResponse(w, x, y, z, timestamp))

                elif frame.msg_code in [MessageCode.ACK, MessageCode.NACK]:
                    # self.ack_msg_buffer.append((msg_code, payload))
                    # self.event.set()
                    request_id = int.from_bytes(frame.payload[:2], 'little')
                    cmd_code_val = frame.payload[2]
                    error_code = frame.payload[3]
                    self.ack_queue.put_nowait(AcknowledgeResponse(MessageCode(frame.msg_code), 
                                                                request_id, 
                                                                cmd_code_val, 
                                                                error_code))

                elif frame.msg_code == MessageCode.IMU_SCALED_MEAS:
                    a_x, a_y, a_z,  \
                    g_x, g_y, g_z,  \
                    m_x, m_y, m_z,  \
                    timestamp, _ = struct.unpack_from('<9f1Q1H', frame.payload, offset=0)
                    self.scaled_meas_queue.put_nowait(ReadImuScaledMeasResponse((a_x, a_y, a_z),
                                                                                (g_x, g_y, g_z), 
                                                                                (m_x, m_y, m_z),
                                                                                timestamp))

            except serial.SerialException as e:
                print(f"UART error: {e}") # log critical
                break
            
            except TransportReadTimeOutError:
                print("STM read procedure exceeded read timeout") # log warn
                continue

            except Empty:
                print(f"{self.name}: Rx queue is empty for too long")
                continue

            except Full:
                print(f"{self.name}: Rx queue is full for too long") # log warn
                continue
            except (ValueError, struct.error, IndexError) as exc:
                print(f"Invalid incoming frame: {exc}")
                continue

        self.quat_meas_logger.flush_buffer()



class AppContorState(Enum):
    INIT                     = 0
    IDLE                     = 1
    QUAT_STREAM_ACTIVE       = 2
    QUAT_STREAM_PAUSED       = 3
    # CALIB_MEAS_ACQUISITION   = 4
    # CALIB_PARAM_ESTIMATION   = 5
    CALIBRATION              = 4
    REF_TIME_ESTABLISHING    = 5


class CommunicationState(Enum):
    NOT_EST           = 1
    ESTABLISHED       = 2
    SCALED_MEAS_RX    = 3
    CALIB_PARAM_TX    = 4
    QUAT_MEAS_RX      = 5



class AcknowledgeTimeoutError(Exception):
    pass

class ApplicationManager(threading.Thread):

    # ACK_MAX_TRY_NUMBER = 2

    def __init__(self,
                transport: Transport,
                cmd_queue: Queue,
                ack_queue: Queue,
                scaled_meas_queue: Queue,
                # event: threading.Event, 
                ack_timeout: Union[int, float],
                # ack_msg_buffer: list,
                calib_mode: str, # auto = use last calib params, manual = wait for calibrate cmd  
                stop_event: threading.Event):
        
        super().__init__(name="AppThread", daemon=True)

        self.transport = transport
        self.cmd_queue = cmd_queue
        self.ack_queue = ack_queue
        self.scaled_meas_queue = scaled_meas_queue
        self.ack_timeout = ack_timeout
        self.stop_event = stop_event 
        self.pending_requests = dict()

        # self.lock = threading.Lock()
        # self._ack_try_cntr = self.ACK_MAX_TRY_NUMBER

        self._is_dev_calibrated = False
        self._calibration_handler = None # CalibrationHandlerThread(scaled_meas_queue, self._calib_stop_event)
        
        self._app_state = AppContorState.INIT 
        self._comm_state = CommunicationState.NOT_EST
        self._request_id = 0
        self.ref_time_us = None

        # init_thread = threading.Thread(target=self.init_communication,
        #                 args=(calib_mode,),
        #                 name = "InitializationThread",
        #                 daemon=True)
        
        # init_thread.start()
        # init_thread.join()

        # if self._comm_state is CommunicationState.NOT_EST:
        #     print("Initialization wasn't performed") # log critical
        
    def check_pending_requests(self):
        for request_id, request in list(self.pending_requests.items()):
            if time.monotonic() >= request["deadline"]:
                print(f"Request {request_id} timeout")
                self.pending_requests.pop(request_id)
                # request["try_count"] -= 1
                # if request["try_count"] == 0:
                #     print("Max try count exceeded!") # log critical
                #     self.pending_requests.pop(request_id)


    
    def run(self):
        while not self.stop_event.is_set():
            # --------------------------------------------------------------
            # Check acknowledge messages (firstly) in non-blocking mode
            # --------------------------------------------------------------
            for _ in range(5):
                try:
                    ack_msg = self.ack_queue.get_nowait()
                except Empty:
                    break
                self.handle_responce(ack_msg)

            # --------------------------------------------------------------
            # Check command messages (keyboard now) with short block (0.05)
            # --------------------------------------------------------------

            try:
                cmd: ControlCommmands = self.cmd_queue.get(timeout = 0.05)
                if cmd == ControlCommmands.START_QUAT_MEAS:
                    if self._app_state == AppContorState.QUAT_STREAM_ACTIVE:
                        print("Quat stream is already active!")
                        continue
                    if not self._is_dev_calibrated:
                        print("Device not calibrated. Use device calib params!")
                    pkg = self._request_id.to_bytes(2, byteorder='little')
                    request = {"command": ControlCommmands.START_QUAT_MEAS,
                                "deadline": time.monotonic() + self.ack_timeout}
                    self.pending_requests[self._request_id] = request
                    self._request_id += 1
                    self.transport.write(MessageCode.START_QUAT_MEAS.value, pkg)   


                elif cmd == ControlCommmands.STOP_QUAT_MEAS:
                    if self._app_state != AppContorState.QUAT_STREAM_ACTIVE:
                        print("Quat stream is not active!") 
                        continue

                    pkg = self._request_id.to_bytes(2, byteorder='little')
                    request = {"command": ControlCommmands.STOP_QUAT_MEAS,
                                "deadline": time.monotonic() + self.ack_timeout}
                    self.pending_requests[self._request_id] = request
                    self._request_id += 1
                    self.transport.write(MessageCode.STOP_QUAT_MEAS.value, pkg)     

            except serial.SerialException as e:
                self.pending_requests.pop(self._request_id - 1, None)
                print(f"UART TX failed: {e}")

            # except Empty:
                # print(f"{self.name}: Cmd queue is empty for too long") #log debug

            # --------------------------------------------------------------
            # Check pending requests
            # --------------------------------------------------------------
            self.check_pending_requests()


    def handle_state(self, request):
        if request["command"] == ControlCommmands.START_QUAT_MEAS:
            self._comm_state = CommunicationState.QUAT_MEAS_RX
            self._app_state = AppContorState.QUAT_STREAM_ACTIVE

        elif request["command"] == ControlCommmands.STOP_QUAT_MEAS:
            self._comm_state = CommunicationState.QUAT_MEAS_RX
            self._app_state = AppContorState.QUAT_STREAM_PAUSED

        elif request["command"] == ControlCommmands.START_CALIBRATE:
            self._comm_state = CommunicationState.SCALED_MEAS_RX
            self._app_state = AppContorState.CALIBRATION
            self.start_calibration()
            
        elif request["command"] == ControlCommmands.STOP_CALIBRATE:
            self._comm_state = CommunicationState.CALIB_PARAM_TX
            self._app_state = AppContorState.CALIBRATION
            print("Calibration params transferring ...")
            self.cmd_queue.put(ControlCommmands.SEND_CALIB_DATA)

    def start_calibration(self):
        while True:
            try:
                self.scaled_meas_queue.get_nowait()
            except Empty:
                break
        # self._calib_finished_event = threading.Event()
        self._calib_stop_event = threading.Event()
        self._calibration_handler = CalibrationHandlerThread(self.scaled_meas_queue, self._calib_stop_event)
        self._calibration_active = True
        self._calibration_handler.start()


    def handle_responce(self, ack_msg: AcknowledgeResponse):
        request = self.pending_requests.get(ack_msg.request_id)
        if request is None:
            print(f"Unknown or expired request ID: {ack_msg.request_id}")
            return

        if ack_msg.command_code != request["command"]:
            print("ACK command code mismatch")
            return
        
        self.pending_requests.pop(ack_msg.request_id)
        
        if ack_msg.msg_code is MessageCode.ACK:
            print(f"STM32 successfully received {ack_msg.msg_code}")
            self.handle_state(request)
            return
                
        if ack_msg.msg_code is MessageCode.NACK:
            self.handle_error_code(ack_msg.error_code)
        else:
            print(f"Incorrect msg code in responce, expected {MessageCode.ACK}")


    def handle_error_code(self, error_code):
        if error_code == ErrorCode.INC_CRC.value:
            print("Incorrect CRC")
        ...

def main():
    stop_event = threading.Event()
    # TODO: cfg for uart comm
    uart_transport = UartCommunicator()
    CmdQueue = Queue()
    AckQueue = Queue()
    ScaledMeasQueue = Queue()

    rx_thread = RxMsgHandlerThread(uart_transport, AckQueue, ScaledMeasQueue, stop_event)
    app_manager_thread = ApplicationManager(uart_transport,
                                            CmdQueue, 
                                            # ack_event,
                                            AckQueue,
                                            ScaledMeasQueue,
                                            1.0,
                                            "manual",
                                            stop_event)
    try:
        rx_thread.start()
        app_manager_thread.start()
        app_manager_thread.join()

    except KeyboardInterrupt:
        pass

    finally:
        stop_event.set()
        rx_thread.join(timeout=2.0)
        if app_manager_thread._calibration_handler is not None:
            app_manager_thread._calib_stop_event.set()
            app_manager_thread._calibration_handler.join(timeout=2.0)

        uart_transport.serial_handler.close()
       


if __name__ == "__main__":
    main()
