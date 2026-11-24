import sys
from pathlib import Path
import threading
from queue import Queue, Empty, Full
from enum import Enum, IntEnum, auto
import serial
import struct
import time
from typing import Union, Optional

from src.communication.com_dataclasses import   AcknowledgeResponse,\
                                                PendingRequest, \
                                                RequestResult

from src.communication.com_protocol_codes import    MessageCode, \
                                                    ControlCommands, \
                                                    ErrorCode
from rx_worker import RxMsgHandlerThread
from ack_worker import AckResponseHandlerThread
# from calib_worker import CalibrationHandlerThread

from src.communication.serial_handler import MsgCommunicator
from src.measlog.imu_csv_logger import ImuLogger

from keyboard_cmd_source import KeyboardListener, KeyBoardCommmands



class AppState(IntEnum):
    # INIT                              = auto()
    IDLE                              = auto()
    # REF_TIME_REQUEST_PENDING          = auto()
    # QUAT_STREAM_REQUEST_PENDING       = auto()
    QUAT_STREAM_ACTIVE                = auto()
    QUAT_STREAM_PAUSED                = auto()
    # CALIB_MEAS_ACQUISITION   = 4
    # CALIB_PARAM_ESTIMATION   = 5
    # CALIBRATE_REQUEST_PENDING         = auto()
    CALIBRATION                       = auto()


class ConnectionState(IntEnum):
    NOT_EST           = auto()
    ESTABLISHED       = auto()
    LOST              = auto()

    # SCALED_MEAS_RX    = auto()
    # CALIB_PARAM_TX    = auto()
    # QUAT_MEAS_RX      = auto()


class ControlCmdState(IntEnum):
    FREE     = auto()
    PENDING  = auto()
    DONE     = auto()
    CANCELED = auto()


class AcknowledgeTimeoutError(Exception):
    pass

class ApplicationManager(threading.Thread):
    
    control_cmd_2_msg_code_mapper = {
        ControlCommands.HANDSHAKE:       MessageCode.HANDSHAKE,
        ControlCommands.START_QUAT_MEAS: MessageCode.START_QUAT_MEAS,
        ControlCommands.STOP_QUAT_MEAS:  MessageCode.STOP_QUAT_MEAS,
        ControlCommands.SET_REF_TIME:    MessageCode.SET_REF_TIME,
        ControlCommands.START_CALIBRATE: MessageCode.START_CALIBRATE,
        ControlCommands.STOP_CALIBRATE:  MessageCode.STOP_CALIBRATE,
        ControlCommands.SET_CALIB_DATA:  MessageCode.SET_CALIB_DATA
    }

    keyboard_2_control_cmd_mapper = {
        KeyBoardCommmands.CHECK_DEVICE_STATUS:  ControlCommands.CHECK_DEVICE_STATUS,
        KeyBoardCommmands.START_QUAT_MEAS:      ControlCommands.START_QUAT_MEAS,
        KeyBoardCommmands.STOP_QUAT_MEAS:       ControlCommands.STOP_QUAT_MEAS,
        KeyBoardCommmands.START_CALIBRATE:      ControlCommands.START_CALIBRATE,
        KeyBoardCommmands.STOP_CALIBRATE:       ControlCommands.STOP_CALIBRATE,
        KeyBoardCommmands.SET_CALIB_DATA:       ControlCommands.SET_CALIB_DATA
    }

    def __init__(self,
                transport: MsgCommunicator,
                man_queue: Queue,
                request_table: dict,
                request_lock: threading.Lock,
                request_timeout: Union[int, float],
                stop_event: threading.Event):
        
        super().__init__(name="AppThread", daemon=True)

        self.transport = transport
        self.man_queue = man_queue
        self._stop_event = stop_event 

        self._request_id = 1
        self._request_table = request_table
        self._request_lock = request_lock
        self._local_req_table = {}
        self._request_timeout = request_timeout
        self._received_responce = None
        self._responce_updated = False

        self._cntrl_cmd_state: Optional[ControlCmdState] = None
        self._app_state = AppState.IDLE 
        # self._app_lock = threading.Lock()
        self._conn_state = ConnectionState.NOT_EST
        # self._condition = threading.Condition()
        self._connect_tries = 2
        # self._connection_wait_time = 2

        self._kb_thread = KeyboardListener(man_queue)

        self.ref_time_us = None


    def run(self):
        # INITIALIZATION
        while not self._stop_event.is_set() and self._conn_state is not ConnectionState.ESTABLISHED and self._connect_tries > 0:
            # Establish connetion firstly
            print("Establishing connection ...")
            self.establish_connection()
            if self._conn_state != ConnectionState.ESTABLISHED:
                continue
            # Initialize reference time
            self.set_reference_time()

        if self._conn_state is not ConnectionState.ESTABLISHED:
            return
       
        # Stat keyboard thread
        self._kb_thread.start()
        
        while not self._stop_event.is_set():
            # Exiting on first lost connection without retry (timeout on last responce)
            if self._conn_state is ConnectionState.LOST:
                print("Device is not responding. Bye-bye")
                return

            try:
                task: Union[RequestResult, KeyBoardCommmands] = self.man_queue.get(timeout = 0.05)
                if not isinstance(task, (RequestResult, KeyBoardCommmands)):
                    raise TypeError(f"Get wrong command type! Expected RequestResult or KeyBoardCommmands")
                if isinstance(task, RequestResult):
                    self.handle_responce_result(task)
                else:
                    self.handle_keyboard_command(task)

            except serial.SerialException as e:
                print(f"UART TX failed: {e}")
            # except Empty:
                # print(f"{self.name}: Cmd queue is empty for too long") #log debug
            # --------------------------------------------------------------
            # Check pending requests
            # --------------------------------------------------------------
            self.check_pending_requests()

                
    def check_pending_requests(self):
        for request_id, request in list(self._local_req_table.items()):
            if time.monotonic() >= request.deadline:
                print(f"Request {request_id} timeout!")
                with self._request_lock:
                    self._request_table.pop(request_id)
                    self._local_req_table.pop(request_id)
                print(f"Command {request.control_cmd} was canceled")
                self._app_state = AppState.IDLE
                # if request.control_cmd == ControlCommands.STOP_CALIBRATE:
                    #  TODO: writing to file should be stopped !
                    # may be with blocking + gui info
                    # pass
                # request["try_count"] -= 1
                # if request["try_count"] == 0:
                #     print("Max try count exceeded!") # log critical
                #     self.pending_requests.pop(request_id)


    def handle_responce_result(self, req_res: RequestResult):
        request: PendingRequest = req_res.request
        response: AcknowledgeResponse = req_res.response
        if response.msg_code == MessageCode.ACK:
            print(f"Request accepted: {request.control_cmd}")
            # print("STM32F767zi is available!")
            self.change_app_state(request)
            self._cntrl_cmd_state = ControlCmdState.DONE
            self._conn_state = ConnectionState.ESTABLISHED
            self._received_responce = response
            self._responce_updated = True
        else:
            print(f"Request NOT accepted: {request.control_cmd}")
            self._conn_state = ConnectionState.LOST

    def handle_keyboard_command(self, kb_cmd: KeyBoardCommmands):
        if self._cntrl_cmd_state is ControlCmdState.PENDING:
            print("Waiting for acknowledge from last control command ...")
            return
        
        if kb_cmd == KeyBoardCommmands.START_QUAT_MEAS:
            if self._app_state == AppState.QUAT_STREAM_ACTIVE:
                print("Quat stream is already active!")
                return
            if (control_cmd := self._get_matching_control_cmd(kb_cmd)) is not None:
                self.handle_request(control_cmd)

        elif kb_cmd == KeyBoardCommmands.STOP_QUAT_MEAS:
            if self._app_state != AppState.QUAT_STREAM_ACTIVE:
                print("Quat stream is not active!") 
                return
            if (control_cmd := self._get_matching_control_cmd(kb_cmd)) is not None:
                self.handle_request(control_cmd)

        # elif cmd == ControlCommands.START_CALIBRATE:
        #     if self._app_state not in [AppState.IDLE, AppState.QUAT_STREAM_PAUSED]:
        #         print("Calibration can't be started")
        #         return
        #     self._request_id += 1
        #     self.handle_request(cmd, self._request_id)

        # elif cmd == ControlCommands.STOP_CALIBRATE:
        #     if self._app_state != AppState.CALIBRATION:
        #         print("Calibration is not active!")
        #         return
        #     self._request_id += 1
        #     self.handle_request(cmd, self._request_id)

    def handle_request(self, cmd: ControlCommands, payload = b'\x00'):
        req_id = self._request_id + 1
        request = PendingRequest(req_id, 
                                    cmd,
                                    deadline= time.monotonic() +  self._request_timeout)
        # Register request BEFORE transmission.
        with self._request_lock:
            if req_id in self._request_table:
                raise ValueError(f"Request ID {req_id} already exists")
            self._request_table[req_id] = request
            self._local_req_table[req_id] = request
        self._request_id = req_id
        self._cntrl_cmd_state = ControlCmdState.PENDING  
        try:
            if (msg_code := self._get_matching_msg_code(cmd)) is not None:
                self.transport.write_package(msg_code.value, 
                                    # req_id.to_bytes(2, 'little'))
                                    req_id.to_bytes(2, 'little') + payload)
        except serial.SerialException:
            with self._request_lock:
                self._request_table.pop(req_id, None)
            raise

    def _get_matching_control_cmd(self, keyboard_cmd) -> Optional[ControlCommands]:
        res = self.keyboard_2_control_cmd_mapper.get(keyboard_cmd, None)
        if not res:
            print("Keyboard command doesn't match the control command")
        return res 
    
    def _get_matching_msg_code(self, control_cmd):
        res = self.control_cmd_2_msg_code_mapper.get(control_cmd, None)
        if not res:
            print("Control command doesn't match the msg code")
        return res 

    def change_app_state(self, request: PendingRequest):
        if request.control_cmd == ControlCommands.START_QUAT_MEAS:
            # self._conn_state = ConnectionState.QUAT_MEAS_RX
            self._app_state = AppState.QUAT_STREAM_ACTIVE

        elif request.control_cmd == ControlCommands.STOP_QUAT_MEAS:
            # self._conn_state = ConnectionState.QUAT_MEAS_RX
            # self._app_state = AppState.QUAT_STREAM_PAUSED
            self._app_state = AppState.IDLE


    def establish_connection(self):
        if self._connect_tries == 0:
            print("Device is not responding. Bye-bye")
            self._stop_event.set()
            return
        # clean all task in manager queue
        self._clean_man_queue()
        self._connect_tries -= 1
        self.handle_request(ControlCommands.HANDSHAKE)
        task: RequestResult = self.man_queue.get(timeout = self._request_timeout + 0.5)
        if not isinstance(task, RequestResult):
            raise TypeError(f"Get wrong command type! Expected RequestResult")
        self.handle_responce_result(task)

    def set_reference_time(self):
        reference_time_us = time.time_ns() // 10**3 # int object in the end
        payload = reference_time_us.to_bytes(8, byteorder='little') #cast to uint64 in little-endian
        self.handle_request(ControlCommands.SET_REF_TIME, payload)
        task: RequestResult = self.man_queue.get(timeout = self._request_timeout + 0.5)
        if not isinstance(task, RequestResult):
            raise TypeError(f"Get wrong command type! Expected RequestResult")
        self.handle_responce_result(task)
        if self._responce_updated:
            self.ref_time_us = reference_time_us

    def _clean_man_queue(self):
        while True:
            try:
                self.man_queue.get_nowait()
            except Empty:
                return

def main():
    stop_event = threading.Event()
    uart_transport = MsgCommunicator()  # TODO: cfg for uart comm
    ManQueue = Queue()
    AckQueue = Queue()
    ack_timeout = 0.1
    ScaledMeasQueue = Queue()

    request_table = {}
    request_lock = threading.Lock()
    request_timeout = 1 # req_wait_time > ack_timeout

    rx_thread = RxMsgHandlerThread(uart_transport, 
                                   AckQueue, 
                                   ScaledMeasQueue, 
                                   stop_event)

    ack_thread = AckResponseHandlerThread(AckQueue, ack_timeout,
                                          request_table, 
                                          request_lock,
                                          ManQueue, 
                                          stop_event)


    app_manager_thread = ApplicationManager(uart_transport,
                                            ManQueue, 
                                            request_table,
                                            request_lock,
                                            request_timeout,
                                            stop_event)
    try:
        rx_thread.start()
        ack_thread.start()
        app_manager_thread.start()
        app_manager_thread.join()

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    except serial.SerialException:
        print("Serial port error!")
    except TypeError as e:
        print(f"Error: {e}")
    finally:
        stop_event.set()
        rx_thread.join(timeout=2.0)
        ack_thread.join(timeout=2.0)
        uart_transport.close_connection()
       


if __name__ == "__main__":
    main()
