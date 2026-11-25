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
                                                DeviceStatusResponse

from src.communication.com_protocol_codes import    MessageCode, \
                                                    ControlCommands, \
                                                    ErrorCode
from rx_worker import RxMsgHandlerThread
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
    PENDING  = auto()
    DONE     = auto()
    CANCELED = auto()


class AcknowledgeTimeoutError(Exception):
    pass



class ApplicationManager(threading.Thread):
    
    control_cmd_2_msg_code_mapper = {
        ControlCommands.HANDSHAKE:       MessageCode.HANDSHAKE,
        ControlCommands.SET_CALIB_DATA:  MessageCode.SET_CALIB_DATA,
        ControlCommands.CHECK_DEVICE_STATUS: MessageCode.CHECK_DEVICE_STATUS,
        ControlCommands.START_QUAT_MEAS: MessageCode.START_QUAT_MEAS,
        ControlCommands.STOP_QUAT_MEAS:  MessageCode.STOP_QUAT_MEAS,
        ControlCommands.SET_REF_TIME:    MessageCode.SET_REF_TIME,
        ControlCommands.START_CALIBRATE: MessageCode.START_CALIBRATE,
        ControlCommands.STOP_CALIBRATE:  MessageCode.STOP_CALIBRATE,
    }

    keyboard_2_control_cmd_mapper = {
        KeyBoardCommmands.CHECK_DEVICE_STATUS:  ControlCommands.CHECK_DEVICE_STATUS,
        KeyBoardCommmands.START_QUAT_MEAS:      ControlCommands.START_QUAT_MEAS,
        KeyBoardCommmands.STOP_QUAT_MEAS:       ControlCommands.STOP_QUAT_MEAS,
        # KeyBoardCommmands.START_CALIBRATE:      ControlCommands.START_CALIBRATE,
        # KeyBoardCommmands.STOP_CALIBRATE:       ControlCommands.STOP_CALIBRATE,
        # KeyBoardCommmands.SET_CALIB_DATA:       ControlCommands.SET_CALIB_DATA
    }

    def __init__(self,
                transport: MsgCommunicator,
                man_queue: Queue,
                request_table: dict,
                request_timeout: Union[int, float],
                stop_event: threading.Event):
        
        super().__init__(name="AppThread", daemon=True)

        self.transport = transport
        self.man_queue = man_queue
        self._stop_event = stop_event 

        self._request_id = 1
        self._request_table = request_table
        self._request_timeout = request_timeout
        self._resp_error_cntr = 0
        self._max_resp_errors = 3
        self._last_response: Optional[AcknowledgeResponse] = None
        # self._response_updated = False

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
    
    def check_device_status(self):
        if self._cntrl_cmd_state is ControlCmdState.PENDING:
            return
        print("Device status unknown. Fetching ...\n", end='')
        self._app_unknown_counter += 1 # resets to zero in change_app_state() method
        if self._app_unknown_counter > self._app_unknown_limit:
            self._conn_state = ConnectionState.LOST
            return
        self.handle_request(ControlCommands.CHECK_DEVICE_STATUS)
        

      

    def handle_keyboard_command(self, kb_cmd: KeyBoardCommmands):
        # Process most priority keyboard cmd
        if kb_cmd == KeyBoardCommmands.SHUTDOWN:
            print("'ESC' pressed. App exitting ...\n", end='')
            self._stop_event.set()
            return
        
        # Skip other keyboard cmds while app_state is UNKNOWN
        if self._app_state is AppState.UNKNOWN:
            print("CMD skipped! Waiting for device status response ...\n", end='')
            return

        # Skip other keyboard cmds until last call is received or expired
        if self._cntrl_cmd_state is ControlCmdState.PENDING:
            print("CMD skipped! Waiting for acknowledge from last control command ...\n", end='')
            return
        
        elif kb_cmd == KeyBoardCommmands.CHECK_DEVICE_STATUS:
            if self._app_state in [AppState.IDLE, 
                                #    AppState.QUAT_STREAM_ACTIVE, # TODO: for simplicity not impl yet
                                   AppState.QUAT_STREAM_PAUSED]:
                self.handle_request(ControlCommands.CHECK_DEVICE_STATUS)
            else:
                print(f"Can't request device status in {self._app_state}\n", end='')
        

        elif kb_cmd == KeyBoardCommmands.START_QUAT_MEAS:
            if self._app_state not in (AppState.IDLE, AppState.QUAT_STREAM_PAUSED):
                print(f"Cannot start quaternion stream in {self._app_state}\n", end='')
                return
            if (control_cmd := self._get_matching_control_cmd(kb_cmd)) is not None:
                self.handle_request(control_cmd)

        elif kb_cmd == KeyBoardCommmands.STOP_QUAT_MEAS:
            if self._app_state != AppState.QUAT_STREAM_ACTIVE:
                print("Quat stream is not active!\n", end='') 
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
        msg_code = self._get_matching_msg_code(cmd)
        if msg_code is None:
            raise ValueError(f"Unsupported command: {cmd}")
        
        req_id = (self._request_id + 1) & 0xFFFF
        if req_id == 0:
            req_id = 1
        request = PendingRequest(req_id, cmd,
                                    deadline= time.monotonic() +  self._request_timeout)
        # Register request BEFORE transmission.
        if req_id in self._request_table:
            raise ValueError(f"Request ID {req_id} already exists")
        self._request_table[req_id] = request
        self._cntrl_cmd_state = ControlCmdState.PENDING
        self._request_id = req_id
        # self._response_updated = False
        try:
                self.transport.write_package(msg_code.value, 
                                    # req_id.to_bytes(2, 'little'))
                                    req_id.to_bytes(2, 'little') + payload)
        except serial.SerialException:
            self._request_table.pop(req_id, None)
            self._cntrl_cmd_state = ControlCmdState.CANCELED
            raise

    def _get_matching_control_cmd(self, keyboard_cmd) -> Optional[ControlCommands]:
        res = self.keyboard_2_control_cmd_mapper.get(keyboard_cmd, None)
        if not res:
            print("Keyboard command doesn't match the control command\n", end='')
        return res 
    
    def _get_matching_msg_code(self, control_cmd):
        res = self.control_cmd_2_msg_code_mapper.get(control_cmd, None)
        if not res:
            print("Control command doesn't match the msg code\n", end='')
        return res 

    
    def handle_response(self, response: Union[AcknowledgeResponse, DeviceStatusResponse]) -> Union[AcknowledgeResponse, DeviceStatusResponse, None]: 
        # if ack_resp is None or not isinstance(ack_resp, AcknowledgeResponse):
        #     self._update_response_errors()
        #     return
        req: Optional[PendingRequest] = self._request_table.get(response.request_id, None)
        if req is None or not isinstance(req, PendingRequest):
            print(f"Unknown or expired request ID: {response.request_id}, msg code: {response.msg_code}\n", end='')
            self._update_response_errors()
            self._app_state = AppState.UNKNOWN
            return None
        
        if response.msg_code not in (MessageCode.ACK, MessageCode.NACK, MessageCode.CHECK_DEVICE_STATUS):
            print(f"Unexpected response code: {response.msg_code}\n",end='')
            self._update_response_errors()
            self._cancel_request(req)
            self._app_state = AppState.UNKNOWN
            return None
            
        if response.control_cmd_val != req.control_cmd.value:
            print(f"response control cmd didn't match pending control cmd. Received: {response.request_id}, {response.msg_code}\n", end = '')
            self._update_response_errors()
            self._cancel_request(req)
            self._app_state = AppState.UNKNOWN
            return None

        # Remove completed request
        self._request_table.pop(req.request_id, None)
        # Reset accumulated response errors 
        self._resp_error_cntr = 0

        if response.msg_code == MessageCode.NACK:
            print(f"Request NOT accepted (NACK): {req.control_cmd}\n",end='')
            # IF NACK is received -> connection is stable -> there are stm-side failures
            self.log_error_code(response)
            print(f"Command {req.control_cmd} was canceled\n", end='')
            self._cntrl_cmd_state = ControlCmdState.CANCELED
            return None

        print(f"Request accepted (ACK): {req.control_cmd}\n", end='')
        self._cntrl_cmd_state = ControlCmdState.DONE
        self.change_app_state(response)
        return response
        # self._last_response = ack_resp

    def _update_response_errors(self):
        self._resp_error_cntr += 1
        if self._resp_error_cntr >= self._max_resp_errors:
            # Exiting on first lost connection without retry (max response errors exceeded)
            print("Device is not responding. Bye-bye")
            self._conn_state = ConnectionState.LOST

    def _cancel_request(self, req):
        print(f"Command {req.control_cmd} was canceled\n", end='')
        self._request_table.pop(req.request_id, None)
        self._cntrl_cmd_state = ControlCmdState.CANCELED

    def _cancel_all_requests(self):
        for req_id in list(self._request_table.keys()):
            # print(f"Command {req.control_cmd} was canceled\n", end='')
            self._request_table.pop(req_id)
        self._cntrl_cmd_state = ControlCmdState.CANCELED

    def log_error_code(self, resp: Union[AcknowledgeResponse, DeviceStatusResponse]):
        if resp.error_code == ErrorCode.INC_CRC.value:
            print("Incorrect CRC")
        ...

    def change_app_state(self, response: Union[AcknowledgeResponse, DeviceStatusResponse]):

        control_cmd = ControlCommands(response.control_cmd_val)
        
        if control_cmd == ControlCommands.CHECK_DEVICE_STATUS and isinstance(response, DeviceStatusResponse):
           # self.log_device_status(response.dev_state)
           # NOTE: dev_state can not be UNKNOWN !
           try: 
                dev_state = AppState(response.dev_state_val)
           except ValueError:
                print(f"Device state is unknown: {response.dev_state_val}\n", end='')
                self._app_state = AppState.UNKNOWN
                return
           else:
                self._app_state = dev_state
                self._app_unknown_counter = 0
                return

        if control_cmd in [ControlCommands.HANDSHAKE, ControlCommands.SET_REF_TIME]:
            self._app_state = AppState.IDLE

        elif control_cmd == ControlCommands.START_QUAT_MEAS:
            # self._conn_state = ConnectionState.QUAT_MEAS_RX
            self._app_state = AppState.QUAT_STREAM_ACTIVE

        elif control_cmd == ControlCommands.STOP_QUAT_MEAS:
            # self._conn_state = ConnectionState.QUAT_MEAS_RX
            # self._app_state = AppState.QUAT_STREAM_PAUSED
            self._app_state = AppState.IDLE


    def establish_connection(self):
        self._clean_man_queue() # clean all tasks in manager queue
        self._cancel_all_requests() # clean all requests in reqest table
        self.handle_request(ControlCommands.HANDSHAKE)
        # blocking !
        try:
            task = self.man_queue.get(timeout = self._request_timeout + 0.5)
        except Empty:
            print("No response during establishing connection!\n", end='')
            self._cancel_all_requests() # clean all requests in reqest table
            return
        resp = self.handle_response(task)
        if resp is not None and resp.control_cmd_val == ControlCommands.HANDSHAKE.value:
           self._conn_state = ConnectionState.ESTABLISHED
        
    def set_reference_time(self):
        self._clean_man_queue() # clean all tasks in manager queue
        self._cancel_all_requests() # clean all requests in reqest table
        reference_time_us = time.time_ns() // 10**3 # int object in the end
        payload = reference_time_us.to_bytes(8, byteorder='little') #cast to uint64 in little-endian
        self.handle_request(ControlCommands.SET_REF_TIME, payload)
        # blocking !
        try:
            task = self.man_queue.get(timeout = self._request_timeout + 0.5)
        except Empty:
            print("No response during setting time!\n", end='')
            self._cancel_all_requests() # clean all requests in reqest table
            return
        
        resp = self.handle_response(task)
        if resp is not None and resp.control_cmd_val == ControlCommands.SET_REF_TIME.value:
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
    ManQueue = Queue() #TODO: change to PriorityQueue
    # AckQueue = Queue()
    ack_timeout = 0.1
    ScaledMeasQueue = Queue()

    request_table = {}
    request_timeout = 1 # req_wait_time > ack_timeout

    rx_thread = RxMsgHandlerThread(uart_transport, 
                                   ManQueue, 
                                   ScaledMeasQueue, 
                                   stop_event)

    app_manager_thread = ApplicationManager(uart_transport,
                                            ManQueue, 
                                            request_table,
                                            request_timeout,
                                            stop_event)
    try:
        rx_thread.start()
        app_manager_thread.start()
        app_manager_thread.join()

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    except serial.SerialException:
        print("Serial port error!")
    except TypeError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # set global event to finish other threads 
        stop_event.set()

        # stop keyboard thread
        app_manager_thread.stop_keyboard_listener()
        
        if app_manager_thread.is_alive():
            app_manager_thread.join(timeout=2.0)

        if rx_thread.is_alive():
            rx_thread.join(timeout=2.0) # wait for meas and log files to be closed 
        
        # close serial port 
        uart_transport.close_connection()
       


if __name__ == "__main__":
    main()
