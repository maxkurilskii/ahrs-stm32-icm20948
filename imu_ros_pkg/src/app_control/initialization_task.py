from pathlib import Path
import threading
import time
from queue import Queue, Empty, Full
from enum import Enum
import serial
from typing import Union, Optional


from src.communication.com_dataclasses import   ReadImuScaledMeasResponse, \
                                                QuaternionMsgResponse, \
                                                AcknowledgeResponse,\
                                                ParsedFrame, \
                                                PendingRequest, \
                                                RequestResult

# from rx_worker import RxMsgHandlerThread
from src.communication.serial_handler import MsgCommunicator
from src.communication.com_protocol_codes import MessageCode, ControlCommands,  ErrorCode

control_cmd_2_msg_code_mapper = {
        ControlCommands.HANDSHAKE:       MessageCode.HANDSHAKE,
        ControlCommands.START_QUAT_MEAS: MessageCode.START_QUAT_MEAS,
        ControlCommands.STOP_QUAT_MEAS:  MessageCode.STOP_QUAT_MEAS,
        ControlCommands.SET_REF_TIME:    MessageCode.SET_REF_TIME,
        ControlCommands.START_CALIBRATE: MessageCode.START_CALIBRATE,
        ControlCommands.STOP_CALIBRATE:  MessageCode.STOP_CALIBRATE,
        ControlCommands.SET_CALIB_DATA:  MessageCode.SET_CALIB_DATA
    }

def request_handler(transport: MsgCommunicator, 
                    control_cmd: ControlCommands,
                    request_id: int,
                    request_table: dict,
                    request_lock: threading.Lock):
                    # cmd_payload: bytes):
    
    request = PendingRequest(request_id,control_cmd,
                             deadline=time.monotonic() + 1.5)

    # Register request BEFORE transmission.
    with request_lock:
        if request_id in request_table:
            raise ValueError(f"Request ID {request_id} already exists")
        request_table[request_id] = request
    
    try:
        transport.write_package(control_cmd_2_msg_code_mapper[control_cmd].value, 
                            request_id.to_bytes(2, 'little'))
                            # request_id.to_bytes(2, 'little') + cmd_payload)
    except serial.SerialException:
        with request_lock:
            request_table.pop(request_id, None)
        raise


class AckResponseHandlerThread(threading.Thread):

    def __init__(
        self,
        ack_queue: Queue,
        ack_timeout: int,
        request_table: dict,
        lock: threading.Lock,
        manager_queue: Queue,
        stop_event: threading.Event
    ):
        super().__init__(
            name="AckResponseHandler",
            daemon=True
        )

        self.ack_queue = ack_queue
        self.ack_timeout = ack_timeout
        self.request_table = request_table
        self.request_lock = lock
        self.manager_queue = manager_queue
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            try:
                ack: AcknowledgeResponse = self.ack_queue.get(timeout=self.ack_timeout)
            except Empty:
                continue
            self.handle_response(ack)
    
    def handle_response(self, ack_resp): 
            with self.request_lock:
                req: Optional[PendingRequest] = self.request_table.get(ack_resp.request_id)
            
            if req is None:
                print(f"Unknown or expired request ID: {ack_resp.request_id}")
                return
                    
            if ack_resp.msg_code == MessageCode.NACK:
                self.log_error_code(ack_resp)
                return
            
            if ack_resp.control_cmd != req.control_cmd.value:
                print(f"No matching command_code. Received: {ack_resp.request_id}, {ack_resp.msg_code}")
                return
            with self.request_lock:
                self.request_table.pop(ack_resp.request_id)

            self.manager_queue.put(RequestResult(req, ack_resp))


    def log_error_code(self, ack: AcknowledgeResponse):
        if ack.error_code == ErrorCode.INC_CRC.value:
            print("Incorrect CRC")
        ...


# def set_time_handler(transport: MsgCommunicator, 
#                     ack_queue: Queue, 
#                     ack_timeout: int):
        
#     global request_id, request_table
#     request_table[request_id] = {"control_cmd": ControlCommmands.SET_REF_TIME.value}
#     transport.write_package(MessageCode.SET_REF_TIME.value, request_id.to_bytes(2, 'little') + b'\x00')
#     request_id += 1
#     try:
#         ack_resp: AcknowledgeResponse = ack_queue.get(timeout = ack_timeout)
#     except Empty:
#         print("Can't establish connection with STM32F767zi")
#     else:
#         if ack_resp.request_id not in request_table:
#             print(f"No matching request id. Received: {ack_resp.request_id}, {ack_resp.msg_code}, {ack_resp.control_cmd}")
#             return
#         if ack_resp.msg_code == MessageCode.NACK:
#             not_ack_handler(ack_resp)
#             return
#         if not ack_resp.control_cmd == control_cmd.value:
#             print(f"No matching command_code. Received: {ack_resp.request_id}, {ack_resp.msg_code}, {ack_resp.control_cmd}")
#             return
                
#         request_table.pop(ack_resp.request_id)
#         print("STM32F767zi reference time is set!")


class RxMsgHandlerThread(threading.Thread):
    def __init__(self, 
                transport: MsgCommunicator,
                ack_queue: Queue,
                stop_event: threading.Event):

        super().__init__(name="RxMsgHandler", daemon=True)

        self.transport = transport
        self.ack_queue = ack_queue
        self._stop_event = stop_event
    
    def run(self):
        while not self._stop_event.is_set():
            try:
                msg: bytes = self.transport.read_package() 
                frame = ParsedFrame(MessageCode(msg[1]), msg[3:-2])

                if frame.msg_code in [MessageCode.ACK, MessageCode.NACK]:
                    request_id = int.from_bytes(frame.payload[:2], 'little')
                    control_cmd_val = frame.payload[2]
                    error_code = frame.payload[3]
                    self.ack_queue.put_nowait(AcknowledgeResponse(frame.msg_code, 
                                                                request_id, 
                                                                control_cmd_val, 
                                                                error_code))
            except serial.SerialException as e:
                print(f"UART error: {e}") # log critical
                break
            
            except Empty:
                print(f"{self.name}: Rx queue is empty for too long")
                continue

            except Full:
                print(f"{self.name}: Rx queue is full for too long") # log warn
                continue


def handle_responce_result(req_res: RequestResult):
    request = req_res.request
    response = req_res.response
    if response.msg_code == MessageCode.ACK:
        print(f"Request accepted: {request.control_cmd}")
        # handle_state(request)


def main():
    transport = MsgCommunicator()
    stop_event = threading.Event()
    ack_queue = Queue(maxsize=10)
    man_queue = Queue(maxsize=10)
    ack_timeout = 2

    request_id = 1
    request_table = {}
    request_lock = threading.Lock()



    rx_thread = RxMsgHandlerThread(transport, ack_queue, stop_event)

    # init_thread = threading.Thread(target=request_handler,
    #                                args= (transport, ack_queue, ack_timeout),
    #                                name = "InitThread", daemon=True)
    

    ack_thread = AckResponseHandlerThread(ack_queue,
                                          ack_timeout,
                                          request_table,
                                          request_lock,
                                          man_queue,
                                          stop_event)
    try:
        rx_thread.start()
        ack_thread.start()
        # init_thread.start()
        # init_thread.join()

        request_handler(transport, ControlCommands.HANDSHAKE, 
                        request_id=request_id, request_table=request_table,
                        request_lock=request_lock)

        try:
            res: RequestResult = man_queue.get(timeout = 5)
        except Empty:
            print("Stm is not responding ...")
            return
        else:
            handle_responce_result(res)

    
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"uart port error: {e}") 
    finally:
        stop_event.set()
        rx_thread.join(timeout=2)
        ack_thread.join(timeout=2)
        transport.close_connection()


if __name__ == "__main__":
    main()