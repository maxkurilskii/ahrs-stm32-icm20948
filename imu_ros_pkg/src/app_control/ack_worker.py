from pathlib import Path
import threading
from queue import Queue, Empty, Full
from enum import Enum
import serial
from typing import Union, Optional

from src.communication.com_dataclasses import   AcknowledgeResponse,\
                                                PendingRequest, \
                                                RequestResult

from src.communication.com_protocol_codes import MessageCode, ErrorCode

class AckResponseHandlerThread(threading.Thread):

    def __init__(
        self,
        ack_queue: Queue,
        ack_timeout: Union[int, float],
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
