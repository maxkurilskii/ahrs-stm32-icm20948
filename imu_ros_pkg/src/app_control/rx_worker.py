import sys
from pathlib import Path
import threading
from queue import Queue, Empty, Full
from enum import Enum
import serial
import struct
# import time
# from typing import Any, Protocol, Callable, Union, Optional, Tuple


sys.path.insert(0, str(Path(__file__).resolve().parents[1])) 

from base_dataclasses import ReadImuScaledMeasResponse, QuaternionMsgResponse, Transport, AcknowledgeResponse, ParsedFrame
from src.uartcom.serial_worker import  TransportReadTimeOutError, TransportError
from src.measlog.imu_csv_logger import ImuLogger
from protocol_codes import MessageCode


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
                    self.ack_queue.put_nowait(AcknowledgeResponse(frame.msg_code, 
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


    # def decode_msg(self, resp: bytes):
    #     pass


