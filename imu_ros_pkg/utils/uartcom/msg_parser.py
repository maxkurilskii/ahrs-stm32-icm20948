from enum import Enum
from typing import Optional
from collections import deque

from utils.uartcom.protocol_codes import PROTOTOL_START_BYTE, MessageCode, ErrorCode
from utils.uartcom.crc16 import calculate_crc16


class ParseStates(Enum):
    """States of the UART frame parser state machine."""

    WAIT_START    = 0
    WAIT_MSG_CODE = 1
    WAIT_LEN      = 2
    WAIT_DATA     = 3
    WAIT_CRC_H    = 4 
    WAIT_CRC_L    = 5 

     
class ParserQueueEmptyError(IndexError):
    """" Raised on accessing message from empty queue (dequeue) """ 
    pass
   

class ParserCrc16Error(ValueError):
    """" Raised on crc16 mismatches of the received message """ 
    pass
   

class ProtocolSerialParser:    
    """
    Incremental parser for the binary UART protocol.

    Processes the input stream byte by byte using a finite-state
    machine.

    Completed frame after successful validations converted to bytes and
    enqueued in DEQUE with fixed maxlen.
    Meaning that old complete frames would be overwritten if reader won't able to 
    process them.
    """

    def __init__(self, max_frame_size, min_frame_size, msg_queue_size) -> None:
        #protocol bytes: [start, cmd, data_len_byte, data, crc_h, crc_l] 
        self._cur_parse_state = ParseStates.WAIT_START
        # buffer for collecting bytes of one frame/msg/package 
        self._msg = bytearray()

        if not 6 <= min_frame_size <= max_frame_size <= 260:
            raise ValueError("Invalid frame size limits")
            
        self._max_frame_size = max_frame_size
        self._min_frame_size = min_frame_size

        if msg_queue_size <= 0:
            raise ValueError("Queue size must be positive")
        
        # queue (fixed size!) to hold completed msgs
        self._msg_queue = deque(maxlen = msg_queue_size)
        
        self._queue_overflow_count = 0        
        self._data_len_cntr = 0
        self._crc16_error_cntr = 0


    def _reset_all(self) -> None:
        """Reset the parser state and discard the current incomplete frame."""
        self._msg = bytearray() 
        self._data_len_cntr = 0
        self._cur_parse_state = ParseStates.WAIT_START

    def is_msg_available(self) -> bool:
        return len(self._msg_queue) > 0
    
    def get_message(self) -> bytes:
        if not self.is_msg_available():
            raise  ParserQueueEmptyError("Message queue is still empty")
        return self._msg_queue.popleft()

    def _is_valid_msg_code(self, raw_byte) -> bool:
        try:
            MessageCode(raw_byte)
        except ValueError:
            print("Incorrect MSG CODE")
            return False
        else:
            return True

    def _is_valid_msg_size(self, raw_byte) -> bool:
        return self._min_frame_size <= raw_byte + 5 <= self._max_frame_size

    def process_bytes(self, stream: bytes) -> None:
        """
        Collects a complete protocol frame using bytes received from the UART stream.
        
        Validates: 
        * Message Code 
        * DATA (payload) length -> should be less then maximum of frame length - 3  
        * CRC16
        Otherwise resets content of inner buffer.

        Args:
            bytes(): Each received byte represented as an integer in the range 0..255.

        Returns: None
        """
        for raw_byte in stream: 
            if  self._cur_parse_state == ParseStates.WAIT_START and \
                raw_byte == PROTOTOL_START_BYTE:
                    self._msg.append(raw_byte)
                    self._cur_parse_state = ParseStates.WAIT_MSG_CODE
        
            elif self._cur_parse_state == ParseStates.WAIT_MSG_CODE:
                if self._is_valid_msg_code(raw_byte):
                    self._msg.append(raw_byte)
                    self._cur_parse_state = ParseStates.WAIT_LEN
                else:
                     self._reset_all()
                    
            elif self._cur_parse_state == ParseStates.WAIT_LEN:
                if not self._is_valid_msg_size(raw_byte):
                    self._reset_all()
                    continue
                self._msg.append(raw_byte)
                self._data_len_cntr = raw_byte
                if self._data_len_cntr == 0:
                    self._cur_parse_state = ParseStates.WAIT_CRC_H
                else:
                    self._cur_parse_state = ParseStates.WAIT_DATA
            
            elif self._cur_parse_state == ParseStates.WAIT_DATA:
                self._msg.append(raw_byte)
                self._data_len_cntr -= 1
                if self._data_len_cntr == 0:
                    self._cur_parse_state = ParseStates.WAIT_CRC_H
                                
            elif self._cur_parse_state == ParseStates.WAIT_CRC_H:
                self._msg.append(raw_byte)
                self._cur_parse_state = ParseStates.WAIT_CRC_L
                
            elif self._cur_parse_state == ParseStates.WAIT_CRC_L:
                self._msg.append(raw_byte)
                crc16 = calculate_crc16(self._msg) 
                if (crc16 & 0xFFFF) != 0x0000:
                    print(f"Incorrect CRC!\nMessage: {self._msg}")  
                    self._crc16_error_cntr += 1
                    self._reset_all()

                # save frame/msg before reseting it 
                msg = bytes(self._msg[:3 + self._msg[2] + 2]) # (3 status + payload + crc16)
                self._reset_all()
                if len(self._msg_queue) == self._msg_queue.maxlen: 
                    self._queue_overflow_count += 1
                self._msg_queue.append(msg)
