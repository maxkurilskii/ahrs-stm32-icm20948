from enum import Enum
from typing import Optional

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


class ProtocolSerialParser:    
    """
    Incremental parser for the binary UART protocol.

    Processes the input stream byte by byte using a finite-state
    machine and returns a complete frame after successful CRC validation.
    """

    def __init__(self) -> None:
        #protocol bytes: [start, cmd, data_len_byte, data, crc_h, crc_l] 
        self._msg = bytearray() #var for collecting bytes of one frame msg 
        self._data_len_cntr = 0
        self._crc16_len_cntr = 2
        self._current_msg_state = ParseStates.WAIT_START

    def reset_all(self) -> None:
        """Reset the parser state and discard the current incomplete frame."""
        self._msg = bytearray() 
        self._data_len_cntr = 0
        self._current_msg_state = ParseStates.WAIT_START
            
    def process_byte(self, raw_byte: int) -> Optional[bytes]:
        """
        Process one byte received from the UART stream.

        Args:
            raw_byte: Received byte represented as an integer in the range 0..255.

        Returns:
            A complete validated frame if one has been received;
            otherwise, None.
        """
        if  self._current_msg_state == ParseStates.WAIT_START:
            if raw_byte == PROTOTOL_START_BYTE:
                self._msg.append(raw_byte)
                self._current_msg_state = ParseStates.WAIT_MSG_CODE
                return None
      
        elif self._current_msg_state == ParseStates.WAIT_MSG_CODE:
            try:
                MessageCode(raw_byte)
            except ValueError:
                print("Incorrect MSG CODE")
                self.reset_all()
            else:
                self._msg.append(raw_byte)
                self._current_msg_state = ParseStates.WAIT_LEN
            return None
                
        elif self._current_msg_state == ParseStates.WAIT_LEN:
            self._msg.append(raw_byte)
            self._data_len_cntr = raw_byte
            if self._data_len_cntr == 0:
                self._current_msg_state = ParseStates.WAIT_CRC_H
            else:
                self._current_msg_state = ParseStates.WAIT_DATA
            return None
        
        elif self._current_msg_state == ParseStates.WAIT_DATA:
            self._msg.append(raw_byte)
            self._data_len_cntr -= 1
            if self._data_len_cntr == 0:
                self._current_msg_state = ParseStates.WAIT_CRC_H
            return None
                            
        elif self._current_msg_state == ParseStates.WAIT_CRC_H:
            self._msg.append(raw_byte)
            self._current_msg_state = ParseStates.WAIT_CRC_L
            return None
            
        elif self._current_msg_state == ParseStates.WAIT_CRC_L:
            self._msg.append(raw_byte)
            crc16 = calculate_crc16(self._msg) 
            if (crc16 & 0xFFFF) != 0x0000:
                print(f"Incorrect CRC!\nMessage: {self._msg}")  
                self.reset_all()
                return None 
            msg = bytes(self._msg)
            self.reset_all()
            return msg
