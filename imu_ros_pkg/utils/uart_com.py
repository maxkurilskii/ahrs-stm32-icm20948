#!/usr/bin/env python3

"""
UART communication and protocol handling for STM32 IMU data exchange.

Provides:
- binary UART frame encoding and byte-by-byte parsing;
- serial port communication;
- protocol message and error definitions;
- IMU measurement reception and transmission.
"""

import argparse
import serial
import time
import struct
from enum import Enum
from typing import Optional, Union, Tuple, List

from . base_dataclasses import *
from . crc16 import calculate_crc16
from . imu_csv_logger import ImuLogger

class MessageCode(Enum):
    """Message identifiers used by the STM32 UART protocol."""

    IMU_MEAS        = 0x42      # IMU measurement message
    START_CMD       = 0x10      # Start command
    STOP_CMD        = 0x11      # Stop command
    CALIBRATE_CMD   = 0x12      # Calibration command
    ERROR_STATUS    = 0x13      # Error status message


class ErrorCode(Enum):
    """Error codes used to report UART protocol errors."""

    NO_ERROR       = 0x00
    INC_START_BYTE = 0x01
    INC_MSG_CODE   = 0x02
    INC_DATA_LEN   = 0x03
    INC_CRC        = 0x04
    INC_FRAME_LEN  = 0x05
    
class ParseStates(Enum):
    """States of the UART frame parser state machine."""

    WAIT_START    = 0
    WAIT_MSG_CODE = 1
    WAIT_LEN      = 2
    WAIT_DATA     = 3
    WAIT_CRC_H    = 4 
    WAIT_CRC_L    = 5 


class ProtocolEncoder:
    """Builds binary UART frames according to the STM32 protocol."""
         
    @staticmethod
    def build_message(msg_code: int, data: bytes) -> bytes:
        """
        Build a UART frame containing the message code and payload.

        The resulting frame has the following format:
            [START][MSG_CODE][DATA_LEN][DATA][CRC_H][CRC_L]

        Args:
            msg_code: Protocol message identifier.
            data: Message payload.

        Returns:
            Encoded UART frame as bytes.
        """
        tx_frame = bytearray()
        tx_frame.extend([UartCommunicator.PROTOTOL_START_BYTE, 
                        msg_code, 
                        len(data),
                        *data])
        crc16_val = calculate_crc16(tx_frame)
        crc16_h = (crc16_val >> 8) & 0xFF
        crc16_l = crc16_val & 0xFF
        tx_frame.extend([crc16_h, crc16_l])
        return bytes(tx_frame)
        
        
    
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
            if raw_byte == UartCommunicator.PROTOTOL_START_BYTE:
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

class UartReadTimeout(serial.SerialException):
    """Raised when the maximum UART read timeout is exceeded."""
    pass
          
class UartCommunicator:
    """
    Provides serial communication with the STM32 device.

    Handles UART port configuration, frame transmission, reception,
    buffering of incomplete data, and protocol parsing.
    """

    PROTOTOL_START_BYTE  = 0x23 #b'\x23' or b'#'(35)
    
    def __init__(self, port = 'COM4', 
                baudrate =  115200, 
                timeout = 1.0,
                max_try_timeout = 4) -> None:
        """
        Initialize the UART connection and protocol handlers.

        Args:
            port: Serial port name.
            baudrate: UART communication speed in baud.
            timeout: Serial read timeout in seconds.
        """
        self.serial_handler = serial.Serial(port = port, 
                                        baudrate = baudrate, 
                                        timeout = timeout, 
                                        write_timeout = 0.1) # 0.1 or None? 
        
        self._read_timeout = timeout
        self._max_try_timeout = max_try_timeout
        self.msg_parser = ProtocolSerialParser()
        self.msg_encoder = ProtocolEncoder()
        self._read_remaining = bytearray()

    def uart_write_package(self, msg_code: int, data: bytes):
        """
        Encode and transmit a protocol message over UART.

        Args:
            msg_code: Protocol message identifier.
            data: Message payload.
        """
        tx_msg: bytes = self.msg_encoder.build_message(msg_code, data)
        # print('Cmd tx message: ', tx_msg.hex(' '))
        self.serial_handler.write(tx_msg)

    def uart_read_package(self) -> bytes:
        """
        Read and parse the next complete UART frame.

        The method preserves bytes belonging to subsequent frames and
        processes them during the next call.

        Returns:
            A complete validated UART frame.

        Raises:
            UartReadTimeout: If no complete frame is received within the
                maximum allowed read time.
            serial.SerialException: If the serial port is closed.
        """
        # parse msg from remaining bytes of last call 
        if self._read_remaining:
            for ind, old_byte in enumerate(self._read_remaining):
                frame_msg = self.msg_parser.process_byte(old_byte)  
                if frame_msg is not None:
                    if ind != len(self._read_remaining) - 1:  self._read_remaining = self._read_remaining[ind + 1:]
                    else:   self._read_remaining = bytearray() 
                    return frame_msg
            # clean remaining bytes!
            self._read_remaining = bytearray() 
        
        # parse new bytes from RX buffer
        try_counter = 0
        while(self.serial_handler.is_open):
            # wait for one byte in serial port
            one_byte = self.serial_handler.read(1)
            if not one_byte:
                try_counter += 1
                if try_counter * self._read_timeout >= self._max_try_timeout:
                    raise UartReadTimeout('Max read timeout was reached')
                continue
            try_counter = 0 # reset if connection established
            frame_msg = self.msg_parser.process_byte(one_byte[0])
            if frame_msg is not None: 
                return frame_msg
            
            # read all available  bytes in serial buffer  
            data: bytes = self.serial_handler.read(self.serial_handler.in_waiting)
            for ind, new_byte in enumerate(data):
                frame_msg = self.msg_parser.process_byte(new_byte)
                if frame_msg is not None:
                    # in case serial buffer (data) still contains bytes that MUST be processes further ->
                    # -> save to next call in self._read_remaining
                    if ind != len(data) - 1: self._read_remaining = data[ind + 1:]
                    return frame_msg 
        raise serial.SerialException('Port is closed!')


def main():
    """
    Run a standalone UART IMU data collection example.

    Receives scaled IMU measurements from the STM32 and stores them
    in a CSV file.
    """
    header = ["Time_us",
                            "A_X", "A_Y", "A_Z", 
                            "G_X", "G_Y", "G_Z", 
                            "M_X", "M_Y", "M_Z"]
    
    header_formatted = [f'{header[0]:>22}'] + [f"{data:>9}" for data in header[1:]]
    scaled_meas_logger = ImuLogger(header = header_formatted)
    try:
        host_com = UartCommunicator()
        if host_com.serial_handler.is_open:
                print(f'Collecting and saving measurements in {scaled_meas_logger.file_name} ')
                print('Press CTRL + C to stop program')
        while(True):
            try:
                msg = host_com.uart_read_package()
                a_x, a_y, a_z,  \
                g_x, g_y, g_z,  \
                m_x, m_y, m_z,  \
                timestamp, _ = struct.unpack_from('<9f1Q1H', msg, offset=3)
                scaled_meas_logger.save_scaled_data(ReadImuScaledMeasResponse((a_x, a_y, a_z),
                                                                                (g_x, g_y, g_z), 
                                                                                (m_x, m_y, m_z),
                                                                                timestamp))
            except UartReadTimeout as e:
                print(e)
                break
            
    except KeyboardInterrupt:
        print("Program stopped")
    except serial.SerialException as e:
        print(e)
    finally:
        scaled_meas_logger.flush_buffer()
        print('Data saved')     
                
  
if __name__ == "__main__":
    main()   
    