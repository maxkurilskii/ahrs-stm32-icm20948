#!/usr/bin/env python3

"""
UART communication and protocol handling for STM32 IMU data exchange.

Provides:
- binary UART frame encoding and byte-by-byte parsing;
- serial port communication;
- protocol message and error definitions;
- IMU measurement reception and transmission.
"""

import serial

from src.app_control.protocol_codes import PROTOTOL_START_BYTE, MessageCode
from src.app_control.base_dataclasses import ReadImuScaledMeasResponse
from src.uartcom.msg_parser import ProtocolSerialParser
from src.uartcom.msg_encoder import ProtocolEncoder

class TransportReadTimeOutError(Exception):
    pass 

class TransportError(Exception):
    pass 

class UartCommunicator:
    """
    Provides serial communication with the STM32 device.

    Handles UART port configuration, frame transmission, reception,
    buffering of incomplete data, and protocol parsing.
    """
    
    def __init__(self, 
                port = 'COM4', 
                baudrate =  115200,   
                # read timeout in secs
                timeout = 1.0,       
                # number of bytes for greatest possible frame (data/payload byte -> 0 .. 255)
                max_frame_size = 255 + 5, 
                # number of bytes for smallest possible frame (minimum one byte for DATA field)
                min_frame_size = 6,  
                # number of messages to hold in queue (with overwriting oldest!)
                msg_queue_size = 5) -> None:
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
        self.msg_parser = ProtocolSerialParser(max_frame_size, min_frame_size, msg_queue_size)
        self.msg_encoder = ProtocolEncoder()

    def write(self, msg_code: int, data: bytes):
        """
        Encode and transmit a protocol message over UART.

        Args:
            msg_code: Protocol message identifier.
            data: Message payload.
        """
        tx_msg: bytes = self.msg_encoder.build_message(msg_code, data)
        # print('Cmd tx message: ', tx_msg.hex(' '))
        self.serial_handler.write(tx_msg)

    def read(self) -> bytes:
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
        
        # parse new bytes from Serial RX buffer
        while(self.serial_handler.is_open):
            # some frames coulde be holded from last call
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
            
            # wait for one byte in serial port
            one_byte = self.serial_handler.read(1) # timeout 
            if not one_byte:
                raise TransportReadTimeOutError
            self.msg_parser.process_bytes(one_byte)
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
    
            # read all available  bytes in serial buffer  
            data: bytes = self.serial_handler.read(self.serial_handler.in_waiting)
            self.msg_parser.process_bytes(data)
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
               
        raise serial.SerialException('Port is closed!')


