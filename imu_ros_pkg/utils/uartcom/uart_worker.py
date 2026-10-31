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

from utils.uartcom.protocol_codes import PROTOTOL_START_BYTE, MessageCode
from utils.uartcom.base_dataclasses import ReadImuScaledMeasResponse
from utils.uartcom.msg_parser import ProtocolSerialParser, ParserQueueEmptyError, ParserCrc16Error
from utils.uartcom.msg_encoder import ProtocolEncoder
from utils.measlog.imu_csv_logger import ImuLogger

    

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
    
    def __init__(self, 
                port = 'COM4', 
                baudrate =  115200,   
                # read timeout in secs
                timeout = 1.0,       
                # timeout in secs to wait for responce before closing connection (depends on read timeout)
                max_try_timeout = 4,  
                # number of bytes for greates possible frame (data/payload byte -> 0 .. 255)
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
        self._max_try_timeout = max_try_timeout
        self.msg_parser = ProtocolSerialParser(max_frame_size, min_frame_size, msg_queue_size)
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
        
        # parse new bytes from Serial RX buffer
        try_counter = 0
        while(self.serial_handler.is_open):
            # some frames coulde be holded from last call
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
            
            # wait for one byte in serial port
            one_byte = self.serial_handler.read(1) # timeout 
            if not one_byte:
                try_counter += 1
                if try_counter * self._read_timeout >= self._max_try_timeout:
                    raise UartReadTimeout('Max read timeout was reached')
                continue
            try_counter = 0 # reset if connection established
            self.msg_parser.process_bytes(one_byte)
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
    
            # read all available  bytes in serial buffer  
            data: bytes = self.serial_handler.read(self.serial_handler.in_waiting)
            self.msg_parser.process_bytes(data)
            if self.msg_parser.is_msg_available(): 
                return self.msg_parser.get_message()
               
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
                msg_code = msg[1]
                if not msg_code == MessageCode.IMU_MEAS:
                    print(f"Expected IMU MEAS, but received: {MessageCode(msg_code)}")
                    return
                a_x, a_y, a_z,  \
                g_x, g_y, g_z,  \
                m_x, m_y, m_z,  \
                timestamp, _ = struct.unpack_from('<9f1Q1H', msg, offset=3)
                scaled_meas_logger.save_scaled_data(ReadImuScaledMeasResponse((a_x, a_y, a_z),
                                                                                (g_x, g_y, g_z), 
                                                                                (m_x, m_y, m_z),
                                                                                timestamp))
            except UartReadTimeout:
                raise
            except ParserQueueEmptyError:
                raise
            except ParserCrc16Error as e:
                print(e)
                continue
          
    except KeyboardInterrupt:
        print("Program stopped")
    except serial.SerialException as e:
        print(e)
    except (UartReadTimeout, ParserQueueEmptyError, ParserCrc16Error) as e:
        print(e)
    finally:
        scaled_meas_logger.flush_buffer()
        print('Data saved')     
                
  
if __name__ == "__main__":
    main()   
    