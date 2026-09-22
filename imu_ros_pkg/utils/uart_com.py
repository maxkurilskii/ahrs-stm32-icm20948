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
    IMU_MEAS      = 0x42  #any imu measurement
    START_CMD     = 0x10 #control cmd
    STOP_CMD      = 0x11  #control cmd
    CALIBRATE_CMD = 0x12  #control cmd   
    ERROR_STATUS  = 0x13 #works the same as acknowledge and notify about an error  


class ErrorCode(Enum):
    NO_ERROR       = 0x00
    INC_START_BYTE = 0x01
    INC_MSG_CODE   = 0x02
    INC_DATA_LEN   = 0x03
    INC_CRC        = 0x04
    INC_FRAME_LEN  = 0x05
    
class ParseStates(Enum):
    WAIT_START    = 0
    WAIT_MSG_CODE = 1
    WAIT_LEN      = 2
    WAIT_DATA     = 3
    WAIT_CRC_H    = 4 
    WAIT_CRC_L    = 5 


class ProtocolEncoder:
    def __init__(self) -> None:
        pass
         
    @staticmethod
    def build_message(msg_code: int, data: bytes) -> bytes:
        tx_frame = bytearray()
        tx_frame.extend([UartCommunicator.START_BYTE, 
                        msg_code, 
                        len(data),
                        *data])
        crc16_val = calculate_crc16(tx_frame)
        crc16_h = (crc16_val >> 8) & 0xFF
        crc16_l = crc16_val & 0xFF
        tx_frame.extend([crc16_h, crc16_l])
        return bytes(tx_frame)
        
        
    
class ProtocolSerialParser:    
    def __init__(self) -> None:
        #protocol bytes: [start, cmd, data_len_byte, data, crc_h, crc_l] 
        self._msg = bytearray() #var for collecting bytes of one frame msg 
        self._data_len_cntr = 0
        self._crc16_len_cntr = 2
        self._current_msg_state = ParseStates.WAIT_START

    def reset_all(self):
        self._msg = bytearray() 
        self._data_len_cntr = 0
        self._current_msg_state = ParseStates.WAIT_START
            
    def process_byte(self, raw_byte: int) -> Optional[bytes]:
        if  self._current_msg_state == ParseStates.WAIT_START:
            if raw_byte == UartCommunicator.START_BYTE:
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
    pass
          
class UartCommunicator:
    START_BYTE  = 35 #b'\x23' or b'#'
    MAX_TRY_TIME_SEC = 4 
    
    def __init__(self, port = 'COM4', baudrate =  115200, timeout = 1.0) -> None:
        self.serial_handler = serial.Serial(port = port, 
                                        baudrate = baudrate, 
                                        timeout = timeout, 
                                        write_timeout = 0.1) # 0.1 or None? 
        
        self._read_timeout = timeout
        self.msg_parser = ProtocolSerialParser()
        self.msg_encoder = ProtocolEncoder()
        self._read_remaining = bytearray()

    def uart_write_package(self, msg_code: int, data: bytes):
        tx_msg = self.msg_encoder.build_message(msg_code, data)
        print('Cmd tx message: ', tx_msg.hex(' '))
        self.serial_handler.write(tx_msg)

    def uart_read_package(self) -> bytes:
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
            one_byte = self.serial_handler.read(1)
            if not one_byte:
                try_counter += 1
                if try_counter * self._read_timeout >= self.MAX_TRY_TIME_SEC:
                    raise UartReadTimeout('Max read timeout was reached')
                continue
            try_counter = 0
            frame_msg = self.msg_parser.process_byte(one_byte[0])
            if frame_msg is not None: 
                return frame_msg
            
            data: bytes = self.serial_handler.read(self.serial_handler.in_waiting)
            for ind, new_byte in enumerate(data):
                frame_msg = self.msg_parser.process_byte(new_byte)
                if frame_msg is not None:
                    if ind != len(data) - 1: self._read_remaining = data[ind + 1:]
                    return frame_msg 
        raise serial.SerialException('Port is closed!')


def main():
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
                scaled_meas_logger.save_scaled_data(ReadImuScaledMeasResponce((a_x, a_y, a_z),
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
    # Script is supposed to read imu scaled meas to test logic of UartCommunicator 
    main()   
    