from enum import Enum
from dataclasses import dataclass, field
from typing import List, Union, Optional, Tuple


@dataclass(frozen=True)
class QuaternionMsgResponce:
    w: float
    x: float
    y: float
    z: float
    timestamp: int


@dataclass(frozen=True)
class ReadImuEulerCommand:
    #start(1) + cmd(1) + len(1) + data(40) + crc(2)   
    cmd_code: int = 66 #b'\x42'
    data_len: int = 16 #b'\x10'

@dataclass(frozen=True)
class ReadImuEulerResponce:    
    roll: float
    pitch: float
    yaw:  float
    timestamp: int

@dataclass(frozen=True)
class ReadImuScaledMeasCommand:
    #start(1) + cmd(1) + len(1) + data(40) + crc(2)   
    cmd_code: int = 66 #b'\x42'
    data_len: int = 40 #b'\x28'
    
@dataclass(frozen=True)
class ReadImuScaledMeasResponce:    
    accel_meas: Tuple[float, float, float]
    gyro_meas: Tuple[float, float, float]
    mag_meas:  Tuple[float, float, float]
    timestamp: int

@dataclass(frozen=True)
class ReadACC_GYRO:
    cmd_code: int = 66 #b'\x42'
    data_len: int = 24  #b'\x18'

@dataclass(frozen=True)
class ReadGYRO:
    cmd_code: int = 66 #b'\x42'
    data_len: int = 4 #b'\x04' 

@dataclass(frozen=True)
class ReadMAG:
    cmd_code: int = 66 #b'\x42'
    data_len: int = 12  #b'\x0C'

@dataclass(frozen=True)
class ReadADC:
    cmd_code: int = 65 #b'\x41'
    data_len: bytes =  b'\x02' #2
    
@dataclass(frozen=True)
class ReadTEMP:
    cmd_code: bytes = b'\x43' #66
    data_len: bytes =  b'\x04' #4