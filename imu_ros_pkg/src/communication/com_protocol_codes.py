from enum import Enum, IntEnum, auto

PROTOTOL_START_BYTE  = 0x23 #b'\x23' or b'#'(35)


class ControlCommands(IntEnum):
    CHECK_DEVICE_STATUS = auto()
    HANDSHAKE           = auto()
    START_QUAT_MEAS     = auto()
    STOP_QUAT_MEAS      = auto()
    SET_REF_TIME        = auto()
    START_CALIBRATE     = auto()
    STOP_CALIBRATE      = auto()
    SET_CALIB_DATA      = auto()

class MessageCode(Enum):
    """Message identifiers used by the STM32 UART protocol."""
    # (HOST -> STM) Control and status msg codes (requests) 
    CHECK_DEVICE_STATUS = 0x08
    HANDSHAKE           = 0x09
    START_QUAT_MEAS     = 0x10      # Start quat measuring command
    STOP_QUAT_MEAS      = 0x11      # Stop quat measuring command
    SET_REF_TIME        = 0x12      # Set reference time based on host posix timestamp 
    START_CALIBRATE     = 0x14      # Start streaming imu scaled measurements command
    STOP_CALIBRATE      = 0x15      # Stop streaming imu scaled measurements command
    SET_CALIB_DATA      = 0x16      # Send calibration parameters for magnetometer
    # (STM -> HOST) Responce msg codes 
    ACK                 = 0x17
    NACK                = 0x18
    # HEARTBEAT         = 0x19
    IMU_SCALED_MEAS     = 0x42      # IMU measurement message (reserved ! not realised yet)
    QUAT_MEAS           = 0x43
    # ERROR_STATUS      = 0x13      # Error status message (reserved ! not realised yet)


# dict for correspondence of msg code and payload length (data len) byte
MsgCodePayloadDict = {
    # MessageCode.START_QUAT_MEAS.value:  46,
    MessageCode.START_QUAT_MEAS.value:    8,
    MessageCode.STOP_QUAT_MEAS.value:     1, 
    MessageCode.SET_REF_TIME.value:       8,
    # MessageCode.ERROR_STATUS.value:     8,
    MessageCode.START_CALIBRATE.value:    8,
    MessageCode.STOP_CALIBRATE.value:     1,
    MessageCode.SET_CALIB_DATA.value:     1
}

class ErrorCode(Enum):
    """Error codes used to report UART protocol errors."""

    NO_ERROR       = 0x00
    INC_START_BYTE = 0x01
    INC_MSG_CODE   = 0x02
    INC_DATA_LEN   = 0x03
    INC_CRC        = 0x04
    INC_FRAME_LEN  = 0x05
    