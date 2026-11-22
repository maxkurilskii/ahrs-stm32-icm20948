from enum import Enum

PROTOTOL_START_BYTE  = 0x23 #b'\x23' or b'#'(35)

class MessageCode(Enum):
    """Message identifiers used by the STM32 UART protocol."""
    # Control and status msg codes
    START_QUAT_MEAS     = 0x10      # Start quat measuring command
    STOP_QUAT_MEAS      = 0x11      # Stop quat measuring command
    SET_REF_TIME        = 0x12      # Set reference time based on host posix timestamp 
    # ERROR_STATUS      = 0x13      # Error status message (reserved ! not realised yet)
    START_CALIBRATE     = 0x14      # Start streaming imu scaled measurements command
    STOP_CALIBRATE      = 0x15      # Stop streaming imu scaled measurements command
    SET_CALIB_DATA      = 0x16      # Send calibration parameters for magnetometer
    ACK                 = 0x17
    NACK                = 0x19
    # Responce msg codes
    IMU_SCALED_MEAS     = 0x42      # IMU measurement message (reserved ! not realised yet)
    QUAT_MEAS           = 0x43

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
    