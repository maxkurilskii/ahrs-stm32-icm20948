from enum import Enum

PROTOTOL_START_BYTE  = 0x23 #b'\x23' or b'#'(35)


class MessageCode(Enum):
    """Message identifiers used by the STM32 UART protocol."""

    IMU_MEAS        = 0x42      # IMU measurement message
    START_CMD       = 0x10      # Start command
    STOP_CMD        = 0x11      # Stop command
    CALIBRATE_CMD   = 0x12      # Calibration command
    ERROR_STATUS    = 0x13      # Error status message
    UPD_TIME_CMD    = 0x15      # Update host time on stm

# dict for correspondence of msg code and payload length (data len) byte
MsgCodePayloadDict = {
    MessageCode.IMU_MEAS.value:         46,
    MessageCode.START_CMD.value:        8,
    MessageCode.STOP_CMD.value:         1, 
    MessageCode.UPD_TIME_CMD.value:     8,
    MessageCode.CALIBRATE_CMD.value:    8,
    MessageCode.ERROR_STATUS.value:     1
}

class ErrorCode(Enum):
    """Error codes used to report UART protocol errors."""

    NO_ERROR       = 0x00
    INC_START_BYTE = 0x01
    INC_MSG_CODE   = 0x02
    INC_DATA_LEN   = 0x03
    INC_CRC        = 0x04
    INC_FRAME_LEN  = 0x05
    