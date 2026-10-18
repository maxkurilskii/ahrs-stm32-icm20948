#ifndef UART_PROTOCOL_TYPES_H
#define UART_PROTOCOL_TYPES_H
// #include <stdint.h>

/* -------------------------- PROTOCOL FRAME STRUCTURE -----------------------------*/
// | START BYTE | MSG_CODE BYTE | DATA LEN BYTE | ... DATA BYTES ... | CRC_H | CRC_L | 

// Min bytes per frame: 5 (data can be left -> status msg: )

#define PROTOCOL_START_BYTE  0x23 //35 or b'#'


typedef enum{
    START_CMD    = 0x10,   // (request msg) start publishing imu measurements (only one of: raw/scaled/euler/quaternion) 
    STOP_CMD     = 0x11,   // (request msg) stop publishing imu measurements
    IMU_MEAS     = 0x42,   // (responce msg) 
    ERROR_ST     = 0x13,   // (responce msg) error status
    CALIB_CMD    = 0x14,   // (request msg) calibrate magnetometer (and gyroscope) + start publishing scaled/raw imu measurements 
    UPD_TIME_CMD = 0x15    // (request msg) update host (master ref) timestamp var included in cmd message  
}message_code_t;

typedef enum{
    NO_ERROR,           // 0 used as acknowledge msg (tests)
    INC_START_BYTE,
    INC_MSG_CODE,
    INC_DATA_LEN,
    INC_CRC,
    INC_FRAME_LEN,
    INC_FRAME_CASE,
    NOT_ENOUGH_DATA
} error_code_t;


#endif
