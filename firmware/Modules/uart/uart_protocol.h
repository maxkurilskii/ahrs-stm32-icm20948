#ifndef UART_PARSER_H
#define UART_PARSER_H

#include "common.h"
#include "crc16.h"
#include "uart_driver.h"

/* -------------------------- PROTOCOL FRAME STRUCTURE -----------------------------*/
// | START BYTE | MSG_CODE BYTE | DATA LEN BYTE | ... DATA BYTES ... | CRC_H | CRC_L | 

#define PROTOCOL_START_BYTE     0x23            // 35 or b'#'
#define MAX_FRAME_SIZE          RX_BUF_SIZE
#define MIN_FRAME_SIZE          6               // Min 1 byte for data field (in some status msgs will be ignored) 

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

typedef enum{
    WAIT_START_BYTE,
    WAIT_MSG_CODE,
    WAIT_DATA_LEN,
    WAIT_DATA,
    WAIT_CRC_H,
    WAIT_CRC_L
}ParseState;


typedef struct{
    uint8_t payload[MAX_FRAME_SIZE - 5];
    message_code_t msg_code;
    uint8_t payload_len;
}PayloadInstance;


error_code_t parse_protocol_frame(PayloadInstance *pld, uint8_t data);
bool is_valid_msg_code(message_code_t v);

void encode_protocol_frame(uint8_t *frame, uint8_t *payload, message_code_t msg_code, uint8_t payload_len);

#endif