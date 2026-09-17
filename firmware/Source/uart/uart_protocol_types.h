#ifndef UART_PROTOCOL_TYPES_H
#define UART_PROTOCOL_TYPES_H
#include <stdint.h>

#define PROTOCOL_START_BYTE  0x23 //35 or b'#'

typedef enum{
    IMU_MEAS  = 0x42, // only one of the meas(raw/scaled/euler/quaternion) 
    START_CMD = 0x10,
    STOP_CMD  = 0x11,
    ACK       = 0x12,
    ERROR     = 0x13,
    CALIB_CMD = 0x14,   
}message_code_t ;

typedef enum{
    NO_ERROR,
    INC_START_BYTE,
    INC_MSG_CODE,
    INC_DATA_LEN,
    INC_CRC,
    INC_FRAME_LEN
} error_code_t;

typedef union{
    uint8_t raw_bytes[6];
    struct{
        uint8_t start_byte;
        uint8_t msg_code;
        uint8_t data_len;
        uint8_t data;
        uint8_t crc_h;
        uint8_t crc_l;
    }msg_byte;
}tx_ack_msg_t;


typedef union{
    uint8_t raw_bytes[6];
    struct{
        uint8_t start_byte;
        uint8_t msg_code;
        uint8_t data_len;
        uint8_t err_code;
        uint8_t crc_h;
        uint8_t crc_l;
    }msg_byte;
}tx_error_msg_t;


#endif