#ifndef UART_PARSER_H
#define UART_PARSER_H

#include "common.h"
#include "uart_protocol_types.h"
#include "uart_driver.h"

#define MAX_FRAME_SIZE    RX_BUF_SIZE
// TODO: надо оли подкбчать явно ? илид остаточно тогоЮ чтоо н в uart_driver уже есть как и uart_protocl_types.h
// #include "ring_buf.h" 


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

#endif