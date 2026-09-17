#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "common.h"
#include "uart_protocol_types.h"
#include "imu_ism20948.h"
#include "filter_proces.h"
#include "crc16.h"

#define UART_TX_PERIOD_MS 20
#define RX_BUF_SIZE  100

typedef enum{
    USART3_FREE,
    USART3_READY,
    USART3_TRANSMITING,
    USART3_DATA_RECEIVED
}usart3_state_t;

extern volatile usart3_state_t cur_usart3_state;

void USART3_Init(void);

void dma_clear_flags(void);

void transmit_byte_usart3(uint8_t data);
void transmit_data_usart3(uint8_t* data_buffer, uint8_t len);
void transmit_imu_sample_usart3(imu_sample_t* imu_meas);
void transmit_euler_orient_usart3(imu_euler_orient_t* eu);
void transmit_quater_orient_usart3(imu_quater_orient_t* q);

void transmit_byte_usart3_debug(uint8_t data);

uint8_t* get_usart_rx_data(void);
uint8_t parse_protocol_frame(void);
uint8_t GetUsartParsedMsgCode(void);

void transmit_control_msg_usart3(uint8_t msg_code, uint8_t msg_data);

#endif
