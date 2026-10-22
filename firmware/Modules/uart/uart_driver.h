#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "common.h"
#include "ring_buf.h"

#define UART_TX_PERIOD_MS   1
#define TX_BUF_SIZE         100
#define RX_BUF_SIZE         100                                                                                                                  

typedef enum{
    USART3_TX_FREE,            // after transmition of data (DMA irq resets)
    USART3_TX_TASKED,          // enough data was collected to be transmitted (not used)
    USART3_TX_TRANSMITING,     // transmission in progress 
}usart3_tx_state_t;

typedef enum{
    USART3_RX_FREE,            // init state and after data is processed in APP
    // USART3_RX_RECEIVING,       // In progress of copying data in on of the interrupts (HC, TC, IDLE)
    USART3_RX_DATA_RECEIVED,   // USART IDLE irq sets 
}usart3_rx_state_t;


/* Global UART3 STATE */
extern volatile usart3_tx_state_t cur_usart3_tx_state;
extern volatile usart3_rx_state_t cur_usart3_rx_state;

/* Extern Ring Buffers */
extern volatile RingBuf rx_ring_buffer;
extern volatile RingBuf tx_ring_buffer;

void USART3_Timer_Start(void);
void USART3_Timer_Stop(void);
void USART3_Init(void);

void dma_clear_flags(void);

void transmit_byte_usart3_debug(uint8_t data);
void transmit_byte_usart3(uint8_t data);
void transmit_data_usart3(uint8_t* data_buffer, uint8_t len);
void transmit_protocol_msg_usart3(uint8_t *data, uint8_t data_len);

// void transmit_imu_sample_usart3(imu_sample_t* imu_meas);
// void transmit_euler_orient_usart3(imu_euler_orient_t* eu);
// void transmit_quater_orient_usart3(imu_quater_orient_t* q);

uint8_t* get_usart_rx_data(void);
uint8_t GetUsartParsedMsgCode(void);

void transmit_control_msg_usart3(uint8_t msg_code, uint8_t msg_data);

uint8_t UartWrite_2_RxRingBuffer(void);
uint8_t UartWrite_2_TxRingBuffer(uint8_t *data, uint8_t data_len);

#endif
