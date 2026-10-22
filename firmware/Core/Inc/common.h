#ifndef COMMON_H
#define COMMON_H

#include "stm32f767xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "system_clock_init.h" 
#include "led_unit.h"

#define SYS_TICK_PERIOD_US     90

typedef union{
    uint8_t  bytes[8];
    uint64_t timestamp;
} PosixTimestamp;


/*
 * RX statistics:
 * rx_frames_received       - number of valid protocol frames successfully received from the host.
 * rx_rb_full_state_count   - number of times the RX ring buffer was found full
 *                            during attempts to write incoming data.
 * rx_bytes_dropped         - number of bytes dropped when copying data from the DMA RX buffer
 *                            to the RX ring buffer because the ring buffer was full.
 * parse_hard_errors        - number of parsing errors classified as hard errors.
 *                            Currently, this counter does not distinguish between error types.
 *
 * TX statistics:
 * tx_frames_enqueued      - number of frames successfully enqueued into the TX ring buffer
 *                            for subsequent UART DMA transmission.
 * tx_dma_start_count      - number of UART TX DMA transfers started.
 * tx_dma_finish_count     - number of UART TX DMA transfers completed.
 *                            Should match tx_dma_start_count if every started transfer completes.
 *                            Counts DMA transfers, not protocol frames.
 * tx_rb_full_state_count  - number of times the TX ring buffer was found full
 *                           during attempts to enqueue outgoing data.
 * Test execution statistics:
 * rx_loop_enters - number of iterations of the receive-test loop during burst testing.
 * loads_cnt      - number of times another task was executed while the receive-test loop was running.
 *                  Used to evaluate how many executions of other tasks were performed
 *                  during RX processing.
 */
typedef struct 
{
    uint16_t rx_frames_received;          
    uint16_t rx_rb_full_state_count;    
    uint16_t rx_bytes_dropped;             
    uint16_t parse_hard_errors;            
    uint16_t tx_frames_enqueued;           
    uint16_t tx_dma_start_count;           
    uint16_t tx_dma_finish_count;         
    uint16_t tx_rb_full_state_count;     
    uint16_t rx_loop_enters;
    uint16_t loads_cnt;

}TestStats;

extern TestStats stats;


void delay_ms(uint16_t milis);
//extern volatile uint32_t msCounter;
extern volatile uint32_t usCounter;
#endif

