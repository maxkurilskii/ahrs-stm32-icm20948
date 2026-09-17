#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "common.h"

// small delay for dma tx(!) and rx to ensure 
// registers are set and IMU is ready
#define DMA_DELAY   100 // ~900 ns at 108 MHz

//pull down ncc/cs (chip select) pin - start of communication
#define SPI1_CS_LOW     GPIOA->BSRR = GPIO_BSRR_BR4
//pull up ncc/cs (chip select) pin - end of communication
#define SPI1_CS_HIGH    GPIOA->BSRR = GPIO_BSRR_BS4

typedef enum{
    SPI_FREE,
    SPI_READY,
    SPI_WRITING,
    SPI_READING,
    SPI_DATA_READY
} spi_state_t;

extern volatile spi_state_t cur_spi_state;
extern volatile uint8_t spi_bytes_received;

void SPI1_Init_All(void); // base initialization of spi 

// writing and reading fully blocking
void spi_write_blocking(uint8_t reg_add, uint8_t data);
void spi_read_blocking(uint8_t reg_add, uint8_t* result_buf, uint8_t byte_quant);

// writing and reading using dma
void spi_write_async(uint8_t reg_addr, uint8_t tx_byte);
void spi_read_async(uint8_t reg_addr, uint8_t byte_number);

// get received data from spi_rx_buffer
uint8_t* get_spi_received_data(void);

#endif

