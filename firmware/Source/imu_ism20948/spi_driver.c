#include "spi_driver.h"

volatile spi_state_t cur_spi_state = SPI_FREE;

uint8_t spi_tx_buffer[30] = {0};
volatile uint8_t spi_rx_buffer[30] = {0};

/* -------- Static functions -------- */
static void transmit_byte_spi(uint8_t data);
static void dma_clear_flags(void);
static void DMA2_SPI1_Init(void);

/* -------- "Public" functions -------- */

void DMA2_Stream0_IRQHandler(void){
    if (DMA2->LISR & DMA_LISR_TCIF0){
        //while (SPI1->SR & SPI_SR_BSY);
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        //dma_clear_flags();
        SPI1_CS_HIGH; 
        cur_spi_state = SPI_DATA_READY;
    }
}


uint8_t* get_spi_received_data(void){
    return (uint8_t*)&spi_rx_buffer[1];
}


void SPI1_Init_All(void){
    DMA2_SPI1_Init();     
    /* ---- SPI1 Configuration ---- */

    // Enable SPI1 clock  from APB2
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    //Set mosi(GPIOA7), miso(GPIOA6), sclk(GPIOA5) pins in alt mode, except nss(chip select) pin
    //Enable GPIOA clock from AHB1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //config gpio for spi1 pins
    GPIOA->MODER |= 2U << GPIO_MODER_MODER5_Pos; //set sclk in alt mode (ob10)
    GPIOA->AFR[0] |= 5U << GPIO_AFRL_AFRL5_Pos; //define af5 for gpio5(0b101)
    GPIOA->MODER |= 2U << GPIO_MODER_MODER6_Pos; //set miso in alt mode (ob10)
    GPIOA->AFR[0] |= 5U << GPIO_AFRL_AFRL6_Pos; //define af5 for gpio6(0b101)
    GPIOA->MODER |= 2U << GPIO_MODER_MODER7_Pos; //set mosi in alt mode (ob10)
    GPIOA->AFR[0] |= 5U << GPIO_AFRL_AFRL7_Pos; //define af5 for gpio7(0b101)
    //select GPIOA4 as nss pin, and set in OUTPUT_PUSH_PULL mode
    GPIOA->MODER |= 1U << GPIO_MODER_MODER4_Pos; //set nss in output mode (ob01)
    
    // No start spi com during init
    SPI1_CS_HIGH;
    for(uint8_t i= 0; i < 3; i++) __NOP();
    
    // Software slave mgmnt(ssm = 1, ssi= 1) + master selection  + cpha and cpol are def 
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;

    // Clock prescaler: 108 Mhz (APB2) / 32  (0b100) =  3.375 MHz
    SPI1->CR1 |= (4U << SPI_CR1_BR_Pos); 
    //SPI1->CR1 |= (7U << SPI_CR1_BR_Pos); //~421 kHz
    
    // Data size (DS) = 8 bit (def), thres of SPI_RX_FIFO to 8 bit
    SPI1->CR2 |=  SPI_CR2_FRXTH;
    
    //Enable SPI1 to send TX and RX requests to DMA 
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    //Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}



void spi_write_async(uint8_t reg_addr, uint8_t tx_byte){
    /*
    Implements single-byte SPI write using async DMA. 
    Blocks at the end with CPU polling to wait for transfer completion.
    */
    cur_spi_state = SPI_WRITING;
    //DMA2 Tx Stream3 and RX Stream0 must be disabled during reconfig
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream3->CR & DMA_SxCR_EN);
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    dma_clear_flags();
    
    /*config num of data that would be send and read*/
    DMA2_Stream3->NDTR = 2; //address byte + data bytes
    DMA2_Stream0->NDTR = 2; //junk bytes
    
    spi_tx_buffer[0] = reg_addr;
    spi_tx_buffer[1] = tx_byte;
    __DSB(); 
    
    //SPI start comm sequence (alr should be SPI_EN = 1, SPI_RXDMA=SPI_TXDMA=1)
    SPI1_CS_LOW; //start spi com
    //small delay to ensure registers are set and IMU is ready
    for(uint8_t i = 0; i < DMA_DELAY; i++) __NOP(); 

    DMA2_Stream0->CR |= DMA_SxCR_EN; //dma2 str0 is ready for rx transactions (listens to rx request)
    DMA2_Stream3->CR |= DMA_SxCR_EN; //dma2 str3 is ready for tx transactions (listens to tx request)
    while(cur_spi_state != SPI_DATA_READY);    
}



void spi_read_async(uint8_t reg_addr, uint8_t byte_number){
    /* 
    Multi-byte asynchronous SPI read using DMA.
    Short CPU polling at the end exist to ensure data 
    is latched and IMU (slave) is ready for next transfer
    */
    cur_spi_state =  SPI_READING;    
    //DMA2 Tx Stream3 and RX Stream0 must be disabled during reconfig
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream3->CR & DMA_SxCR_EN);
    dma_clear_flags();
    
    //config num of data that would be send and read (SPI DR should be read to clear RX_FIFO before next transfer)
    DMA2_Stream3->NDTR = byte_number + 1; //address byte + data bytes
    DMA2_Stream0->NDTR = byte_number + 1; //junk bytes
    
    spi_tx_buffer[0] = reg_addr | 0x80;
    __DSB();
    
     //SPI start comm sequence (alr should be SPI_EN = 1, SPI_RXDMA=SPI_TXDMA=1)
    SPI1_CS_LOW; 
    for(uint8_t i = 0; i < DMA_DELAY; i++) __NOP(); //little delay for slave 
    DMA2_Stream0->CR |= DMA_SxCR_EN; //dma2 str0 is ready for rx transactions (listens to rx request)
    DMA2_Stream3->CR |= DMA_SxCR_EN; //dma2 str3 is ready for tx transactions (listens to tx request)
}

/* ------------------------------------------- */


static void transmit_byte_spi(uint8_t tx_byte){
	while (!(SPI1->SR & SPI_SR_TXE));
	*(volatile uint8_t *)&SPI1->DR = tx_byte;
}


static void dma_clear_flags(void){
    DMA2->LIFCR =
          DMA_LIFCR_CFEIF0
        | DMA_LIFCR_CDMEIF0
        | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0
        | DMA_LIFCR_CTCIF0

        | DMA_LIFCR_CFEIF3
        | DMA_LIFCR_CDMEIF3
        | DMA_LIFCR_CTEIF3
        | DMA_LIFCR_CHTIF3
        | DMA_LIFCR_CTCIF3;
}

static void DMA2_SPI1_Init(void){
    /* ---- Config DMA for SPI1 ---- */
    // Enable DMA2 streams for SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    // DMA2 TX(Stream3) and RX(Stream0)
    DMA2_Stream0->CR &= ~DMA_SxCR_EN; //disable stream0 during config
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream3->CR &= ~DMA_SxCR_EN; //disable stream3 during config
    while(DMA2_Stream3->CR & DMA_SxCR_EN);
    dma_clear_flags();
    
    //Select channel 3 for both streams 
    DMA2_Stream0->CR |= (3U << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream3->CR |= (3U << DMA_SxCR_CHSEL_Pos);
    
    //MSIZE = PSIZE = 8 bit (def 0x00); Priority level(PL) - High (0b10)
    DMA2_Stream0->CR |= DMA_SxCR_PL_1;
    DMA2_Stream3->CR |= DMA_SxCR_PL_0;
    
    //Rx: perif->memory direction (def) + mem incrementation (MINC)
    DMA2_Stream0->CR |= DMA_SxCR_MINC;
    //Tx: memory->perif direction + mem incrementation (MINC)
    DMA2_Stream3->CR |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC;
    
    //select mem address for dma TX transfer
    DMA2_Stream3->M0AR = (uint32_t)spi_tx_buffer;
    // Select mem address for dma RX transfer 
    DMA2_Stream0->M0AR = (uint32_t)spi_rx_buffer;
    
    // Set peripheral addresses
    DMA2_Stream0->PAR = (uint32_t)(volatile uint8_t*)&SPI1->DR; // take only LSB of spi data reg ADDRESS(!)
    DMA2_Stream3->PAR = (uint32_t)(volatile uint8_t*)&SPI1->DR; // take only LSB of spi data reg ADDRESS(!)
    
    //Enable interrupts only for RX Stream
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;
    NVIC_EnableIRQ(DMA2_Stream0_IRQn); //global interrupt enable
    
    //DO NOT ENABLE ANY DMA STREAM IN INITIALIZATION !!!   
}





