#include "uart_driver.h"

/*  UART3 TX BUFFER */
uint8_t uart_tx_dma_buf[TX_BUF_SIZE] = {0};

/*  UART3 RX BUFFER */
volatile uint8_t uart_rx_dma_buf[RX_BUF_SIZE] = {0};

/*  UART3 RX RING BUFFER */
volatile RingBuf rx_ring_buffer;

/* Global UART3 STATE */
volatile usart3_tx_state_t cur_usart3_tx_state;
volatile usart3_rx_state_t cur_usart3_rx_state;
volatile uint8_t received_len = 0;


void usart_write_to_ring_buffer()
{
    static uint32_t last_write_ind = 0;
    uint32_t new_write_ind = RX_BUF_SIZE - DMA1_Stream1->NDTR; // should be always >= 0
    if  (new_write_ind == last_write_ind)
        return;

    if (new_write_ind > last_write_ind)
    {
        for (uint32_t i = last_write_ind; i < new_write_ind; ++i)
        {
            rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i]);
        }
    }
    /* Received data wraped around usart_rx_dma_buffer -> need to copy 2 parts */
    else if (new_write_ind < last_write_ind)
    {
        // copy the farthest to the end of usart_rx_dma_buffer
        for (uint32_t i = last_write_ind; i < RX_BUF_SIZE; i++)
        {
            rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i]);
        }
        // copy the closest to the start of usart_rx_dma_buffer
        for (uint32_t i = 0; i < new_write_ind; i++)
        {
            rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i]);
        }
    }

    last_write_ind = new_write_ind;
}

volatile uint32_t ndtr;

/* DMA RX Stream 1 */
void DMA1_Stream1_IRQHandler(void)
{
    // Half Transfer Complete
    if (DMA1->LISR & DMA_LISR_HTIF1)
    {
        DMA1->LIFCR = DMA_LIFCR_CHTIF1;
        // Copying ...
        usart_write_to_ring_buffer();

        cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
    }
    // Transfer Complete
    if (DMA1->LISR & DMA_LISR_TCIF1)
    {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        // Copying ...
        usart_write_to_ring_buffer();
        cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
    }
}

/* DMA TX stream 3 */
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF3)
    {
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream3->CR & DMA_SxCR_EN)
            ;
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;
        cur_usart3_tx_state = USART3_TX_FREE;
    }
}

void USART3_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_IDLE)
    {

        // Clear idle line flag
        USART3->ICR = USART_ICR_IDLECF;

        // Update USART RX FSM flag
        cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;

        // DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        // while(DMA1_Stream1->CR & DMA_SxCR_EN);
        // DMA1->LIFCR = DMA_LIFCR_CTCIF1;

        // Copying ...
        usart_write_to_ring_buffer();

        cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
    }
}

uint8_t *get_usart_rx_data(void)
{
    return (uint8_t *)uart_tx_dma_buf;
}

void USART3_Init(void)
{

    /* ---------------- DMA1 for USART3 init -------------- */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Disable DMA1 Stream1 for TX, DMA1 Stream1 for RX dur config
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN)
        ;
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream1->CR & DMA_SxCR_EN)
        ;

    // Enable usart3 TX channel 4 stream 3; RX channel 4 stream 1
    DMA1_Stream3->CR |= 4U << DMA_SxCR_CHSEL_Pos;
    DMA1_Stream1->CR |= 4U << DMA_SxCR_CHSEL_Pos;

    /* TX congif: MSize = PSize = 8 bit; Mem incr enabled; mem -> perif; TC int enable */
    DMA1_Stream3->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE | (1U << DMA_SxCR_DIR_Pos);
    DMA1_Stream3->PAR = (uint32_t)&(USART3->TDR);
    DMA1_Stream3->M0AR = (uint32_t)uart_tx_dma_buf;
    DMA1_Stream3->NDTR = X_BUF_SIZE;
    
    /* RX config:  MSize = PSize = 8 bit; Mem inc; Perif->Mem;
        CIRC mode: data wraps around automatically (end -> start)
        HT/TC interrupts: notify app to save data before buffer overwrite */
    DMA1_Stream1->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_HTIE;
    DMA1_Stream1->PAR = (uint32_t)&(USART3->RDR);
    DMA1_Stream1->M0AR = (uint32_t)uart_rx_dma_buf; // pointer to array, not stucture
    DMA1_Stream1->NDTR = RX_BUF_SIZE;

    // Globally enable DMA TX and RX Interrupts
    // TODO:  NVIC_SetPriority(); for DMA RX and TX(?)
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* ------------- USART3 initialization ---------------- */

    // Enable USART3 on APB1(54 Mhz)
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    // Enable clock on GPIOD from AHB1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    // Pin GPIOD8 (USART3_TX) connected to ST-LINK: alt mode + pull up + no push-pull
    GPIOD->MODER |= 2U << GPIO_MODER_MODER8_Pos;
    GPIOD->AFR[1] |= 7U << GPIO_AFRH_AFRH0_Pos;
    // Pin GPIOD9 (USART3_RX) connected to ST-LINK: alt mode + pull up + no push-pull
    GPIOD->MODER |= 2U << GPIO_MODER_MODER9_Pos;
    GPIOD->AFR[1] |= 7U << GPIO_AFRH_AFRH1_Pos;

    // Configures USART for 115200 baud rate with oversampling = 16 (over8 = 0):
    // 54000000/(16 * 115200) = 29.29687 -> matisa = 29; frac = 0.29 * 16 = 4.75 = 5
    USART3->BRR = (29U << USART_BRR_DIV_MANTISSA_Pos) | (5U << USART_BRR_DIV_FRACTION_Pos);

    // Configures USART for 921600 baud rate with oversampling = 16 (over8 = 0)::
    // 54000000/(16 * 921600) = 3.66 -> matisa = 3; frac = 0.66 * 16 ~ 11
    // USART3->BRR = (3U << USART_BRR_DIV_MANTISSA_Pos) | (11U << USART_BRR_DIV_FRACTION_Pos);

    USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // use dma for tx and rx

    // Word len = 8bit (M1=M2=0) + enable UART3 transmit and receive
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;

    // Globally enable USART3 Interrupts
    // TODO:  NVIC_SetPriority(); for USART IDLE
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    // Enable DMA RX before USART3
    DMA1_Stream1->CR |= DMA_SxCR_EN;
    // Turn on USART3
    USART3->CR1 |= USART_CR1_UE;

    // Wait for USART TX/RX enable ACK (TEACK/REACK) (on HW level)
    while (!(USART3->ISR & (USART_ISR_TEACK | USART_ISR_REACK)))
        ;

    // set TX and RX FSM flags
    cur_usart3_tx_state = USART3_TX_FREE;
    cur_usart3_rx_state = USART3_RX_FREE;

    /* Inittialize ring buffer for RX */
    rb_init(&rx_ring_buffer);
    
}

void transmit_byte_usart3(uint8_t data)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN)
        ;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = 1;
    uart_tx_dma_buf[0] = data;

    __DSB();
    // SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, 1);
    // for(uint8_t i = 0; i < 20; i++) __NOP(); // ~180 ns
    // Start transmitting
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void transmit_byte_usart3_debug(uint8_t data)
{
    // blocking function
    while (!(USART3->ISR & USART_ISR_TXE))
        ;
    USART3->TDR = data;
    while (!(USART3->ISR & USART_ISR_TC))
        ;
}

void transmit_data_usart3(uint8_t *data_buffer, uint8_t len)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN)
        ;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = len;
    memcpy((uint8_t *)uart_tx_dma_buf, data_buffer, len);
    __DSB();
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void transmit_control_msg_usart3(uint8_t msg_code, uint8_t msg_data)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    uint8_t ack_msg[6] = {PROTOCOL_START_BYTE,
                          msg_code,
                          0x01,
                          msg_data,
                          0x00, 0x00}; /* crc part  */
    uint16_t crc16_val = CRC16_Calculate(ack_msg, 4);
    ack_msg[4] = (uint8_t)(crc16_val >> 8); // crc_h
    ack_msg[5] = (uint8_t)(crc16_val);      // crc_l
    memcpy(uart_tx_dma_buf, ack_msg, 6);

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = 6;
    __DSB();
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void transmit_imu_sample_usart3(imu_sample_t *imu_m)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    uint8_t *p_acc = (uint8_t *)imu_m->accel;
    uint8_t *p_gyro = (uint8_t *)imu_m->gyro;
    uint8_t *p_mag = (uint8_t *)imu_m->mag;
    uint8_t *p_time = (uint8_t *)&(imu_m->time_us);

    uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
    uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
    uart_tx_dma_buf[2] = 0x2C;                // length of data = 44 bytes (36 for ac+gyro+mag and 8 for timestmp)

    // fill tx_buffer with measurements: 3-14(12 bytes)
    for (uint8_t i = 0; i < 12; i++)
    {
        uart_tx_dma_buf[3 + i] = *(p_acc + i);
        uart_tx_dma_buf[15 + i] = *(p_gyro + i);
        uart_tx_dma_buf[27 + i] = *(p_mag + i);
    }
    // timestamp
    for (uint8_t i = 0; i < 8; i++)
        uart_tx_dma_buf[39 + i] = *(p_time + i);

    uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
    // calc crc16 for saved bytes
    uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
    // save in big endian
    uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
    uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

    __DSB();

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    // start(1) + cmd(1) + len(1) + data[40] + crc(2)
    DMA1_Stream3->NDTR = num_bytes;
    // Start transmitting
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void transmit_euler_orient_usart3(imu_euler_orient_t *eu)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    // start(1) + cmd(1) + len(1) + data[20] + crc(2)
    uint8_t *p_roll = (uint8_t *)&eu->roll;
    uint8_t *p_pitch = (uint8_t *)&eu->pitch;
    uint8_t *p_yaw = (uint8_t *)&eu->yaw;
    uint8_t *p_time = (uint8_t *)&eu->time_us;

    uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
    uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
    uart_tx_dma_buf[2] = 0x14;                // length of data = 12 + 8 = 20
    // measurements
    for (uint8_t i = 0; i < 4; i++)
    {
        uart_tx_dma_buf[3 + i] = *(p_roll + i);
        uart_tx_dma_buf[7 + i] = *(p_pitch + i);
        uart_tx_dma_buf[11 + i] = *(p_yaw + i);
    }
    // timestamp
    for (uint8_t i = 0; i < 8; i++)
        uart_tx_dma_buf[15 + i] = *(p_time + i);

    uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
    // calc crc16 for saved bytes
    uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
    // save in big endian
    uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
    uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

    __DSB();

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = num_bytes;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void transmit_quater_orient_usart3(imu_quater_orient_t *q)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    // start(1) + cmd(1) + len(1) + data[24] + crc(2)
    uint8_t *p_w = (uint8_t *)&q->w;
    uint8_t *p_x = (uint8_t *)&q->x;
    uint8_t *p_y = (uint8_t *)&q->y;
    uint8_t *p_z = (uint8_t *)&q->z;
    uint8_t *p_time = (uint8_t *)&q->time_us;

    uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
    uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
    uart_tx_dma_buf[2] = 0x18;                // length of data = 16 + 8 = 24
    // measurements
    for (uint8_t i = 0; i < 4; i++)
    {
        uart_tx_dma_buf[3 + i] = *(p_w + i);
        uart_tx_dma_buf[7 + i] = *(p_x + i);
        uart_tx_dma_buf[11 + i] = *(p_y + i);
        uart_tx_dma_buf[15 + i] = *(p_z + i);
    }
    // timestamp
    for (uint8_t i = 0; i < 8; i++)
        uart_tx_dma_buf[19 + i] = *(p_time + i);

    uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
    // calc crc16 for saved bytes
    uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
    // save in big endian
    uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
    uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

    __DSB();

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = num_bytes;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}
