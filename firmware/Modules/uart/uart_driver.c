#include "uart_driver.h"

/* Global UART3 STATE */
volatile usart3_tx_state_t cur_usart3_tx_state;
volatile usart3_rx_state_t cur_usart3_rx_state;

/*  UART3 TX BUFFER */
uint8_t uart_tx_dma_buf[TX_BUF_SIZE] = {0};

/*  UART3 TX RING BUFFER */
volatile RingBuf tx_ring_buffer;

/*  UART3 RX BUFFER */
volatile uint8_t uart_rx_dma_buf[RX_BUF_SIZE] = {0};

/*  UART3 RX RING BUFFER */
volatile RingBuf rx_ring_buffer;

// Private functions declarations
void Timer10_Init(void);
void UartTX_Task();
uint8_t UartWrite_2_RxRingBuffer(void);


void TIM1_UP_TIM10_IRQHandler(void){
    
    if(TIM10->SR & TIM_SR_UIF){
        TIM10->SR &= ~TIM_SR_UIF;
        UartTX_Task();
    }
}


/*
 * Adds a complete frame to the TX ring buffer for transmission to the host.
 * First checks whether enough free space is available for the entire frame.
 * If there is insufficient space, the frame is rejected.
 *
 * If enough space is available, the frame bytes are copied into the ring
 * buffer and the enqueued frame counter is incremented.
 * If a write fails, LED3 indicates buffer saturation and the overflow
 * counter is updated.
 *
 * Returns 1 if the frame is accepted, or 0 if there is insufficient space.
 */

uint8_t UartWrite_2_TxRingBuffer(uint8_t *data, uint8_t data_len)
{
    if(rb_free_space(&tx_ring_buffer) < data_len)
        return 0;
    stats.tx_frames_enqueued += 1;
    for(uint8_t i = 0; i < data_len; i++){
        if (!rb_write_data(&tx_ring_buffer, data[i])){
            set_led(LED3); // visualize buffer saturation
            stats.tx_rb_full_state_count += 1;
        }
    }
    return 1;
}


/*
 * Checks whether the TX ring buffer contains data and the previous
 * DMA/USART TX transfer has completed.
 * If the buffer is empty or transmission is still in progress, returns.
 *
 * Otherwise, reads all available bytes (not frames!) from the TX ring buffer into
 * the DMA TX buffer and starts a new DMA transfer.
 * The TX state prevents starting a new transfer while the previous
 * one is in progress. It is updated by the DMA1 Stream3 interrupt
 * handler when the Transfer Complete (TC) flag is set.
 */
void UartTX_Task(){
    if (rb_is_empty(&tx_ring_buffer) || cur_usart3_tx_state == USART3_TX_TRANSMITING)
        return;
    stats.tx_dma_start_count++;

    cur_usart3_tx_state = USART3_TX_TRANSMITING;
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    uint8_t n_read = rb_read_all_data(uart_tx_dma_buf, &tx_ring_buffer);
    DMA1_Stream3->NDTR = n_read;
    __DSB();
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}


/*
 * UART RX DMA continuously receives data into a circular DMA buffer.
 * This function is called from DMA Half Transfer (HT), Transfer
 * Complete (TC), USART3 (IDLE) interrupt handlers to copy newly received bytes
 * into the software RX ring buffer.
 *
 * If the ring buffer becomes full, copying stops and the remaining
 * bytes are dropped.
 *
 * The function assumes that each DMA buffer segment is processed
 * before DMA overwrites its data.
 */

uint8_t UartWrite_2_RxRingBuffer(void)
{
    static uint32_t last_write_ind = 0;
    uint32_t new_write_ind = (RX_BUF_SIZE - DMA1_Stream1->NDTR) % RX_BUF_SIZE; // 0 .. RX_BUF_SIZE - 1
    if  (new_write_ind == last_write_ind) // dma rx buffer has not changed
        return 1;  
    
    if (new_write_ind > last_write_ind)
    {
        for (uint32_t i = last_write_ind; i < new_write_ind; ++i)
        {
            if (!rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i])){
                set_led(LED2); // visualize buffer saturation
                last_write_ind = new_write_ind;
                stats.rx_rb_full_state_count += 1;    
                stats.rx_bytes_dropped += new_write_ind - i;
                return 0;  
            }
        }
    }
    /* Received data wraped around usart_rx_dma_buffer -> need to copy 2 parts */
    else {
        // copy the farthest to the end of usart_rx_dma_buffer
        for (uint32_t i = last_write_ind; i < RX_BUF_SIZE; i++)
        {
            if (!rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i])){
                set_led(LED2); // visualize buffer saturation
                last_write_ind = new_write_ind;
                stats.rx_rb_full_state_count += 1;    
                stats.rx_bytes_dropped += RX_BUF_SIZE - i + new_write_ind;
                return 0;
            }
        }
        // copy the closest to the start of usart_rx_dma_buffer
        for (uint32_t i = 0; i < new_write_ind; i++)
        {
            if (!rb_write_data(&rx_ring_buffer, uart_rx_dma_buf[i])){
                set_led(LED2);
                last_write_ind = new_write_ind;
                stats.rx_rb_full_state_count += 1;    
                stats.rx_bytes_dropped += new_write_ind - i;
                return 0;     
            }
        }
    }

    last_write_ind = new_write_ind;
    return 1;  
}


/* DMA RX Stream 1 */
void DMA1_Stream1_IRQHandler(void)
{
    // Half Transfer Complete
    if (DMA1->LISR & DMA_LISR_HTIF1)
    {
        DMA1->LIFCR = DMA_LIFCR_CHTIF1;
        // Copying ...
        UartWrite_2_RxRingBuffer();
        // cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
    }
    // Transfer Complete
    if (DMA1->LISR & DMA_LISR_TCIF1)
    {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        // Copying ...
        UartWrite_2_RxRingBuffer();
        // cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
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

        stats.tx_dma_finish_count++;
    }
}

void USART3_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_IDLE){
        // Clear idle line flag
        USART3->ICR = USART_ICR_IDLECF;
        // Copying ...
        UartWrite_2_RxRingBuffer();
        // cur_usart3_rx_state = USART3_RX_DATA_RECEIVED;
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
    DMA1_Stream3->NDTR = RX_BUF_SIZE;
    
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

    /* Timer 10 for TX Tasks init */
    Timer10_Init();

    /* Inittialize ring buffers for RX and TX */
    rb_init(&rx_ring_buffer);
    rb_init(&tx_ring_buffer);
}


void Timer10_Init(void){
    // Enable TIM10 clock from APB2 (108 Mhz)
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    // Prescaler: 108 Mhz / 108 = 1 MHz 
    TIM10->PSC = 107;
    //AUto-reload: (UART_TX_PERIOD_MS / 1000) * Ftim_tick = ARR
    TIM10->ARR = UART_TX_PERIOD_MS * 1000;
    
    // Update Prescaler and ARR registers before start
    TIM10->EGR |= TIM_EGR_UG;
    TIM10->SR &= ~TIM_SR_UIF; //cleat flag!
    
    // Enable interrupts
    TIM10->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    
    // DO NOT Start timer in INIT!
    // TIM10->CR1 |= TIM_CR1_CEN; 
}

void USART3_Timer_Start(void){
    TIM10->CR1 |= TIM_CR1_CEN;
}

void USART3_Timer_Stop(void){
    TIM10->CR1 &= ~TIM_CR1_CEN;
    TIM10->SR &= ~TIM_SR_UIF; 
    TIM10->CNT = 0;
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

void transmit_protocol_msg_usart3(uint8_t *data, uint8_t data_len)
{
    cur_usart3_tx_state = USART3_TX_TRANSMITING;

    memcpy(uart_tx_dma_buf, data, data_len);

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;
    DMA1_Stream3->NDTR = data_len;
    __DSB();
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}


// void transmit_imu_sample_usart3(imu_sample_t *imu_m)
// {
//     cur_usart3_tx_state = USART3_TX_TRANSMITING;
//     uint8_t *p_acc = (uint8_t *)imu_m->accel;
//     uint8_t *p_gyro = (uint8_t *)imu_m->gyro;
//     uint8_t *p_mag = (uint8_t *)imu_m->mag;
//     uint8_t *p_time = (uint8_t *)&(imu_m->time_us);

//     uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
//     uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
//     uart_tx_dma_buf[2] = 0x2C;                // length of data = 44 bytes (36 for ac+gyro+mag and 8 for timestmp)

//     // fill tx_buffer with measurements: 3-14(12 bytes)
//     for (uint8_t i = 0; i < 12; i++)
//     {
//         uart_tx_dma_buf[3 + i] = *(p_acc + i);
//         uart_tx_dma_buf[15 + i] = *(p_gyro + i);
//         uart_tx_dma_buf[27 + i] = *(p_mag + i);
//     }
//     // timestamp
//     for (uint8_t i = 0; i < 8; i++)
//         uart_tx_dma_buf[39 + i] = *(p_time + i);

//     uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
//     // calc crc16 for saved bytes
//     uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
//     // save in big endian
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

//     __DSB();

//     DMA1_Stream3->CR &= ~DMA_SxCR_EN;
//     DMA1->LIFCR = DMA_LIFCR_CTCIF3;
//     // start(1) + cmd(1) + len(1) + data[40] + crc(2)
//     DMA1_Stream3->NDTR = num_bytes;
//     // Start transmitting
//     DMA1_Stream3->CR |= DMA_SxCR_EN;
// }

// void transmit_euler_orient_usart3(imu_euler_orient_t *eu)
// {
//     cur_usart3_tx_state = USART3_TX_TRANSMITING;
//     // start(1) + cmd(1) + len(1) + data[20] + crc(2)
//     uint8_t *p_roll = (uint8_t *)&eu->roll;
//     uint8_t *p_pitch = (uint8_t *)&eu->pitch;
//     uint8_t *p_yaw = (uint8_t *)&eu->yaw;
//     uint8_t *p_time = (uint8_t *)&eu->time_us;

//     uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
//     uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
//     uart_tx_dma_buf[2] = 0x14;                // length of data = 12 + 8 = 20
//     // measurements
//     for (uint8_t i = 0; i < 4; i++)
//     {
//         uart_tx_dma_buf[3 + i] = *(p_roll + i);
//         uart_tx_dma_buf[7 + i] = *(p_pitch + i);
//         uart_tx_dma_buf[11 + i] = *(p_yaw + i);
//     }
//     // timestamp
//     for (uint8_t i = 0; i < 8; i++)
//         uart_tx_dma_buf[15 + i] = *(p_time + i);

//     uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
//     // calc crc16 for saved bytes
//     uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
//     // save in big endian
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

//     __DSB();

//     DMA1_Stream3->CR &= ~DMA_SxCR_EN;
//     DMA1->LIFCR = DMA_LIFCR_CTCIF3;
//     DMA1_Stream3->NDTR = num_bytes;
//     DMA1_Stream3->CR |= DMA_SxCR_EN;
// }

// void transmit_quater_orient_usart3(imu_quater_orient_t *q)
// {
//     cur_usart3_tx_state = USART3_TX_TRANSMITING;
//     // start(1) + cmd(1) + len(1) + data[24] + crc(2)
//     uint8_t *p_w = (uint8_t *)&q->w;
//     uint8_t *p_x = (uint8_t *)&q->x;
//     uint8_t *p_y = (uint8_t *)&q->y;
//     uint8_t *p_z = (uint8_t *)&q->z;
//     uint8_t *p_time = (uint8_t *)&q->time_us;

//     uart_tx_dma_buf[0] = PROTOCOL_START_BYTE; // start byte
//     uart_tx_dma_buf[1] = IMU_MEAS;            // imu cmd code
//     uart_tx_dma_buf[2] = 0x18;                // length of data = 16 + 8 = 24
//     // measurements
//     for (uint8_t i = 0; i < 4; i++)
//     {
//         uart_tx_dma_buf[3 + i] = *(p_w + i);
//         uart_tx_dma_buf[7 + i] = *(p_x + i);
//         uart_tx_dma_buf[11 + i] = *(p_y + i);
//         uart_tx_dma_buf[15 + i] = *(p_z + i);
//     }
//     // timestamp
//     for (uint8_t i = 0; i < 8; i++)
//         uart_tx_dma_buf[19 + i] = *(p_time + i);

//     uint8_t num_bytes = 3 + uart_tx_dma_buf[2];
//     // calc crc16 for saved bytes
//     uint16_t crc16 = CRC16_Calculate((uint8_t *)uart_tx_dma_buf, num_bytes);
//     // save in big endian
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)((crc16 >> 8) & 0xFF); // high
//     uart_tx_dma_buf[num_bytes++] = (uint8_t)(crc16 & 0xFF);        // low

//     __DSB();

//     DMA1_Stream3->CR &= ~DMA_SxCR_EN;
//     DMA1->LIFCR = DMA_LIFCR_CTCIF3;
//     DMA1_Stream3->NDTR = num_bytes;
//     DMA1_Stream3->CR |= DMA_SxCR_EN;
// }
