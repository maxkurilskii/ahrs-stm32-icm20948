#include "common.h"
#include "uart_driver.h"
#include "uart_parser.h"
#include "ism20948_driver.h"
#include "data_prepr.h"

#define IMU_RAW_MEAS                0
#define IMU_SCALED_MEAS             1
#define EULER_ORIENTATION           2
#define QUATERNION_ORIENTATION      3    

#define SEND_MODE   IMU_SCALED_MEAS

// volatile uint32_t msCounter = 0;
uint32_t start_delay_us = 0;
volatile uint32_t usCounter = 0;

uint16_t tmp_rx_buffer = 0;
uint8_t mag_device_id = 0;

void SysTick_Handler(void)
{
    usCounter++; //~90 us for one period
}

void delay_ms(uint16_t millis)
{
    start_delay_us = usCounter;
    while ((usCounter - start_delay_us) * SYS_TICK_PERIOD_US < millis * 1000)
    {
        __NOP();
    }
}



int main(void)
{
    SysClockInit();
    SysTickInit();
    
    LEDs_Init();
    
    USART3_Init();

    IMU20948_Init();
    Madgwick_Filter_Init();

    // delay_ms(1000);
    uint64_t master_ref_time = 0;
    uint32_t stm_ref_us_ticks = 0;

    /* Parsed payload instance from received msg */
    PayloadInstance pld; 

    /* Flag to launch any imu measurements -> orientation estimation */
    uint8_t imu_running = 0;
    /* Used to control when to send imu meas / orientation over uart */
    uint8_t imu_sample_cnt = 0;


    imu_sample_t imu_sample = {.accel = {0.5, 0.5, 1.0},
                                .gyro = {1.0, -1.0, 0.0},
                                .mag = {10.0, -1.0, 33.0},
                                .time_us = 0}; 

    while (1) {

        /* ---------------------------------------------------------------- */
        //  CMD LISTENING PART  
        /* ---------------------------------------------------------------- */


        /* Check if new cmd has arrived */
        while (!rb_is_empty(&rx_ring_buffer)){
            // save current next_write_ind (snapshot) 
            uint32_t nwi_snap = rx_ring_buffer.next_write_ind; 
            int n_available = rb_available(&rx_ring_buffer, nwi_snap);
            if (n_available < 0)
                continue; // sanity case 

            for(int i = 0; i < n_available; i++){
                uint8_t data;
                int n_read = rb_read_partial_data(&data, &rx_ring_buffer, 1);
                if (n_read <= 0)
                    break;
                error_code_t err_code = parse_protocol_frame(&pld, data);
                if (!(err_code == NO_ERROR)){
                    continue;
                }
                else if (pld.msg_code == START_CMD)
                {
                    if(pld.payload_len <= 0)
                        break;

                    // save app timestamp (need to save it as early as possible)
                    uint32_t elaped_us_ticks = (usCounter - stm_ref_us_ticks);
                    imu_sample.time_us = (uint64_t)elaped_us_ticks * SYS_TICK_PERIOD_US + master_ref_time; 

                    memcpy((uint8_t *)&master_ref_time, pld.payload, pld.payload_len); // skip first 3 bytes and last 2 (crc_h, crc_l)
                    IMU_Timer_Start();
                    imu_running = 1;
                }
                else if (pld.msg_code == STOP_CMD){
                    IMU_Timer_Stop();
                    imu_running = 0;
                }
                else if (pld.msg_code == CALIB_CMD){
                    // TODO:   
                    //  - init calibration: gyro + magnet + accel(?)
                    //  - transnmit back measurements during calibration
                    //  - transmit back resulted biases (?) to check validness of calculation
                    // imu_running = 1;
                    continue;
                }
            }
        }

        /* ---------------------------------------------------------------- */
        //  IMU CONTROL PART  
        /* ---------------------------------------------------------------- */

        if (!imu_running)
            continue;

        if (cur_spi_state == SPI_READY)
        {
            cur_spi_state = SPI_READING;
            // save time in imu sample before taking measurements
            uint32_t elaped_us_ticks =  (usCounter - stm_ref_us_ticks);
            imu_sample.time_us = (uint64_t)elaped_us_ticks * SYS_TICK_PERIOD_US + master_ref_time;        
            start_reading_imu_measurement();
        }

        /* ------------------------------------------ */
        if (cur_spi_state == SPI_DATA_READY)
        {
            imu_scaled_meas_t *meas = get_imu_corrected_measurement();
            for (uint16_t i = 0; i < 3; i++){
                imu_sample.accel[i] = meas->s_accel[i];
                imu_sample.gyro[i] = meas->s_gyro[i];
                imu_sample.mag[i] = meas->s_mag[i];
            }

            imu_sample_cnt++;
            update_orientation(&imu_sample); // may take > 1-5 ms (?)
            cur_spi_state = SPI_FREE;
        }

        /* ------------------------------------------ */
        if ((imu_sample_cnt * TIM9_PERIOD_MS >= UART_TX_PERIOD_MS) &&
            (cur_usart3_tx_state == USART3_TX_FREE) && (cur_spi_state == SPI_FREE))
        {
            imu_sample_cnt = 0;
            #if SEND_MODE == IMU_SCALED_MEAS
                transmit_imu_sample_usart3(&imu_sample);
            #elif SEND_MODE == EULER_ORIENTATION
                transmit_euler_orient_usart3(get_euler_orient_sample());
            #elif SEND_MODE == QUATERNION_ORIENTATION
                transmit_quater_orient_usart3(get_quater_orient_sample());
            #endif
        }

           
    }
 

    return 0;
}

// char str_buf[30];
// uint8_t n_recorded = snprintf(str_buf, sizeof(str_buf), "%llu", ref_time);
// transmit_data_usart3((uint8_t*)str_buf, n_recorded + 1);
// return 0;

// while(!(USART3->ISR & USART_ISR_RXNE));
// uint8_t data = USART3->RDR;
// transmit_byte_usart3_debug(data);
