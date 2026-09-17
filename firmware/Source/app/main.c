#include "common.h"
#include "uart_driver.h"
#include "imu_ism20948.h"
#include "filter_proces.h"

#define IMU_RAW_MEAS            0
#define IMU_SCALED_MEAS         1
#define EULER_ORIENTATION       2
#define QUATERNION_ORIENTATION  3

#define SEND_MODE   EULER_ORIENTATION  

//volatile uint32_t msCounter = 0;
volatile uint32_t usCounter = 0;
uint32_t start_delay_us = 0;


uint16_t tmp_rx_buffer = 0;
uint8_t mag_device_id = 0;

void SysTick_Handler(void){
	usCounter++; //~90 us for one period
}

void delay_ms(uint16_t millis){
	start_delay_us = usCounter;
	while ((usCounter - start_delay_us)* SYS_TICK_PERIOD_US < millis*1000){
		__NOP();
	}
}


int main(void){
	SysClockInit();
	SysTickInit();
	LEDs_Init();
    USART3_Init();
    IMU20948_Init();
    Madgwick_Filter_Init();
    delay_ms(1000);
   
    uint64_t master_ref_time = 0;
    uint32_t stm_ref_time = 0;
    
    imu_sample_t imu_sample = {.accel = {0.5, 0.5, 1.0}, 
                               .gyro = {1.0, -1.0, 0.0}, 
                               .mag = {10.0, -1.0, 33.0}, 
                               .time_us = usCounter * SYS_TICK_PERIOD_US};
    uint8_t imu_sample_cnt = 0;
                               
    while(1){                     
        while(cur_usart3_state != USART3_DATA_RECEIVED) __NOP();
        // save app timestamp of received frame (expected to include host time)  
        stm_ref_time = usCounter * SYS_TICK_PERIOD_US;
        cur_usart3_state = USART3_FREE;
        uint8_t err_code = parse_protocol_frame();
        if (err_code == NO_ERROR){
            /* NO_ERROR = 0x00 -> ack msg */
            transmit_control_msg_usart3(ERROR, NO_ERROR); 
            break;  
        }
        else{
            transmit_control_msg_usart3(ERROR, err_code); 
        }
    }

    delay_ms(500);

    //copy received timestamp (uint64_t)  in ref_time var
    uint8_t* frame  = get_usart_rx_data();
    memcpy((uint8_t*)&master_ref_time, &frame[3], 8); //skip first 3 bytes

    IMU_Timer_Start();
                               
	while(1){   
        if (cur_spi_state == SPI_READY){
            cur_spi_state = SPI_READING;
            // save time in imu sample before taking measurements
            uint32_t now = usCounter * SYS_TICK_PERIOD_US;
            imu_sample.time_us = (now - stm_ref_time) + master_ref_time; 
            start_reading_imu_measurement();
        }
        
        if(cur_spi_state == SPI_DATA_READY){
            imu_scaled_meas_t* meas = get_imu_corrected_measurement();
            for(uint16_t i = 0; i < 3; i++){
                imu_sample.accel[i] = meas->s_accel[i];
                imu_sample.gyro[i] = meas->s_gyro[i];
                imu_sample.mag[i] = meas->s_mag[i];
            }
            imu_sample_cnt++;      
            update_orientation(&imu_sample); //may take > 1-5 ms (?)
            cur_spi_state = SPI_FREE;
        }
        
        if ((imu_sample_cnt * TIM9_PERIOD_MS >= UART_TX_PERIOD_MS) &&
             (cur_usart3_state == USART3_FREE) && (cur_spi_state == SPI_FREE)){
                imu_sample_cnt = 0;
                #if SEND_MODE == IMU_SCALED_MEAS
                    transmit_imu_sample_usart3( &imu_sample );
                #elif SEND_MODE == EULER_ORIENTATION
                    transmit_euler_orient_usart3( get_euler_orient_sample() );
                #elif SEND_MODE == QUATERNION_ORIENTATION
                    transmit_quater_orient_usart3( get_quater_orient_sample() ); 
                #endif
        }
    }
}

//char str_buf[30];
//uint8_t n_recorded = snprintf(str_buf, sizeof(str_buf), "%llu", ref_time);
//transmit_data_usart3((uint8_t*)str_buf, n_recorded + 1);
//return 0;


//while(!(USART3->ISR & USART_ISR_RXNE));
//uint8_t data = USART3->RDR;
//transmit_byte_usart3_debug(data);
