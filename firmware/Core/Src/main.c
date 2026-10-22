#include "common.h"
#include "uart_driver.h"
#include "uart_protocol.h"
#include "ism20948_driver.h"
#include "data_prepr.h"

#define IMU_RAW_MEAS                0
#define IMU_SCALED_MEAS             1
#define EULER_ORIENTATION           2
#define QUATERNION_ORIENTATION      3    

#define SEND_MODE   IMU_SCALED_MEAS

#define TEST_LOAD_0_MS      0
#define TEST_LOAD_5_MS      5
#define TEST_LOAD_10_MS     10
#define TEST_LOAD_20_MS     20
#define TEST_LOAD_50_MS     50
#define TEST_LOAD_100_MS    100

#define CURRENT_LOAD_DELAY  TEST_LOAD_10_MS

#define MAX_TEST_TIME_MS    5000
/* TEST Instance  */
TestStats stats = {0};
bool stats_updated = 0;
uint16_t prev_rx_loop_enters = 0;
void send_stats(void);
#define SEND_TEST_HEADER    {char *header = "---- TEST RESULTS ---- \n"; \
                                    for(char *p = header; *p; p++) \
                                        transmit_byte_usart3_debug(*p);}

#define SEND_TEST_STATS(buf, buf_size, format, data)  { int n = snprintf((buf), (buf_size), (format), (data)); \
                                                        for(int i = 0; i < n; i++) \
                                                            transmit_byte_usart3_debug(buf[i]);}

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

    // IMU20948_Init();
    // Madgwick_Filter_Init();
    // delay_ms(1000);

    PosixTimestamp master_ref_time = {0};
    uint32_t stm_ref_us_ticks = 0;

    /* Parsed payload instance from received msg */
    PayloadInstance pld; 

    /* Responce frame */
    uint8_t resp_frame[MAX_FRAME_SIZE] = {};


    USART3_Timer_Start();

    while (1) {

        /* ---------------------------------------------------------------- */
        //  CMD LISTENING PART  
        /* ---------------------------------------------------------------- */


        /* Check if new cmd has arrived */
        while (!rb_is_empty(&rx_ring_buffer)){
            stats_updated = 1;
            stats.rx_loop_enters++;

            // save app timestamp (need to save it as early as possible)
            uint32_t app_tmp_time = usCounter;
            // save current next_write_ind (snapshot) to know exact read bound
            // real next_write_ind can change during reading
            uint32_t nwi_snap = rx_ring_buffer.next_write_ind; 
            int n_available = rb_available(&rx_ring_buffer);
            if (n_available < 0)
                continue; // sanity case 

            for(int i = 0; i < n_available; i++){
                uint8_t data;
                rb_read_partial_data(&data, &rx_ring_buffer, 1);
                error_code_t err_code = parse_protocol_frame(&pld, data);
                if (!(err_code == NO_ERROR)){
                    continue;
                }
                else if (pld.msg_code == START_CMD)
                {
                    stats.rx_frames_received += 1;
                    // imu_running = 1;

                    // master ref time received 
                    // memcpy(master_ref_time.bytes, pld.payload, pld.payload_len); // skip first 3 bytes and last 2 (crc_h, crc_l)
                    // stm_ref_us_ticks = app_tmp_time;
                    
                    encode_protocol_frame(resp_frame, pld.payload, START_CMD, pld.payload_len);       
                    if (!UartWrite_2_TxRingBuffer(resp_frame, pld.payload_len + 5)){
                        set_led(LED3);
                        stats.tx_rb_full_state_count+=1;
                    }
                    // IMU_Timer_Start();
                }
                else if (pld.msg_code == STOP_CMD){
                    // imu_running = 0;
                    stats.rx_frames_received += 1;

                    encode_protocol_frame(resp_frame, pld.payload, STOP_CMD, pld.payload_len);
                    if (!UartWrite_2_TxRingBuffer(resp_frame, pld.payload_len + 5)){
                        set_led(LED3);
                        stats.tx_rb_full_state_count+=1;
                    }
                    // IMU_Timer_Stop();
                }
                else if (pld.msg_code == UPD_TIME_CMD){
                    stats.rx_frames_received += 1;
                    // memcpy(master_ref_time.bytes, pld.payload, pld.payload_len); // skip first 3 bytes and last 2 (crc_h, crc_l)
                    // stm_ref_us_ticks = app_tmp_time;
                    
                    encode_protocol_frame(resp_frame, pld.payload, UPD_TIME_CMD, pld.payload_len);
                    if (!UartWrite_2_TxRingBuffer(resp_frame, pld.payload_len + 5)){
                        set_led(LED3);
                        stats.tx_rb_full_state_count+=1;
                    }
                    // while(cur_usart3_tx_state != USART3_TX_FREE);    
                    // transmit_protocol_msg_usart3(resp_frame, pld.payload_len + 5);
                }
                else if (pld.msg_code == CALIB_CMD){
                    stats.rx_frames_received++;

                    encode_protocol_frame(resp_frame, pld.payload, CALIB_CMD, pld.payload_len);
                    if (!UartWrite_2_TxRingBuffer(resp_frame, pld.payload_len + 5)){
                        set_led(LED3);
                        stats.tx_rb_full_state_count+=1;
                    }
                    // TODO:   
                    //  - init calibration: gyro + magnet + accel(?)
                    //  - transnmit back measurements during calibration
                    //  - transmit back resulted biases (?) to check validness of calculation
                }
            }
        }
        /* ---------------------------------------------------------------- */
        // TEST LOAD: 
        /* ---------------------------------------------------------------- */
        if (stats.rx_loop_enters != prev_rx_loop_enters){
            prev_rx_loop_enters = stats.rx_loop_enters;
            stats.loads_cnt++;
        }
        delay_ms(CURRENT_LOAD_DELAY);


        /* Check if stats is updated and test time elapsed  -> leave loop to send stats */
        if (stats_updated && 
                (uint64_t)(usCounter - stm_ref_us_ticks) * SYS_TICK_PERIOD_US > MAX_TEST_TIME_MS * 1000){
            while(cur_usart3_tx_state != USART3_TX_FREE);
            set_led(LED1);
            break; 
        }
           
    }

    send_stats();
 

    return 0;
}

/* LAST results for 20 burst messages with 10 ms delay (load) */
// ---- TEST RESULTS ---- 
// Parse errors cnt: 13
// RX frames received: 17
// RX ring buf overflows: 3
// RX bytes dropped: 12
// TX frames enqueued: 17
// TX ring buf overflows: 0
// RX loop enters: 3
// LOADs number: 2
// TX DMA start count: 4
// TX DMA finish count: 4

void send_stats(void)
{
    while(1){
        static char buf[40];
        SEND_TEST_HEADER
        SEND_TEST_STATS(buf, sizeof(buf), "Parse errors cnt: %d\n", stats.parse_hard_errors);
        SEND_TEST_STATS(buf, sizeof(buf), "RX frames received: %d\n", stats.rx_frames_received);
        SEND_TEST_STATS(buf, sizeof(buf), "RX ring buf overflows: %d\n", stats.rx_rb_full_state_count);
        SEND_TEST_STATS(buf, sizeof(buf), "RX bytes dropped: %d\n", stats.rx_bytes_dropped);
        SEND_TEST_STATS(buf, sizeof(buf), "TX frames enqueued: %d\n", stats.tx_frames_enqueued);
        SEND_TEST_STATS(buf, sizeof(buf), "TX ring buf overflows: %d\n", stats.tx_rb_full_state_count);
        SEND_TEST_STATS(buf, sizeof(buf), "RX loop enters: %d\n", stats.rx_loop_enters);
        SEND_TEST_STATS(buf, sizeof(buf), "LOADs number: %d\n", stats.loads_cnt - 1); // last increment is not useful
        SEND_TEST_STATS(buf, sizeof(buf), "TX DMA start count: %d\n", stats.tx_dma_start_count);
        SEND_TEST_STATS(buf, sizeof(buf), "TX DMA finish count: %d\n", stats.tx_dma_finish_count);

        delay_ms(3000);
    }
}


