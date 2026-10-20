#include "uart_protocol.h"

static void reset_routine(ParseState *parse_state, uint8_t *frame_index){
    *parse_state = WAIT_START_BYTE;
    *frame_index = 0;
}


error_code_t parse_protocol_frame(PayloadInstance *pld, uint8_t data)
{
    static ParseState parse_state = WAIT_START_BYTE;
    static uint8_t frame[MAX_FRAME_SIZE] = {};
    static uint8_t i = 0;


    switch(parse_state){
        case WAIT_START_BYTE: {
            if (data == PROTOCOL_START_BYTE){
                parse_state = WAIT_MSG_CODE;
                frame[i++] = data;
            } else{
                reset_routine(&parse_state, &i);
                // transmit_control_msg_usart3(ERROR_ST, INC_START_BYTE);
                return INC_START_BYTE;
            }
            break;
        }
        case WAIT_MSG_CODE: {
            if (is_valid_msg_code(data)){
                parse_state = WAIT_DATA_LEN;
                pld->msg_code = frame[i++] = data;
            } else{
                reset_routine(&parse_state, &i);
                // transmit_control_msg_usart3(ERROR_ST, INC_MSG_CODE);
                return INC_MSG_CODE;
            }
            break;
        }
        case WAIT_DATA_LEN: {
            if (0 < data && data < MAX_FRAME_SIZE - 5){
                parse_state = WAIT_DATA;
                pld->payload_len = frame[i++] = data;
            } else {
                reset_routine(&parse_state, &i);
                // transmit_control_msg_usart3(ERROR_ST, INC_FRAME_LEN);
                return INC_FRAME_LEN;
            }
            break;
        }
        case WAIT_DATA: {
            frame[i++] = data;
            if (i - 3 == pld->payload_len)
                parse_state = WAIT_CRC_H;
            break;
        }
        case WAIT_CRC_H: {
            frame[i++] = data;
            parse_state = WAIT_CRC_L;
            break;
        }
        case WAIT_CRC_L: {
            frame[i] = data;
            uint16_t crc16 = CRC16_Calculate(frame, i + 1);
            if(crc16 != 0x00){
                reset_routine(&parse_state, &i);
                // transmit_control_msg_usart3(ERROR_ST, INC_CRC);
                return INC_CRC;
            }
            else{
                memcpy(pld->payload, &frame[i - 1 - pld->payload_len], pld->payload_len);
                reset_routine(&parse_state, &i);
                return NO_ERROR;
            }
        }
        default:{
            reset_routine(&parse_state, &i);
            // transmit_control_msg_usart3(ERROR_ST, INC_FRAME_CASE);
            return INC_FRAME_CASE;
        }
    }

    return NOT_ENOUGH_DATA;
}


bool is_valid_msg_code(message_code_t v)
{
    return (v == START_CMD) || (v == STOP_CMD) || (v == CALIB_CMD) || (v == UPD_TIME_CMD);
}


void encode_protocol_frame(uint8_t *frame, uint8_t *payload, message_code_t msg_code, uint8_t payload_len)
{
    frame[0] = PROTOCOL_START_BYTE;
    frame[1] = msg_code;
    frame[2] = payload_len;
    memcpy(&frame[3], payload, payload_len);
    uint8_t frame_length = payload_len + 5;  
    uint16_t crc16_val = CRC16_Calculate(frame, frame_length - 2); // except crc itself (2 bytes)
    frame[frame_length - 2] = (uint8_t)(crc16_val >> 8); // crc_h
    frame[frame_length - 1] = (uint8_t)(crc16_val);      // crc_l
}



// typedef struct{
//     float	    s_accel[3];
//     float	    s_gyro[3];
//     float       s_mag[3];
// }imu_scaled_meas_t;

// typedef union 
// {
//     uint8_t array[36];

//     struct{
//         float	    s_accel[3];
//         float	    s_gyro[3];
//         float       s_mag[3];
//     } sensor;

// }imu_scaled_meas_t;
