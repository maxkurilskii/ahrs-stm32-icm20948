#ifndef CRC16_H
#define CRC16_H

#include "common.h"
#define POLY_CRC16      0x1021 // CRC-16/IBM-3740; CRC-16/CCITT-FALSE


void generate_table_crc16(uint16_t* table_crc16);
uint16_t CRC16_Calculate(uint8_t *data_arr, uint8_t data_len);

    
#endif