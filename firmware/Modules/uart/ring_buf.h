#ifndef RING_BUF_H
#define RING_BUF_H

#include "common.h"
#include <stdio.h>
#include <stdint.h>

#define W_IND      -1
#define R_IND       0
#define RB_SIZE    100  // 0 < RING_BUF_SIZE < 255 (uint8T)

typedef uint8_t Data;

typedef struct
{
    Data arr[RB_SIZE];
    size_t size;                  // размер массива-буфера
    int next_write_ind;           // Следующий индекс для записи (head)
    int next_read_ind;            // Следующий индекс для чтения (tail) 
} RingBuf;


void rb_init(volatile RingBuf *rb);

void rb_print_array(RingBuf *rb, int len);
Data rb_pick_element(volatile RingBuf *rb);

uint8_t rb_is_full(const volatile RingBuf *rb);
uint8_t rb_is_empty(const volatile RingBuf *rb);
int rb_free_space(const volatile RingBuf *rb);
int rb_available(const volatile RingBuf *rb, const uint32_t next_write_index);

void rb_write_data(volatile RingBuf *rb, Data val);
uint8_t rb_read_partial_data(Data *dest, volatile RingBuf *rb, int len);
uint8_t rb_read_partial_data(Data *dest, volatile RingBuf *rb, int len);


#endif