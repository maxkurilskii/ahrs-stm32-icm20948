#include "common.h"

#define W_IND      -1
#define R_IND       0
#define RB_SIZE    10  // 0 < RING_BUF_SIZE < 65535 (uint16)

typedef uint8_t Data;

typedef struct
{
    Data arr[RB_SIZE];
    size_t size;                  // размер массива-буфера
    int next_write_ind;           // Следующий индекс для записи (head)
    int next_read_ind;            // Следующий индекс для чтения (tail) 
    uint8_t is_empty;             // пуст ли буфер
} RingBuf;


void rb_init(RingBuf *rb);

void rb_print_array(RingBuf *rb, int len);
Data rb_pick_element(RingBuf *rb);

uint8_t rb_is_full(RingBuf *rb);
uint8_t rb_is_empty(RingBuf *rb);
int rb_free_space(RingBuf *rb);

void rb_write_data(RingBuf *rb, Data val);
uint16_t rb_read_all_data(Data *dest, RingBuf *rb);
uint16_t rb_read_partial_data(Data *dest, RingBuf *rb, int len);
