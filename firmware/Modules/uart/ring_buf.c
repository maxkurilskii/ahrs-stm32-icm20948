#include "ring_buf.h"


void rb_init(RingBuf *rb)
{
    rb->next_write_ind = 0;
    rb->next_read_ind = 0;
    rb->is_empty = 1;
}


// void rb_print_array(RingBuf *rb, int len)
// {
//     for(int i = 0; i < len; i++){
//         printf("%d ", rb->arr[i]);
//     }
//     printf("\n");
// }


Data rb_pick_element(RingBuf *rb)
{
    return (rb->arr[rb->next_read_ind]);
}


uint8_t rb_is_full(RingBuf *rb)
{
    // если в буфере уже есть данные И совпадают индексы 
    return (rb->is_empty == 0) && (rb->next_write_ind == rb->next_read_ind);
}

uint8_t rb_is_empty(RingBuf *rb)
{
    return rb->is_empty;
}

int rb_free_space(RingBuf *rb)
{
    if(rb->is_empty)
        return rb->size;

    if(rb->next_write_ind > rb->next_read_ind)
        return rb->size - (rb->next_write_ind - rb->next_read_ind);
        
    if(rb->next_write_ind < rb->next_read_ind)
        return rb->next_read_ind - rb->next_write_ind;
    
    return 0;
}


void rb_write_data(RingBuf *rb, Data val)
{
    /* Записываем даже если буфер полный -> затираем старые данные */
    // если буфер заполнен -> сдвигаем R индекс
    if(rb_is_full(rb)){
            // инкрементируем Read индекс с учетом его возможного переполнения
            rb->next_read_ind = (rb->next_read_ind + 1) % rb->size;
    }

    rb->arr[rb->next_write_ind] = val;
    // инкрементируем Write индекс с учетом его возможного переполнения
    rb->next_write_ind = (rb->next_write_ind + 1) % rb->size;
    
    rb->is_empty = 0;
}


uint16_t rb_read_all_data(Data *dest, RingBuf *rb, int len)
{
    if(rb->is_empty)
        return 0;

    uint16_t i = 0; // счетчик прочитанных элементов
    while(1){
        dest[i] = rb->arr[rb->next_read_ind];
        i++; 
        rb->next_read_ind = (rb->next_read_ind + 1) % rb->size;   
        // Если R индекс достиг W индекса -> прочитали все что было 
        if(rb->next_read_ind == rb->next_write_ind){
            rb->is_empty = 1;
            break;
        }
    }
    
    return i;
}


uint16_t rb_read_partial_data(Data *dest, RingBuf *rb, int len)
{
    if(rb->is_empty || len <= 0)
        return 0;

    uint16_t i = 0; // счетчик прочитанных элементов
    while(i < len){
        dest[i] = rb->arr[rb->next_read_ind];
        i++; 
        rb->next_read_ind = (rb->next_read_ind + 1) % rb->size;   
        // Если R индекс достиг W индекса -> прочитали все что было 
        if(rb->next_read_ind == rb->next_write_ind){
            rb->is_empty = 1;
            break;
        }
    }
    
    return i;
}

