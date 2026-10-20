#include "ring_buf.h"


void rb_init(volatile RingBuf *rb)
{
    rb->size = RB_SIZE;
    rb->next_write_ind = 0;
    rb->next_read_ind = 0;
}


// void rb_print_array(RingBuf *rb, int len)
// {
//     for(int i = 0; i < len; i++){
//         printf("%d ", rb->arr[i]);
//     }
//     printf("\n");
// }


Data rb_pick_element(volatile RingBuf *rb)
{
    return (rb->arr[rb->next_read_ind]);
}


uint8_t rb_is_full(const volatile RingBuf *rb)
{
    // NOTE: In that case capacity of RING bufer is SIZE - 1 
    return (rb->next_write_ind + 1) % rb->size  == rb->next_read_ind;
}

uint8_t rb_is_empty(const volatile RingBuf *rb)
{
    return rb->next_write_ind == rb->next_read_ind;
}


int rb_available(const volatile RingBuf *rb, const uint32_t next_write_index)
{
    if (rb_is_empty(rb))
        return 0;

    if (next_write_index > rb->next_read_ind)
        return next_write_index - rb->next_read_ind;
        
    if (next_write_index < rb->next_read_ind)
        return rb->size - (rb->next_read_ind - next_write_index);
    
    return 0;
}



int rb_free_space(const volatile RingBuf *rb)
{
    if (rb_is_empty(rb))
        return rb->size;

    if (rb->next_write_ind > rb->next_read_ind)
        return rb->size - (rb->next_write_ind - rb->next_read_ind);
        
    if (rb->next_write_ind < rb->next_read_ind)
        return rb->next_read_ind - rb->next_write_ind;
    
    return 0;
}


void rb_write_data(volatile RingBuf *rb, Data val)
{
    // если буфер заполнен -> не пишем
    if(rb_is_full(rb)){
        set_led(LED2); // visualize buffer saturation
        return;
    }

    rb->arr[rb->next_write_ind] = val;
    // инкрементируем Write индекс с учетом его возможного переполнения
    rb->next_write_ind = (rb->next_write_ind + 1) % rb->size;

}


uint8_t rb_read_all_data(Data *dest, volatile RingBuf *rb)
{
    if(rb_is_empty(rb))
        return 0;

    uint8_t i = 0; // счетчик прочитанных элементов
    while(1){
        dest[i] = rb->arr[rb->next_read_ind];
        i++; 
        rb->next_read_ind = (rb->next_read_ind + 1) % rb->size;   
        // Если R индекс достиг W индекса -> прочитали все что было 
        if(rb->next_read_ind == rb->next_write_ind){
            break;
        }
    }
    
    return i;
}


uint8_t rb_read_partial_data(Data *dest, volatile RingBuf *rb, int len)
{
    if(rb_is_empty(rb) || len <= 0)
        return 0;

    uint8_t i = 0; // счетчик прочитанных элементов
    while(i < len){
        dest[i] = rb->arr[rb->next_read_ind];
        i++; 
        rb->next_read_ind = (rb->next_read_ind + 1) % rb->size;   
        if(rb->next_read_ind == rb->next_write_ind){
            break;
        }
    }
    
    return i;
}

