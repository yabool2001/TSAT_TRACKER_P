#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint8_t* buffer;
    uint32_t total_size;
    uint32_t cur_size;
    uint32_t in_index;
    uint32_t out_index;
} fifo_t;
 
 
int32_t fifo_init(uint8_t* worker_buffer, uint32_t size, volatile fifo_t* fifo)
{
    int32_t retval = 0L;
 
    if (NULL == worker_buffer) {
        retval = -1L;
        goto exit;
    }
 
    if (0LU == size) {
        retval = -2L;
        goto exit;
    }
 
    if (NULL == fifo) {
        retval = -3L;
        goto exit;
    }
 
    fifo->buffer = worker_buffer;
    fifo->total_size = size;
    fifo->cur_size = 0LU;
    fifo->in_index = 0LU;
    fifo->out_index = 0LU;
 
exit:
    return retval;
}
 
bool fifo_is_empty(fifo_t* fifo)
{
    bool is_empty = true;
 
    if (0LU != fifo->cur_size) {
        is_empty = false;
    }
 
    return is_empty;
}
 
int32_t fifo_put(fifo_t* fifo, uint8_t byte)
{
    int32_t retval = 0L;
    
    if (NULL == fifo) {
        retval = -1L;
        goto exit;
    }
    
    /* Check if fifo is not full */
    if (fifo->total_size == fifo->cur_size) {
        retval = -2L;
        goto exit;
    }
 
    /* Enqueue the byte */
    fifo->buffer[fifo->in_index] = byte;
    /* Increase the input index */
    fifo->in_index += 1LU;
    /* Increase the count of the bytes placed in the fifo */
    fifo->cur_size += 1LU;
 
    /* Wrap around the buffer */
    if (0LU == (fifo->in_index % fifo->total_size)) {
        fifo->in_index = 0LU;
    }
 
exit:
    return retval;
}
 
int32_t fifo_get(fifo_t* fifo, uint8_t* byte)
{
    int32_t retval = 0L;
    
    if (NULL == fifo) {
        retval = -1L;
        goto exit;
    }
    
    if (NULL == byte) {
        retval = -2L;
        goto exit;
    }
 
    /* Check if fifo is not full */
    if (0LU == fifo->cur_size) {
        retval = -3L;
        goto exit;
    }
 
    /* Dequeue the byte */
    *byte = fifo->buffer[fifo->out_index];
    /* Increase the output index */
    fifo->out_index += 1LU;
    /* Decrement the count of the bytes placed in the fifo */
    fifo->cur_size -= 1LU;
 
    /* Wrap around the buffer */
    if (0LU == (fifo->out_index % fifo->total_size)) {
        fifo->out_index = 0LU;
    }
 
exit:
    return retval;
}