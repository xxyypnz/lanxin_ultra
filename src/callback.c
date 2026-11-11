#include "callback.h"
#include "device.h"
#include "macro.h"

#define PRODUCER_COPY(src, dst_buffer, capacity, width, height, channel, sensor_ts, recv_ts, tag, frame_num) do{ \
    /* (dst_info) = (src); */ \
    (sensor_ts) = (src).sensor_timestamp; \
    (recv_ts) = (src).recv_timestamp; \
    \
    (width) = (src).frame_width; \
    (height) = (src).frame_height; \
    (channel) = (src).frame_channel; \
    \
    size_t element_size = 0; \
    switch ((src).frame_data_type) { \
case LX_DATA_UNSIGNED_CHAR: \
        element_size = sizeof(unsigned char); \
        break; \
    case LX_DATA_UNSIGNED_SHORT: \
        element_size = sizeof(unsigned short); \
        break; \
    case LX_DATA_SIGNED_SHORT: \
        element_size = sizeof(short); \
        break; \
    case LX_DATA_FLOAT: \
        element_size = sizeof(float); \
        break; \
    default: \
        fprintf(stderr, "producer: unsupported frame data type: %d\n", (int)(src).frame_data_type); \
        break; \
    } \
    size_t frame_size = (size_t)(width) * (size_t)(height) * \
                        (size_t)(channel) * element_size; \
    \
    if((src).frame_data == NULL || frame_size == 0 || frame_size > (capacity)){ \
        fprintf(stderr, "producer %s---(FRAME %llu): invalid frame copy, frame_size=%zu capacity=%zu\n", \
                tag, frame_num, frame_size, (size_t)(capacity)); \
    } else { \
        memcpy((dst_buffer), (src).frame_data, frame_size); \
    } \
}while(0)

// 回调函数本体
// 数据帧深拷贝到缓冲区
static unsigned long long frame_num = 0;
static int producer_index = 0;

void producer(FrameInfo* frame, void* usr_data){
    if(frame == NULL || frame->frame_state != LX_SUCCESS){
        fprintf(stderr, "producer: frame is NULL or frame_state is not LX_SUCCESS\n");
        return;
    }

    // int frame_count = atomic_load(&g_buffer_queue.frame_count);
    // if(frame_count >= BUFFER_QUEUE_SIZE) return;
    // Buffer* dst = &g_buffer_queue.buffer[g_buffer_queue.tail];
    // pthread_mutex_lock(&dst->mutex);

    frame_num++;
    Buffer* dst = &g_buffer_array[producer_index];
    if(sem_wait(&dst->A) != 0){
        perror("producer: sem_wait A failed");
        return;
    }

    PRODUCER_COPY(frame->depth_data,
         dst->depth_buffer,
         dst->depth_capacity,
         dst->depth_width,
         dst->depth_height,
         dst->depth_channel,
         dst->depth_sensor_timestamp,
         dst->depth_recv_timestamp,
         "depth", frame_num);

    PRODUCER_COPY(frame->amp_data,
         dst->amp_buffer,
         dst->amp_capacity,
         dst->amp_width,
         dst->amp_height,
         dst->amp_channel,
         dst->amp_sensor_timestamp,
         dst->amp_recv_timestamp,
         "amp", frame_num);

    PRODUCER_COPY(frame->rgb_data,
         dst->rgb_buffer,
         dst->rgb_capacity,
         dst->rgb_width,
         dst->rgb_height,
         dst->rgb_channel,
         dst->rgb_sensor_timestamp,
         dst->rgb_recv_timestamp,
         "rgb", frame_num);

    if(sem_post(&dst->B) != 0){
        perror("producer: sem_post B failed");
        return;
    }

    producer_index = (producer_index + 1) % BUFFER_SIZE;

    // pthread_mutex_unlock(&dst->mutex);
    // g_buffer_queue.tail = (g_buffer_queue.tail + 1) % BUFFER_QUEUE_SIZE;
    // atomic_fetch_add_explicit(&g_buffer_queue.frame_count, 1, memory_order_release);

    return;
}

void callback_init(void){
    LX_STATE lx_state = DcRegisterFrameCallback(g_handle, producer, NULL);
    if(lx_state != LX_SUCCESS){
        fprintf(stderr, "callback_init: 注册回调函数失败\n");
        exit(-1);
    }

    return;
}

void callback_free(void){
    LX_STATE lx_state = DcUnregisterFrameCallback(g_handle);
    if(lx_state != LX_SUCCESS){
        fprintf(stderr, "callback_free: 注销回调函数失败\n");
        exit(-1);
    }

    return;
}
