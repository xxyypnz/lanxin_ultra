#include "buffer.h"
#include "device.h"
#include "macro.h"

BufferQueue g_buffer_queue;

// 记得打反斜杠续行符
#define ALLOC(src, dst_buffer, dst_capacity, tag) do{ \
    size_t element_size = 0; \
    size_t data_size = 0; \
    \
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
/* 加入算法模块的特殊数据类型... */ \
default: \
    break; \
    } \
    size_t multiple = 1; \
    LxIntValueInfo info; \
    CHECK(DcGetIntValue(g_handle, LX_INT_2D_BINNING_MODE, &info)); \
    switch(info.cur_value){ \
        case LX_BINNING_1X1: \
            multiple = 1; \
            break; \
        case LX_BINNING_2X2: \
            multiple = 4; \
            break; \
        case LX_BINNING_4X4: \
            multiple = 16; \
            break; \
        default: \
            multiple = 1; \
            break; \
    } \
    data_size = ((size_t)(src).frame_width) *  \
                ((size_t)(src).frame_height) * \
                ((size_t)(src).frame_channel) * \
                (element_size) * (multiple); \
    /* 注意： 已经包含了数据类型的字节数！ */ \
    (dst_buffer) = malloc(data_size); \
    (dst_capacity) = data_size; \
    \
    if((dst_buffer) == NULL){ \
        fprintf(stderr, "ALLOC: 内存分配失败，容量: %zu 字节\n", data_size); \
        exit(-1); \
    } \
    printf("ALLOC: 分配 %s 缓冲区，容量: %zu 字节，宽: %d，高: %d，通道数: %d，数据类型: %d\n", \
           (tag), data_size, (src).frame_width, (src).frame_height, \
           (src).frame_channel, (int)(src).frame_data_type); \
}while(0)

#define FREE(buffer, capacity) do{ \
    free((buffer)); \
    (buffer) = NULL; \
    (capacity) = 0; \
}while(0)

// 使用非回调的函数先获取一帧数据
// 获取帧数据参数，从而确定 buffer 的容量
void buffer_init(void){
    int max_retry_count = 5, retry_count = 0;

retry:
    DcSetCmd(g_handle, LX_CMD_GET_NEW_FRAME);
    void* raw_frame = NULL;
    DcGetPtrValue(g_handle, LX_PTR_FRAME_DATA, &raw_frame);
    FrameInfo* frame = (FrameInfo*)raw_frame;

    if(frame == NULL){
        retry_count++;
        if(retry_count > max_retry_count){
            fprintf(stderr, "buffer_init: 获取帧数据失败，放弃重试\n");
            exit(-1);
        }

        int sleep_time = 1 << (retry_count - 1);
        fprintf(stderr, "buffer_init: 获取帧数据失败，等待 %d 秒后重试...\n", sleep_time);
        sleep(sleep_time);

        goto retry;
    }

    else{
        g_buffer_queue.head = 0, g_buffer_queue.tail = 0;
        atomic_init(&g_buffer_queue.frame_count, 0);

        for(int i = 0; i < BUFFER_QUEUE_SIZE; i++){
            ALLOC(frame->depth_data, g_buffer_queue.buffer[i].depth_buffer, g_buffer_queue.buffer[i].depth_capacity, "depth");
            ALLOC(frame->amp_data, g_buffer_queue.buffer[i].amp_buffer, g_buffer_queue.buffer[i].amp_capacity, "amp");
            ALLOC(frame->rgb_data, g_buffer_queue.buffer[i].rgb_buffer, g_buffer_queue.buffer[i].rgb_capacity, "rgb");
            // ALLOC(frame->app_data ...);

            pthread_mutex_init(&g_buffer_queue.buffer[i].mutex, NULL);
        }
    }

    // printf("Buffer 初始化成功\n");
    return;
}

void buffer_free(void){
    for(int i = 0; i < BUFFER_QUEUE_SIZE; i++){
        pthread_mutex_destroy(&g_buffer_queue.buffer[i].mutex);

        FREE(g_buffer_queue.buffer[i].depth_buffer, g_buffer_queue.buffer[i].depth_capacity);
        FREE(g_buffer_queue.buffer[i].amp_buffer, g_buffer_queue.buffer[i].amp_capacity);
        FREE(g_buffer_queue.buffer[i].rgb_buffer, g_buffer_queue.buffer[i].rgb_capacity);
        // FREE();
    }

    return;
}
