
#ifndef LANXIN_ULTRA_BUFFER_H
#define LANXIN_ULTRA_BUFFER_H

#include "system.h"

// #define BUFFER_QUEUE_SIZE 4
#define BUFFER_SIZE 2

typedef struct{

    void* depth_buffer;
    size_t depth_capacity;
    int depth_width;
    int depth_height;
    int depth_channel;
    unsigned long long depth_sensor_timestamp;
    unsigned long long depth_recv_timestamp;

    void* amp_buffer;
    size_t amp_capacity;
    int amp_width;
    int amp_height;
    int amp_channel;
    unsigned long long amp_sensor_timestamp;
    unsigned long long amp_recv_timestamp;
    
    void* rgb_buffer;
    size_t rgb_capacity;
    int rgb_width;
    int rgb_height;
    int rgb_channel;
    unsigned long long rgb_sensor_timestamp;
    unsigned long long rgb_recv_timestamp;

    // void* app_buffer;
    // size_t app_capacity;

    // pthread_mutex_t mutex;
    sem_t A;
    sem_t B;
}Buffer;
extern Buffer g_buffer_array[BUFFER_SIZE];

// typedef struct{
//     Buffer buffer[BUFFER_QUEUE_SIZE];
//     int head;
//     int tail;
//     atomic_int frame_count;
// }BufferQueue;
// extern BufferQueue g_buffer_queue;

void buffer_init(void);
void buffer_free(void);

#endif
