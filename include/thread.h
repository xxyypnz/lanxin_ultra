#ifndef LANXIN_ULTRA_THREAD_H
#define LANXIN_ULTRA_THREAD_H

#include "consumer.h"
#include "server.h"
#include "system.h"

#define THREAD_NUM 2

typedef void*(*ThreadFunc)(void*);
typedef struct{
    pthread_t id;
    pthread_attr_t attr;
    ThreadFunc func;
    void* arg;
    bool valid;
}ThreadInfo;

extern ThreadInfo thread_map[THREAD_NUM];

void thread_init(void);
void thread_free(void);

#endif
