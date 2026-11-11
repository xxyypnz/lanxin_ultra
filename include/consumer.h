
#ifndef LANXIN_ULTRA_CONSUMER_H
#define LANXIN_ULTRA_CONSUMER_H

#include "system.h"

#define PACKAGE_MAX_SIZE (1024U * 1024U * 8U)

typedef struct{
    int width;
    int height;
    int channel;
    unsigned long long sensor_timestamp;
    unsigned long long recv_timestamp;

    char data[PACKAGE_MAX_SIZE];
    size_t length;
}Frame;

typedef struct{
    char package_data[PACKAGE_MAX_SIZE];
    int package_length;
}Package;

void* consumer(void* arg);

#endif
