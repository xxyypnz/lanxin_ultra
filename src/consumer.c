
#include "buffer.h"
#include "consumer.h"
#include "server.h"
#include <png.h>
#include "detector_api.h"

unsigned long long empty_times = 0;
extern int g_flag;

#define CONSUMER_COPY(src_buffer, src_capacity, dst, src_width, src_height, src_channel, sensor_ts, recv_ts) do{ \
    (dst).width = (src_width); \
    (dst).height = (src_height); \
    (dst).channel = (src_channel); \
    (dst).sensor_timestamp = (sensor_ts); \
    (dst).recv_timestamp = (recv_ts); \
    (dst).length = (src_capacity); \
    memcpy((dst).data, (src_buffer), (src_capacity)); \
}while(0)

#define DETECT(frame, ready, output_length, detections, error) do{ \
    if((ready) && (frame).channel == 3){ \
        (output_length) = 0; \
        (error)[0] = '\0'; \
        if(detector_detect_rgb( \
                (const unsigned char*)(frame).data, \
                (frame).width, (frame).height, \
                (unsigned char*)(frame).data, sizeof((frame).data), \
                &(output_length), (detections), sizeof(detections), \
                (error), sizeof(error)) == 0){ \
            (frame).length = (output_length); \
        } else { \
            fprintf(stderr, "consumer: RGB detection failed: %s\n", (error)); \
        } \
    } \
}while(0)

#define ENCODE(frame, channel) do{ \
    png_image image = {0}; \
    unsigned char* png_data = NULL; \
    unsigned char* raw_data = NULL; \
    png_alloc_size_t png_size = 0; \
    bool encode_failed = false; \
    image.version = PNG_IMAGE_VERSION; \
    if((frame).width <= 0 || (frame).height <= 0){ \
        fprintf(stderr, "consumer: invalid frame dimensions for PNG encoding\n"); \
        encode_failed = true; \
    } else { \
        image.width = (png_uint_32)(frame).width; \
        image.height = (png_uint_32)(frame).height; \
        if((channel) == 1){ \
            image.format = PNG_FORMAT_LINEAR_Y; \
        } else if((channel) == 3){ \
            image.format = PNG_FORMAT_RGB; \
        } else { \
            fprintf(stderr, "consumer: unsupported pixel channel count: %d\n", (channel)); \
            encode_failed = true; \
        } \
    } \
    if(!encode_failed){ \
        png_alloc_size_t raw_size = PNG_IMAGE_SIZE(image); \
        if((frame).length < raw_size){ \
            fprintf(stderr, "consumer: raw frame is smaller than PNG input size\n"); \
            encode_failed = true; \
        } else { \
            raw_data = malloc(raw_size); \
            if(raw_data == NULL){ \
                fprintf(stderr, "consumer: failed to allocate raw PNG input\n"); \
                encode_failed = true; \
            } else { \
                memcpy(raw_data, (frame).data, raw_size); \
            } \
        } \
    } \
    if(!encode_failed && !png_image_write_to_memory( \
           &image, NULL, &png_size, 0, raw_data, 0, NULL)){ \
        fprintf(stderr, "consumer: PNG size query failed: %s\n", image.message); \
        encode_failed = true; \
    } \
    if(!encode_failed && png_size > sizeof((frame).data)){ \
        fprintf(stderr, "consumer: encoded PNG exceeds frame buffer capacity\n"); \
        encode_failed = true; \
    } \
    if(!encode_failed){ \
        png_data = malloc(png_size); \
        if(png_data == NULL){ \
            fprintf(stderr, "consumer: failed to allocate PNG output\n"); \
            encode_failed = true; \
        } \
    } \
    if(!encode_failed && !png_image_write_to_memory( \
           &image, png_data, &png_size, 0, raw_data, 0, NULL)){ \
        fprintf(stderr, "consumer: PNG encoding failed: %s\n", image.message); \
        encode_failed = true; \
    } \
    if(!encode_failed){ \
        memcpy((frame).data, png_data, png_size); \
        (frame).length = (size_t)png_size; \
    } \
    free(raw_data); \
    free(png_data); \
    png_image_free(&image); \
    if(encode_failed){ \
        (frame).length = 0; \
    } \
}while(0)

#define PACK(channel, package) do{ \
    size_t offset = 0; \
    size_t image_length = (channel).length; \
    if(image_length > PACKAGE_MAX_SIZE - 24){ \
        (package).package_length = 0; \
    } \
    else{ \
        /* 4字节时间戳长度 */ \
        (package).package_data[offset++] = 0; \
        (package).package_data[offset++] = 0; \
        (package).package_data[offset++] = 0; \
        (package).package_data[offset++] = 1; /* 服务端定义回传帧数据的 msg_type */ \
        /* 8字节 sensor 时间戳数据 */ \
        for(int i = 7; i >= 0; i--){ \
            (package).package_data[offset++] = (char)((channel).sensor_timestamp >> (i * 8)); \
        } \
        /* 8字节 recv 时间戳数据 */ \
        for(int i = 7; i >= 0; i--){ \
            (package).package_data[offset++] = (char)((channel).recv_timestamp >> (i * 8)); \
        } \
        /* 4字节 image 长度 */ \
        for(int i = 3; i >= 0; i--){ \
            (package).package_data[offset++] = (char)(image_length >> (i * 8)); \
        } \
        /* n字节 image 净荷 */ \
        memcpy((package).package_data + offset, (channel).data, image_length); \
        offset += image_length; \
        (package).package_length = (int)(offset); \
    } \
}while(0)

#define SENDALL(channel, buf, length, client_map) do{ \
    if((channel) && !send_failed_flag && client_map[i].valid){ \
        int fd = client_map[i].fd; \
        size_t total = 0; \
        while(total < (size_t)(length)){ \
            ssize_t len = send(fd, (const char*)(buf) + total, \
                               (size_t)(length) - total, MSG_NOSIGNAL); \
            if(len < 0 && errno == EINTR){ \
                continue; \
            } \
            if(len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)){ \
                usleep(1000); \
                continue; \
            } \
            if(len <= 0){ \
                if(len < 0){ \
                    perror("consumer: send failed"); \
                } else { \
                    fprintf(stderr, "consumer: send returned 0, fd=%d\n", fd); \
                } \
                send_failed_flag = true; \
                shutdown(fd, SHUT_RDWR); \
                break; \
            } \
            total += (size_t)len; \
        } \
    } \
}while(0)

void* consumer(void* arg){
    // 宏定义中的dst
    static Frame depth_frame, amp_frame, rgb_frame;
    static Package depth_package, amp_package, rgb_package;

/*detect 必要变量声明*/
    bool detector_ready = false;
    size_t detected_rgb_length = 0;
    char detections_json[16384] = {0};
    char detector_error[512] = {0};

    if (detector_init("./detector/python",
                      "./models/yolov8n.onnx",
                      "./models/coco.names",
                      detector_error, sizeof(detector_error)) == 0) {
        detector_ready = true;
    } else {
        fprintf(stderr, "detector init failed: %s\n", detector_error);
    }

/*detect 必要变量声明*/

    while(g_flag){

        int frame_count = atomic_load(&g_buffer_queue.frame_count);
        if(frame_count <= 0){
            empty_times++;
            if(empty_times % 10 == 0){
                printf("consumer: shared buffer is empty, waiting for producer...\n");
            }
            usleep(100000); // 100ms
            continue;
        }

        // 1.从 shared buffer 拷贝数据到 consumer 函数内的局部变量
        Buffer* src = &g_buffer_queue.buffer[g_buffer_queue.head];
        pthread_mutex_lock(&src->mutex);

        CONSUMER_COPY(src->depth_buffer, src->depth_capacity, depth_frame, src->depth_width, src->depth_height, src->depth_channel, src->depth_sensor_timestamp, src->depth_recv_timestamp);
        CONSUMER_COPY(src->amp_buffer, src->amp_capacity, amp_frame, src->amp_width, src->amp_height, src->amp_channel, src->amp_sensor_timestamp, src->amp_recv_timestamp);
        CONSUMER_COPY(src->rgb_buffer, src->rgb_capacity, rgb_frame, src->rgb_width, src->rgb_height, src->rgb_channel, src->rgb_sensor_timestamp, src->rgb_recv_timestamp);

        pthread_mutex_unlock(&src->mutex);

        // 2.调用 detector 对 rgb_frame 进行目标检测
        DETECT(rgb_frame, detector_ready, detected_rgb_length,
               detections_json, detector_error);

        // 3.编码成 PNG 格式
        ENCODE(depth_frame, depth_frame.channel);
        ENCODE(amp_frame, amp_frame.channel);
        ENCODE(rgb_frame, rgb_frame.channel);


        // 4.数据打包
        PACK(depth_frame, depth_package);
        PACK(amp_frame, amp_package);
        PACK(rgb_frame, rgb_package);

        // 5.回传到对应客户端
        pthread_rwlock_rdlock(&g_client_map_lock);
        for(int i = 0; i < MAX_CONNECTIONS; i++){
            bool send_failed_flag = false;
            SENDALL(g_client_map[i].channel[0], depth_package.package_data, depth_package.package_length, g_client_map);
            SENDALL(g_client_map[i].channel[1], amp_package.package_data, amp_package.package_length, g_client_map);
            // SENDALL(g_client_map[i].channel[2], rgb_package.package_data, rgb_package.package_length);
        }
        pthread_rwlock_unlock(&g_client_map_lock);


        pthread_rwlock_rdlock(&g_clientAI_map_lock);
        for(int i = 0; i < MAX_CONNECTIONS; i++){
            bool send_failed_flag = false;
            // SENDALL(g_clientAI_map[i].channel[0], depth_package.package_data, depth_package.package_length);
            // SENDALL(g_clientAI_map[i].channel[1], amp_package.package_data, amp_package.package_length);
            SENDALL(g_clientAI_map[i].channel[2], rgb_package.package_data, rgb_package.package_length, g_clientAI_map);
        }
        pthread_rwlock_unlock(&g_clientAI_map_lock);

        g_buffer_queue.head = (g_buffer_queue.head + 1) % BUFFER_QUEUE_SIZE;
        atomic_fetch_sub_explicit(&g_buffer_queue.frame_count, 1, memory_order_release);
    }

    return NULL;
}
