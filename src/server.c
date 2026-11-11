
#include "device.h"
#include "server.h"

#include <cjson/cJSON.h>

extern int g_flag;

ClientInfo g_client_map[MAX_CONNECTIONS];
pthread_rwlock_t g_client_map_lock = PTHREAD_RWLOCK_INITIALIZER;
// static int client_count = 0;

// 新添加的视频端口处理
ClientInfo g_clientAI_map[MAX_CONNECTIONS];
pthread_rwlock_t g_clientAI_map_lock = PTHREAD_RWLOCK_INITIALIZER;
// static int clientAI_count = 0;

/**/
/** 主线程和 server 线程共享的停止通知 fd。 */
int g_server_stop_fd = -1;

int server_stop_init(void){
    /** 使用非阻塞 eventfd，避免退出通知因重复写入而阻塞。 */
    g_server_stop_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if(g_server_stop_fd < 0){
        perror("server_stop_init: eventfd 失败");
        return -1;
    }
    return 0;
}

void server_stop_notify(void){
    /** 写入 eventfd 会让已阻塞的 epoll_wait 返回一个可读事件。 */
    if(g_server_stop_fd >= 0){
        uint64_t value = 1;
        ssize_t result = write(g_server_stop_fd, &value, sizeof(value));
        if(result < 0 && errno != EAGAIN){
            perror("server_stop_notify: write 失败");
        }
    }
}
/**/

void start_handler(int fd, int chn_id_integer, cJSON* status, ClientInfo* client_map, pthread_rwlock_t* client_map_lock){
    int status_integer = status->valueint;
    pthread_rwlock_wrlock(client_map_lock);
    for(int i = 0; i < MAX_CONNECTIONS; i++){
        if(client_map[i].fd == fd){
            client_map[i].channel[chn_id_integer] = (status_integer) == 1 ? true : false;
        }
    }
    pthread_rwlock_unlock(client_map_lock);

    return;
}

void resolution_handler(int fd, int chn_id_integer, cJSON* resolution, ClientInfo* client_map, pthread_rwlock_t* client_map_lock){
    int resolution_integer = resolution->valueint;
    pthread_rwlock_rdlock(client_map_lock);
    for(int i = 0; i < MAX_CONNECTIONS; i++){
        if(client_map[i].fd == fd){
            DcSetIntValue(g_handle, LX_INT_2D_BINNING_MODE, resolution_integer);
        }
    }
    pthread_rwlock_unlock(client_map_lock);

    return;
}

void autoexplevel_handler(int fd, int chn_id_integer, cJSON* autoexplevel, ClientInfo* client_map, pthread_rwlock_t* client_map_lock){
    int autoexplevel_integer = autoexplevel->valueint;
    pthread_rwlock_rdlock(client_map_lock);
    for(int i = 0; i < MAX_CONNECTIONS; i++){
        if(client_map[i].fd == fd){
            DcSetIntValue(g_handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, autoexplevel_integer);
        }
    }
    pthread_rwlock_unlock(client_map_lock);

    return;
}

#define CMD_RESPOND(fd, cmd) do{ \
    cJSON* response = cJSON_CreateObject(); \
    if(response == NULL){ \
        fprintf(stderr, "CMD_RESPOND: create JSON object failed\n"); \
        return; \
    } \
    if(cJSON_AddNumberToObject((response), "cmd_id", (cmd)) == NULL){ \
        fprintf(stderr, "CMD_RESPOND: add cmd_id to JSON object failed\n"); \
        cJSON_Delete(response); \
        return; \
    } \
    char* response_payload = cJSON_PrintUnformatted(response); \
    if(response_payload == NULL){ \
        fprintf(stderr, "CMD_RESPOND: print JSON object failed\n"); \
        cJSON_Delete(response); \
        return; \
    } \
    \
    char response_package[MAX_JSON]; \
    int offset = 0; \
    response_package[offset++] = 0; \
    response_package[offset++] = 0; \
    response_package[offset++] = 0; \
    response_package[offset++] = 2; /* 服务端定义回传消息的 msg_type */ \
    \
    size_t payload_length = strlen(response_payload); \
    uint32_t payload_length_be = htonl((uint32_t)payload_length); \
    \
    memcpy(response_package + offset, &(payload_length_be), sizeof(payload_length_be)); \
    offset += sizeof(payload_length_be); \
    \
    memcpy(response_package + offset, response_payload, payload_length); \
    offset += (int)(payload_length); \
    \
    send((fd), response_package, (size_t)(offset), 0); \
    \
    cJSON_free(response_payload); \
    cJSON_Delete(response); \
}while(0)

// 客户端保证发送的 json 一定合法
// 服务端接收时保证接收到的 json 不为空
void json_handler(int fd, char* buf, size_t buf_len, int port_type){
    cJSON* root = cJSON_ParseWithLength(buf, buf_len);
    if(root == NULL){
        fprintf(stderr, "json_handler: parse JSON failed\n");
        goto json_cleanup;
    }

    cJSON* cmd_id = cJSON_GetObjectItemCaseSensitive(root, "cmd_id");
    cJSON* dst = cJSON_GetObjectItemCaseSensitive(root, "dst");
    // cJSON* status = cJSON_GetObjectItemCaseSensitive(root, "status");
    if(!cJSON_IsNumber(cmd_id) || !cJSON_IsObject(dst)){
        fprintf(stderr, "json_handler: invalid JSON format\n");
        goto json_cleanup;
    }

    cJSON* dev_id = cJSON_GetObjectItemCaseSensitive(dst, "dev_id");
    cJSON* chn_id = cJSON_GetObjectItemCaseSensitive(dst, "chn_id");
    if(!cJSON_IsNumber(dev_id) || !cJSON_IsNumber(chn_id)){
        fprintf(stderr, "json_handler: invalid dst format\n");
        goto json_cleanup;
    }

    int cmd_id_integer = cmd_id->valueint, chn_id_integer = chn_id->valueint;

    // 选择其中的三个字段作为测试
    // 宏定义统一化非常困难，字段并非高度一致
    // CMD_HANDLER(CMD_CAMERA_START_SET, "status", CMD_CAMERA_START_SET_RES);
    // CMD_HANDLER(CMD_CAMERA_RESOLUTION_SET, "resolution", CMD_CAMERA_RESOLUTION_SET_RES);
    // CMD_HANDLER(CMD_CAMERA_AUTOEXPLEVEL_SET , "autoexplevel", CMD_CAMERA_AUTOEXPLEVEL_SET _RES);

    ClientInfo* client_map = NULL;
    pthread_rwlock_t* client_map_lock = NULL;

    if(port_type == MESSAGE_PORT){
        client_map = g_client_map;
        client_map_lock = &g_client_map_lock;
    }
    else{
        client_map = g_clientAI_map;
        client_map_lock = &g_clientAI_map_lock;
    }

    // status 只会有 1/0 两种情况，分别表示通道的 打开/关闭
    if(cmd_id_integer == CMD_CAMERA_START_SET){
        cJSON* status = cJSON_GetObjectItemCaseSensitive(root, "status");
        start_handler(fd, chn_id_integer, status, client_map, client_map_lock);
        CMD_RESPOND(fd, CMD_CAMERA_START_SET_RES);
    }

    if(cmd_id_integer == CMD_CAMERA_RESOLUTION_SET){
        cJSON* resolution = cJSON_GetObjectItemCaseSensitive(root, "resolution");
        resolution_handler(fd, chn_id_integer, resolution, client_map, client_map_lock);
        CMD_RESPOND(fd, CMD_CAMERA_RESOLUTION_SET_RES);
    }

    if(cmd_id_integer == CMD_CAMERA_AUTOEXPLEVEL_SET){
        cJSON* autoexplevel = cJSON_GetObjectItemCaseSensitive(root, "autoexplevel");
        autoexplevel_handler(fd, chn_id_integer, autoexplevel, client_map, client_map_lock);
        CMD_RESPOND(fd, CMD_CAMERA_AUTOEXPLEVEL_SET_RES);
    }

json_cleanup:
    cJSON_Delete(root);

    return;
}

static int set_nonblocking(int fd){
    int flags = fcntl(fd, F_GETFL, 0);
    if(flags == -1) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// ===创建+配置+启动一个监听 socket
#define SOCKET_INIT(fd, port, fail_label) do{ \
    \
    /* ===创建监听 socket */ \
    (fd) = socket(AF_INET, SOCK_STREAM, 0); \
    if((fd) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
    /* 设置端口复用 */ \
    int opt = 1; \
    if(setsockopt((fd), SOL_SOCKET, SO_REUSEADDR, &(opt), sizeof(opt)) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
    /* 设置 IP 地址和端口号 */ \
    struct sockaddr_in server_addr; \
    memset(&server_addr, 0, sizeof(server_addr)); \
    server_addr.sin_family = AF_INET; \
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); \
    server_addr.sin_port = htons((port)); \
    /* 绑定 */ \
    if(bind((fd), (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
    /* 开始监听 */ \
    if(listen((fd), BACKLOG) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
    /* 设置 socket 为非阻塞 */ \
    if(set_nonblocking((fd)) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
}while(0)

// ===将监听 socket 注册到 epoll
#define SOCKET_REGISTER(epoll_fd, listen_fd, fail_label) do{ \
    struct epoll_event ev; \
    memset(&ev, 0, sizeof(ev)); \
    ev.events = EPOLLIN; \
    ev.data.fd = (listen_fd); \
    if(epoll_ctl((epoll_fd), EPOLL_CTL_ADD, (listen_fd), &(ev)) < 0){ \
        perror(""); \
        goto fail_label; \
    } \
}while(0)

#define CLIENT_MAP_INIT(client_map, lock) do{ \
    pthread_rwlock_wrlock(&(lock)); \
    for(int i = 0; i < MAX_CONNECTIONS; i++){ \
        (client_map)[i].fd = -1; \
        (client_map)[i].valid = false; \
        for(int j = 0; j < CHANNEL_COUNT; j++){ \
            (client_map)[i].channel[j] = false; \
        } \
    } \
    pthread_rwlock_unlock(&(lock)); \
}while(0)

bool fd_in_map(int fd, ClientInfo* client_map, pthread_rwlock_t* client_map_lock){
    pthread_rwlock_rdlock(client_map_lock);
    for(int i = 0; i < MAX_CONNECTIONS; i++){
        if(client_map[i].fd == fd){
            pthread_rwlock_unlock(client_map_lock);
            return true;
        }
    }
    pthread_rwlock_unlock(client_map_lock);
    return false;
}

void* server(void* arg){

    // 创建 epoll 实例
    int epoll_fd = -1;
    struct epoll_event evs[MAX_EVENTS];
    epoll_fd = epoll_create1(0);
    if(epoll_fd < 0){
        perror("epoll_create1 失败");
        goto cleanup;
    }

    int message_fd = -1, video_fd = -1;
    SOCKET_INIT(message_fd, MESSAGE_PORT, cleanup);
    SOCKET_INIT(video_fd, VIDEO_PORT, cleanup);

    /**/
    /** 将停止通知 fd 纳入同一个 epoll，保持 epoll_wait 的 timeout=-1。 */
    struct epoll_event stop_event;
    memset(&stop_event, 0, sizeof(stop_event));
    stop_event.events = EPOLLIN;
    stop_event.data.fd = g_server_stop_fd;
    if(g_server_stop_fd < 0 || epoll_ctl(epoll_fd, EPOLL_CTL_ADD,
                                         g_server_stop_fd, &stop_event) < 0){
        perror("server: 注册 stop_fd 失败");
        goto cleanup;
    }
    /**/

    // 两个 socket 都注册到同一个 epoll
    SOCKET_REGISTER(epoll_fd, message_fd, cleanup);
    SOCKET_REGISTER(epoll_fd, video_fd, cleanup);

    // 准备接收客户端数据
    char buf[MAX_MESSAGES];
    size_t buf_len;

    // 服务端侧的客户端列表初始化，包含消息端口和视频端口
    CLIENT_MAP_INIT(g_client_map, g_client_map_lock);
    CLIENT_MAP_INIT(g_clientAI_map, g_clientAI_map_lock);

    while(g_flag){
        int ready_count;
        ready_count = epoll_wait(epoll_fd, evs, MAX_EVENTS, -1);
        if(ready_count < 0){
            if(errno == EINTR) continue;
            perror("epoll_wait 失败");
            break;
        }
            
        bool stop_requested = false;
        for(int i = 0; i < ready_count; i++){

            // epoll 当中取出的连接（描述符以及标志位）
            int current_fd = evs[i].data.fd;
            uint32_t event_flags = evs[i].events;

            /**/
            if(current_fd == g_server_stop_fd){
                /** 消费 eventfd 中的通知值，然后结束 server 线程。 */
                uint64_t value;
                (void)read(g_server_stop_fd, &value, sizeof(value));
                stop_requested = true;
                break;
            }
            /**/

            struct epoll_event ev;
            // 新建客户端信息
            int conn_fd;
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            memset(&client_addr, 0, sizeof(client_addr));

            // 情况一：消息端口有新连接
            if(current_fd == message_fd){
                while((conn_fd = accept(message_fd, (struct sockaddr*)&client_addr, &client_len)) >= 0){
                    if(set_nonblocking(conn_fd) < 0){
                        perror("message_port, set_nonblocking failed");
                        close(conn_fd);
                        continue;
                    }

                    ev.events = EPOLLIN;
                    ev.data.fd = conn_fd;
                    if(epoll_ctl(epoll_fd, EPOLL_CTL_ADD, conn_fd, &ev) < 0){
                        fprintf(stderr, "epoll_ctl 添加 fd 失败: %s\n", strerror(errno));
                        close(conn_fd);
                        continue;
                    }

                    memset(&client_addr, 0, sizeof(client_addr));
                    client_len = sizeof(client_addr);

                    pthread_rwlock_wrlock(&g_client_map_lock);
                    for(int i = 0; i < MAX_CONNECTIONS; i++){
                        if(g_client_map[i].valid == false){
                            g_client_map[i].fd = conn_fd;
                            g_client_map[i].valid = true;
                            break;
                        }
                    }
                    pthread_rwlock_unlock(&g_client_map_lock);
                }

                continue;
            }

            // 情况二：视频端口有新连接
            if(current_fd == video_fd){
                while((conn_fd = accept(video_fd, (struct sockaddr*)&client_addr, &client_len)) >= 0){
                    if(set_nonblocking(conn_fd) < 0){
                        perror("video_port, set_nonblocking failed");
                        close(conn_fd);
                        continue;
                    }

                    ev.events = EPOLLIN;
                    ev.data.fd = conn_fd;
                    if(epoll_ctl(epoll_fd, EPOLL_CTL_ADD, conn_fd, &ev) < 0){
                        fprintf(stderr, "epoll_ctl 添加 fd 失败: %s\n", strerror(errno));
                        close(conn_fd);
                        continue;
                    }

                    memset(&client_addr, 0, sizeof(client_addr));
                    client_len = sizeof(client_addr);

                    pthread_rwlock_wrlock(&g_clientAI_map_lock);
                    for(int i = 0; i < MAX_CONNECTIONS; i++){
                        if(g_clientAI_map[i].valid == false){
                            g_clientAI_map[i].fd = conn_fd;
                            g_clientAI_map[i].valid = true;
                            break;
                        }
                    }
                    pthread_rwlock_unlock(&g_clientAI_map_lock);
                }

                continue;
            }

            // 情况三：已连接客户端发生断开或者异常
            if(event_flags & (EPOLLERR | EPOLLHUP | EPOLLRDHUP)){
                if(epoll_ctl(epoll_fd, EPOLL_CTL_DEL, current_fd, NULL) < 0){
                    perror("epoll_ctl 删除 fd 失败");
                }

                pthread_rwlock_wrlock(&g_client_map_lock);
                for(int i = 0; i < MAX_CONNECTIONS; i++){
                    if(g_client_map[i].fd == current_fd){
                        g_client_map[i].fd = -1;
                        g_client_map[i].valid = false;
                        for(int j = 0; j < CHANNEL_COUNT; j++){
                            g_client_map[i].channel[j] = false;
                        }
                        break;
                    }
                }
                pthread_rwlock_unlock(&g_client_map_lock);

                pthread_rwlock_wrlock(&g_clientAI_map_lock);
                for(int i = 0; i < MAX_CONNECTIONS; i++){
                    if(g_clientAI_map[i].fd == current_fd){
                        g_clientAI_map[i].fd = -1;
                        g_clientAI_map[i].valid = false;
                        for(int j = 0; j < CHANNEL_COUNT; j++){
                            g_clientAI_map[i].channel[j] = false;
                        }
                        break;
                    }
                }
                pthread_rwlock_unlock(&g_clientAI_map_lock);

                close(current_fd);
                continue;
            }

            // 情况四：已有的连接有数据到达
            if(event_flags & EPOLLIN){
                ssize_t recv_len = recv(current_fd, buf, sizeof(buf), 0);

                if(recv_len > 0){
                    buf_len = (size_t)recv_len;

                    if(fd_in_map(current_fd, g_client_map, &g_client_map_lock)){
                        json_handler(current_fd, buf, buf_len, MESSAGE_PORT);
                    }
                    else if(fd_in_map(current_fd, g_clientAI_map, &g_clientAI_map_lock)){
                        json_handler(current_fd, buf, buf_len, VIDEO_PORT);
                    }
                }

                else if(recv_len == 0){
                    // 客户端关闭连接
                    if(epoll_ctl(epoll_fd, EPOLL_CTL_DEL, current_fd, NULL) < 0){
                        perror("epoll_ctl 删除 fd 失败");
                    }

                    pthread_rwlock_wrlock(&g_client_map_lock);
                    for(int i = 0; i < MAX_CONNECTIONS; i++){
                        if(g_client_map[i].fd == current_fd){
                            g_client_map[i].fd = -1;
                            g_client_map[i].valid = false;
                            for(int j = 0; j < CHANNEL_COUNT; j++){
                                g_client_map[i].channel[j] = false;
                            }
                            break;
                        }
                    }
                    pthread_rwlock_unlock(&g_client_map_lock);

                    pthread_rwlock_wrlock(&g_clientAI_map_lock);
                    for(int i = 0; i < MAX_CONNECTIONS; i++){
                        if(g_clientAI_map[i].fd == current_fd){
                            g_clientAI_map[i].fd = -1;
                            g_clientAI_map[i].valid = false;
                            for(int j = 0; j < CHANNEL_COUNT; j++){
                                g_clientAI_map[i].channel[j] = false;
                            }
                            break;
                        }
                    }
                    pthread_rwlock_unlock(&g_clientAI_map_lock);

                    close(current_fd);
                }

                else{
                    perror("recv 失败");
                }
            }
        }

        if(stop_requested) break;
    }

cleanup:
    if(epoll_fd >= 0) close(epoll_fd);
    if(message_fd >= 0) close(message_fd);
    if(video_fd >= 0) close(video_fd);

    /**/
    /** stop_fd 由 server 线程在退出路径统一关闭，主线程只负责写入。 */
    if(g_server_stop_fd >= 0){
        close(g_server_stop_fd);
        g_server_stop_fd = -1;
    }
    /**/

    return NULL;
}
