//frame_callback.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <string>
#include "lx_camera_api.h"
#include "lx_camera_application.h"
#include <lx_camera_define.h>
#include <png.h>

#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
#endif

#include <jpeglib.h>
#include "opencv2/opencv.hpp"

//图像水印函数
#include <chrono>
#include <iomanip>
#include <sstream>


/*****************pgh for server thread*****************/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/un.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>  // 添加这个头文件以声明 getnameinfo 和 NI_NUMERICHOST
#include <net/if.h>  // 添加这个头文件以声明 struct ifreq 和 IFNAMSIZ

#include <ifaddrs.h>
#include <arpa/inet.h>

#include <pthread.h>

#include "./yolov8-seg/include/yolov8_seg.h"
//#include "./yolov8-seg/yolov8_seg.h"

#if 1
// json解析
#include <json/json.h>
#include <iostream>
#include <string>
#endif

#include "defer.h"
#include <signal.h>
#include <sanitizer/lsan_interface.h>


#define FRAME_MAX_SIZE 1024*1024*4      // 一帧数据的最大长度；最大的数据：时间戳长度+时间戳净荷+AI信息长度+AI信息净荷+图片长度+图片净荷
#define FRAME_INFO_MAX_SIZE 1024*1024   // 一帧数据的信息最大长度：最大的数据：时间戳长度+时间戳净荷+AI信息长度+AI信息净荷

#define SRV_MSG_PORT 10001  // 消息端口
#define SRV_VID_PORT 10002  // 视频端口
#define BACKLOG 10
#define MAX_CONNECTED_NO 10
#define MAXDATASIZE 1024*1024*8
#define MAX_MSG_RES 1024
#define MAX_MSG_REQ 1024

#define MAX_CONNECTIONS 16


#define WEATHER_SUNNY       1       // 晴天
#define WEATHER_CLOUDY      2       // 多云
#define BRI_DEFAULT         50     // 亮度
#define CNT_DEFAULT         50     // 对比度
#define SAT_DEFAULT         50     // 饱和度
#define EXP_DEFAULT         50     // 曝光度


typedef struct {
    int fd;
    int chn;         // 客户端请求x通道的数据： 0，1，2，3：相机的4个通道，4：AI处理后的通道
    int frame_cnt;
} s_fd_info;
typedef struct s_client_info {
    int         fd_num;
    s_fd_info   fd_info[MAX_CONNECTIONS];
}s_client_info;

//int g_client_fd[MAX_CONNECTIONS] = {0};
//int g_client_fd_num = 0;
s_client_info g_client_info;

fd_set readfd;
int svr_msg_fd;
int svr_vid_fd;
int max_fd;
int sendto_client_counter[8]={0};
pthread_mutex_t socket_mutex = PTHREAD_MUTEX_INITIALIZER;

DcHandle g_handle = 0;
pthread_mutex_t g_handle_mutex = PTHREAD_MUTEX_INITIALIZER;

/*****************pgh server thread end*****************/

/*****************rt data handle ********************/
// {"cmdId":2002,"dst": {"devId": val,"chnId": val}, "status":val}
#define CMD_ID_START    2002        // 摄像头启动/停止/复位请求,status:启动：0;停止：1;复位：2
#define STA_STOP        0           // 停止
#define STA_START       1           // 启动
#define STA_RESET       2           // 复位

/****** 命令汇总
{
  "cmdId": 2006,
  "dst": {"devId": val,"chnId": val},
  "weather": 1,
  "vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50}
}
 ******/
#define CMD_ID_SET_VID_PARA 2006    // 设置视频参数

// 调式用
#define rknn_using 1

// "weather": {"name":"晴天","value":"待定"},

// 通信命令字枚举类型
typedef enum {
    CMD_CAMERA_MODE_SET             = 2000,     // 摄像头2D模式设置{"cmdId": 2000,"dst": {"devId": 1,"chnId": 1},"mode": 1}   mode 0: 自动, 1: 手动
    CMD_CAMERA_MODE_SET_RES         = 2001,     // 摄像头2D模式设置响应{"cmdId": 2001,"dst": {"devId": 1,"chnId": 1},"mode": 1}
    CMD_CAMERA_START_SET            = 2002,     // 摄像头启动/停止/复位设置{"cmdId": 2002,"dst": {"devId": 1,"chnId": 1},"status": 1}
    CMD_CAMERA_START_SET_RES        = 2003,     // 摄像头启动/停止/复位设置响应{"cmdId": 2003,"dst": {"devId": 1,"chnId": 1},"status": 1}
    CMD_CAMERA_PARA_SET             = 2004,     // 摄像头2D参数设置，目前硬件不支持亮度，对比度，饱和度设置，故只能对曝光、增益进行设置{"cmdId": 2004,"dst": {"devId": 1,"chnId": 1},"vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50,"gai":80}}
    CMD_CAMERA_PARA_SET_RES         = 2005,     // 摄像头2D参数设置响应{"cmdId": 2005,"dst": {"devId": 1,"chnId": 1},"weather": {"name":"晴天","value":"待定"}, "vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50}}
    CMD_CAMERA_RESOLUTION_SET       = 2006,     // 摄像头分辨率设置，3D图像向2D图像分辨率对齐;0：1*1; 1：2*2；2：4*4  {"cmdId": 2006,"dst": {"devId": 1,"chnId": 1},"resolution": 1}
    CMD_CAMERA_RESOLUTION_SET_RES   = 2007,     // 摄像头分辨率设置响应{"cmdId": 2007,"dst": {"devId": 1,"chnId": 1},"resolution": 1}
    CMD_CAMERA_AUTOEXPLEVEL_SET     = 2008,     // 摄像头2D自动曝光级别设置{"cmdId": 2008,"dst": {"devId": 1,"chnId": 1},"autoexplevel": 1}
    CMD_CAMERA_AUTOEXPLEVEL_SET_RES = 2009,     // 摄像头2D自动曝光级别设置响应{"cmdId": 2009,"dst": {"devId": 1,"chnId": 1},"autoExpLevel": 1}
    CMD_CAMERA_AUTOEXP_SET          = 2010,     // 摄像头自动曝光设置{"cmdId": 2010,"dst": {"devId": 1,"chnId": 2},"autoExp": 1}  autoExp: 0: 手动（关闭自动曝光）1：自动
    CMD_CAMERA_AUTOEXP_SET_RES      = 2011,     // 摄像头自动曝光设置响应{"cmdId": 2011,"dst": {"devId": 1,"chnId": 1},"autoExp": 1}
    CMD_CAMERA_3DMODE_SET           = 2012,     // 摄像头3D模式设置{"cmdId": 2012,"dst": {"devId": 1,"chnId": 1},"mode": 1}   mode 0: 自动, 1: 手动
    CMD_CAMERA_3DMODE_SET_RES       = 2013,     // 摄像头3D模式设置响应{"cmdId": 2013,"dst": {"devId": 1,"chnId": 1},"mode": 1}
    CMD_CAMERA_3DPARA_SET           = 2014,     // 摄像头3D参数设置，目前硬件不支持亮度，对比度，饱和度设置，故只能对曝光、增益进行设置{"cmdId": 2004,"dst": {"devId": 1,"chnId": 1},"vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50,"gai":80}}
    CMD_CAMERA_3DPARA_SET_RES       = 2015,     // 摄像头3D参数设置响应{"cmdId": 2005,"dst": {"devId": 1,"chnId": 1},"weather": {"name":"晴天","value":"待定"}, "vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50}}
    CMD_CAMERA_3DAUTOEXPLEVEL_SET   = 2016,     // 摄像头3D自动曝光级别设置{"cmdId": 2008,"dst": {"devId": 1,"chnId": 1},"autoexplevel": 1}
    CMD_CAMERA_3DAUTOEXPLEVEL_SET_RES= 2017,    // 摄像头3D自动曝光级别设置响应{"cmdId": 2009,"dst": {"devId": 1,"chnId": 1},"autoExpLevel": 1}
    CMD_CAMERA_3D_UNDISTORT_EN_SET  = 2018,     // 摄像头3D图像反畸变使能设置{"cmdId": 2018,"dst": {"devId": 1,"chnId": 1},"undistort": 1}  undistort: 0: 关闭，1：开启
    CMD_CAMERA_3D_UNDISTORT_EN_SET_RES= 2019,   // 摄像头3D图像反畸变使能设置响应{"cmdId": 2019,"dst": {"devId": 1,"chnId": 1},"undistort": 1}

    CMD_IMAG_TPYE_SET               = 2020,     // 获取摄像头图像类型设置：0 深度图，1，强度图，2，RGB图，3，APP图，4，AI处理后图 {"cmdId": 2020,"dst": {"devId": 1,"chnId": 1}}
    CMD_IMAG_TPYE_SET_RES           = 2021,     // 获取摄像头图像类型设置响应{"cmdId": 2021,"dst": {"devId": 1,"chnId": 1}}
    CMD_AI_PICKSEQ_SET              = 2040,     // 采摘序列请求设置  {"cmdId": 2040,"dst": {"devId": 1,"chnId": 1}, "pickSeq": "xxx"}
    CMD_AI_PICKSEQ_SET_RES          = 2041,     // 采摘序列请求设置响应{"cmdId": 2041,"dst": {"devId": 1,"chnId": 1}, "pickSeq": "xxx"}
    CMD_AI_MODEL_SET                = 2042,     // 推理模型设置     {"cmdId": 2042,"dst": {"devId": 1,"chnId": 1}, "model": 0}
    CMD_AI_MODEL_SET_RES            = 2043,     // 推理模型设置响应{"cmdId": 2043,"dst": {"devId": 1,"chnId": 1}, "model": 0}
    CMD_SYS_IP_GET                  = 2044,     // 获取系统IP地址   {"cmdId": 2044,"dst": {"devId": 1,"chnId": 1}}
    CMD_SYS_IP_GET_RES              = 2045,     // 获取系统IP地址响应{"cmdId": 2045,"dst": {"devId": 1,"chnId": 1}, "IP": "xxx"}
    CMD_DISP_INFO_SET               = 2060,     // 摄像头滚动信息显示设置  {"cmdId": 2060,"dst": {"devId": 1,"chnId": 1}}
    CMD_DISP_INFO_SET_RES           = 2061,     // 摄像头滚动信息显示设置响应{"cmdId": 2002,"dst": {"devId": 1,"chnId": 1}, "disp":"xxx"}
    CMD_ERR_INFO_REQ                = 2062,     // 错误信息请求  {"cmdId": 2062,"dst": {"devId": 1,"chnId": 1}}
    CMD_ERR_INFO_REQ_RES            = 2063      // 错误信息请求响应{"cmdId": 2063,"dst": {"devId": 1,"chnId": 1}, "err": "xxx"}
} e_cmd;

// Json 解析错误，这个字符串发回给客户端
char ERR_JSON_PARSE[] = "Failed to parse JSON, please check your request commander";

// 定义一个数据结构
typedef struct {
    int cmd;
    int func;
} s_map_cmd2func;

// 跟随common.h中的定义,只是将virt_addr改为静态分配，避免malloc/free带来的释放问题
#define MAX_RGB_IMAGE_SIZE 1024*1024*64
typedef struct {
    int width;
    int height;
    int width_stride;           // reserved
    int height_stride;          // reserved
    image_format_t format;      // IMAGE_FORMAT_RGB888
    unsigned char  virt_addr[MAX_RGB_IMAGE_SIZE];   
    int size;                   // width*height*3
    int fd;                     // reserved
} my_image_buffer_t;


// 将seg_mask指向的图片直接进行空间分配，让这个数据结构直接包含完整的数据，便于将整个数据结构直接发送
#define MAX_SEG_IMAGE_SIZE 1024*16*1 // pghpghpgh,注意这个大小，1024*1024*1出现断错误
typedef struct
{
    int width;
    int height;
    uint8_t pixel_data_length;  // 代表图像每个点的数据长度，目前为sizeof(uint8_t)
    uint8_t type;               // 代表图像类型，0：深度图，1：强度图，2：RGB图，3：APP图。目前固定为2RGB图
    uint8_t seg_mask[MAX_SEG_IMAGE_SIZE];
} object_segment_result_ai;

typedef struct
{
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
    object_segment_result_ai results_seg[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list_ai;

// AI图像处理SDK全部变量

#if rknn_using
rknnNet g_rknnNet;
pthread_mutex_t g_rknnNetMutex = PTHREAD_MUTEX_INITIALIZER;
#endif



int g_ai_model = 0;     // 记录当前AI模型

void rt_camera_start_set(const char* data, int len) {
    ;// 解析data，设置摄像头启动/停止/复位
}  

void rt_disp_info_set(const char* data, int len) {
    ;// 解析data，设置摄像头滚动信息显示
}   

void rt_camera_tpye_set(const char* data, int len) {
    ;// 解析data，设置摄像头图像类型
}   

void rt_camera_para_set(const char* data, int len) {
    ;// 解析data，设置摄像头参数
}   

void rt_ai_pickSeq_set(const char* data, int len) {
    ;// 解析data，设置采摘序列请求
}   

void rt_camera_resolution_set(const char* data, int len) {
    ;// 解析data，设置推理模型
}

void rt_camera_err_set(const char* data, int len) {
    ;// 解析data，设置推理模型
} 

static char wait_key = '0';
#define checkTC(state) {LX_STATE val=state;                                 \
    if(val!= LX_SUCCESS){                                                   \
        if(val == LX_E_NOT_SUPPORT) {                                       \
            printf("mrdvs, not support this operator\n");                          \
        }else if(val == LX_E_RECONNECTING){                                 \
            printf("mrdvs, device reconnecting\n");                                \
        } else  if(val == LX_ERROR || val == LX_E_INPUT_ILLEGAL){           \
            printf("mrdvs, press any key to exit!\n");                             \
            wait_key = getchar();                                        \
         } else{                                                            \
            printf("mrdvs, press any key to continue\n");                          \
            wait_key = getchar();                                        \
         }                                                                  \
     }                                                                      \
}

void CallbackFunc(FrameInfo* frame, void* usr_data);


// 根据fd移除socket连接
int remove_connction_by_fd(int fd, fd_set *read_fds) {
    int i;
    for(i=0; i<MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd == fd){
            g_client_info.fd_info[i].fd = -1; // 标记为无效
            g_client_info.fd_info[i].chn = -1;
            g_client_info.fd_info[i].frame_cnt = 0;
            g_client_info.fd_num--;

            shutdown(fd, SHUT_RDWR);
            close(fd);
            FD_CLR(fd, read_fds); // 从read_fds中移除
            printf("mrdvs, remove connection fd=%d\n", fd);
            return 0;
        }
    }
        
    return -1;
}

// 移除连接
void remove_connection(int index, fd_set *read_fds) {
    int fd = g_client_info.fd_info[index].fd;

    // 文件描述符无效
    if(fd <0){
        printf("mrdvs, remove client_fd[%d]=%d wrong\n", index, g_client_info.fd_info[index].fd);
        return;
    }

    //printf("will remove client_fd[%d]=%d conn. Current connections: %d\n", index, g_client_info.fd_info[index].fd, g_client_info.fd_num);

    // 尝试读取数据以检测客户端是否断开连接
    char buffer[1];
    ssize_t result = recv(g_client_info.fd_info[index].fd, buffer, sizeof(buffer), MSG_PEEK);
    
    if (result == 0 || (result < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) {
        // 客户端已断开连接或发生错误
        shutdown(fd, SHUT_RDWR);
        close(fd);
        g_client_info.fd_info[index].fd = -1; // 标记为无效
        g_client_info.fd_info[index].chn = -1;
        g_client_info.fd_info[index].frame_cnt = 0;
        g_client_info.fd_num--;
        FD_CLR(fd, read_fds); // 从read_fds中移除
        printf("mrdvs, Removed Client client_fd[%d]=%d, Current connections: %d\n", index, fd, g_client_info.fd_num);
    } else {
        // 客户端未断开连接
        printf("mrdvs, Client at index %d still connected\n", index);
    }

    return;
}

static void setreuseport(int fd)
{
    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("mrdvs, setsockopt");
        exit(1);
    }
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        perror("mrdvs, setsockopt");
        exit(1);
    }

    int send_buf_size = 32 * 1024 * 1024; // 设置发送缓冲区为 2MB
    int recv_buf_size = 32 * 1024 * 1024; // 设置接收缓冲区为 2MB
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size)) < 0) {
        perror("mrdvs, setsockopt");
        exit(1);
    }
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size)) < 0) {
        perror("mrdvs, setsockopt");
        exit(1);
    } 

    return;
}

// 获取摄像头模式：0: 手动; 1: 自动
int rt_camera_mode_get(int *mode) {
    int ret = 0;
    bool enable_auto_exposure = false;
    ret = DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, &enable_auto_exposure);
    if(ret != LX_SUCCESS){
        printf("mrdvs, get 2D_AUTO_EXPOSURE failed\n");
    }
    printf("mrdvs, get 2D_AUTO_EXPOSURE success, enable_auto_exposure=%d\n", enable_auto_exposure);
    *mode = enable_auto_exposure ? 1 : 0;
    return ret;
}

// 设置摄像头模式: 0: 手动; 1: 自动; 第2个参数buf是将错误打印信息返回给上层
// LX_BOOL_ENABLE_3D_AUTO_EXPOSURE设置返回-2，摄像头不支持3D自动/手动曝光设置
#define EN_3D_AUTO_EXPOSURE 0
int rt_camera_mode_set(int mode, char *buf, int buf_len) {
    int ret = -1;

#if 0
    //关闭数据流
    checkTC(DcStopStream(g_handle));
#endif

    if(mode != 0 && mode != 1){
        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "mode value out of range, should be 0(auto mode) or 1(manual mode)");
        goto ret_to_father;
    }

    if(mode == 0){
        // 设置手动模式                       
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, false);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 2D_AUTO_EXPOSURE to disable failed");
            goto ret_to_father;
        }

#if EN_3D_AUTO_EXPOSURE
        // LX_BOOL_ENABLE_3D_AUTO_EXPOSURE设置返回-2，摄像头不支持
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, false);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 3D_AUTO_EXPOSURE to disable failed");
            goto ret_to_father;
        }
#endif

        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 2D_AUTO_EXPOSURE to disable success");

    } else {
        // 设置自动模式
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, true);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 2D_AUTO_EXPOSURE to enable failed");
            goto ret_to_father;
        }

#if EN_3D_AUTO_EXPOSURE
        // LX_BOOL_ENABLE_3D_AUTO_EXPOSURE设置返回-2，摄像头不支持
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, true);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"model\":%d,\"ret\":\"%d,%s,%s\"", mode, ret, "failed","set 3D_AUTO_EXPOSURE to enable failed");
            goto ret_to_father;
        }
#endif
        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 2D_AUTO_EXPOSURE to enable success");
    }


#if 0
    
    //开启RGB数据流
    //bool enable_rgb = true;
    //printf("mrdvs, DcSetIntValue RGB set to enable_rgb=%d\n", enable_rgb);
    //checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    //checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));
#endif

ret_to_father:
    printf("%s, %s\n", "mrdvs", buf);
    return ret;

}

int rt_camera_3dmode_set(int mode, char *buf, int buf_len) {
    int ret = -1;

#if 0
    //关闭数据流
    checkTC(DcStopStream(g_handle));
#endif

    if(mode != 0 && mode != 1){
        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "mode value out of range, should be 0(auto mode) or 1(manual mode)");
        goto ret_to_father;
    }

    if(mode == 0){
        // 设置手动模式                       
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, false);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 3D_AUTO_EXPOSURE to disable failed");
            goto ret_to_father;
        }
        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 3D_AUTO_EXPOSURE to disable success");

    } else {
        // 设置自动模式
        ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, true);
        if(ret != LX_SUCCESS){
            snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 3D_AUTO_EXPOSURE to enable failed");
            goto ret_to_father;
        }
        snprintf(buf, buf_len, "\"mode\":%d,\"ret\":\"%d,%s\"", mode, ret, "set 3D_AUTO_EXPOSURE to enable success");
    }


#if 0
    
    //开启RGB数据流
    //bool enable_rgb = true;
    //printf("mrdvs, DcSetIntValue RGB set to enable_rgb=%d\n", enable_rgb);
    //checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    //checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));
#endif

ret_to_father:
    printf("%s, %s\n", "mrdvs", buf);
    return ret;

}

// 发送数据给所有客户端，这个函数只发送系统错误消息，不发送二进制数据
// CMD_ERR_INFO_REQ_RES            = 2063      
// 错误信息请求响应{"cmdId": 2063,"dst": {"devId": 1,"chnId": 1}, "err": "xxx"}
int rt_msg_send_all(const char *buf){
    int i;
    int len_sent = 0;
    char send_buf[MAX_MSG_RES] = {0};

    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }

        // 组包消息{"cmdId": 2063,"dst":{"devId": 1,"chnId": 1}, "err": "xxx"}
        snprintf(send_buf, sizeof(send_buf),
             "{\"cmdId\": %d,\"dst\":{\"devId\": 1,\"chnId\": 1}, \"err\":\"%s\"}",
             CMD_ERR_INFO_REQ_RES , buf);

        // 发送给所有有效的客户端
        len_sent = send(g_client_info.fd_info[i].fd, buf, strlen(buf), 0);
        if (len_sent < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, i, g_client_info.fd_info[i].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                remove_connection(i, &readfd);
                printf("mrdvs, %s, msg send error, client disconnected, errno == EPIPE || errno == ECONNRESET\n", __FUNCTION__);

            } else {
                perror("mrdvs, send error msg error");
            }
        }
        printf("mrdvs, send error msg success, len_sent=%d, client_fd[%d]=%d, send buf:%s\n", len_sent, i, g_client_info.fd_info[i].fd, buf);
    }
    return len_sent;
}   


// 从socket获取端口号
int socket_port_get(int fd, int *port)
{
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    if (getsockname(fd, (struct sockaddr *)&addr, &len) == -1) {
        perror("mrdvs, getsockname");
        rt_msg_send_all("socket port get: getsockname error");
        return -1; 
    }
    *port = ntohs(addr.sin_port);
    return 0;
}

int sendall(int sockfd, const void *buf, size_t len) {
    size_t total = 0;        // 已发送的字节数
    size_t bytesleft = len;  // 剩余要发送的字节数
    ssize_t n;

    while (total < len) {
        n = send(sockfd, (const char *)buf + total, bytesleft, 0);
        if (n == -1) {
            if (errno == EINTR) {
                continue; // 如果被信号中断，继续发送
            }
            break; // 发送失败
        }
        total += n;
        bytesleft -= n;
    }

    return n == -1 ? -1 : 0; // 返回-1表示失败，0表示成功
}

/**********img数据格式************/
// 1. 数据头：4字节 0xff 0xff 0xff 0xff
// 2. 数据长度：4字节，表示后续数据的长度
/**********img数据格式 end************/
// fd_socket: select 的监听fd；fd_src: 数据来源fd；buf: 数据缓冲区；len: 数据长度
void data_send(int fd_src, int fd_socket, char *buf, int len){
    int ret=0;
    int i=0;

    for (int i = 1; i <= max_fd; i++){
        if (FD_ISSET(i, &readfd)) {
            if((i == fd_socket)||(i == fd_src)){
                continue;
            }
            
            //ret = sendall(i, buf, len);
            ret = send(i, buf, len, 0);
            if (ret <= 0) {
                //perror("send data error socke id=%ld, len=%d, ret=%ld\n", i, len, ret);
                printf("mrdvs, send data error socke id=%d, len=%d, ret=%d\n", i, len, ret);
                rt_msg_send_all("send data error");
                continue;
            }

            g_client_info.fd_info[i].frame_cnt++;
            //printf("%s:send success! fd_src=%d, fd_dst=%d, data_len=%d,sent_len=%d, send_counter=%d\n", 
            //        __FUNCTION__, fd_src, fd_socket, i, ret, g_client_info.fd_info[i].frame_cnt);       
        }
    }

    return;
}

// 目前不实现数据头的解析，只是简单的解析json数据：{"cmdId":2002,"dst": {"devId": val,"chnId": val},"status":val}
// 定义结构体来存储解析后的数据
struct dst {
    int dev_id;
    int chn_id;
};

struct parsed_data {
    int cmd_id;
    dst dst_info;
    int status;
};

// AI处理后的数据转发给其他客户端
void data_forward(int chn, int index, char *buf, int len){
    int j;
    int port = 0;
    unsigned long  ret = 0;

    for(j = 0; j < MAX_CONNECTIONS; j++){
        if(g_client_info.fd_info[j].fd < 0){
            continue;
        }

        // 查看客户端的chn，只有请求channel 4 的客户端才转发数据
        if(g_client_info.fd_info[j].chn != chn){
            continue;
        }

        ret = send(g_client_info.fd_info[j].fd, buf, len, 0);
        if (ret < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, j, g_client_info.fd_info[j].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                //remove_connection(i, &readfd);
                printf("mrdvs, forward data error, client disconnected, errno == EPIPE || errno == ECONNRESET\n");

            } else {
                perror("mrdvs, forward error");
                rt_msg_send_all("mrdvs, forward data error");
            }
        }
        g_client_info.fd_info[j].frame_cnt++;   // 发送帧计数
        if(g_client_info.fd_info[j].frame_cnt % 100 == 0){
            printf("mrdvs, %s: Server --> client AI data. chn:%d, [%d]:%d, frame_cnt:%d, sent len:%ld\n", 
                __FUNCTION__, chn, index,g_client_info.fd_info[j].fd, g_client_info.fd_info[j].frame_cnt, ret); 
        }
    } 
    return;

}

// 注意这个函数只发送json消息，不发送二进制数据
int rt_msg_send(int index, char *buf){
    int len_sent = 0;

    // 发送给指定客户端
    len_sent = send(g_client_info.fd_info[index].fd, buf, strlen(buf), 0);
    if (len_sent < 0) {
        printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                errno, index, g_client_info.fd_info[index].fd);
        if (errno == EPIPE || errno == ECONNRESET) {
            // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
            remove_connection(index, &readfd);
            printf("mrdvs, %s, msg send error, client disconnected, errno == EPIPE || errno == ECONNRESET\n", __FUNCTION__);

        } else {
            perror("mrdvs, msg send error");
            rt_msg_send_all("mrdvs, send error msg error");
        }
    }
    printf("mrdvs, msg send :    Server --> Client. len_sent=%d, client_fd[%d]=%d, send buf:%s\n", 
        len_sent, index, g_client_info.fd_info[index].fd, buf);
    return len_sent;
}

// 通用消息响应处理函数
int rt_common_msg_res(int index, int cmd, int chn, char *buf){
    int ret = 0;

    char send_buf[MAX_MSG_RES] = {0};

    // 组包消息头{"cmdId": 2006,"dst": {"devId": 1,"chnId": 1},"IP": "pghpghpgh"}
    snprintf(send_buf, sizeof(send_buf),
             "{\"cmdId\": %d, \"dst\": {\"devId\": 1,\"chnId\": %d}, %s}",
             cmd , chn, buf);
    ret = rt_msg_send(index, send_buf);
    if(ret < 0){
        printf("mrdvs, %s, msg send error, client_fd[%d]=%d\n", __FUNCTION__, index, g_client_info.fd_info[index].fd);
    }
    if(ret != strlen(send_buf)){
        printf("mrdvs, %s, msg sent error, sent len != send len, client_fd[%d]=%d, ret=%d, send_buf=%s\n", 
                __FUNCTION__, index, g_client_info.fd_info[index].fd, ret, send_buf);
    }
    return 0;
}


    // 硬件现在不支持对比度、饱和度设置；只支持曝光度、增益设置。深度图是3D，可以直接设置曝光值; 2D图像是RGB，需要关闭自动曝光，然后设置曝光值
    // 1. 设置自动曝光等级 0-100： 需要先打开自动曝光
        // DcSetIntValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, true);
        // DcSetIntValue(g_handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, exp);
    
    // 2. 设置手动曝光等级 0-1000： 需要先关闭自动曝光
    // DcSetIntValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, false);
    // DcSetIntValue(g_handle, LX_INT_2D_MANUAL_EXPOSURE, exp);
        /*
    2D图像曝光、增益参数设置：2D图像是RGB，需要关闭自动曝光，然后设置曝光值
    LX_INT_2D_MANUAL_EXPOSURE = 1051,   //2D手动曝光时的曝光值
    LX_INT_2D_MANUAL_GAIN = 1052,       //2D手动曝光时的增益值
    LX_INT_2D_AUTO_EXPOSURE_LEVEL = 1054,//2D图像自动曝光时曝光等级[0-100], 0-整体更暗， 100-整体更亮
    */
/*
    
*/

//摄像头参数设置，目前硬件不支持亮度，对比度，饱和度设置，故只能对曝光(0~10000)、增益进行设置(~500)
//{"cmdId": 2004,"dst": {"devId": 1,"chnId": 1},"vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50,"gai":80}}
#define MAX_EXP_3D_VALUE 3000 
#define MIN_EXP_3D_VALUE 1
#define MAX_EXP_VALUE 10000 // 定义 MAX_EXP_VALUE 常量
#define MIN_EXP_VALUE 0
#define MAX_GAI_VALUE 11088
#define MIN_GAI_VALUE 100
#define MAX_EXPLEVEL_VALUE 100 
#define MIN_EXPLEVEL_VALUE 0

/*先设置曝光参数，如果错误，就不会设置增益参数*/
int set_camera_para(int index, int bri, int cnt, int sat, int exp, int gai){
    int ret = -1;
    int ret_gai = -1;
    char buf[256]={0};

    // 检查 exp 参数是否在有效范围内
    if (exp < 0 || exp > MAX_EXP_VALUE) {
        printf("mrdvs, exp value out of range, should be %d-%d\n", MIN_EXP_VALUE, MAX_EXP_VALUE);
        //返回消息
        snprintf(buf, sizeof(buf), "\"bri\":%d, \"cnt\":%d, \"sat\":%d, \"exp\":%d, \"gai\":%d, \"ret\":\"%d, %s %d-%d\"", 
                    bri, cnt, sat, exp, gai, ret, "exp value invalid, should be",MIN_EXP_VALUE, MAX_EXP_VALUE);
        ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES, 1, buf);      
        return -1;
    }
    // 检查 gai 参数是否在有效范围内
    if(gai < MIN_GAI_VALUE || gai > MAX_GAI_VALUE){
        printf("mrdvs, gai value out of range, should be %d-%d\n", MIN_GAI_VALUE, MAX_GAI_VALUE);
        //返回消息
        snprintf(buf, sizeof(buf), "\"bri\":%d,\"cnt\":%d,\"sat\":%d,\"exp\":%d,\"gai\":%d,\"ret\":\"%d,%s %d-%d\"", 
                    bri, cnt, sat, exp, gai, ret, "gain value invalid, should be", MIN_GAI_VALUE, MAX_GAI_VALUE);
        ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES, 1, buf);
        return -1;
    }

    // 判断模式，自动和手动模式，自动模式不能进行曝光、增益设置
    bool auto_exp = false;
    ret = DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, &auto_exp);
    if(ret != LX_SUCCESS){
        printf("mrdvs, get device 2D_AUTO_EXPOSURE value failed\n");
        //返回消息
        snprintf(buf, sizeof(buf), "\"bri\":%d, \"cnt\":%d, \"sat\":%d, \"exp\":%d, \"gai\":%d, \"ret\":\"%d, %s\"", 
                    bri, cnt, sat, exp, gai, ret, "get device 2D_AUTO_EXPOSURE value failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES, 1, buf);
        return -1;
    }

    ret = -1; // 重新初始化ret为-1，用于后续判断和返回打印信息
    if(auto_exp){
        // 自动模式，不能进行曝光、增益设置
        printf("mrdvs, device 2D_AUTO_EXPOSURE is auto mode, can not set exp and gai, current auto_exp:%d\n", auto_exp);
        //返回消息
        snprintf(buf, sizeof(buf), "\"bri\":%d, \"cnt\":%d, \"sat\":%d, \"exp\":%d, \"gai\":%d, \"ret\":\"%d, %s\"", 
                    bri, cnt, sat, exp, gai, ret, "device 2D_AUTO_EXPOSURE is auto mode, can not set exp and gai, current auto_exp=true");
        ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES, 1, buf);
        return -1;
    }

    // 2D  并设置曝光值和增益值
    ret = DcSetIntValue(g_handle, LX_INT_2D_MANUAL_EXPOSURE, exp);
    ret_gai = DcSetIntValue(g_handle, LX_INT_2D_MANUAL_GAIN, gai);
    if(ret != LX_SUCCESS){
        printf("mrdvs, set LX_INT_2D_MANUAL_EXPOSURE/GAIN failed\n");
        //返回消息
        snprintf(buf, sizeof(buf), "\"bri\":%d, \"cnt\":%d, \"sat\":%d, \"exp\":%d, \"gai\":%d, \"ret\":\"%d, %s\"", 
                    bri, cnt, sat, exp, gai, ret, "failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES,1, buf);
        return -1;
    }

    //返回消息
    snprintf(buf, sizeof(buf), "\"bri\":%d, \"cnt\":%d, \"sat\":%d, \"exp\":%d, \"gai\":%d, \"ret\":\"%d, %s\"", 
                    bri, cnt, sat, exp, gai, ret, "success");
    ret = rt_common_msg_res(index, CMD_CAMERA_PARA_SET_RES, 1, buf);

#if 0
    //关闭数据流
    checkTC(DcStopStream(g_handle));     
    //开启RGB数据流
    bool enable_rgb = true;
    printf("mrdvs, DcSetIntValue RGB set to enable_rgb=%d\n", enable_rgb);
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));
#endif

    return ret;
}

int set_camera_3Dpara(int index, int exp){
    int ret = -1;
    int ret_gai = -1;
    char buf[256]={0};

    // 检查 exp 参数是否在有效范围内
    if (exp < MIN_EXP_3D_VALUE || exp > MAX_EXP_3D_VALUE) {
        printf("mrdvs, 3D exp value invalid, should be %d-%d\n", MIN_EXP_3D_VALUE, MAX_EXP_3D_VALUE);
        //返回消息
        snprintf(buf, sizeof(buf), "\"exp\":%d, \"ret\":\"%d, %s %d-%d\"", 
                    exp, ret, "3D exp value invalid, should be",MIN_EXP_3D_VALUE, MAX_EXP_3D_VALUE);
        ret = rt_common_msg_res(index, CMD_CAMERA_3DPARA_SET_RES, 1, buf);      
        return -1;
    }

    // 判断模式，自动和手动模式，自动模式不能进行曝光、增益设置
    // 目前硬件不支持3D LX_BOOL_ENABLE_3D_AUTO_EXPOSURE 自动/手动曝光获取
#if 0
    bool auto_exp = false;
    ret = DcGetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, &auto_exp);
    if(ret != LX_SUCCESS){
        printf("mrdvs, get device 3D_AUTO_EXPOSURE value failed\n");
        //返回消息
        snprintf(buf, sizeof(buf), "\"exp\":%d, \"ret\":\"%d, %s\"", exp, ret, "get device 3D_AUTO_EXPOSURE value failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_3DPARA_SET_RES, 1, buf);
        return -1;
    }

    ret = -1; // 重新初始化ret为-1，用于后续判断和返回打印信息
    if(auto_exp){
        // 自动模式，不能进行曝光、增益设置
        printf("mrdvs, device 3D_AUTO_EXPOSURE is auto mode, can not set exp, current auto_exp:%d\n", auto_exp);
        //返回消息
        snprintf(buf, sizeof(buf), "\"exp\":%d, \"ret\":\"%d, %s\"", 
                    exp, ret, "device 3D_AUTO_EXPOSURE is auto mode, can not set exp, current auto_exp=true");
        ret = rt_common_msg_res(index, CMD_CAMERA_3DPARA_SET_RES, 1, buf);
        return -1;
    }
#endif

    // 3D 曝光设置
    ret = DcSetIntValue(g_handle, LX_INT_FIRST_EXPOSURE, exp);
    if(ret != LX_SUCCESS){
        printf("mrdvs, set LX_INT_FIRST_EXPOSURE failed\n");
        //返回消息
        snprintf(buf, sizeof(buf), "\"exp\":%d, \"ret\":\"%d, %s\"", exp, ret, "failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_3DPARA_SET_RES,1, buf);
        return -1;
    }

    //返回消息
    snprintf(buf, sizeof(buf), "\"exp\":%d, \"ret\":\"%d, %s\"",exp,ret, "success");
    ret = rt_common_msg_res(index, CMD_CAMERA_3DPARA_SET_RES, 1, buf);

#if 0
    //关闭数据流
    checkTC(DcStopStream(g_handle));     
    //开启RGB数据流
    bool enable_rgb = true;
    printf("mrdvs, DcSetIntValue RGB set to enable_rgb=%d\n", enable_rgb);
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));
#endif

    return ret;
}

// 摄像头自动曝光设置: 0: 关闭; 1: 打开
int set_camera_autoexp(int index, int chn, int autoexp){
    int ret = -1;
    char buf[256]={0};
    bool enable_rgb = true;

    if(autoexp != 0 && autoexp != 1){
        printf("mrdvs, autoexp=%d value out of range, should be 0 or 1\n", autoexp);
        snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "autoexp value out of range, should be 0 or 1");

        goto ret_to_father;
    }
    if(chn !=2){
        printf("mrdvs, chn=%d value error, should be 2\n", chn);
        snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "chn value error, should be 2");
        goto ret_to_father;
    }

    if(chn == 2){
        if(autoexp == 0){
            // 关闭自动曝光
            ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, false);
            if(ret != LX_SUCCESS){
                printf("mrdvs, DcSetBoolValue failed\n");
                snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "DcSetBoolValue failed");
                goto ret_to_father;
            }
            printf("mrdvs, DcSetBoolValue set LX_BOOL_ENABLE_2D_AUTO_EXPOSURE success \n");
            snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "DcSetBoolValue set LX_BOOL_ENABLE_2D_AUTO_EXPOSURE success");
        } else {
            // 打开自动曝光
            ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, true);
            if(ret != LX_SUCCESS){
                printf("mrdvs, DcSetBoolValue failed\n");
                snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "DcSetBoolValue failed");
                goto ret_to_father;
            }
            printf("mrdvs, DcSetBoolValue set LX_BOOL_ENABLE_2D_AUTO_EXPOSURE success \n");
            snprintf(buf, sizeof(buf), "\"autoexp\":%d,\"ret\":\"%d,%s\"", autoexp, ret, "DcSetBool Value set LX_BOOL_ENABLE_2D_AUTO_EXPOSURE success");
        }
    }

    //关闭数据流
    checkTC(DcStopStream(g_handle));
    
    //开启RGB数据流
    printf("mrdvs, DcSetBoolValue set and LX_BOOL_ENABLE_2D_AUTO_EXPOSURE=%d\n", autoexp);
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));

 ret_to_father:
    ret = rt_common_msg_res(index, CMD_CAMERA_AUTOEXP_SET_RES, chn, buf);   
    return ret;

}

// CMD_CAMERA_3D_UNDISTORT_EN_SET = 2019, 3D畸变校正使能设置
int set_camera_3Dundistort(int index, int undistort_en){
    int ret = -1;
    char buf[256]={0};

    if(undistort_en != 0 && undistort_en != 1){
        printf("mrdvs, undistort_en=%d value invalid, should be 0 or 1\n", undistort_en);
        snprintf(buf, sizeof(buf), "\"undistort_en\":%d,\"ret\":\"%d,%s\"", undistort_en, ret, "undistort_en value invalid, should be 0 or 1");
        goto ret_to_father;
    }

    if(undistort_en == 0){
            ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_UNDISTORT, false);
            if(ret != LX_SUCCESS){
                printf("mrdvs, set LX_BOOL_ENABLE_3D_UNDISTORT failed\n");
                snprintf(buf, sizeof(buf), "\"undistort_en\":%d,\"ret\":\"%d,%s\"", undistort_en, ret, "DcSetBoolValue LX_BOOL_ENABLE_3D_UNDISTORT failed");
                goto ret_to_father;
            }
            printf("mrdvs, DcSetBoolValue set LX_BOOL_ENABLE_3D_UNDISTORT to false success \n");
            snprintf(buf, sizeof(buf), "\"undistort_en\":%d,\"ret\":\"%d,%s\"", undistort_en, ret, "DcSetBoolValue set LX_BOOL_ENABLE_3D_UNDISTORT to false success");
        } else {
            ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_UNDISTORT, true);
            if(ret != LX_SUCCESS){
                printf("mrdvs, DcSetBoolValue failed\n");
                snprintf(buf, sizeof(buf), "\"undistort_en\":%d,\"ret\":\"%d,%s\"", undistort_en, ret, "DcSetBoolValue set LX_BOOL_ENABLE_3D_UNDISTORT to true failed");
                goto ret_to_father;
            }
            printf("mrdvs, DcSetBoolValue set LX_BOOL_ENABLE_3D_UNDISTORT to true success \n");
            snprintf(buf, sizeof(buf), "\"undistort_en\":%d,\"ret\":\"%d,%s\"", undistort_en, ret, "DcSetBool Value set LX_BOOL_ENABLE_3D_UNDISTORT to true success");
        }

 ret_to_father:
    ret = rt_common_msg_res(index, CMD_CAMERA_3D_UNDISTORT_EN_SET_RES, 1, buf);   
    return ret;
}


    // 硬件现在不支持对比度、饱和度设置；只支持曝光度、增益设置。深度图是3D，可以直接设置曝光值; 2D图像是RGB，需要关闭自动曝光，然后设置曝光值
    // 目前摄像头底层SDK不支持自动曝光等级设置
    // 1. 设置自动曝光等级 0-100： 需要先打开自动曝光
        // DcSetIntValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, true);
        // DcSetIntValue(g_handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, exp);
    /*
    2D图像曝光、增益参数设置：2D图像是RGB，需要关闭自动曝光，然后设置曝光值
    LX_INT_2D_MANUAL_EXPOSURE = 1051,   //2D手动曝光时的曝光值
    LX_INT_2D_MANUAL_GAIN = 1052,       //2D手动曝光时的增益值
    LX_INT_2D_AUTO_EXPOSURE_LEVEL = 1054,//2D图像自动曝光时曝光等级[0-100], 0-整体更暗， 100-整体更亮
    */
int set_camera_autoexplevel(int index, int autoexplevel){
    int ret = -1;
    char buf[256]={0};

    // 检查 exp 参数是否在有效范围内
    if (autoexplevel < MIN_EXPLEVEL_VALUE || autoexplevel > MAX_EXPLEVEL_VALUE) {
        printf("mrdvs, Exposure value out of range\n");
        snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "Exposure value out of range, should be 0-100");
        ret = rt_common_msg_res(index, CMD_CAMERA_AUTOEXPLEVEL_SET_RES, 1, buf);
        return -1;
    }      
    
    //关闭数据流
    checkTC(DcStopStream(g_handle));
    
    // 打开自动曝光 2D  并设置曝光值和增益值                        
    ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, true);
    ret = DcSetIntValue(g_handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, autoexplevel);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcSetIntValue failed\n");
        snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "DcSetIntValue failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_AUTOEXPLEVEL_SET_RES, 1, buf);
        return -1;
    }
        

    sleep(1);
    //开启RGB数据流
    bool enable_rgb = true;
    printf("mrdvs, DcSetIntValue set and LX_INT_2D_AUTO_EXPOSURE_LEVEL=%d\n", autoexplevel);
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));

    printf("mrdvs, DcSetIntValue set and LX_INT_2D_AUTO_EXPOSURE_LEVEL success \n");
    snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "success");
    ret = rt_common_msg_res(index, CMD_CAMERA_AUTOEXPLEVEL_SET_RES, 1, buf);
    return ret;
}

int set_camera_3Dautoexplevel(int index, int autoexplevel){
    int ret = -1;
    char buf[256]={0};

    // 检查 exp 参数是否在有效范围内
    if (autoexplevel < MIN_EXPLEVEL_VALUE || autoexplevel > MAX_EXPLEVEL_VALUE) {
        printf("mrdvs, 3D Exposure value out of range\n");
        snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "3D Exposure value out of range, should be 0-100");
        ret = rt_common_msg_res(index, CMD_CAMERA_AUTOEXPLEVEL_SET_RES, 1, buf);
        return -1;
    }      
    
    //关闭数据流
    checkTC(DcStopStream(g_handle));
    
    // 打开自动曝光 2D  并设置曝光值和增益值                        
    ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, true);
    if(ret != LX_SUCCESS){
        printf("mrdvs, 3D LX_BOOL_ENABLE_3D_AUTO_EXPOSURE failed\n");
        snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "3D LX_BOOL_ENABLE_3D_AUTO_EXPOSURE failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_3DAUTOEXPLEVEL_SET_RES, 1, buf);
        return -1;
    }
    ret = DcSetIntValue(g_handle, LX_INT_3D_AUTO_EXPOSURE_LEVEL, autoexplevel);
    if(ret != LX_SUCCESS){
        printf("mrdvs, 3D LX_INT_3D_AUTO_EXPOSURE_LEVEL failed\n");
        snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "3D LX_INT_3D_AUTO_EXPOSURE_LEVEL failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_3DAUTOEXPLEVEL_SET_RES, 1, buf);
        return -1;
    }
        
    sleep(1);
    //开启RGB数据流
    bool enable_rgb = true;
    printf("mrdvs, DcSetIntValue set and LX_INT_2D_AUTO_EXPOSURE_LEVEL=%d\n", autoexplevel);
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(g_handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &enable_rgb));
    //开启数据流
    checkTC(DcStartStream(g_handle));

    printf("mrdvs, DcSetIntValue set and LX_INT_2D_AUTO_EXPOSURE_LEVEL success \n");
    snprintf(buf, sizeof(buf), "\"autoexplevel\":%d,\"ret\":\"%d,%s\"", autoexplevel, ret, "success");
    ret = rt_common_msg_res(index, CMD_CAMERA_3DAUTOEXPLEVEL_SET_RES, 1, buf);
    return ret;
}


void rt_resolution_get(){
    ;
}

/*设置摄像头分辨率*/
#define LX_STREAM_OFFON 1
int rt_resolution_set(int index, int mode){
    int lx_binning_mode = -1;
    int ret = -1;
    char buf[256]={0};

    // 设置摄像头分辨率, 目前摄像头SDK API只支持1X1, 2X2
    switch(mode){
        case 0:
            lx_binning_mode = LX_BINNING_1X1;
            break;
        case 1:
            lx_binning_mode = LX_BINNING_2X2;
            break;
        case 2:
            lx_binning_mode = LX_BINNING_4X4;;
            break;
        default:
            printf("mrdvs, set to invalid resolution=%d, should be 0:1X1, 1:2X2,2:4X4\n", mode);
            snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s%d,%s\"", mode, ret, "set to invalid resolution:",mode,"should be 0:1X1, 1:2X2,2:4X4");
            ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
            return -1;
    }

#if LX_STREAM_OFFON

    ret = DcUnregisterFrameCallback(g_handle);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcUnregisterFrameCallback failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcUnregisterFrameCallback failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }

    //关闭数据流
    ret = DcStopStream(g_handle);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcStopStream failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcStopStream failed");
            ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }
#endif

    sleep(1);
    ret = DcSetIntValue(g_handle, LX_INT_2D_BINNING_MODE, lx_binning_mode);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcSetIntValue failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcSetIntValue failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }

#if LX_STREAM_OFFON
    //开启RGB数据流
    bool enable_rgb = true;
    printf("mrdvs, DcSetIntValue RGB set to enable_rgb=%d,LX_BINNING_MODE=%dn", enable_rgb, mode);
    ret = DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcSetBoolValue failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcSetBoolValue failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES,1, buf);
        return -1;
    }
    ret = DcGetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcGetBoolValue failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcGetBoolValue failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }

    //开启数据流
    //开启深度图、强度图和RGB图
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, true));
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_3D_AMP_STREAM, true));
    checkTC(DcSetBoolValue(g_handle, LX_BOOL_ENABLE_2D_STREAM, true));

    ret = DcRegisterFrameCallback(g_handle, CallbackFunc, nullptr);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcRegisterFrameCallback failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcRegisterFrameCallback failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }

    ret = DcStartStream(g_handle);
    if(ret != LX_SUCCESS){
        printf("mrdvs, DcStartStream failed\n");
        snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "DcStartStream failed");
        ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
        return -1;
    }   
#endif

    snprintf(buf, sizeof(buf), "\"resolution\":%d,\"ret\":\"%d,%s\"", lx_binning_mode, ret, "success");
    printf("mrdvs, set resolution=%d success\n", lx_binning_mode);
    ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);

    return ret;
}

int rt_ai_model_set(int index, int model) {
    int ret = -1;
    char buf[256]={0};

    // 设置摄像头分辨率, 目前摄像头SDK API只支持1X1, 2X2
    switch(model){
        case 0:
            break;
        case 1:
            break;
        default:
            printf("mrdvs, set to invalid AI model=%d, should be 0 or 1\n", model);
            snprintf(buf, sizeof(buf), "\"model\":%d,\"ret\":\"%d,%s%d,%s\"", model, ret, "set to invalid AI model:",model,"should be 0 or 1");
            ret = rt_common_msg_res(index, CMD_CAMERA_RESOLUTION_SET_RES, 1, buf);
            return -1;
    }

#if rknn_using    
    pthread_mutex_lock(&g_rknnNetMutex);
    ret = g_rknnNet.rknn_envs_switch(model);
    pthread_mutex_unlock(&g_rknnNetMutex);
#endif

    if(ret == 0){
        printf("mrdvs, set AI model=%d success\n", model);
        snprintf(buf, sizeof(buf), "\"model\":%d,\"ret\":\"%d,%s%d,%s\"", model, ret, "set AI model to ",model,"success");
        g_ai_model = model;
    }else{
        printf("mrdvs, set AI model=%d failed\n", model);
        snprintf(buf, sizeof(buf), "\"model\":%d,\"ret\":\"%d,%s%d,%s\"", model, ret, "set AI model to ",model,"failed");
    }
    ret = rt_common_msg_res(index, CMD_AI_MODEL_SET_RES, 1, buf);
    return ret;
}   


int get_mac_address(const char* ifname, char* mac, size_t mac_len) {
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/address", ifname);
    FILE* file = fopen(path, "r");
    if (file == NULL) {
        perror("fopen");
        rt_msg_send_all("mrdvs, get_mac_address: fopen error");
        return -1;
    }
    if (fgets(mac, mac_len, file) == NULL) {
        perror("fgets");
        rt_msg_send_all("mrdvs, get_mac_address: fgets error");
        fclose(file);
        return -1;
    }
    // 去掉换行符
    mac[strcspn(mac, "\n")] = '\0';
    fclose(file);
    return 0;
}

int rt_system_ip_get(int index, int cmd){
    int ret=-1;
    struct ifaddrs *ifaddr, *ifa;
    char send_buf[MAX_MSG_RES] = {0};
    char ip[32]={0};
    char mac[18]={0};  // MAC 地址的格式为 "XX:XX:XX:XX:XX:XX"

    if (getifaddrs(&ifaddr) == -1) {
        perror("mrdvs, getifaddrs");
        rt_msg_send_all("mrdvs, getifaddrs error");
        return ret;
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        int family = ifa->ifa_addr->sa_family;

        if (family == AF_INET) {
            if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            ip, MAX_MSG_RES, NULL, 0, NI_NUMERICHOST) == 0) {
                // 过滤掉 127.0.0.1
                if (strcmp(ip, "127.0.0.1") != 0) {
                    // 获取 MAC 地址
                    if (get_mac_address(ifa->ifa_name, mac, sizeof(mac)) == 0) {
                        printf("mrdvs, Interface: %s\tAddress: %s\tMAC: %s\n", ifa->ifa_name, ip, mac);

                        // 组包消息并发送
                        snprintf(send_buf, sizeof(send_buf),
                                "{\"cmdId\":%d, \"dst\": {\"devId\":1,\"chnId\":1}, \"IP\":\"%s\", \"MAC\":\"%s\"}",
                                cmd + 1, ip, mac);
                        ret = rt_msg_send(index, send_buf);
                    } else {
                        perror("mrdvs, ioctl");
                        rt_msg_send_all("mrdvs, get_mac_address error");
                    }
                }
            }
        }
    }
    
    freeifaddrs(ifaddr);
    return ret;
}

// 针对客户端消息请求的消息响应函数
int msg_send_res(int index, int cmd, char *buf){
    int ret = 0;
    char send_buf[MAX_MSG_RES] = {0};

    // 组包消息头{"cmdId": 2006,"dst": {"devId": 1,"chnId": 1},"IP": "pghpghpgh"}
    snprintf(send_buf, sizeof(send_buf),
             "{\"cmdId\": %d, \"dst\": {\"devId\": 1,\"chnId\": 1}, \"%s}",
             cmd , buf);
    ret = rt_msg_send(index, send_buf);
    return ret;
}

int max_connection_check(){
    if(g_client_info.fd_num >= MAX_CONNECTIONS){
        printf("mrdvs, max connection reached connection=%d, can't accept more connections\n", g_client_info.fd_num);
        return -1;
    }
    printf("mrdvs, current connection number=%d, will start to send to this new socket\n", g_client_info.fd_num);
    return 0;
}

// 解析消息，并进行处理
void data_handler_msg(int index, char *buf, int len){
    int ret = 0;
    char buf_res[MAX_MSG_RES] = {0};

    buf[len] = '\0';
            
    printf("mrdvs, msg received : Server <-- Client. client_fd[%d]=%d, len=%d, buf:%s\n", 
            index, g_client_info.fd_info[index].fd, len, buf);
    // 将 buf 转换为 std::string
    std::string json_str(buf, len);

    // 解析 JSON 字符串
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;

    std::istringstream s(json_str);
    if (!Json::parseFromStream(reader, s, &root, &errs)) {
        //std::cerr << "mrdvs, Failed to parse JSON: " << errs << std::endl;
        std::cerr << "mrdvs, Failed to parse JSON: " << errs << ", received message: " << json_str << std::endl;
        ret = rt_common_msg_res(index, CMD_ERR_INFO_REQ_RES, 1, ERR_JSON_PARSE);
        return;
    }

/****提取数据并填充结构体, 目前解析以下几个消息 
1. {"cmdId":2002,"dst": {"devId": val,"chnId": val},"status":val} 摄像头启动/停止/复位请求
2. {"cmdId":2012} AI处理后视频发送请求, 
*************************************************/ 

    parsed_data parsedData;
    parsedData.cmd_id = root["cmdId"].asInt();
    parsedData.dst_info.dev_id = root["dst"]["devId"].asInt();
    parsedData.dst_info.chn_id = root["dst"]["chnId"].asInt();

    // ==================== pnz 2026/10/19 新增：通用命令分发 BEGIN ====================
    /*
     * 用 X-Macro 清单描述差异，用处理宏生成重复的 if 分支。
     * 仅使用标准 C/C++ 预处理语法，不依赖 typeof、语句表达式或编译器扩展。
     *
     * MODE 清单列：命令名片段、函数名片段、JSON 字段。
     * 例如 (MODE, mode, "mode") 中的 ## 会生成 CMD_CAMERA_MODE_SET、
     * CMD_CAMERA_MODE_SET_RES、rt_camera_mode_set；大小写差异显式列出。
     * 新增同签名模式命令，只需增加一行 X(...)，无需复制处理流程。
     */
#define PNZ_CAMERA_MODE_COMMANDS(X) \
    X(MODE,   mode,   "mode")       \
    X(3DMODE, 3dmode, "mode")

    CMD_CAMERA_PGH_SET


    define PNZ_CAMERA_MODE_CASE(COMMAND, FUNCTION, JSON_KEY, p1);


    xxx_req;
    xxx_res;
    xxx_set;
    xxx_get;


#define PNZ_CAMERA_MODE_CASE(COMMAND, FUNCTION, JSON_KEY, p1)                 \
    do {                                                               \
        if (parsedData.cmd_id == CMD_CAMERA_##COMMAND##_SET) {          \
            const int pnz_value = root[JSON_KEY].asInt();               \
            char pnz_response[256] = {0};                               \
            ret = rt_camera_##FUNCTION##_set(                          \
                pnz_value, pnz_response, sizeof(pnz_response));          \
            ret = rt_common_msg_res(                                   \
                index, CMD_CAMERA_##COMMAND##_SET_RES, 1, pnz_response); \
            return;                                                    \
        }                                                              \
    } while (0);

    PNZ_CAMERA_MODE_COMMANDS(PNZ_CAMERA_MODE_CASE)

    /*
     * INT 清单列：完整命令常量、JSON 字段、处理函数、实参列表、
     * 读取前操作、读取后操作（两项用于保持原日志相对 JSON 读取的顺序）。
     * 命令和函数命名不统一，故允许显式传入；ARGUMENTS 用括号包住逗号，
     * 避免被预处理器当作多个宏参数。pnz_value 是处理宏读取的整数值。
     * 无日志时传 (void)0；增加同类命令只需增加一行。
     *
     * 这些函数内部负责回包，不能像 MODE 一样再次调用 rt_common_msg_res。
     * 保留原字段大小写（autoexp）、目标通道和日志；缺失字段仍按 asInt 处理。
     * 未命中时继续后面的原始特殊流程；命中后 return 退出 data_handler_msg。
     */
#define PNZ_CAMERA_INT_COMMANDS(X)                                      \
    X(CMD_CAMERA_AUTOEXP_SET, "autoexp", set_camera_autoexp,             \
      (index, parsedData.dst_info.chn_id, pnz_value), (void)0,            \
      printf("mrdvs, *****************chnId=%d, autoexp=%d\n",            \
             parsedData.dst_info.chn_id, pnz_value))                     \
    X(CMD_CAMERA_AUTOEXPLEVEL_SET, "autoexplevel", set_camera_autoexplevel, \
      (index, pnz_value), (void)0, (void)0)                             \
    X(CMD_CAMERA_3DAUTOEXPLEVEL_SET, "autoexplevel", set_camera_3Dautoexplevel, \
      (index, pnz_value), (void)0, (void)0)                             \
    X(CMD_CAMERA_3D_UNDISTORT_EN_SET, "undistort", set_camera_3Dundistort, \
      (index, pnz_value), (void)0, (void)0)                             \
    X(CMD_CAMERA_RESOLUTION_SET, "resolution", rt_resolution_set,       \
      (index, pnz_value), (void)0, (void)0)                             \
    X(CMD_AI_MODEL_SET, "model", rt_ai_model_set,                        \
      (index, pnz_value),                                               \
      printf("mrdvs, AI will set camera CMD_AI_MODEL_SET \n"), (void)0)

#define PNZ_CAMERA_INT_CASE(COMMAND, JSON_KEY, HANDLER, ARGUMENTS, BEFORE_READ, AFTER_READ) \
    do {                                                                \
        if (parsedData.cmd_id == (COMMAND)) {                            \
            BEFORE_READ;                                                \
            const int pnz_value = root[JSON_KEY].asInt();                \
            AFTER_READ;                                                 \
            ret = HANDLER ARGUMENTS;                                    \
            return;                                                     \
        }                                                               \
    } while (0);

    PNZ_CAMERA_INT_COMMANDS(PNZ_CAMERA_INT_CASE)

    // 宏没有 C++ 块作用域，使用后显式取消，避免影响本文件后续原始代码。
#undef PNZ_CAMERA_INT_CASE
#undef PNZ_CAMERA_INT_COMMANDS
#undef PNZ_CAMERA_MODE_CASE
#undef PNZ_CAMERA_MODE_COMMANDS
    // ==================== pnz 2026/10/19 新增：通用命令分发 END ======================
    // ==================== pnz 2026/10/19 以下保留原始特殊命令处理 ===================
    
    // 2002: 摄像头启动/停止/复位请求
    // {"cmdId": 2002,"dst": {"devId": 1,"chnId": 1},"status": 1}
    if(parsedData.cmd_id == CMD_CAMERA_START_SET){
        parsedData.status = root["status"].asInt();

        char buf[128]={0};
        switch (parsedData.status){
            case STA_STOP:
                printf("mrdvs, will stop send to this socket\n");
                g_client_info.fd_info[index].chn =  -1;
                snprintf(buf, sizeof(buf), "\"status\":%d,\"ret\":\"%d,%s\"", parsedData.status, ret, "will stop send to this socket");
                break;
            case STA_START:
                ret = max_connection_check();
                if(ret != 0){
                    snprintf(buf, sizeof(buf), "\"status\":%d,\"ret\":\"%d,%s%d,%s\"", parsedData.status, ret, "max connection reached connection:",g_client_info.fd_num,"can't accept more connections");
                }else{ 
                    printf("mrdvs, will start send to this socket\n");
                    g_client_info.fd_info[index].chn =  parsedData.dst_info.chn_id;
                    snprintf(buf, sizeof(buf), "\"status\":%d,\"ret\":\"%d,%s\"", parsedData.status, ret, "will start send to this socket");
                }
                break;
            case STA_RESET:
                printf("mrdvs, will reset camera. \n");
                g_client_info.fd_info[index].chn =  parsedData.dst_info.chn_id;
                snprintf(buf, sizeof(buf), "\"status\":%d,\"ret\":\"%d,%s\"", parsedData.status, ret, "will reset camera");
                break;
            default:
                break;
        }

        // 请求消息和响应消息一样，所以直接将buf发送给客户端
        //ret = rt_common_msg_res(index, CMD_CAMERA_START_SET_RES, buf);
        return;
    }

    // 2004: 设置视频参数 {"cmdId": 2004,"dst": {"devId": 1,"chnId": 1},"vPara":{"bri":100,"cnt":80,"sat":90,"exp": 50,"gai":80}}
    // 手动模式下，2D手动曝光时的曝光值：0~10000；2D手动曝光时的增益值：0~500
    if(parsedData.cmd_id == CMD_CAMERA_PARA_SET){
        printf("mrdvs, AI will set video 2Dpara\n");

        // 当client传过来的数据格式不是下面希望的格式时，解析会出现段错误，后续优化
        //int weather = root["weather"].asInt();
        int bri = root["vPara"]["bri"].asInt();
        int cnt = root["vPara"]["cnt"].asInt();
        int sat = root["vPara"]["sat"].asInt();
        int exp = root["vPara"]["exp"].asInt();
        int gai = root["vPara"]["gai"].asInt();

        printf("mrdvs, data_parser:cmd_id=%d,bri=%d,cnt=%d,sat=%d,exp=%d,gai=%d\n", 
                    parsedData.cmd_id, bri, cnt, sat, exp, gai);

        // 设置摄像头参数,这个函数内部已经做了消息响应
        ret = set_camera_para(index, bri, cnt, sat, exp,gai);
        if(ret != 0){
            printf("mrdvs, set_camera_2Dpara failed\n");
            return;
        }
        printf("mrdvs, set_camera_2Dpara success\n");

        
        return;
    }

    // 2014: 设置3D曝光亮度 {"cmdId": 2014,"dst": {"devId": 1,"chnId": 1},"vPara":{"exp": 50}}
    // interface_gige.cpp 2061 camera_M4Pro_127.0.0.1_3956 LX_INT_FIRST_EXPOSURE available range [1, 3000]
    if(parsedData.cmd_id == CMD_CAMERA_3DPARA_SET){
        // 当client传过来的数据格式不是下面希望的格式时，解析会出现段错误，后续优化
        int exp = root["vPara"]["exp"].asInt();

        printf("mrdvs, data_parser:cmd_id=%d,exp=%d\n", parsedData.cmd_id,exp);

        // 设置摄像头参数,这个函数内部已经做了消息响应
        ret = set_camera_3Dpara(index, exp);
        if(ret != 0){
            printf("mrdvs, set_camera_3Dpara failed\n");
            return;
        }
        printf("mrdvs, set_camera_3Dpara success\n");

        
        return;
    }

/*
    // 2012: AI处理后视频发送请求
    if(parsedData.cmd_id = 2012){
        printf("AI will send post-img data to VID port\n");
        return;
    }
*/
    // 2020: 设置摄像头图像类型
    if(parsedData.cmd_id == CMD_IMAG_TPYE_SET){
        printf("AI will set camera CMD_IMAG_TPYE_SET \n");
        return;
    } 

    // 2040: 采摘序列请求
    if(parsedData.cmd_id == CMD_AI_PICKSEQ_SET){
        printf("mrdvs, AI will set camera CMD_AI_PICKSEQ_SET \n");
        return;
    } 

    // 2044: 获取系统IP地址
    if(parsedData.cmd_id == CMD_SYS_IP_GET){
        printf("mrdvs, sysetem will get system IP address\n");
        ret = rt_system_ip_get(index, parsedData.cmd_id);
        return;
    }

    // 2060: 摄像头滚动信息显示设置
    if(parsedData.cmd_id == CMD_DISP_INFO_SET){
        printf("mrdvs, system will set camera CMD_DISP_INFO_SET \n");
        return;
    }

    // 2062: 错误信息请求
    if(parsedData.cmd_id == CMD_ERR_INFO_REQ){
        printf("mrdvs, AI will get error info\n");
        return;
    }
    

    return;
}


#if 0
void data_handler_msg_old(int index, char *buf, int len){
    int ret = 0;
    buf[len] = '\0';
    printf("%s: client_fd[%d]=%d, len=%d, buf:%s\n", 
            __FUNCTION__, index, g_client_info.fd_info[index].fd, len, buf);
    // 将 buf 转换为 std::string
    std::string json_str(buf, len);

    // 解析 JSON 字符串
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;

    std::istringstream s(json_str);
    if (!Json::parseFromStream(reader, s, &root, &errs)) {
        std::cerr << "Failed to parse JSON: " << errs << std::endl;
        return;
    }

/****提取数据并填充结构体, 目前解析以下几个消息 
1. {"cmdId":2002,"dst": {"devId": val,"chnId": val},"status":val} 摄像头启动/停止/复位请求
2. {"cmdId":2012} AI处理后视频发送请求, 
*************************************************/ 

    parsed_data parsedData;
    parsedData.cmd_id = root["cmdId"].asInt();

    if(parsedData.cmd_id == CMD_ID_START){
        parsedData.dst_info.dev_id = root["dst"]["devId"].asInt();
        parsedData.dst_info.chn_id = root["dst"]["chnId"].asInt();
        parsedData.status = root["status"].asInt();

        // 打印解析后的数据    
        printf("data_parser:cmd_id=%d,dev_id=%d,chn_id=%d, status=%d\n", 
                    parsedData.cmd_id,parsedData.dst_info.dev_id, parsedData.dst_info.chn_id, parsedData.status);

        switch (parsedData.status){
            case STA_STOP:
                printf("will stop send to this socket\n");
                g_client_info.fd_info[index].chn =  -1;
                break;
            case STA_START:
                printf("current connection=%d, will start to send to this socket\n", g_client_info.fd_num);
                g_client_info.fd_info[index].chn =  parsedData.dst_info.chn_id;
                break;
            case STA_RESET:
                printf("will reset camera. \n");
                g_client_info.fd_info[index].chn =  parsedData.dst_info.chn_id;
                break;
            default:
                break;
        }
        return;
    }

    if(parsedData.cmd_id == CMD_CAMERA_PARA_SET){
        printf("AI will set video para\n");

        int weather = root["weather"].asInt();
        int bri = root["vPara"]["bri"].asInt();
        int cnt = root["vPara"]["cnt"].asInt();
        int sat = root["vPara"]["sat"].asInt();
        int exp = root["vPara"]["exp"].asInt();

        // 打印解析后的数据
        printf("data_parser:cmd_id=%d,weather=%d,bri=%d,cnt=%d,sat=%d,exp=%d\n", 
                    parsedData.cmd_id, weather, bri, cnt, sat, exp);

        // 设置摄像头参数
        ret = set_camera_para(index, weather, bri, cnt, sat, exp);
        return;
    }

    if(parsedData.cmd_id == 2012){
        printf("AI will send post-img data to VID port\n");
        return;
    }

    return;
}
#endif

void data_handler_vid(int index, char *buf, int len){
    data_forward(4, index, buf, len);
    return;
}


void data_handler(int index, char *buf, int len){
    int socket_port = 0;
    int ret = 0;

    int fd = g_client_info.fd_info[index].fd;

    ret = socket_port_get(fd, &socket_port);
    if(ret < 0){ // 获取socket端口失败，则不发送数据
        printf("mrdvs, data_handler:fd get socket port failed, fd=%d, len=%d\n", fd, len);
        return;
    }

    // 消息端口，解析数据
    if(socket_port ==  SRV_MSG_PORT){ 
        //printf("data_handler:fd is SRV_MSG_PORT, fd=%d, len=%d\n", fd, len);
        data_handler_msg(index, buf, len);
        return;
    }  

    // 视频端口，调用 data_forward 函数
    if(socket_port ==  SRV_VID_PORT){
        // 打印接受的到数据信息，每100帧打印一次
        g_client_info.fd_info[index].frame_cnt++;
        if(g_client_info.fd_info[index].frame_cnt % 100 == 0){
            printf("%s:mrdvs,  Server <-- AI, [%d].fd=%d, frame_cnt:%d, len=%d\n", 
                __FUNCTION__, index, fd, g_client_info.fd_info[index].frame_cnt, len);
        }
        data_handler_vid(index, buf, len);
    }

    return;
}


// 初始化全局数组
void init_connections() {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        g_client_info.fd_info[i].fd = -1;
        g_client_info.fd_info[i].chn = -1;
        g_client_info.fd_info[i].frame_cnt = 0;
    }
    return;
}

int set_socket_chn(int chn){
    int i;
    int ret = -1;
    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }
        g_client_info.fd_info[i].chn = chn;
        printf("mrdvs, %s:set chn=%d, client_fd[%d]=%d\n",
            __FUNCTION__, chn, i, g_client_info.fd_info[i].fd);
        ret = i;
    }
    return ret;
}

// 添加新连接
int add_connection(int new_socket, fd_set *read_fds, int *max_fd) {
    if (g_client_info.fd_num < MAX_CONNECTIONS) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (g_client_info.fd_info[i].fd == -1) {
                g_client_info.fd_info[i].fd = new_socket;
                g_client_info.fd_info[i].chn = -1;
                g_client_info.fd_info[i].frame_cnt = 0;
                g_client_info.fd_num++;    

                FD_SET(new_socket, read_fds);
                
                if (new_socket > *max_fd) {
                    *max_fd = new_socket;
                }

                // 获取对方的 IP 地址
                struct sockaddr_in peer_address;
                socklen_t peer_addrlen = sizeof(peer_address);
                char ip_str[INET_ADDRSTRLEN];
                if (getpeername(new_socket, (struct sockaddr *)&peer_address, &peer_addrlen) == -1) {
                    perror("gmrdvs, etpeername");
                    rt_msg_send_all("mrdvs, getpeername error");
                    memcpy(ip_str, "unknown", sizeof("unknown"));
                } else {
                    inet_ntop(AF_INET, &peer_address.sin_addr, ip_str, sizeof(ip_str));
                    //std::cout << "New connection from " << ip_str << ":" << ntohs(peer_address.sin_port) << std::endl;
                }
                printf("mrdvs, New conn added. Client ip:%s, socket id=%d. Current conn num=%d\n",ip_str, new_socket, g_client_info.fd_num);
                return i;
            }
        }
    } else {
        printf("mrdvs, Maximum number of connections reached. Closing new connection. Current conn num=%d\n", g_client_info.fd_num);
        close(new_socket);
        return -1;
    }
    return -1;
}



void* server_thread(void *arg)
{
    struct sockaddr_in svr_msg_sockaddr, svr_vid_sockaddr, client_sockaddr;
    int recvbytes;
    int client_fd;
    socklen_t sin_size;
    int i;
    
    fd_set writefd;
    char buf[MAX_MSG_REQ];

    // 初始化全局数组
    init_connections();

    if ((svr_msg_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }
    if ((svr_vid_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }

    setreuseport(svr_msg_fd);
    setreuseport(svr_vid_fd);
    printf("mrdvs, %s:socket success!, svr_msg_fd=%d, svr_vid_fd=%d\n", __FUNCTION__, svr_msg_fd, svr_vid_fd);

    svr_msg_sockaddr.sin_family = AF_INET;
    svr_msg_sockaddr.sin_port = htons(SRV_MSG_PORT);
    svr_msg_sockaddr.sin_addr.s_addr = INADDR_ANY;
    bzero(&(svr_msg_sockaddr.sin_zero), 8);
    svr_vid_sockaddr.sin_family = AF_INET;
    svr_vid_sockaddr.sin_port = htons(SRV_VID_PORT);
    svr_vid_sockaddr.sin_addr.s_addr = INADDR_ANY;
    bzero(&(svr_vid_sockaddr.sin_zero), 8);


    if (bind(svr_msg_fd, (struct sockaddr *)&svr_msg_sockaddr, sizeof(struct sockaddr)) == -1) {
        perror("mrdvs, bind");
        exit(1);
    }
    if (bind(svr_vid_fd, (struct sockaddr *)&svr_vid_sockaddr, sizeof(struct sockaddr)) == -1) {
        perror("bind");
        exit(1);
    }
    printf("mrdvs, bind success!\n");

    if (listen(svr_msg_fd, BACKLOG) == -1) {
        perror("mrdvs, listen");
        exit(1);
    }
    if (listen(svr_vid_fd, BACKLOG) == -1) {
        perror("mrdvs, listen");
        exit(1);
    }
    printf("mrdvs, listening....\n");

    // 添加服务器套接字到 read_fds
    FD_ZERO(&readfd);
    FD_SET(svr_msg_fd, &readfd);
    FD_SET(svr_vid_fd, &readfd);
    max_fd = svr_msg_fd > svr_vid_fd ? svr_msg_fd : svr_vid_fd;

    while (1) {
        fd_set tmp_fds = readfd;
        int ready = select(max_fd + 1, &tmp_fds, NULL, NULL, NULL);
        if (ready < 0) {
            printf("mrdvs, Error in select, server will quit.\n");
            return NULL;
        }

        // 检查是否有新连接
        if (FD_ISSET(svr_msg_fd, &tmp_fds)) {
            sin_size = sizeof(struct sockaddr_in);
            client_fd = accept(svr_msg_fd, (struct sockaddr *)&client_sockaddr, &sin_size);
            if(client_fd < 0){
                printf("mrdvs, Error accepting msg new connection.\n");
                continue;
            }else{
                //printf("received a new MSG connection, i=%d\n", client_fd);
                pthread_mutex_lock(&socket_mutex);
                i = add_connection(client_fd, &readfd, &max_fd);
                pthread_mutex_unlock(&socket_mutex);
           }
        }
        if(FD_ISSET(svr_vid_fd, &tmp_fds)) {
            sin_size = sizeof(struct sockaddr_in);
            client_fd = accept(svr_vid_fd, (struct sockaddr *)&client_sockaddr, &sin_size);
            if(client_fd < 0){
                printf("mrdvs, Error accepting div new connection.\n");
                continue;
            }
            printf("mrdvs, received a new DIV connection, i=%d\n", client_fd);
            pthread_mutex_lock(&socket_mutex);
            i = add_connection(client_fd, &readfd, &max_fd);
            pthread_mutex_unlock(&socket_mutex);
            continue;
        }

        // 检查已连接的客户端是否有数据
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if(g_client_info.fd_info[i].fd < 0){
                continue;
            }

            if(g_client_info.fd_info[i].fd >= 0 && FD_ISSET(g_client_info.fd_info[i].fd, &readfd)){
                recvbytes = recv(g_client_info.fd_info[i].fd, buf, MAX_MSG_REQ, MSG_DONTWAIT);

                //检查 errno 变量是否等于 EWOULDBLOCK。这个错误码表示套接字当前没有数据可读。
                if(recvbytes < 0 && errno == EWOULDBLOCK){
                    continue;
                }

                // ECONNRESET,表示连接被对方重置。应该关闭这个套接字,并从客户端列表中移除。
                if(recvbytes < 0 && errno == ECONNRESET){
                    printf("mrdvs, conn resetted from client %d\n", g_client_info.fd_info[i].fd);

                    pthread_mutex_lock(&socket_mutex);
                    remove_connection(i, &readfd);
                    pthread_mutex_unlock(&socket_mutex);
                    continue;
                } 

                // 对方已经关闭了连接。应该关闭这个套接字,并从客户端列表中移除.
                // 注意这个时候可能有数据正在向这个套接字写,所以应该先把数据停止写，再关闭套接字。
                if(recvbytes == 0) {
                    printf("mrdvs, Client client_fd[%d]=%d disconnected\n", i, g_client_info.fd_info[i].fd);
                    remove_connection(i, &readfd);
                    continue;
                } 

                // 接收到数据    
                if(recvbytes > 0){
                    //printf("Received data from client sock id:%d, data len:%d\n", i, recvbytes);
                    //buf[recvbytes] = '\0';
                    data_handler(i, buf, recvbytes);
                }
            }
        }
    }
    return NULL;
}

//数据格式。3D强度图和深度图有不同的数据格式，获取数据指针需要使用对应的feature
/*********************************
typedef enum LX_DATA_TYPE
{
    LX_DATA_UNSIGNED_CHAR = 0,
    LX_DATA_UNSIGNED_SHORT = 2,
    LX_DATA_SIGNED_SHORT = 3,
    LX_DATA_FLOAT = 5,
    LX_DATA_OBSTACLE = 16,
    LX_DATA_PALLET = 17,
    LX_DATA_LOCATION =18,
    LX_DATA_OBSTACLE2 = 19,
}LX_DATA_TYPE;   
*********************************/

void* frame_encode_depth(unsigned int type, const void *data, unsigned int width, unsigned int height, unsigned long *depth_size){
    //=================================
    //摄像头设置没有  亮度 对比度 饱和度，在这里用openncv处理
    //参考连接：https://blog.csdn.net/u012294613/article/details/140975761
    //=================================
    cv::Mat depth_mat= cv::Mat(height, width, CV_16UC1, (void *)data);
    cv::Mat depth_mat_xu;

    // 确保数据类型为 CV_16U
    if (type != CV_16U) {
        printf("mrdvs, Error: Expected data type CV_16U, but got type=%d\n", type);
        return NULL;
    }

    // 这里不需要转换数据类型，只需进行比例因子调整
    depth_mat.convertTo(depth_mat_xu, CV_16U, 1.0);

    //depth_mat.convertTo(depth_mat_8u, CV_8U, 255.0/4096);
    //depth_mat.convertTo(depth_mat_xu, type, 255.0/65535);

    //cv::resize(depth_mat_8u, depth_mat_8u, cv::Size(width / 2, height / 2), 0, 0, cv::INTER_NEAREST);
    JSAMPLE *depth_data = (JSAMPLE *)depth_mat_xu.data;
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    unsigned char *depth_buffer = NULL;
    JSAMPROW row_pointer[1];
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &depth_buffer, depth_size);

    cinfo.image_width = depth_mat_xu.cols;
    cinfo.image_height = depth_mat_xu.rows;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 100, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &depth_data[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    return depth_buffer;
}

// 将PNG格式的数据保存到文件，其中文件名自动按照时间戳生成
void frame_save_png(const void *data, unsigned long size){  
    //printf("===========================frame_save_png: size=%ld\n", size);
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char filename[64];
    sprintf(filename, "./png/depth_%04d%02d%02d%02d%02d%02d.png", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

    FILE *fp = fopen(filename, "wb");
    if (fp == NULL) {
        printf("mrdvs, Error: Failed to open file %s\n", filename);
        return;
    }

    fwrite(data, 1, size, fp);
    fclose(fp);

    //printf("Saved PNG file: %s\n", filename);
    return;
}

#if 0
// 将16位深度图转换为PNG格式，要求必须是无损压缩
void *frame_encode_png_old(unsigned int type, const void *data, unsigned int width, unsigned int height, unsigned long *png_size){
    if (type != 16) {
        printf("Error: Expected data type 16-bit, but got type=%d\n", type);
        return NULL;
    }

    // 创建PNG写结构
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr) {
        printf("Error: png_create_write_struct failed\n");
        return NULL;
    }

    // 创建PNG信息结构
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        printf("Error: png_create_info_struct failed\n");
        return NULL;
    }

    // 设置错误处理
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        printf("Error: setjmp failed\n");
        return NULL;
    }

    // 创建内存写入结构
    png_bytep png_data = NULL;
    png_size_t png_data_size = 0;
    png_set_write_fn(png_ptr, &png_data, (png_rw_ptr)png_malloc, (png_flush_ptr)png_free);

    // 设置PNG头信息
    png_set_IHDR(png_ptr, info_ptr, width, height, 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    // 写入PNG头信息
    png_write_info(png_ptr, info_ptr);

    // 写入图像数据
    for (unsigned int y = 0; y < height; y++) {
        png_write_row(png_ptr, (png_bytep)(data + y * width * 2));
    }

    // 写入PNG文件尾部
    png_write_end(png_ptr, info_ptr);

    // 获取PNG数据大小
    *png_size = png_data_size;

    //frame_save_png(png_data, *png_size);

    // 清理
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return png_data;
    
}
#endif

void *frame_encode_png(unsigned int type, const void *data, unsigned int width, unsigned int height, unsigned long *png_size)
{
    //pthread_mutex_lock(&g_handle_mutex);
    cv::Mat depth_mat = cv::Mat(height, width, CV_16UC1, (void *)data);
    cv::Mat depth_mat_xu;

    // 确保数据类型为 CV_16U
    if (type != CV_16U) {
        printf("mrdvs, Error: Expected data type CV_16U, but got type=%d\n", type);
        return NULL;
    }

    // 这里不需要转换数据类型，只需进行比例因子调整
    depth_mat.convertTo(depth_mat_xu, CV_16U, 1.0);

    // 创建 PNG 编码器
    std::vector<uchar> png_buffer;
    cv::imencode(".png", depth_mat_xu, png_buffer);

    // 分配内存并拷贝数据
    *png_size = png_buffer.size();
    void *png_data = malloc(*png_size);
    if (png_data == NULL) {
        printf("mrdvs, Error: Failed to allocate memory for PNG data\n");
        return NULL;
    }
    memcpy(png_data, png_buffer.data(), *png_size);

    //frame_save_png(png_data, *png_size);

    //pthread_mutex_unlock(&g_handle_mutex);
    return png_data;
}



// flag 为 0，表示深度图；为 1，表示RGB图
void* frame_encode_jpg(const void *bgr, unsigned int width, unsigned int height, unsigned long *jpg_size)
{
    //pthread_mutex_lock(&g_handle_mutex);
    //=================================
    //摄像头设置没有  亮度 对比度 饱和度，在这里用openncv处理
    //参考连接：https://blog.csdn.net/u012294613/article/details/140975761
    //=================================
void memtest(const void *data, int width, int height);
void memtest_once(const void *data, int width, int height);

    //memtest_once(bgr, width, height);
    cv::Mat bgr_mat= cv::Mat(height, width, CV_8UC3, (void *)bgr);
    cv::Mat rgb_mat;
    cv::cvtColor(bgr_mat, rgb_mat, cv::COLOR_BGR2RGB);
    //cv::resize(rgb_mat, rgb_mat, cv::Size(width / 2, height / 2), 0, 0, cv::INTER_NEAREST);
    //cv::resize(rgb_mat, rgb_mat, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);

    JSAMPLE *rgb_data = (JSAMPLE *)rgb_mat.data;
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    unsigned char *jpg_buffer = NULL;
    JSAMPROW row_pointer[1];
    int row_stride;

    

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &jpg_buffer, jpg_size);

    cinfo.image_width = rgb_mat.cols;
    cinfo.image_height = rgb_mat.rows;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 30, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &rgb_data[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    //pthread_mutex_unlock(&g_handle_mutex);
    /*
    FILE *pfile = fopen("test.jpg", "wb");
    fwrite(jpg_buffer, 1, *jpg_size, pfile);
    fclose(pfile);
    */

    //memtest_once(bgr, width, height);
    return jpg_buffer;
}

static void statfps()
{
    static struct timespec last_report = {0, 0};
    static int frame_count = 0;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (last_report.tv_sec == 0) {
        last_report = now;
        return;
    }

    frame_count++;
    if (now.tv_sec - last_report.tv_sec >= 1) {
        double elapsed = (now.tv_sec - last_report.tv_sec) + (now.tv_nsec - last_report.tv_nsec) / 1e9;
        double fps = frame_count / elapsed;
        printf("fps: %.2f\n", fps);
        last_report = now;
        frame_count = 0;
    }
}


#if 0
//将原来 results 中的数据拷贝到buf，buf指针由调用者传入，buf的空间在该函数中动态分配，记得在外面传输完成后，释放该buf
int results2buf(uint8_t **buf, object_detect_result_list_ai* results_ai){
    int i;


    // 计算总数据长度
    unsigned long total_length = 0;
    total_length += sizeof(results_ai->id);
    total_length += sizeof(results_ai->count);
    total_length += results_ai->count * sizeof(object_detect_result);
    for (i = 0; i < results_ai->count; i++) {
        total_length += sizeof(results_ai->results_seg[i].width);
        total_length += sizeof(results_ai->results_seg[i].height);
        total_length += sizeof(results_ai->results_seg[i].pixel_data_length);
        total_length += sizeof(results_ai->results_seg[i].type);
        total_length += results_ai->results_seg[i].width * results_ai->results_seg[i].height * results_ai->results_seg[i].pixel_data_length;
    }

    // 对buf分配空间
    *buf = (uint8_t *)malloc(total_length);
    if (buf == NULL) {
        printf("Error: %s,Failed to allocate memory for buf, needed size:%ld\n", __FUNCTION__, total_length);
        return -1;
    }

    // 拷贝数据
    uint8_t *p = *buf;
    memcpy(p, &results_ai->id, sizeof(results_ai->id));
    p += sizeof(results_ai->id);
    memcpy(p, &results_ai->count, sizeof(results_ai->count));
    p += sizeof(results_ai->count);
    for (i = 0; i < results_ai->count; i++) {
        memcpy(p, &results_ai->results[i], sizeof(object_detect_result));
        p += sizeof(object_detect_result);
        memcpy(p, &results_ai->results_seg[i].width, sizeof(results_ai->results_seg[i].width));
        p += sizeof(results_ai->results_seg[i].width);
        memcpy(p, &results_ai->results_seg[i].height, sizeof(results_ai->results_seg[i].height));
        p += sizeof(results_ai->results_seg[i].height);
        memcpy(p, &results_ai->results_seg[i].pixel_data_length, sizeof(results_ai->results_seg[i].pixel_data_length));
        p += sizeof(results_ai->results_seg[i].pixel_data_length);
        memcpy(p, &results_ai->results_seg[i].type, sizeof(results_ai->results_seg[i].type));
        p += sizeof(results_ai->results_seg[i].type);
        memcpy(p, results_ai->results_seg[i].seg_mask, results_ai->results_seg[i].width * results_ai->results_seg[i].height * results_ai->results_seg[i].pixel_data_length);
        p += results_ai->results_seg[i].width * results_ai->results_seg[i].height * results_ai->results_seg[i].pixel_data_length;
    }

    return total_length;
}
#endif



// 为了减少数据拷贝，将原来的 object_detect_result_list 结构体中的数据拷贝到 object_detect_result_list_ai 结构体中
void seg_mask_cp(uint8_t img_width,uint8_t img_height, object_detect_result_list *results, object_detect_result_list_ai *results_ai){
    int i;

    results_ai->id = results->id;
    results_ai->count = results->count;

    for (i = 0; i < results->count; i++) {
        // 拷贝 object_detect_result
        results_ai->results[i].box.left = results->results[i].box.left;
        results_ai->results[i].box.top = results->results[i].box.top;
        results_ai->results[i].box.right = results->results[i].box.right;
        results_ai->results[i].box.bottom = results->results[i].box.bottom;
        results_ai->results[i].prop = results->results[i].prop;
        results_ai->results[i].cls_id = results->results[i].cls_id;

        // 拷贝 object_segment_result
        memset(results_ai->results_seg[i].seg_mask, 0, MAX_SEG_IMAGE_SIZE);
        if (results->results_seg[i].seg_mask != NULL) {
            results_ai->results_seg[i].width = img_width;
            results_ai->results_seg[i].height = img_height;
            results_ai->results_seg[i].pixel_data_length = sizeof(uint8_t);
            results_ai->results_seg[i].type = 2;

            if(results->results_seg[i].seg_mask == NULL){
                printf("mrdvs, seg_mask_cp: seg_mask is NULL\n");
                continue;
            }
            if((results_ai->results_seg[i].width*results_ai->results_seg[i].height*results_ai->results_seg[i].pixel_data_length)
                > MAX_SEG_IMAGE_SIZE){
                    printf("mrdvs, seg_mask_cp: seg_mask size is too large, exceed MAX_SEG_IMAGE_SIZE:%d, seg_mask size:%d\n", 
                        MAX_SEG_IMAGE_SIZE, results_ai->results_seg[i].width*results_ai->results_seg[i].height*results_ai->results_seg[i].pixel_data_length);
                    continue;
            }
            memcpy(results_ai->results_seg[i].seg_mask, results->results_seg[i].seg_mask, img_width*img_height*sizeof(uint8_t));
        }
    }
    return;
}


void seg_mask_cp_fixdata(uint8_t img_width,uint8_t img_height, object_detect_result_list *results, object_detect_result_list_ai *results_ai){
    int i;

    results_ai->id = 100;
    results_ai->count = 2;

    results_ai->results[0].box.left = 10;
    results_ai->results[0].box.top = 20;
    results_ai->results[0].box.right = 30;
    results_ai->results[0].box.bottom = 40;
    results_ai->results[0].prop = 0.5;
    results_ai->results[0].cls_id = 1;
    results_ai->results[1].box.left = 10;
    results_ai->results[1].box.top = 20;
    results_ai->results[1].box.right = 30;
    results_ai->results[1].box.bottom = 40;
    results_ai->results[1].prop = 0.5;
    results_ai->results[1].cls_id = 1;

    results_ai->results_seg[0].width = 10;
    results_ai->results_seg[0].height = 20;
    results_ai->results_seg[0].pixel_data_length = sizeof(uint8_t);
    results_ai->results_seg[0].type = 2;
    //memset(results_ai->results_seg[0].seg_mask, 0, 5);

    
    results_ai->results_seg[1].width = 10;
    results_ai->results_seg[1].height = 20;
    results_ai->results_seg[1].pixel_data_length = sizeof(uint8_t);
    results_ai->results_seg[1].type = 2;
    //memset(results_ai->results_seg[1].seg_mask, 0, 5);
    

    return;
}

// 发送帧数据+AI处理字符串,此函数仅仅发送chn==4的数据
int frame_cnt_ai = 0;

int frame_send_ai_fixdata(int chn, int frame_width, int frame_height, int frame_channel, void *frame_data){
    int i;
    unsigned char *buf;
    int  buf_len = 0;

    void *jpg_data = NULL;
    unsigned long jpg_len = 0;

    unsigned char send_data[FRAME_MAX_SIZE];
    unsigned long send_len = 0;
    unsigned long  len_sent = 0;

    object_detect_result_list_ai results_ai;
    results_ai.id = 1;
    results_ai.count = 2; // 示例值，根据实际需要调整

    // 填写假数据
    for (int i = 0; i < results_ai.count; i++) {
        results_ai.results[i].box.left = i * 10;
        results_ai.results[i].box.top = i * 20;
        results_ai.results[i].box.right = i * 30;
        results_ai.results[i].box.bottom = i * 40;
        results_ai.results[i].prop = 0.5f + i;
        results_ai.results[i].cls_id = i;

        results_ai.results_seg[i].width = i * 50;
        results_ai.results_seg[i].height = i * 60;
        results_ai.results_seg[i].pixel_data_length = sizeof(uint8_t);
        results_ai.results_seg[i].type = 2;
        memset(results_ai.results_seg[i].seg_mask, i, MAX_SEG_IMAGE_SIZE);
    }
    

    // 计算动态分配的缓冲区大小
    size_t buf_size = sizeof(results_ai.id) + sizeof(results_ai.count) +
                      results_ai.count * sizeof(object_detect_result) +
                      results_ai.count * sizeof(object_segment_result_ai);
    // 动态分配缓冲区
    buf = (unsigned char *)malloc(buf_size);
    if (buf == NULL) {
        printf("mrdvs, Error: Unable to allocate memory for buffer.\n");
        return -1;
    }

    // 拷贝数据到缓冲区
    unsigned char *ptr = buf;
    memcpy(ptr, &results_ai.id, sizeof(results_ai.id));
    buf_len += sizeof(results_ai.id);
    ptr += sizeof(results_ai.id);
    memcpy(ptr, &results_ai.count, sizeof(results_ai.count));
    buf_len += sizeof(results_ai.count);
    ptr += sizeof(results_ai.count);
    memcpy(ptr, results_ai.results, results_ai.count * sizeof(object_detect_result));
    buf_len += results_ai.count * sizeof(object_detect_result);
    ptr += results_ai.count * sizeof(object_detect_result);
    memcpy(ptr, results_ai.results_seg, results_ai.count * sizeof(object_segment_result_ai));
    buf_len += results_ai.count * sizeof(object_segment_result_ai);

    // frame_data进行jpg编码
    frame_cnt_ai++;
    jpg_data = frame_encode_jpg(frame_data, frame_width, frame_height, &jpg_len);
    if (jpg_data == NULL) {
        printf("mrdvs, frame_send: encode jpg failed\n");
        rt_msg_send_all("frame_send: encode jpg failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
        return -1;
    }

    // chnn 4的数据帧格式：buf_len(4字节)+ buf + jpg_len(4字节) + jpg_data
    // 其他chnn的数据帧格式：jpg_len(4字节) + jpg_data
    memset(send_data, 0, FRAME_MAX_SIZE);
    send_data[0] = (unsigned char)((buf_len >> 24) & 0xFF);
    send_data[1] = (unsigned char)((buf_len >> 16) & 0xFF);
    send_data[2] = (unsigned char)((buf_len >> 8) & 0xFF);
    send_data[3] = (unsigned char)((buf_len ) & 0xFF);
    memcpy(&send_data[4], buf, buf_len);

    send_data[4+buf_len] = (unsigned char)((jpg_len >> 24) & 0xFF);
    send_data[4+buf_len+1] = (unsigned char)((jpg_len >> 16) & 0xFF);
    send_data[4+buf_len+2] = (unsigned char)((jpg_len >> 8) & 0xFF);
    send_data[4+buf_len+3] = (unsigned char)((jpg_len ) & 0xFF);
    memcpy(&send_data[4+buf_len+4], jpg_data, jpg_len);

    free(jpg_data);
    free(buf);

    send_len = 4 + buf_len + 4 + jpg_len;

    // 发送给需要的客户端, 注意只是发给请求了chnn 4的客户端
    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }

        // socket chn为这个视频，才发送数据
        if(g_client_info.fd_info[i].chn != chn){
            if(frame_cnt_ai % 100 == 0){
                ;//printf("will not send data:fd chn=%d, index=%d, fd=%d\n", g_client_info.fd_info[i].chn, i, g_client_info.fd_info[i].fd);
            }
            continue;
        }

        //printf("will send data through g_client_fd[%d]=%d, chn=%d\n", i, g_client_info.fd_info[i].fd, chn); 
        len_sent = send(g_client_info.fd_info[i].fd, send_data, send_len, 0);
        if (len_sent < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, i, g_client_info.fd_info[i].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                //remove_connection(i, &readfd);
                printf("mrdvs, send data error, client disconnected, errno == EPIPE || errno == ECONNRESET\n");

            } else {
                perror("mrdvs, send error");
                rt_msg_send_all("send data length<0, send error");
            }
        }

        g_client_info.fd_info[i].frame_cnt++;

        if(g_client_info.fd_info[i].frame_cnt % 100 == 0){
            printf("mrdvs, %s:   Server --> Client. w:%d, h:%d, jpg_len:%ld,send_len:%ld, sent_len:%ld, chn:%d, frame_cnt:%d, client_fd[%d]:%d\n", 
                __FUNCTION__, frame_width,frame_height, jpg_len, send_len, len_sent, g_client_info.fd_info[i].chn, g_client_info.fd_info[i].frame_cnt, i, g_client_info.fd_info[i].fd);
        }   
    }
    //pthread_mutex_unlock(&socket_mutex); 

    return 0;
}

// 判断g_client_info.fd_info中是否有指定chn?(chn0，chn1，chn2，chn3，chn4)的连接需求。有则返回1， 无则返回0
int frame_send_data_judge(int chn){
    int i;
    int ret = 0;

    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0)
            continue;
        if(g_client_info.fd_info[i].chn == chn){
            ret = 1;
            break;
        }
    }

    return ret;
}
// 发送帧数据：将打包好的data发送给客户端
int frame_send_data(int chn, unsigned char *data, int data_len){
    int i;
    int len_sent = 0;

    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }

        // socket chn为这个视频，才发送数据
        if(g_client_info.fd_info[i].chn != chn){
            if(frame_cnt_ai % 100 == 0){
                ;//printf("will not send data:fd chn=%d, index=%d, fd=%d\n", g_client_info.fd_info[i].chn, i, g_client_info.fd_info[i].fd);
            }
            continue;
        }

        //printf("will send data through g_client_fd[%d]=%d, chn=%d\n", i, g_client_info.fd_info[i].fd, chn); 
        len_sent = send(g_client_info.fd_info[i].fd, data, data_len, 0);
        if (len_sent < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, i, g_client_info.fd_info[i].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                //remove_connection(i, &readfd);
                printf("mrdvs, send data error, client disconnected, errno == EPIPE || errno == ECONNRESET\n");
                rt_msg_send_all("mrdvs, send data error, client disconnected, errno == EPIPE || errno == ECONNRESET");
            } else {
                perror("mrdvs, send error");
                rt_msg_send_all("mrdvs, send data length<0, send error");
            }
        }

        g_client_info.fd_info[i].frame_cnt++;

        if(g_client_info.fd_info[i].frame_cnt % 100 == 0){
            printf("mrdvs, %s:   Server --> Client. send_len:%d, sent_len:%d, chn:%d, frame_cnt:%d, client_fd[%d]:%d\n", 
                __FUNCTION__, data_len, len_sent, g_client_info.fd_info[i].chn, g_client_info.fd_info[i].frame_cnt, i, g_client_info.fd_info[i].fd);
        }   
    }

    return 0;
}

// 发送帧数据+长度的字符串,其中buf是数据，buf_len是数据长度。这里的buf可以是字符串，也可以是二进制数据
// pic_encode_type: 0表示jpg，1表示png
int frame_send_wi_buf(int chn, uint32_t frame_id, int frame_width, int frame_height, int frame_channel, void *frame_data, int pic_encode_type, unsigned char *buf, int buf_len){
    int i, ret = 0;
    unsigned long  len_sent = 0;
    unsigned long encoded_len = 0;
    unsigned long result_len = 0;
    unsigned long send_len = 0;
    void *encoded_data = NULL;
    unsigned char send_data[FRAME_MAX_SIZE];

    // 根据pic_encode_type进行编码：0表示jpg，1表示png
    if(pic_encode_type == 0){
        encoded_data = frame_encode_jpg(frame_data, frame_width, frame_height, &encoded_len);
        if (encoded_data == NULL) {
            printf("mrdvs, frame_send: encode jpg failed\n");
            rt_msg_send_all("frame_send: encode jpg failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
            return -1;
        }
    }
    else if(pic_encode_type == 1){
        encoded_data = frame_encode_png(CV_16U, frame_data, frame_width, frame_height, &encoded_len);
        if (encoded_data == NULL) {
            printf("mrdvs, frame_send: encode png failed\n");
            rt_msg_send_all("frame_send: encode png failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
            return -1;
        }
    }
    else{
        printf("mrdvs, frame_send: pic_encode_type error, pic_encode_type=%d\n", pic_encode_type);
        rt_msg_send_all("frame_send: pic_encode_type error"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
        return -1;
    }   

    // 数据帧格式：frame_id(4字节)+buf_len(4字节)+ buf + jpg_len(4字节) + jpg_data
    memset(send_data, 0, FRAME_MAX_SIZE);
    if(buf == NULL){
        buf_len = 0;  
    }

    // 4个字节存放frame_id
    send_data[send_len+0] = (unsigned char)((frame_id >> 24) & 0xFF);
    send_data[send_len+1] = (unsigned char)((frame_id >> 16) & 0xFF);
    send_data[send_len+2] = (unsigned char)((frame_id >> 8) & 0xFF);
    send_data[send_len+3] = (unsigned char)((frame_id ) & 0xFF);
    send_len += 4;

    // 时间戳长度+时间戳净荷
    send_data[send_len+0] = (unsigned char)((buf_len >> 24) & 0xFF);
    send_data[send_len+1] = (unsigned char)((buf_len >> 16) & 0xFF);
    send_data[send_len+2] = (unsigned char)((buf_len >> 8) & 0xFF);
    send_data[send_len+3] = (unsigned char)((buf_len ) & 0xFF);
    send_len += 4;
    if (buf != NULL && buf_len > 0) {
        memcpy(&send_data[send_len], buf, buf_len);
        send_len += buf_len;
    }

    // 图片长度+图片净荷
    send_data[send_len+0] = (unsigned char)((encoded_len >> 24) & 0xFF);
    send_data[send_len+1] = (unsigned char)((encoded_len >> 16) & 0xFF);
    send_data[send_len+2] = (unsigned char)((encoded_len >> 8) & 0xFF);
    send_data[send_len+3] = (unsigned char)((encoded_len ) & 0xFF);
    send_len += 4;
    memcpy(&send_data[send_len], encoded_data, encoded_len);
    send_len += encoded_len;
    send_len = send_len;

    free(encoded_data);  // 释放编码动态申请的内存

    // 发送给需要的客户端
    frame_send_data(chn, send_data, send_len);

    return ret;
}

// 发送带时间戳信息和AI信息的图片：时间戳长度+时间戳净荷+AI信息长度+AI信息净荷+图片长度+图片净荷
int frame_send_wi_timestamp_ai(int chn, uint32_t frame_id, int frame_width, int frame_height, int frame_channel, void *frame_data, int pic_encode_type, unsigned long long sensor_timestamp, unsigned long long recv_timestamp, unsigned char *buf, int buf_len){
    int ret = 0;

    unsigned char send_data[FRAME_MAX_SIZE];
    int send_data_len = 0;

    memset(send_data, 0, send_data_len);

    // 根据pic_encode_type进行编码：0表示jpg，1表示png
    void *encoded_data = NULL;
    unsigned long encoded_data_len = 0;
    if(pic_encode_type == 0){
        encoded_data = frame_encode_jpg(frame_data, frame_width, frame_height, &encoded_data_len);
        if (encoded_data == NULL) {
            printf("mrdvs, frame_send: encode jpg failed\n");
            rt_msg_send_all("frame_send: encode jpg failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
            return -1;
        }
    }
    else if(pic_encode_type == 1){
        encoded_data = frame_encode_png(CV_16U, frame_data, frame_width, frame_height, &encoded_data_len);
        if (encoded_data == NULL) {
            printf("mrdvs, frame_send: encode png failed\n");
            rt_msg_send_all("frame_send: encode png failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
            return -1;
        }
    }
    else{
        printf("mrdvs, frame_send: pic_encode_type error, pic_encode_type=%d\n", pic_encode_type);
        rt_msg_send_all("frame_send: pic_encode_type error"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送
        return -1;
    }

// 数据帧格式：帧ID（4字节） + 时间戳信息长度(4字节)+ 时间戳信息(8字节); + AI信息长度(4字节) + AI信息(n字节)；+ 图片长度(4字节) + 图片数据(n字节)
    // 1. 第1部分：存放帧ID
    send_data[send_data_len+0] = (unsigned char)((frame_id >> 24) & 0xFF);
    send_data[send_data_len+1] = (unsigned char)((frame_id >> 16) & 0xFF);
    send_data[send_data_len+2] = (unsigned char)((frame_id >> 8) & 0xFF);
    send_data[send_data_len+3] = (unsigned char)((frame_id ) & 0xFF);
    send_data_len += 4;

    // 2. 第2部分：时间戳信息长度和时间戳净荷
    unsigned long long buf_timestamp_len = 2 * sizeof(unsigned long long);
    send_data[send_data_len+0] = (unsigned char)((buf_timestamp_len >> 24) & 0xFF);
    send_data[send_data_len+1] = (unsigned char)((buf_timestamp_len >> 16) & 0xFF);
    send_data[send_data_len+2] = (unsigned char)((buf_timestamp_len >> 8) & 0xFF);
    send_data[send_data_len+3] = (unsigned char)((buf_timestamp_len ) & 0xFF);
    send_data_len += 4;

    // 将时间戳不转换为字符串，直接以8字节的long long 拷贝到buf_info中
    memcpy(send_data + send_data_len, &sensor_timestamp, sizeof(unsigned long long));
    send_data_len += sizeof(unsigned long long);
    memcpy(send_data + send_data_len, &recv_timestamp, sizeof(unsigned long long));
    send_data_len += sizeof(unsigned long long);

    // 3. 第3部分：存放AI信息长度和AI信息净荷
    send_data[send_data_len] = (unsigned char)((buf_len >> 24) & 0xFF);
    send_data[send_data_len+1] = (unsigned char)((buf_len >> 16) & 0xFF);
    send_data[send_data_len+2] = (unsigned char)((buf_len >> 8) & 0xFF);
    send_data[send_data_len+3] = (unsigned char)((buf_len ) & 0xFF);
    send_data_len += 4;
    if (buf != NULL && buf_len > 0) {
        memcpy(&send_data[send_data_len], buf, buf_len);
    }
    send_data_len += buf_len;

    // 4. 第4部分：存放图片长度和图片净荷
    send_data[send_data_len] = (unsigned char)((encoded_data_len >> 24) & 0xFF);
    send_data[send_data_len+1] = (unsigned char)((encoded_data_len >> 16) & 0xFF);
    send_data[send_data_len+2] = (unsigned char)((encoded_data_len >> 8) & 0xFF);
    send_data[send_data_len+3] = (unsigned char)((encoded_data_len ) & 0xFF);
    send_data_len += 4;
    memcpy(&send_data[send_data_len], encoded_data, encoded_data_len);
    send_data_len += encoded_data_len;
    free(encoded_data);  // 释放编码动态申请的内存

    // 发送给需要的客户端
    frame_send_data(chn, send_data, send_data_len);

    return ret;
}

// unsigned long long sensor_timestamp;     //sensor出图时间戳
// unsigned long long recv_timestamp;       //接收完帧数据时的时间戳
// 发送带时间戳信息的图片：时间戳长度+时间戳净荷+图片长度+图片净荷
int frame_send_wi_timestamp(int chn, uint32_t frame_id, int frame_width, int frame_height, int frame_channel, void *frame_data, int pic_encode_type, unsigned long long sensor_timestamp, unsigned long long recv_timestam){
    int ret = 0;
    unsigned long long buf_timestamp[2];
    int buf_timestamp_len = 2 * sizeof(unsigned long long);

    // 不将时间戳转换为字符串，而是直接放到buf_timestamp中，注意高低字节序的问题
    buf_timestamp[0] = sensor_timestamp;
    buf_timestamp[1] = recv_timestam;
    ret = frame_send_wi_buf(chn, frame_id, frame_width, frame_height, frame_channel, frame_data, pic_encode_type, (unsigned char *)buf_timestamp, buf_timestamp_len);
    return ret;
}

// 发送带AI信息的图片：AI信息长度+AI信息净荷+图片长度+图片净荷
int frame_send_ai(int chn, int frame_width, int frame_height, int frame_channel, void *frame_data, unsigned char *buf, int buf_len){
    int i;
    int ret;
    int socket_port = 0;
    unsigned long  len_sent = 0;
    unsigned long jpg_len = 0;
    unsigned long result_len = 0;
    unsigned long send_len = 0;
    void *jpg_data = NULL;
    unsigned char send_data[FRAME_MAX_SIZE];

    frame_cnt_ai++;
    jpg_data = frame_encode_jpg(frame_data, frame_width, frame_height, &jpg_len);
    if (jpg_data == NULL) {
        printf("mrdvs, frame_send: encode jpg failed\n");
        rt_msg_send_all("frame_send: encode jpg failed"); // pghpghpgh 后续可以改为仅仅对这个连接进行错误信息发送

        return -1;
    }
    // chnn 4的数据帧格式：buf_len(4字节)+ buf + jpg_len(4字节) + jpg_data，注意当buf为NULL时，buf_len=0，buf不需要拷贝到send_data中，也就是说buf_len=0时，send_data的长度为4+4+jpg_len
    // 其他chnn的数据帧格式：jpg_len(4字节) + jpg_data
    memset(send_data, 0, FRAME_MAX_SIZE);

    if(buf == NULL){
        buf_len = 0;  
    }

    send_data[0] = (unsigned char)((buf_len >> 24) & 0xFF);
    send_data[1] = (unsigned char)((buf_len >> 16) & 0xFF);
    send_data[2] = (unsigned char)((buf_len >> 8) & 0xFF);
    send_data[3] = (unsigned char)((buf_len ) & 0xFF);
    if (buf != NULL && buf_len > 0) {
        memcpy(&send_data[4], buf, buf_len);
    }
    send_data[4+buf_len] = (unsigned char)((jpg_len >> 24) & 0xFF);
    send_data[4+buf_len+1] = (unsigned char)((jpg_len >> 16) & 0xFF);
    send_data[4+buf_len+2] = (unsigned char)((jpg_len >> 8) & 0xFF);
    send_data[4+buf_len+3] = (unsigned char)((jpg_len ) & 0xFF);
    memcpy(&send_data[4+buf_len+4], jpg_data, jpg_len);
    send_len = 4 + buf_len + 4 + jpg_len;

    free(jpg_data);

    // 发送给需要的客户端, 注意只是发给请求了chnn 4的客户端
    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }

        // socket chn为这个视频，才发送数据
        if(g_client_info.fd_info[i].chn != chn){
            if(frame_cnt_ai % 100 == 0){
                ;//printf("will not send data:fd chn=%d, index=%d, fd=%d\n", g_client_info.fd_info[i].chn, i, g_client_info.fd_info[i].fd);
            }
            continue;
        }

        //printf("will send data through g_client_fd[%d]=%d, chn=%d\n", i, g_client_info.fd_info[i].fd, chn); 
        len_sent = send(g_client_info.fd_info[i].fd, send_data, send_len, 0);
        if (len_sent < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, i, g_client_info.fd_info[i].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                //remove_connection(i, &readfd);
                printf("mrdvs, send data error, client disconnected, errno == EPIPE || errno == ECONNRESET\n");

            } else {
                perror("mrdvs, send error");
                rt_msg_send_all("mrdvs, send data length<0, send error");
            }
        }

        g_client_info.fd_info[i].frame_cnt++;

        if(g_client_info.fd_info[i].frame_cnt % 100 == 0){
            printf("mrdvs, %s:   Server --> Client. w:%d, h:%d, jpg_len:%ld,send_len:%ld, sent_len:%ld, chn:%d, frame_cnt:%d, client_fd[%d]:%d\n", 
                __FUNCTION__, frame_width,frame_height, jpg_len, send_len, len_sent, g_client_info.fd_info[i].chn, g_client_info.fd_info[i].frame_cnt, i, g_client_info.fd_info[i].fd);
        }   
    }
    //pthread_mutex_unlock(&socket_mutex); 

    return 0;
}

int frame_cnt = 0;
int frame_send(int chn, int frame_width, int frame_height, int frame_channel, void *frame_data){
    // statfps();
    int i;
    int ret;
    int socket_port = 0;
    unsigned long  len_sent = 0;
    unsigned long jpg_len = 0;
    unsigned long send_len = 0;
    void *jpg_data = NULL;
    unsigned char send_data[FRAME_MAX_SIZE];

    frame_cnt++;

    //chn 0, 1, 2, 3, 4分别对应深度图, 强度图, RGB图, 3D图(点云？), AI处理后图
    // 强度图, RGB图, 3D图(点云) ，AI处理后图 编码为 JPG 格式
    if(chn == 1 || chn == 2 || chn == 3 || chn == 4){
        jpg_data = frame_encode_jpg(frame_data, frame_width, frame_height, &jpg_len);
        if(chn == 2)
            ;//printf("frame_send: encode jpg success! w=%d, h=%d, jpg_len=%ld\n", frame_width,frame_height, jpg_len);
        if (jpg_data == NULL) {
            printf("mrdvs, frame_send: encode jpg failed\n");
            rt_msg_send_all("mrdvs, frame_send: encode jpg failed");
            return -1;
        }
    }else{  // 深度图编码为无损PNG格式
        // jpg_data = frame_data;
        // jpg_len = frame_width * frame_height * 2;
        jpg_data = frame_encode_png(CV_16U, frame_data, frame_width, frame_height, &jpg_len);
        if (jpg_data == NULL) {
            printf("mrdvs, frame_send: encode jpg failed\n");
            rt_msg_send_all("mrdvs, frame_send: encode jpg failed");
            return -1;
        }
    }

    // chnn 4的数据帧格式：result_len(4字节)+ result_data + jpg_len(4字节) + jpg_data
    // 其他chnn的数据帧格式：jpg_len(4字节) + jpg_data
    memset(send_data, 0, FRAME_MAX_SIZE);
    send_data[0] = (unsigned char)((jpg_len >> 24) & 0xFF);
    send_data[1] = (unsigned char)((jpg_len >> 16) & 0xFF);
    send_data[2] = (unsigned char)((jpg_len >> 8) & 0xFF);
    send_data[3] = (unsigned char)((jpg_len ) & 0xFF);
    memcpy(&send_data[4], jpg_data, jpg_len);
    free(jpg_data);

    send_len = jpg_len + 4;

    // 硬件抓帧，发送给需要的客户端
    //pthread_mutex_lock(&socket_mutex); 
    for(i = 0; i < MAX_CONNECTIONS; i++){
        if(g_client_info.fd_info[i].fd < 0){
            continue;
        }

        // socket chn为这个视频，才发送数据
        if(g_client_info.fd_info[i].chn != chn){
            if(frame_cnt % 100 == 0){
                ;//printf("will not send data:fd chn=%d, index=%d, fd=%d\n", g_client_info.fd_info[i].chn, i, g_client_info.fd_info[i].fd);
            }
            continue;
        }

        //printf("will send data through g_client_fd[%d]=%d, chn=%d\n", i, g_client_info.fd_info[i].fd, chn); 
        len_sent = send(g_client_info.fd_info[i].fd, send_data, send_len, 0);
        if (len_sent < 0) {
            printf("mrdvs, send data error, errno=%d, client_fd[%d]=%d\n", 
                    errno, i, g_client_info.fd_info[i].fd);
            if (errno == EPIPE || errno == ECONNRESET) {
                // 客户端已经断开连接,关闭 socket 并从客户端列表中移除
                //remove_connection(i, &readfd);
                printf("mrdvs, send data error, client disconnected, errno == EPIPE || errno == ECONNRESET\n");

            } else {
                perror("mrdvs, send error");
                rt_msg_send_all("mrdvs, send data length<0, send error");
            }
        }

        g_client_info.fd_info[i].frame_cnt++;

        if(g_client_info.fd_info[i].frame_cnt % 100 == 0){
            printf("%s:mrdvs,    Server --> Client. w:%d, h:%d, jpg_len:%ld,send_len:%ld, sent_len:%ld, chn:%d, frame_cnt:%d, client_fd[%d]:%d\n", 
                __FUNCTION__, frame_width,frame_height, jpg_len, send_len, len_sent, g_client_info.fd_info[i].chn, g_client_info.fd_info[i].frame_cnt, i, g_client_info.fd_info[i].fd);
        }   
    }
    //pthread_mutex_unlock(&socket_mutex); 

    return 0;
}

// 内存检查
#if 0
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Caught SIGINT, checking for leaks..." << std::endl;
        __lsan_do_leak_check();
        exit(0);
    }
}
#endif


int main(int argc, char** argv)
{
    // 内存检查
    //signal(SIGINT, signal_handler);

    pthread_t id_srv_th;
    size_t stacksize;
    int ret = 0;

    pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	pthread_attr_setdetachstate(&attr,PTHREAD_SCOPE_SYSTEM);

    // 设置堆栈大小
    stacksize = MAXDATASIZE*2;
    pthread_attr_setstacksize(&attr, stacksize);

    ret=pthread_create(&id_srv_th,&attr,(void* (*)(void*))server_thread, NULL);
	if(ret!=0){
		printf ("mrdvs, Create pthread error!\n");
	exit (1);
	}
    printf("mrdvs, server_thread: start success\n");


    // 初始化AI模块
    char rknn_model_path[256] = "/home/user/deploy_yolov8seg/train_yolov8n_best.rknn";
    char rknn_label_path[256] = "/home/user/deploy_yolov8seg/labeles.txt";

#if rknn_using
    pthread_mutex_lock(&g_rknnNetMutex);
    ret = g_rknnNet.rknn_envs_init(0);
    pthread_mutex_unlock(&g_rknnNetMutex);
#endif


    if(ret != 0){
        printf("mrdvs, rknn_envs_init failed, ret=%d\n", ret);
        return -1;
    }
    printf("mrdvs, rknn_envs_init success\n");

    checkTC(DcSetInfoOutput(1, true, ""));
    printf("mrdvs, call api version: %s\n", DcGetApiVersion());

    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));
    if (device_num <= 0)
    {
        printf("mrdvs, not found any device, press any key to exit");
        wait_key = getchar();
        return -1;
    }
    printf("mrdvs, DcGetDeviceList success list: %d\n", device_num);

    std::string open_param;
    //int open_mode = OPEN_BY_INDEX; //OPEN_BY_IP;
    int open_mode = OPEN_BY_IP; //OPEN_BY_IP;//
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "127.0.0.1";
        //open_param = "172.30.3.99";
        break;
        //根据sn打开设备
    case OPEN_BY_SN:
        open_param = "ccf8981cc50b66b6";
        break;
        //根据搜索设备列表索引打开设备
    case OPEN_BY_INDEX:
    default:
        open_param = "0";
        break;
    }

    DcHandle handle = 0;
    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state) {
        printf("mrdvs, open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }

    g_handle =  handle;
    printf("mrdvs, device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);
    //===============================================================================

    bool depth_enable = false, amp_enable = false, rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &depth_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &amp_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable));

    //开启深度图、强度图和RGB图
    depth_enable=true;
    amp_enable=true;
    rgb_enable=true;
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, depth_enable));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, amp_enable));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, rgb_enable));
    

    int app_mode = LX_ALGORITHM_MODE::MODE_ALL_OFF;
    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value));
    app_mode = int_value.cur_value;

    printf("mrdvs, depth_enable:%d amp_enable:%d rgb_enable:%d app_mode:%d\n", depth_enable, amp_enable, rgb_enable, app_mode);

    //启流DcRegisterFrameCallback
    checkTC(DcStartStream(handle));

    //设置回调
    checkTC(DcRegisterFrameCallback(handle, CallbackFunc, nullptr));
    
    printf("mrdvs, **********press 'q' to exit********\n\n");
    while (wait_key != 'q')
    {
        wait_key = getchar();
    }

    /********* pgh join thread ***********/
    pthread_join(id_srv_th,NULL);
    /********* pgh join thread end ***********/

    checkTC(DcUnregisterFrameCallback(handle));
    checkTC(DcStopStream(handle));
    checkTC(DcCloseDevice(handle));

    // AI模块释放

#if rknn_using
    pthread_mutex_lock(&g_rknnNetMutex);
    g_rknnNet.rknn_envs_release();
    pthread_mutex_unlock(&g_rknnNetMutex);
#endif

    return 0;
}

// 在图像上添加时间戳水印的函数
void add_timestamp_watermark(FrameDataInfo& rgb_data) {
    static const int font = cv::FONT_HERSHEY_SIMPLEX;
    static const cv::Point text_position(20, 40);
    static const double font_scale = 1.0;
    static const cv::Scalar font_color(255, 255, 255);
    static const int thickness = 2;
    
    cv::Mat image(rgb_data.frame_height, rgb_data.frame_width, CV_8UC3, rgb_data.frame_data);
    
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);
    
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &now_tm);
    
    cv::putText(image, timestamp, text_position, font, font_scale, font_color, thickness);
}

void add_timestamp_watermark_old(FrameDataInfo& rgb_data) {

    //pthread_mutex_lock(&g_handle_mutex);

    // 将 frame_data 转换为 OpenCV 的 Mat 对象
    cv::Mat image(rgb_data.frame_height, rgb_data.frame_width, CV_8UC3, rgb_data.frame_data);

    // 获取当前时间戳
    std::time_t now = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", std::localtime(&now));

    // 设置文本位置、字体、大小和颜色
    //cv::Point text_position(10, image.rows - 10);   // 左下
    cv::Point text_position(20, 40);
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    cv::Scalar font_color(255, 255, 255);  // 白色
    int thickness = 2;

    // 在图像上绘制时间戳
    cv::putText(image, timestamp, text_position, font, font_scale, font_color, thickness);

    //pthread_mutex_unlock(&g_handle_mutex);
}

// 在图像左上角添加时间戳水印，g_ai_model的值在日期后面加吧，用小括号括起来:整体例如：13：19：23（1）
void add_watermark_ai(FrameDataInfo& rgb_data) {
    static const int font = cv::FONT_HERSHEY_SIMPLEX;
    static const cv::Point text_position(20, 40);
    static const double font_scale = 1.0;
    static const cv::Scalar font_color(255, 255, 255);
    static const int thickness = 2;
    
    // 创建图像Mat对象
    cv::Mat image(rgb_data.frame_height, rgb_data.frame_width, CV_8UC3, rgb_data.frame_data);
    
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);
    
    // 格式化时间戳和模型编号
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &now_tm);
    
    // 直接在原数组上追加模型编号，避免额外内存分配
    int len = strlen(timestamp);
    snprintf(timestamp + len, sizeof(timestamp) - len, " (%d)", g_ai_model);
    
    // 绘制水印
    cv::putText(image, timestamp, text_position, font, font_scale, font_color, thickness);
}

void add_watermark_ai_old(FrameDataInfo& rgb_data){

    //pthread_mutex_lock(&g_handle_mutex);

    // 将 frame_data 转换为 OpenCV 的 Mat 对象
    cv::Mat image(rgb_data.frame_height, rgb_data.frame_width, CV_8UC3, rgb_data.frame_data);

    // 获取当前时间戳
    std::time_t now = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", std::localtime(&now));

    // 右上角有其他元素干扰，模型编号:g_ai_model的值在日期后面加吧，用小括号括起来:整体例如：13：19：23（1）
    // 将模型编号 g_ai_model 转换为字符串并拼接到时间戳后面
    std::sprintf(timestamp + std::strlen(timestamp), " (%d)", g_ai_model);

    // 设置文本位置、字体、大小和颜色
    cv::Point text_position(20, 40);
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    cv::Scalar font_color(255, 255, 255);  // 白色
    int thickness = 2;

    // 在图像左上角绘制时间戳和g_ai_model的值
    cv::putText(image, timestamp, text_position, font, font_scale, font_color, thickness);

    // 在图像右上角绘制当前应用的模型编号:g_ai_model的值
    //cv::Point model_position(image.cols - 200, 40);
    //cv::putText(image, "AIModel:"+ std::to_string(g_ai_model), model_position, font, font_scale, font_color, thickness);

    //pthread_mutex_unlock(&g_handle_mutex);
    return;
}


// AI API: int interface(image_buffer_t *img, object_detect_result_list *results); 
int ai_interface(FrameDataInfo *rgb_img, image_buffer_t *rgb_img_out, object_detect_result_list *results) {
    int ret = 0;

    // 将rgb_img指针中的数据拷贝到rgb_img_out中
    rgb_img_out->width = rgb_img->frame_width;
    rgb_img_out->height = rgb_img->frame_height;
    rgb_img_out->size = rgb_img->frame_width * rgb_img->frame_height * rgb_img->frame_channel;
    //rgb_img_out->channel = rgb_img->frame_channel;
    //rgb_img_out->data = (unsigned char *)malloc(rgb_img_out->width * rgb_img_out->height * rgb_img_out->channel);
    memcpy(rgb_img_out->virt_addr, rgb_img->frame_data, (rgb_img->frame_width * rgb_img->frame_height * rgb_img->frame_channel));
    rgb_img_out->format =  IMAGE_FORMAT_RGB888;
    //rgb_img_out->timestamp = 0;
    //rgb_img_out->frame_id = 0;
    //ret = g_rknnNet.interface(rgb_img_out, results, );
    //ret = g_rknnNet.draw_bbox_mask(rgb_img_out, results);
    return ret;
}

void memtest_once(const void *data, int width, int height){
    unsigned pixels = width * height;
    unsigned char *p = (unsigned char *)data;

    unsigned char t1 = 0;
    volatile unsigned char t2;

    printf("mrdvs, memtest: %p %dx%d\n", data, width, height);
    for (unsigned i = 0; i < pixels; i++){
        t1 ^= p[2*i + 0];
        t1 ^= p[2*i + 1];
        // t1 ^= p[3*i + 2];
    }

    for (unsigned i = 0; i < pixels; i++){
        t1 ^= p[3*i + 0];
        t1 ^= p[3*i + 1];
        t1 ^= p[3*i + 2];
    }
    t2 = t1;
    (void)t2;
}

void memtest(const void *data, int width, int height){
    for (int i = 0; i < 100; i++){
        if (i % 10 == 0)
            printf("mrdvs, memtest[%d]: %p %dx%d\n", i, data, width, height);
        memtest_once(data, width, height);
    }
}

// 获取摄像头自动曝光等级： DcGetIntValue(handle, LX_INT_AUTO_EXP_LEVEL, &exp_level);
// 设置摄像头自动曝光等级： DcSetIntValue(handle, LX_INT_AUTO_EXP_LEVEL, exp_level);

// 用于帧同步和校验，一次callback回来的数据进行相同的编号；如果帧指针为空，frame_id也+1，这样上层通过编号可以知道callback是否为空
uint32_t frame_id = 0; 

void CallbackFunc(FrameInfo* frame_ptr, void* usr_data)
{
    //pthread_mutex_lock(&g_handle_mutex);

    int chn_need_send = 0;
    frame_id++;

    //此函数内部,尽量不要做复杂操作,否则可能阻塞内部数据回调
    if (frame_ptr == nullptr)
    {
        goto ret_func;
    }
    if (frame_ptr->frame_state != LX_SUCCESS)
    {
        if (frame_ptr->frame_state == LX_E_FRAME_ID_NOT_MATCH) {
            printf("mrdvs, is enabled sync frame, but frame_state:%d\n", frame_ptr->frame_state);
            goto ret_func;
        }
        else if (frame_ptr->frame_state == LX_E_FRAME_MULTI_MACHINE) {
            printf("mrdvs, recv multi machine singal, frame_state:%d\n", frame_ptr->frame_state);
            goto ret_func;
        }
        else {
            printf("mrdvs, CallbackFunc; frame_state:%d\n", frame_ptr->frame_state);
            goto ret_func;
        }
    }

    //若需要帧id信息,可选通过此结构体获取, 若需要帧同步, 则可通过将此处接收到的数据缓存到队列中,后续在队列中匹配帧id方式实现
    /* 需求：
        1. AI图增加时间戳+AI模型水印：13:19:23（1）
        2. 所有通道协议都统一增加时间戳信息
    */
/*
    FrameExtendInfo* p_extend_info = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        p_extend_info = (FrameExtendInfo*)frame_ptr->reserve_data;
*/

    // 深度图， chn 0, 1, 2, 3 分别对应深度图, 强度图, RGB图, 3D图(点云？)
    // 点云图：经过和摄像头技术支持沟通，摄像头SDK不输出点云图，点云图是应用层获取深度图，在深度图基础之上进行计算得到。
    // Chn 3是SDK算法输出结果，大致包含SDK实现的避障等信息

handler_chn0:       
    chn_need_send = frame_send_data_judge(0);
    if(chn_need_send == 0)
        goto handler_chn1;

    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        // 获取深度图数据
        FrameDataInfo rgb_data = frame_ptr->depth_data;
        if(rgb_data.frame_data == nullptr)
            goto ret_func;
        //add_timestamp_watermark(rgb_data);
        // memtest(rgb_data.frame_data, rgb_data.frame_width, rgb_data.frame_height);
        frame_send_wi_timestamp(0, frame_id, rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel, rgb_data.frame_data,1, rgb_data.sensor_timestamp, rgb_data.recv_timestamp);

        //获取点云数据(可选, 假如需要的话)      3·D图像
        float* xyz_data = nullptr;
        if(frame_ptr->handle == 0)
            goto ret_func;
        if (DcGetPtrValue(frame_ptr->handle, LX_PTR_XYZ_DATA, (void**)&xyz_data) != LX_SUCCESS)
            printf("mrdvs, get xyz failed");

#if 0
        if (xyz_data != nullptr)
        {
            //第yRows行xCol列点云数据
            int yRow = 100, xCol = 100;
            int pose = yRow * frame_ptr->depth_data.frame_width + xCol;
            float x = xyz_data[pose * 3];
            float y = xyz_data[pose * 3 + 1];
            float z = xyz_data[pose * 3 + 2];
            //std::cout << " x:" << x << " y:" << y << " z:" << z << std::endl;
            // FrameDataInfo rgb_data = frame_ptr->depth_data;
            // if(rgb_data.frame_data == nullptr)
            //     return;
            // add_timestamp_watermark(rgb_data);
            // //printf("CallbackFunc: get fame data, frame chn=%d\n", 3);
            // frame_send(3, rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel, rgb_data.frame_data);  
        }
#endif
    }

handler_chn1:       
    chn_need_send = frame_send_data_judge(1);
    if(chn_need_send == 0)
        goto handler_chn2;   

    //强度图，chn 0, 1, 2, 3 分别对应深度图, 强度图, RGB图, 3D图(点云？)
    if (frame_ptr->amp_data.frame_data != nullptr)
    {
        FrameDataInfo rgb_data = frame_ptr->amp_data;
        if(rgb_data.frame_data == nullptr){
            printf("mrdvs, amp_data.frame_data is nullptr\n");
            goto ret_func;
        }   
            
        //add_timestamp_watermark(rgb_data);
        frame_send_wi_timestamp(1, frame_id, rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel, rgb_data.frame_data, 1, rgb_data.sensor_timestamp, rgb_data.recv_timestamp);
        //frame_send(1, rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel,rgb_data.frame_data);
        //if (p_extend_info != nullptr)
        //    printf("amp frame_id:%d\n", p_extend_info->amp_frame_id);
    }

handler_chn2:       
    chn_need_send = frame_send_data_judge(2);
    if(chn_need_send == 0)
        goto handler_chn3; 

    //RGB图，chn 0, 1, 2, 3 分别对应深度图, 强度图, RGB图, 3D图(点云？)
    if (frame_ptr->rgb_data.frame_data != nullptr)
    {   
        FrameDataInfo rgb_data = frame_ptr->rgb_data;
        if(rgb_data.frame_data == nullptr)
            goto ret_func;
      
        add_timestamp_watermark(rgb_data);

        // RGB图，发送数据：时间戳长度+时间戳净荷+图片长度+图片净荷
        frame_send_wi_timestamp(2, frame_id,rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel, rgb_data.frame_data, 0, rgb_data.sensor_timestamp, rgb_data.recv_timestamp);
    }

handler_chn3:       
    chn_need_send = frame_send_data_judge(3);
    if(chn_need_send == 0)
        goto handler_chn4; 

    //chn 0, 1, 2, 3 分别对应深度图, 强度图, RGB图, 3D图(点云？)
    // 点云图：经过和摄像头技术支持沟通，摄像头SDK不输出点云图，点云图是应用层获取深度图，在深度图基础之上进行计算得到。
    // Chn 3是SDK算法输出结果，大致包含SDK实现的避障等信息，不是3D图，也不是点云图
    if (frame_ptr->app_data.frame_data != nullptr)
    {   
        FrameDataInfo rgb_data = frame_ptr->app_data;
        if(rgb_data.frame_data == nullptr)
            goto ret_func;
        //add_timestamp_watermark(rgb_data);
        frame_send_wi_timestamp(3, frame_id, rgb_data.frame_width, rgb_data.frame_height, rgb_data.frame_channel, rgb_data.frame_data, 0, rgb_data.sensor_timestamp, rgb_data.recv_timestamp);
        //printf("chn=%d, rgb sensor_timestamp:%lld\n", 2, frame_ptr->rgb_data.sensor_timestamp);
        //if (p_extend_info != nullptr)
        //    printf("rgb frame_id:%d\n", p_extend_info->rgb_frame_id);
    }

handler_chn4:       
    chn_need_send = frame_send_data_judge(4);
    if(chn_need_send == 0)
        goto ret_func;

    if (frame_ptr->rgb_data.frame_data != nullptr)
    {   
        FrameDataInfo rgb_data = frame_ptr->rgb_data;
        if(rgb_data.frame_data == nullptr)
            goto ret_func;

        // AI处理
        //my_image_buffer_t rgb_img_out;
        image_buffer_t rgb_img_out = {};
        FrameDataInfo rgb_img_ai = {};
        object_detect_result_list results = {};

        // 分配内存给 rgb_img_out.virt_addr
        rgb_img_out.virt_addr = (unsigned char *)malloc(MAX_RGB_IMAGE_SIZE);
        Defer virt_addr_deleter { [&] { free(rgb_img_out.virt_addr); } };
    
        if (!rgb_img_out.virt_addr) {
            fprintf(stderr, "mrdvs, Failed to allocate memory for rgb_img_out.virt_addr\n");
            goto ret_func;
        }
        memset(rgb_img_out.virt_addr, 0, MAX_RGB_IMAGE_SIZE);
        
        //ai_interface(&rgb_data, &rgb_img_out, &results);

        // 将rgb_img指针中的数据拷贝到rgb_img_out中
        rgb_img_out.width = rgb_data.frame_width;
        rgb_img_out.height = rgb_data.frame_height;
        rgb_img_out.size = rgb_data.frame_width * rgb_data.frame_height * rgb_data.frame_channel;
        //rgb_img_out.virt_addr = (unsigned char*)rgb_data.frame_data;
        memcpy(rgb_img_out.virt_addr, rgb_data.frame_data, (rgb_data.frame_width * rgb_data.frame_height * rgb_data.frame_channel));
        rgb_img_out.format =  IMAGE_FORMAT_RGB888;
        rgb_img_out.fd = 0;
        rgb_img_out.width_stride = 0;
        rgb_img_out.height_stride = 0;

        uint8_t *buf; // 这个buf是用来存储AI处理结果的
        unsigned long buf_len = 0;
        int ret = 0;

#if rknn_using
        pthread_mutex_lock(&g_rknnNetMutex);
        buf = g_rknnNet.interface(&rgb_img_out, &results, &buf_len);
        pthread_mutex_unlock(&g_rknnNetMutex);

        pthread_mutex_lock(&g_rknnNetMutex);
        ret = g_rknnNet.draw_bbox_mask(&rgb_img_out, &results);
        pthread_mutex_unlock(&g_rknnNetMutex);
#endif

        if(buf == NULL){
            ;//printf(" AI process failed, buf_len:%ld\n", buf_len);
        }
        
        rgb_img_ai.frame_data_type = LX_DATA_UNSIGNED_CHAR;
        rgb_img_ai.frame_width = rgb_img_out.width;
        rgb_img_ai.frame_height = rgb_img_out.height;
        rgb_img_ai.frame_channel = 3;
        rgb_img_ai.frame_data = rgb_img_out.virt_addr;
        rgb_img_ai.sensor_timestamp = rgb_data.sensor_timestamp;
        rgb_img_ai.recv_timestamp = rgb_data.recv_timestamp;
        
        add_watermark_ai(rgb_img_ai);

        // AI图，发送数据：时间戳长度+时间戳净荷+ AI信息长度+AI信息净荷+图片长度+图片净荷
        //frame_send_ai(4, rgb_img_ai.frame_width, rgb_img_ai.frame_height, rgb_img_ai.frame_channel, rgb_img_ai.frame_data, buf, buf_len);
        frame_send_wi_timestamp_ai(4, frame_id, rgb_img_ai.frame_width, rgb_img_ai.frame_height, rgb_img_ai.frame_channel, rgb_img_ai.frame_data, 0, rgb_data.sensor_timestamp, rgb_data.recv_timestamp, buf, buf_len);

        free(buf);  // 释放AI处理结果动态申请的内存
    }

#ifdef HAS_OPENCV
    if (frame_ptr->depth_data.frame_data != nullptr) {
        printf("11111111111111111111111111\n");
        FrameDataInfo depth_data = frame_ptr->depth_data;
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data);

        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
        printf("222222222222222222222222222222\n");
    }

    if (frame_ptr->amp_data.frame_data != nullptr) {
        FrameDataInfo amp_data = frame_ptr->amp_data;
        cv::Mat amp_image = cv::Mat(amp_data.frame_height, amp_data.frame_width,
            CV_MAKETYPE(amp_data.frame_data_type, amp_data.frame_channel), amp_data.frame_data);

        cv::Mat show;
        if (amp_image.type() == CV_16U)
            amp_image.convertTo(show, CV_8U, 1.0 / 8);
        else
            show = amp_image;

        cv::namedWindow("amp", 0);
        cv::resizeWindow("amp", 640, 480);
        cv::imshow("amp", show);
    }

    if (frame_ptr->rgb_data.frame_data != nullptr) {
        FrameDataInfo rgb_data = frame_ptr->rgb_data;
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data);

        cv::namedWindow("rgb", 0);
        cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
    } 
    cv::waitKey(1);
#endif

#if 0
    //process application
    if (frame_ptr->app_data.frame_data != nullptr)
    {
        if (frame_ptr->app_data.frame_data_type == LX_DATA_PALLET)
        {
            LxPalletPose* palletdata = (LxPalletPose*)frame_ptr->app_data.frame_data;
            //printf("palletdata ret:%d x:%f, y:%f yaw:%f\n",palletdata->return_val, palletdata->x, palletdata->y, palletdata->yaw);
        }
        else if (frame_ptr->app_data.frame_data_type == LX_DATA_OBSTACLE)
        {
            LxAvoidanceOutput* obstacle = (LxAvoidanceOutput*)frame_ptr->app_data.frame_data;
            if (obstacle->state != LxAvSuccess) {
                printf("state is not success\n");
                return;
            }

            if (obstacle->obstacleBoxs != nullptr)
            {
                printf(" obstacle data ");
                int box_num = obstacle->number_box < 10 ? obstacle->number_box : 10;
                for (int i = 0; i < box_num; i++)
                {
                    printf("index:%d width:%f height:%f depth:%f center:%f %f %f",
                        i, obstacle->obstacleBoxs[i].width, obstacle->obstacleBoxs[i].height, obstacle->obstacleBoxs[i].depth,
                        obstacle->obstacleBoxs[i].center[0], obstacle->obstacleBoxs[i].center[1], obstacle->obstacleBoxs[i].center[2]);
                }
            }
        }
    }
#endif

ret_func:
    //pthread_mutex_unlock(&g_handle_mutex);
    return;
}
