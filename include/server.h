#ifndef LANXIN_ULTRA_SERVER_H
#define LANXIN_ULTRA_SERVER_H

#include "system.h"


#define MAX_EVENTS 1024
#define BACKLOG 64
#define MESSAGE_PORT 10001 // 消息端口实际管理客户端发送消息和服务端返回非 AI 识别视频帧
#define VIDEO_PORT 10002 // 视频端口功能一样，区别是专门返回 AI 识别的视频帧
#define MAX_MESSAGES 1024
#define MAX_CONNECTIONS 16
#define CHANNEL_COUNT 3
#define MAX_JSON 128

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

typedef struct{
    int fd;
    bool valid;
    bool channel[CHANNEL_COUNT];
}ClientInfo;

extern ClientInfo g_client_map[MAX_CONNECTIONS];
extern pthread_rwlock_t g_client_map_lock;
extern ClientInfo g_clientAI_map[MAX_CONNECTIONS];
extern pthread_rwlock_t g_clientAI_map_lock;

/**/
/** 停止通知 eventfd，由主线程写入、server 线程注册到 epoll 并读取。 */
extern int g_server_stop_fd;

/** 在创建 server 线程前创建停止通知 fd。 */
int server_stop_init(void);

/** 通知 epoll_wait 退出，不改变 epoll 的无限阻塞模式。 */
void server_stop_notify(void);
/**/

void* server(void* arg);

#endif
