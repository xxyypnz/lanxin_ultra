#include "device.h"
#include "macro.h"

// 多个摄像头的情况考虑使用 array 或者 map 保存句柄集合
// 释放的时候同样考虑操作 g_handle 的集合
DcHandle g_handle = 0;

void device_init(void){
    CHECK(DcSetInfoOutput(2, false, "./log/"));
    
    int dev_num = 0;
    LxDeviceInfo* dev_list = NULL;

    CHECK(DcGetDeviceList(&dev_list, &dev_num));
    if(dev_num <= 0){
        fprintf(stderr, "未找到设备，请检查设备连接\n");
        exit(-1);
    }

    int open_mode = OPEN_BY_IP;
    const char* open_param = "127.0.0.1";
    DcHandle handle = 0;
    LxDeviceInfo dev_info;

    LX_STATE lx_state = DcOpenDevice(open_mode, open_param, &handle, &dev_info);
    if(lx_state != LX_SUCCESS){
        fprintf(stderr, "打开设备失败，请检查设备连接\n");
        exit(-1);
    }

    g_handle = handle;

    bool depth_enable = true, amp_enable = true, rgb_enable = true;
    CHECK(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, depth_enable));
    CHECK(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, amp_enable));
    CHECK(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, rgb_enable));

    // 是否开启算法模式
    // ...
    
    // 启流，但是不注册回调
    lx_state = DcStartStream(handle);
    if(lx_state != LX_SUCCESS){
        fprintf(stderr, "启流失败，请检查设备连接\n");
        exit(-1);
    }
    
    // printf("设备启动成功\n");
    return;
}

void device_free(void){
    // CHECK(DcUnregisterFrameCallback(g_handle));
    CHECK(DcStopStream(g_handle));
    CHECK(DcCloseDevice(g_handle));

    return;
}
