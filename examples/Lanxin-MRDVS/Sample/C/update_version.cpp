// update_version.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <vector>
#ifdef _WIN32
#include <Windows.h>
#include <io.h>
#else
#include <unistd.h>
#include <dirent.h>
#include "string.h"
#include <regex>
#endif
#include "lx_camera_api.h"

using namespace std;

static char wait_key = '0';
static DcHandle handle = 0;
#define checkTC(state) {LX_STATE val=state;                            \
    if(val != LX_SUCCESS){                                             \
        if(val == LX_E_RECONNECTING){                                  \
            std::cout << " device reconnecting" << std::endl;}         \
        else{                                                          \
            std::cout << DcGetErrorString(val)<<std::endl;             \
            std::cout << " press any key to exit!" << std::endl;       \
            DcCloseDevice(handle);                                     \
            wait_key = getchar();                                      \
            return -1;                                                 \
        }                                                              \
    }                                                                  \
}

void get_files_in_dir(const char* dir_, std::vector<std::string>* fileNames, const char* subfix);

int main(int argc, char** argv)
{
    checkTC(DcSetInfoOutput(2, true, "./"));
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;

    //查找相机
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));    
    if (device_num <= 0) 
    {
        std::cout << "not found any device, press any key to exit" << std::endl;
        wait_key = getchar();
        return -1;
    }
    std::cout << "DcGetDeviceList success list: " << device_num << std::endl;

    //打开相机
    std::string open_param;
    LX_OPEN_MODE open_mode = OPEN_BY_INDEX;
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.12";
        break;
        //根据id打开设备
    case OPEN_BY_ID:
        open_param = "F13301122647";
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

    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice(open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state){
        std::cout << "open device failed, open_mode:" << open_mode << " open_param:" << open_param << " press any key to exit" << std::endl;
        wait_key = getchar();
        return -1;
    }

    std::cout << "device_info\n cameraid:" << device_info.id << "\n uniqueid:" << handle
        << "\n cameraip:" << device_info.ip << "\n firmware_ver:" << device_info.firmware_ver << "\n sn:" << device_info.sn
        << "\n name:" << device_info.name << "\n img_algor_ver:" << device_info.algor_ver << std::endl << endl;

    std::vector<std::string> fileNames;
    const char* subfix = nullptr;
    if (device_info.dev_type == LX_DEVICE_M4
        || device_info.dev_type == LX_DEVICE_M4Pro)
        subfix = "M4-update";
    else if(device_info.dev_type == LX_DEVICE_S2)
        subfix = "S2-update";

    get_files_in_dir("", &fileNames, subfix);
    if (fileNames.empty())
        get_files_in_dir("../Firmware", &fileNames, subfix);
    if (fileNames.empty())
        get_files_in_dir("../../Firmware", &fileNames, subfix);
    if (fileNames.empty())
    {
        wait_key = getchar();
        return -1;
    }

    if (fileNames.size() > 1)
    {
        std::cout << "Found Too Many Matched Files, Please Check " << endl;
        for (auto name : fileNames)
            cout << name << endl;
        wait_key = getchar();
        return -1;
    }

    std::cout << endl << "Update Firmware " << fileNames[0] <<"  (Y/N)?" << endl;
    wait_key = getchar();
    if (wait_key == 'Y' || wait_key == 'y')
    {
        while (getchar() != '\n') {}
        checkTC(DcSetStringValue(handle, LX_STRING_FIRMWARE_NAME, fileNames[0].c_str()));
        std::cout << "Update Done. any key to exit" << std::endl;
        wait_key = getchar();
    }
    DcCloseDevice(handle);
    return 0;
}

void get_files_in_dir(const char* dir_, std::vector<std::string>* fileNames,
    const char* subfix) {
    fileNames->clear();
#ifdef _WIN32
    string basedir_ = dir_;
    string second_dir = subfix;
    if (!basedir_.empty())
    {
        basedir_ += "/";
        second_dir = basedir_ + subfix + "*";
    }

    WIN32_FIND_DATA findData_;
    HANDLE hFind_ = FindFirstFile(second_dir.c_str(), &findData_);
    if (hFind_ != INVALID_HANDLE_VALUE) {
        do {
            if (strcmp(findData_.cFileName, ".") == 0 ||
                strcmp(findData_.cFileName, "..") == 0)
                continue;
            fileNames->push_back(basedir_ + findData_.cFileName);

        } while (FindNextFile(hFind_, &findData_));
    }
#else
    string basedir_ = dir_;
    if (basedir_.empty())
    {
        char* dir = getcwd(NULL, 0);
        basedir_ = dir;
        free(dir);
    }

    basedir_ += "/";
    string second_dir = subfix;
    DIR* dir_sys_base = opendir(basedir_.c_str());
    if (dir_sys_base == NULL) {
        std::cout << "open dir dev failed , cannot open " << basedir_ << std::endl;
        return;
    }

    struct dirent* ptr;
    while ((ptr = readdir(dir_sys_base)) != NULL) {
        if (ptr->d_type != DT_REG)
            continue;
        if (regex_match(ptr->d_name, regex(second_dir + ".*")))
        {
            fileNames->push_back(basedir_ + ptr->d_name);
        }
    }
#endif
    if (fileNames->empty())
        std::cout << "Not Found Matched File. PATH: " << basedir_ << " NAME: " << subfix << endl;
}
