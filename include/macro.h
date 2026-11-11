#ifndef LANXIN_ULTRA_MACRO_H
#define LANXIN_ULTRA_MACRO_H

#include "lx_camera_define.h"
#include "system.h"

#define CHECK(state) do { \
    LX_STATE check_state_ = (state); \
    if (check_state_ != LX_SUCCESS) { \
        if (check_state_ == LX_E_NOT_SUPPORT) { \
            fprintf(stderr, "mrdvs, not support this operator\n"); \
        } else if (check_state_ == LX_E_RECONNECTING) { \
            fprintf(stderr, "mrdvs, device reconnecting\n"); \
        } else if (check_state_ == LX_ERROR || check_state_ == LX_E_INPUT_ILLEGAL) { \
            fprintf(stderr, "mrdvs, press any key to exit!\n"); \
            (void)getchar(); \
        } else { \
            fprintf(stderr, "mrdvs, press any key to continue\n"); \
            (void)getchar(); \
        } \
    } \
} while (0)

#endif
