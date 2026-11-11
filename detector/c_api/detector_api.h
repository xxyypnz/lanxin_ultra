#ifndef LANXIN_ULTRA_DETECTOR_API_H
#define LANXIN_ULTRA_DETECTOR_API_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int detector_init(const char *python_dir, const char *model_path,
                  const char *labels_path, char *error, size_t error_capacity);
int detector_detect_rgb(const unsigned char *rgb, int width, int height,
                        unsigned char *rgb_out, size_t rgb_out_capacity,
                        size_t *rgb_out_length, char *detections,
                        size_t detections_capacity, char *error,
                        size_t error_capacity);
void detector_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif
