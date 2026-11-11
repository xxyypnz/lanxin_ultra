#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "detector_api.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

static pthread_mutex_t g_detector_mutex = PTHREAD_MUTEX_INITIALIZER;
static PyObject *g_detector_module;
static PyObject *g_detect_function;
static int g_python_started_here;

static void set_error(char *error, size_t capacity, const char *message) {
    if (error == NULL || capacity == 0) return;
    snprintf(error, capacity, "%s", message != NULL ? message : "unknown Python error");
}

static void set_python_error(char *error, size_t capacity) {
    PyObject *type = NULL, *value = NULL, *traceback = NULL;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);
    PyObject *text = value != NULL ? PyObject_Str(value) : NULL;
    const char *message = text != NULL ? PyUnicode_AsUTF8(text) : NULL;
    set_error(error, capacity, message != NULL ? message : "Python detector call failed");
    Py_XDECREF(text);
    Py_XDECREF(type);
    Py_XDECREF(value);
    Py_XDECREF(traceback);
}

int detector_init(const char *python_dir, const char *model_path,
                  const char *labels_path, char *error, size_t error_capacity) {
    if (python_dir == NULL || model_path == NULL || labels_path == NULL) {
        set_error(error, error_capacity, "detector paths must not be null");
        return -1;
    }

    pthread_mutex_lock(&g_detector_mutex);
    if (g_detect_function != NULL) {
        pthread_mutex_unlock(&g_detector_mutex);
        return 0;
    }

    if (!Py_IsInitialized()) {
        Py_Initialize();
        g_python_started_here = 1;
    }
    PyGILState_STATE gil = PyGILState_Ensure();

    PyObject *sys_path = PySys_GetObject("path");
    PyObject *path = PyUnicode_FromString(python_dir);
    if (sys_path == NULL || path == NULL || PyList_Insert(sys_path, 0, path) != 0) {
        Py_XDECREF(path);
        set_python_error(error, error_capacity);
        PyGILState_Release(gil);
        pthread_mutex_unlock(&g_detector_mutex);
        return -1;
    }
    Py_DECREF(path);

    g_detector_module = PyImport_ImportModule("yolo_detector");
    if (g_detector_module != NULL) {
        PyObject *initialize = PyObject_GetAttrString(g_detector_module, "initialize");
        PyObject *args = Py_BuildValue("(ss)", model_path, labels_path);
        PyObject *result = initialize != NULL && args != NULL
                               ? PyObject_CallObject(initialize, args)
                               : NULL;
        Py_XDECREF(args);
        Py_XDECREF(initialize);
        if (result != NULL) {
            Py_DECREF(result);
            g_detect_function = PyObject_GetAttrString(g_detector_module, "detect_rgb");
        }
    }

    if (g_detect_function == NULL) {
        set_python_error(error, error_capacity);
        Py_XDECREF(g_detector_module);
        g_detector_module = NULL;
        PyGILState_Release(gil);
        pthread_mutex_unlock(&g_detector_mutex);
        return -1;
    }

    PyGILState_Release(gil);
    if (g_python_started_here) (void)PyEval_SaveThread();
    pthread_mutex_unlock(&g_detector_mutex);
    return 0;
}

int detector_detect_rgb(const unsigned char *rgb, int width, int height,
                        unsigned char *rgb_out, size_t rgb_out_capacity,
                        size_t *rgb_out_length, char *detections,
                        size_t detections_capacity, char *error,
                        size_t error_capacity) {
    if (rgb == NULL || rgb_out == NULL || rgb_out_length == NULL || width <= 0 || height <= 0) {
        set_error(error, error_capacity, "invalid RGB detector arguments");
        return -1;
    }
    size_t expected = (size_t)width * (size_t)height * 3U;
    if (expected > rgb_out_capacity || expected > (size_t)PY_SSIZE_T_MAX) {
        set_error(error, error_capacity, "RGB output buffer is too small");
        return -1;
    }

    pthread_mutex_lock(&g_detector_mutex);
    if (g_detect_function == NULL) {
        set_error(error, error_capacity, "detector is not initialized");
        pthread_mutex_unlock(&g_detector_mutex);
        return -1;
    }

    PyGILState_STATE gil = PyGILState_Ensure();
    PyObject *input = PyBytes_FromStringAndSize((const char *)rgb, (Py_ssize_t)expected);
    PyObject *args = input != NULL ? Py_BuildValue("Oii", input, width, height) : NULL;
    PyObject *result = args != NULL ? PyObject_CallObject(g_detect_function, args) : NULL;
    Py_XDECREF(args);
    Py_XDECREF(input);

    int status = -1;
    if (result == NULL || !PyTuple_Check(result) || PyTuple_Size(result) != 2) {
        set_python_error(error, error_capacity);
    } else {
        PyObject *image = PyTuple_GetItem(result, 0);
        PyObject *json = PyTuple_GetItem(result, 1);
        char *image_bytes = NULL;
        Py_ssize_t image_size = 0;
        if (PyBytes_AsStringAndSize(image, &image_bytes, &image_size) == 0 &&
            (size_t)image_size == expected && PyUnicode_Check(json)) {
            Py_ssize_t json_size = 0;
            const char *json_text = PyUnicode_AsUTF8AndSize(json, &json_size);
            if (json_text != NULL && detections_capacity > (size_t)json_size) {
                memcpy(rgb_out, image_bytes, expected);
                if (detections != NULL) {
                    memcpy(detections, json_text, (size_t)json_size);
                    detections[json_size] = '\0';
                }
                *rgb_out_length = expected;
                status = 0;
            } else {
                set_error(error, error_capacity, "detection JSON output buffer is too small");
            }
        } else {
            set_error(error, error_capacity, "Python detector returned invalid image or metadata");
        }
    }
    Py_XDECREF(result);
    if (status != 0 && PyErr_Occurred()) PyErr_Clear();
    PyGILState_Release(gil);
    pthread_mutex_unlock(&g_detector_mutex);
    return status;
}

void detector_shutdown(void) {
    pthread_mutex_lock(&g_detector_mutex);
    if (Py_IsInitialized() && g_detector_module != NULL) {
        PyGILState_STATE gil = PyGILState_Ensure();
        Py_XDECREF(g_detect_function);
        Py_XDECREF(g_detector_module);
        g_detect_function = NULL;
        g_detector_module = NULL;
        PyGILState_Release(gil);
    }
    /* Embedded CPython is intentionally left alive; interpreter finalization is unsafe with SDK threads. */
    pthread_mutex_unlock(&g_detector_mutex);
}
