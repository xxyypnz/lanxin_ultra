# ARM64 RGB detector deployment

The detector is an embedded CPython module behind a small C ABI shared library.
`consumer` passes RGB888 pixels to `detector_detect_rgb`; the returned RGB888
buffer contains boxes and `label confidence%` overlays. Detection JSON is
available to the C caller for logging or a future metadata protocol.

## Target dependencies

Build and run on Ubuntu 20.04 ARM64, or use an AArch64 cross toolchain together
with an Ubuntu 20.04 ARM64 sysroot containing the same CPython development
package and camera SDK. Native target compilation is the recommended path;
this avoids mixing x86 headers/libraries into an ARM64 `.so`.

Install the compiler, Python embedding headers, and runtime packages on the
device:

```sh
sudo apt update
sudo apt install -y build-essential python3-dev python3-numpy python3-opencv \
    libpng-dev zlib1g-dev libcjson-dev
```

The camera SDK runtime/development library must also be installed at the path
configured by the existing Makefile (`/opt/Lanxin-MRDVS`). Check that the
interpreter and compiler are both AArch64/ARM64 and that `python3-config`
belongs to the same Python installation used at runtime:

```sh
uname -m
python3 --version
python3-config --includes --embed --ldflags
```

Obtain a YOLOv8n model exported as a fixed-shape, RGB, 640x640 ONNX model with
80 COCO classes, preferably opset 12 for the OpenCV DNN version shipped with
Ubuntu 20.04. Place it at `models/yolov8n.onnx`. The model weights are not
bundled in source control; use a model whose license and target accuracy are
appropriate for the deployment. The matching class order is
`models/coco.names`.

Build on the device from the project root:

```sh
make clean
make PYTHON_CONFIG=python3-config
```

This creates `build/libdetector.so` and links the executable against it and
the target's embedded Python library. The executable has an `$ORIGIN/build`
runtime search path for the detector library. Run from the project root so
the default module/model paths resolve:

```sh
./lanxin_ultra
```

Paths and thresholds can be overridden without rebuilding:

```sh
DETECTOR_PYTHON_DIR=./detector/python \
DETECTOR_MODEL=./models/yolov8n.onnx \
DETECTOR_LABELS=./models/coco.names \
DETECT_CONFIDENCE=0.35 DETECT_NMS=0.45 ./lanxin_ultra
```

If the model or Python runtime is unavailable, depth and intensity continue to
stream on port 10001 and RGB is sent unannotated on port 10002, with the
detector initialization error printed to stderr. For production, treat that
startup message as a deployment failure and provision the model and packages
before starting the service.
