# Linux/GNU Make build for the POSIX server.

TARGET      ?= lanxin_ultra
BUILD_DIR   ?= build
CC          ?= gcc
PYTHON_CONFIG ?= python3-config

# SDK headers are vendored; the runtime SDK library is installed under /opt/Lanxin-MRDVS.
CPPFLAGS    += -Iinclude -I3rd_party -Idetector/c_api $(CJSON_INCLUDE_FLAGS)
CFLAGS      ?= -std=gnu11 -O1 -g -Wall -Wextra -pthread -fsanitize=address -fno-omit-frame-pointer
LDFLAGS     += $(LX_SDK_LIBRARY_PATH) -L$(BUILD_DIR) -Wl,-rpath,'$$ORIGIN/$(BUILD_DIR)'
LDLIBS      += -pthread $(CJSON_LIBS) $(LX_SDK_LIBS) -ldetector -lpng -lz -fsanitize=address
DETECTOR_SO := $(BUILD_DIR)/libdetector.so
PYTHON_CFLAGS  := $(shell $(PYTHON_CONFIG) --includes 2>/dev/null)
PYTHON_LDFLAGS := $(shell $(PYTHON_CONFIG) --embed --ldflags 2>/dev/null)

# Override these when the dependency names or install paths differ on the target.
CJSON_INCLUDE_FLAGS ?=
CJSON_LIBS           ?= -lcjson
LX_SDK_LIBRARY_PATH ?= -L/opt/Lanxin-MRDVS/lib
LX_SDK_LIBS          ?= -lLxCameraApi

SOURCES := $(wildcard src/*.c)
OBJECTS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SOURCES))
DEPFILES := $(OBJECTS:.o=.d)
ASAN_DEFAULT_OPTIONS_OBJ := $(BUILD_DIR)/asan_default_options.o

.PHONY: all clean detector

all: $(TARGET)

# Link all project objects with pthread, cJSON, and the camera SDK.
$(TARGET): $(OBJECTS) $(ASAN_DEFAULT_OPTIONS_OBJ) $(DETECTOR_SO)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) $(ASAN_DEFAULT_OPTIONS_OBJ) $(LDLIBS)

# Bake the vendor SDK mismatch suppression into the executable, avoiding a
# required ASAN_OPTIONS environment variable while keeping other ASan checks.
$(ASAN_DEFAULT_OPTIONS_OBJ): | $(BUILD_DIR)
	printf '%s\n' 'const char *__asan_default_options(void) { return "alloc_dealloc_mismatch=0"; }' | \
		$(CC) $(CFLAGS) -x c -c -o $@ -

# Build on the Ubuntu 20.04 ARM64 target (or with a matching aarch64 sysroot).
# CPython, NumPy and OpenCV are runtime dependencies; python3-config supplies
# the target interpreter's headers and embedding linker flags.
detector: $(DETECTOR_SO)

$(DETECTOR_SO): detector/c_api/detector_api.c detector/c_api/detector_api.h | $(BUILD_DIR)
	$(CC) -std=gnu11 -O2 -Wall -Wextra -fPIC -shared \
		-I detector/c_api $(PYTHON_CFLAGS) $< -o $@ $(PYTHON_LDFLAGS) -pthread

# Compile each source file and emit header dependency files for incremental builds.
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

# Remove generated objects, dependency files, and executable.
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

-include $(DEPFILES)
