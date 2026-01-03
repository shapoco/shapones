.PHONY: all build flash clean

GRPDIR := EMU
TARGET := SHAPONES
DEVICE := picopad20

REPO_DIR := $(shell cd ../.. && pwd)
LIB_DIR := $(REPO_DIR)/lib
WORK_DIR := $(shell pwd)
WORK_TARGET_DIR := $(WORK_DIR)/$(GRPDIR)/$(TARGET)
BUILD_DIR := $(PICOLIBSDK_PATH)/PicoPad/$(GRPDIR)/$(TARGET)

PICO_DRIVE := e
BIN_DIR := $(REPO_DIR)/bin/$(DEVICE)
BIN := $(BIN_DIR)/$(TARGET).uf2

DEPENDENCIES := \
	Makefile \
	./build.sh \
	$(wildcard $(LIB_DIR)/include/fixbrot/*.hpp) \
	$(wildcard $(WORK_TARGET_DIR)/*.h) \
	$(wildcard $(WORK_TARGET_DIR)/src/*.h) \
	$(wildcard $(WORK_TARGET_DIR)/src/*.cpp)

all: build

build: $(BIN)

$(BIN): $(DEPENDENCIES)
	@mkdir -p "$(BIN_DIR)"
	./build.sh -g "$(GRPDIR)" -t "$(TARGET)" -d "$(DEVICE)" -o "$(BIN_DIR)"

flash: $(BIN)
	sudo mkdir -p /mnt/$(PICO_DRIVE)
	sudo mount -t drvfs $(PICO_DRIVE): /mnt/$(PICO_DRIVE)
	cp "$(BIN)" /mnt/$(PICO_DRIVE)/.

clean:
	rm -rf "$(BIN_DIR)"
	cd $(BUILD_DIR) && ./d.sh || true
