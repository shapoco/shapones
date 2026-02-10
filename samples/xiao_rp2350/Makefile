.PHONY: all install clean

APP_NAME := shapones_xiao_rp2350

BOARD := pico2

PICO_DRIVE := e

REPO_DIR := $(shell cd ../.. && pwd)
SRC_DIR := .
BUILD_DIR := build/$(BOARD)
BIN_DIR := $(REPO_DIR)/bin/$(BOARD)

BIN_NAME := $(APP_NAME).uf2
BIN := $(BIN_DIR)/$(BIN_NAME)

CORE_DIR := ../../core
FATFS_DIR := ../fatfs/source
SRC_LIST := \
	$(wildcard $(SRC_DIR)/src/*.*) \
	$(wildcard $(SRC_DIR)/include/*.*) \
	$(wildcard $(CORE_DIR)/src/*.*) \
	$(wildcard $(CORE_DIR)/include/shapones/*.*)  \
	$(wildcard $(FATFS_DIR)/*.*) 

all: $(BIN)

$(BIN): $(SRC_LIST) CMakeLists.txt Makefile
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) \
		&& cmake -DPICO_BOARD=$(BOARD) ../.. \
		&& make -j
	@mkdir -p $(BIN_DIR)
	cp $(BUILD_DIR)/$(BIN_NAME) $(BIN_DIR)/.
	@echo "------------------------------"
	@echo "UF2 File:"
	@echo $(BIN)
	@ls -l $(BIN)

flash: $(BIN)
	sudo mkdir -p /mnt/$(PICO_DRIVE)
	sudo mount -t drvfs $(PICO_DRIVE): /mnt/$(PICO_DRIVE)
	cp $(BIN) /mnt/$(PICO_DRIVE)/.

clean:
	rm -rf $(BUILD_DIR)

