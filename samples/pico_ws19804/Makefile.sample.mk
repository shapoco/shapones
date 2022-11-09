.PHONY: all install clean

REPO_DIR=$(shell pwd)
SRC_DIR=.
BUILD_DIR=build

BIN_NAME=pico_nes_ws19804.uf2
BIN=$(BUILD_DIR)/$(BIN_NAME)

CORE_DIR=../../core
FATFS_DIR=../fatfs/source

SRC_LIST=\
	$(wildcard $(SRC_DIR)/*.*) \
	$(wildcard $(SRC_DIR)/../font/mono8x16/*.*) \
	$(wildcard $(CORE_DIR)/src/*.*) \
	$(wildcard $(CORE_DIR)/include/shapones/*.*)  \
	$(wildcard $(FATFS_DIR)/*.*) 

all: $(BIN)

$(BIN): $(SRC_LIST) CMakeLists.txt
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) \
		&& cmake .. \
		&& make -j
	@echo "------------------------------"
	@echo "UF2 File:"
	@echo $(REPO_DIR)/$(BIN)
	@ls -l $(REPO_DIR)/$(BIN)

install: $(BIN)
	sudo mkdir -p /mnt/e
	sudo mount -t drvfs e: /mnt/e
	cp $(BIN) /mnt/e/.

clean:
	rm -rf build
