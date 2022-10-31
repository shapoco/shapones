.PHONY: all install clean

WXWIDGETS_PATH := $(shell echo $$HOME)/local/wxWidgets-3.2.1

export PATH = $(shell echo $$PATH):$(WXWIDGETS_PATH)/bin
export LIBRARY_PATH = $(shell echo $$LIBRARY_PATH):$(WXWIDGETS_PATH)/lib
export LD_LIBRARY_PATH = $(shell echo $$LD_LIBRARY_PATH):$(WXWIDGETS_PATH)/lib

SRC_DIR = .
BIN_DIR = build
ROM := ../../roms/nestest.nes

BIN_NAME = shapones
BIN = $(BIN_DIR)/$(BIN_NAME)

SRC_LIST = $(wildcard $(SRC_DIR)/*.*)

all: $(BIN)

$(BIN): $(SRC_LIST)
	mkdir -p $(BIN_DIR)
	g++ \
		../../core/src/*.cpp \
		*.cpp \
		-O4 \
		-I../../core/include \
		-I. \
		`wx-config --cflags` \
		`wx-config --libs` \
		-o $(BIN)

run: $(BIN)
	./$(BIN) $(ROM)

clean:
	rm -rf $(BIN_DIR)
