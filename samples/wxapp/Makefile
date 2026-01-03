.PHONY: all install clean

SRC_DIR = .
BIN_DIR = build
ROM := ../../roms/nestest.nes

CORE_DIR=../../core

BIN_NAME = shapones
BIN = $(BIN_DIR)/$(BIN_NAME)

SRC_LIST = \
	$(wildcard $(SRC_DIR)/*.*) \
	$(wildcard $(CORE_DIR)/src/*.*) \
	$(wildcard $(CORE_DIR)/include/shapones/*.*) 

all: $(BIN)

$(BIN): $(SRC_LIST)
	mkdir -p $(BIN_DIR)
	g++ \
		$(CORE_DIR)/src/*.cpp \
		*.cpp \
		-O4 \
		-I$(CORE_DIR)/include \
		-I. \
		-DSHAPONES_SUPPORT_INES_FILE=1 \
		-DSHAPONES_ENABLE_LOG=1 \
		`wx-config --cflags` \
		`wx-config --libs` \
		-lpulse -lpulse-simple \
		-o $(BIN)

run: $(BIN)
	./$(BIN) $(ROM)

clean:
	rm -rf $(BIN_DIR)
