.PHONY: all build install clean

ROM := ../../roms/nestest.nes

BIN_DIR = bin
BUILD_DIR = build

BIN_NAME = shapones
BIN = $(BIN_DIR)/$(BIN_NAME)

CORE_DIR=../../core
CORE_BUILD_DIR = $(BUILD_DIR)/core
CORE_HPP_LIST = $(wildcard $(CORE_DIR)/include/shapones/*.hpp)
CORE_CPP_LIST = $(wildcard $(CORE_DIR)/src/*.cpp)
CORE_OBJ_LIST = $(patsubst $(CORE_DIR)/src/%.cpp,$(CORE_BUILD_DIR)/%.o,$(CORE_CPP_LIST))

APP_SRC_DIR = .
APP_BUILD_DIR = $(BUILD_DIR)/wxapp
APP_HPP_LIST = $(wildcard $(APP_SRC_DIR)/*.hpp)
APP_CPP_LIST = $(wildcard $(APP_SRC_DIR)/*.cpp)
APP_OBJ_LIST = $(patsubst $(APP_SRC_DIR)/%.cpp,$(APP_BUILD_DIR)/%.o,$(APP_CPP_LIST))

CXX_FLAGS = \
	-O2 \
	-I$(CORE_DIR)/include \
	-I$(APP_SRC_DIR) \
	-DSHAPONES_SUPPORT_INES_FILE=1 \
	-DSHAPONES_ENABLE_LOG=1 \
	`wx-config --cflags`

CXX_LDFLAGS = \
	`wx-config --libs` \
	-lpulse \
	-lpulse-simple

all: build

build: $(BIN)

$(BIN): $(CORE_OBJ_LIST) $(APP_OBJ_LIST)
	@mkdir -p $(BIN_DIR)
	g++ \
		$(CORE_OBJ_LIST) \
		$(APP_OBJ_LIST) \
		-o $(BIN) \
		$(CXX_LDFLAGS)

$(CORE_BUILD_DIR)/%.o: $(CORE_DIR)/src/%.cpp $(CORE_HPP_LIST)
	@mkdir -p $(CORE_BUILD_DIR)
	g++ $(CXX_FLAGS) -c $< -o $@

$(APP_BUILD_DIR)/%.o: $(APP_SRC_DIR)/%.cpp $(APP_HPP_LIST)
	@mkdir -p $(APP_BUILD_DIR)
	g++ $(CXX_FLAGS) -c $< -o $@

run: $(BIN)
	./$(BIN) $(ROM)

clean:
	rm -rf $(BIN_DIR)
