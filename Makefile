# Bitbanged SWD <-> CMSIS-DAP TCP proxy.
#
#   make                 build ./cmsis_dap_tcp for the host
#   make -j$(nproc)      ... in parallel
#   make static          fully static binary (needs a static libstdc++/libc)
#   make clean           remove the binary and object files
#
# The target board is a Raspberry Pi 1-4 / CM1, CM3, CM4; building on the Pi
# itself needs nothing but g++ and make. To cross-build a static arm64 binary
# from an x86 host, use ./build_static_arm64.sh, which runs this Makefile
# inside an Alpine (musl) container.
#
# Common overrides:
#   make CXX=aarch64-linux-gnu-g++      cross-compile
#   make BUILD_DIR=obj-arm64 OUT=foo    separate object dir / binary name

CXX      ?= g++
OUT      ?= cmsis_dap_tcp
BUILD_DIR ?= build

CXXFLAGS ?= -std=gnu++17 -O2 -Wall -Wextra
LDFLAGS  ?=
LDLIBS   ?=

SRCS := DAP.cpp calibrate.cpp dp_connect.cpp gpio.cpp logging.cpp main.cpp \
        node_query.cpp rp2040.cpp rtt.cpp swdmux.cpp target_mem.cpp \
        tcp_server.cpp
OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

.PHONY: all static clean test sim_probe

all: $(OUT)

# -s strips symbols at link time: the static musl libc/libstdc++ archives carry
# embedded debug info, which roughly doubles the output size if left in.
static: LDFLAGS += -static -s
static: $(OUT)

$(OUT): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

# Tests run on the build host, not on a Pi: they stub out the GPIO and stand a
# simulated DP/MEM-AP in for the wire. See tests/README.md.
TEST_SRCS := rtt.cpp rp2040.cpp target_mem.cpp dp_connect.cpp logging.cpp
TEST_FLAGS := -std=gnu++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -I.

test: $(BUILD_DIR)/rtt_test $(BUILD_DIR)/shadow_test $(BUILD_DIR)/rp2040_test
	$(BUILD_DIR)/rtt_test
	$(BUILD_DIR)/shadow_test
	$(BUILD_DIR)/rp2040_test

$(BUILD_DIR)/rtt_test: tests/rtt_test.cpp $(TEST_SRCS) | $(BUILD_DIR)
	$(CXX) $(TEST_FLAGS) -o $@ $^

$(BUILD_DIR)/shadow_test: tests/shadow_test.cpp DAP.cpp calibrate.cpp node_query.cpp $(TEST_SRCS) | $(BUILD_DIR)
	$(CXX) $(TEST_FLAGS) -o $@ tests/shadow_test.cpp calibrate.cpp node_query.cpp $(TEST_SRCS)

sim_probe: $(BUILD_DIR)/sim_probe

$(BUILD_DIR)/rp2040_test: tests/rp2040_test.cpp rp2040.cpp target_mem.cpp logging.cpp | $(BUILD_DIR)
	$(CXX) $(TEST_FLAGS) -o $@ $^

$(BUILD_DIR)/sim_probe: tests/sim_probe.cpp node_query.cpp $(TEST_SRCS) tcp_server.cpp | $(BUILD_DIR)
	$(CXX) -std=gnu++17 -O1 -g -Wall -Wextra -I. -o $@ $^

clean:
	rm -rf $(BUILD_DIR) $(OUT)

-include $(DEPS)
