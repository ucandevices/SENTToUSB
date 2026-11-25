# Host tests of the application layer (sent_bridge, sent_core, slcan) on top
# of the open-sent-c library.  Built twice: 4 channels (STM32C071 build) and
# 1 channel (STM32F042 build).
#   make run     -- build and run both
#   make clean

CC ?= gcc

CORE    := $(abspath $(CURDIR)/../Core)
LIBDIR  := $(CORE)/Inc/sent/implementation
LIBTEST := $(LIBDIR)/Tests
BUILD   := $(CURDIR)/build

LIB_SRCS := $(LIBDIR)/mode_manager.c $(LIBDIR)/sent_crc.c $(LIBDIR)/sent_decoder.c \
            $(LIBDIR)/sent_encoder.c $(LIBDIR)/sent_protocol.c $(LIBDIR)/sent_slow_channel.c
APP_SRCS := $(CORE)/Src/sent_bridge.c $(CORE)/Src/sent_core.c $(CORE)/Src/slcan.c
TEST_SRCS := test_app_main.c test_bridge_multi.c test_core.c sim_signal.c

CFLAGS := -std=c99 -Wall -Wextra -Wpedantic -O1 -g -DSENT_HAL_HOST \
          -I$(CORE)/Inc -I$(LIBTEST) -I$(CURDIR)

SRCS := $(LIB_SRCS) $(APP_SRCS) $(TEST_SRCS)

.PHONY: all run clean

all: $(BUILD)/app_tests_4ch $(BUILD)/app_tests_1ch

$(BUILD)/app_tests_4ch: $(SRCS) $(wildcard $(CORE)/Inc/*.h $(CORE)/Inc/sent/*.h *.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DSENT_NUM_CHANNELS=4 -o $@ $(SRCS)

$(BUILD)/app_tests_1ch: $(SRCS) $(wildcard $(CORE)/Inc/*.h $(CORE)/Inc/sent/*.h *.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DSENT_NUM_CHANNELS=1 -o $@ $(SRCS)

run: all
	$(BUILD)/app_tests_4ch
	$(BUILD)/app_tests_1ch

clean:
	rm -rf $(BUILD)
