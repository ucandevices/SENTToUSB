# Build + run unit tests with gcov coverage instrumentation.
# Run from this directory:
#   make            -- build the test binary
#   make run        -- build and run tests
#   make coverage   -- build, run, and emit gcov line+branch summary
#   make clean      -- remove build artifacts

CC ?= gcc

TESTS_DIR := $(CURDIR)
IMPL_DIR  := $(abspath $(TESTS_DIR)/..)
ROOT_DIR  := $(abspath $(IMPL_DIR)/..)

BUILD     := $(TESTS_DIR)/build
STAGE_INC := $(BUILD)/include
STAGE_SENT:= $(STAGE_INC)/sent

# Stage public headers under .../include/sent/ so #include "sent/foo.h" resolves.
LIB_HEADERS := $(wildcard $(ROOT_DIR)/*.h)

LIB_SRCS := \
    $(IMPL_DIR)/mode_manager.c \
    $(IMPL_DIR)/sent_crc.c \
    $(IMPL_DIR)/sent_decoder.c \
    $(IMPL_DIR)/sent_encoder.c \
    $(IMPL_DIR)/sent_protocol.c \
    $(IMPL_DIR)/sent_slow_channel.c

TEST_SRCS := \
    test_main.c \
    test_sent_crc.c \
    test_sent_protocol.c \
    test_sent_encoder.c \
    test_sent_slow_channel.c \
    test_sent_decoder.c \
    test_sent_decoder_equiv.c \
    ref_decoder_v11.c \
    test_mode_manager.c

LIB_OBJS  := $(patsubst $(IMPL_DIR)/%.c,$(BUILD)/lib/%.o,$(LIB_SRCS))
TEST_OBJS := $(patsubst %.c,$(BUILD)/tests/%.o,$(TEST_SRCS))

# -DSENT_HAL_HOST keeps sent_assert.h on the libc assert() path.
INCLUDES := -I$(STAGE_INC)
CFLAGS   := -std=c99 -Wall -Wextra -Wpedantic -O0 -g \
            --coverage -fprofile-arcs -ftest-coverage \
            -DSENT_HAL_HOST $(INCLUDES)
LDFLAGS  := --coverage

BIN := $(BUILD)/run_tests

.PHONY: all stage run coverage clean

all: $(BIN)

stage: $(STAGE_SENT)/.stamp

$(STAGE_SENT)/.stamp: $(LIB_HEADERS)
	@mkdir -p $(STAGE_SENT)
	@cp $(LIB_HEADERS) $(STAGE_SENT)/
	@touch $@

$(BUILD)/lib/%.o: $(IMPL_DIR)/%.c | stage
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/tests/%.o: %.c | stage
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(LIB_OBJS) $(TEST_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

run: $(BIN)
	$(BIN)

coverage: run
	@echo "--- gcov line + branch summary ---"
	@gcov -b -o $(BUILD)/lib $(LIB_SRCS) | \
	  awk '/^File/{f=$$0} /^Lines executed/{print f, $$0} /^Branches executed/{print "  ", $$0} /^Taken at least once/{print "  ", $$0}'
	@mv *.gcov $(BUILD)/ 2>/dev/null || true
	@echo "--- gcovr summary ---"
	@gcovr $(BUILD)/lib --root $(ROOT_DIR) \
	  --exclude '.*hal_host\.c' \
	  --exclude '.*hal_stm32\.c' \
	  --exclude-lines-by-pattern '.*GCOV_EXCL_LINE.*' \
	  --exclude-branches-by-pattern '.*GCOV_EXCL_BR_LINE.*' \
	  --exclude-branches-by-pattern '.*SENT_ASSERT.*' \
	  --exclude-branches-by-pattern '.*manager != NULL.*' \
	  --gcov-ignore-errors=no_working_dir_found \
	  --print-summary

clean:
	rm -rf $(BUILD) *.gcov *.gcda *.gcno
