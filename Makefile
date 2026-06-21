# 面向模型失配的自适应EKF系统 - 构建脚本（重构版）
#
# 关键变化：
#   - 只有一份 matrix.c（NEON 内核以 #if __ARM_NEON 内联其中），
#     不再有 matrix_neon.c 与之产生重复符号 / 缺函数；ARM 构建因此可用。
#   - 新增可移植命令行 demo（examples/ekf_demo.c，无 GUI 依赖）。
#   - make test 自动编译并运行全部单元测试。

# ===== 编译器设置 =====
CC      = gcc
CSTD    = -std=c99
WARN    = -Wall -Wextra
OPT     = -O2
CFLAGS  = $(WARN) $(OPT) $(CSTD)
LDFLAGS = -lm

# ARM 交叉编译（NEON 由 -mfpu=neon 自动开启 __ARM_NEON 宏）
ARM_CC      = arm-linux-gnueabihf-gcc
ARM_CFLAGS  = $(WARN) $(OPT) $(CSTD) -mfpu=neon -mfloat-abi=hard
ARM_AR      = arm-linux-gnueabihf-ar

# ===== 目录 =====
SRC_DIR   = src
INC_DIR   = include
TEST_DIR  = tests
EX_DIR    = examples
BUILD_DIR = build
LIB_DIR   = lib
INCLUDES  = -I$(INC_DIR)

# ===== 源文件 / 目标 =====
SRC_FILES = $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRC_FILES))
LIB_FILE  = $(LIB_DIR)/libekf.a

# 单元测试（test_benchmark* 作为独立 perf 目标，不进入回归集）
TEST_SRC  = $(filter-out $(TEST_DIR)/test_benchmark.c $(TEST_DIR)/test_benchmark_fixed.c, \
              $(wildcard $(TEST_DIR)/*.c))
TEST_BIN  = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/%, $(TEST_SRC))

DEMO_BIN     = $(BUILD_DIR)/ekf_demo
ATT_DEMO_BIN = $(BUILD_DIR)/attitude_demo
BENCH_BIN    = $(BUILD_DIR)/test_benchmark_fixed

.PHONY: all lib tests demo bench test run-demo run-attitude asan clean stats arm_lib arm_test arm_all arm_clean

# ===== 默认目标 =====
all: lib tests demo

$(shell mkdir -p $(BUILD_DIR) $(LIB_DIR))

# 编译源文件
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# 静态库
lib: $(OBJ_FILES)
	ar rcs $(LIB_FILE) $(OBJ_FILES)
	@echo "库文件已创建: $(LIB_FILE)"

# 单元测试
tests: $(TEST_BIN)
$(BUILD_DIR)/%: $(TEST_DIR)/%.c $(LIB_FILE)
	$(CC) $(CFLAGS) $(INCLUDES) $< -L$(LIB_DIR) -lekf $(LDFLAGS) -o $@

# 可移植命令行 demo（1D/2D 算法对比 + 四旋翼姿态估计）
demo: $(DEMO_BIN) $(ATT_DEMO_BIN)
$(DEMO_BIN): $(EX_DIR)/ekf_demo.c $(LIB_FILE)
	$(CC) $(CFLAGS) $(INCLUDES) $< -L$(LIB_DIR) -lekf $(LDFLAGS) -o $@
	@echo "命令行 demo 已创建: $(DEMO_BIN)"
$(ATT_DEMO_BIN): $(EX_DIR)/attitude_demo.c $(LIB_FILE)
	$(CC) $(CFLAGS) $(INCLUDES) $< -L$(LIB_DIR) -lekf $(LDFLAGS) -o $@
	@echo "姿态估计 demo 已创建: $(ATT_DEMO_BIN)"

run-attitude: $(ATT_DEMO_BIN)
	@./$(ATT_DEMO_BIN)

# 性能基准
bench: $(BENCH_BIN)
$(BENCH_BIN): $(TEST_DIR)/test_benchmark_fixed.c $(LIB_FILE)
	$(CC) $(CFLAGS) $(INCLUDES) $< -L$(LIB_DIR) -lekf $(LDFLAGS) -o $@

# 运行测试
test: tests
	@echo "运行单元测试..."
	@fail=0; for t in $(TEST_BIN); do \
		echo "── $$t"; ./$$t || fail=1; echo; \
	done; \
	if [ $$fail -eq 0 ]; then echo "全部测试通过 ✓"; else echo "存在失败 ✗"; exit 1; fi

run-demo: demo
	@./$(DEMO_BIN)

# 内存/未定义行为消毒构建并跑测试（CI 用）
asan: CFLAGS += -g -fsanitize=address,undefined -fno-omit-frame-pointer
asan: LDFLAGS += -fsanitize=address,undefined
asan: clean test

stats:
	@find $(SRC_DIR) $(INC_DIR) $(TEST_DIR) $(EX_DIR) \( -name "*.c" -o -name "*.h" \) | xargs wc -l

clean:
	rm -rf $(BUILD_DIR) $(LIB_DIR)
	@echo "清理完成"

# ===== ARM NEON 版本（与 x86 共用同一份 matrix.c） =====
ARM_BUILD_DIR = build_arm
ARM_LIB_DIR   = lib_arm
ARM_OBJ       = $(patsubst $(SRC_DIR)/%.c, $(ARM_BUILD_DIR)/%.o, $(SRC_FILES))

$(shell mkdir -p $(ARM_BUILD_DIR) $(ARM_LIB_DIR))

$(ARM_BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) -c $< -o $@

arm_lib: $(ARM_OBJ)
	$(ARM_AR) rcs $(ARM_LIB_DIR)/libekf.a $(ARM_OBJ)
	@echo "ARM 库文件已创建: $(ARM_LIB_DIR)/libekf.a"

arm_test: arm_lib
	$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) $(TEST_DIR)/test_matrix.c \
		-L$(ARM_LIB_DIR) -lekf -lm -o $(ARM_BUILD_DIR)/test_matrix_arm
	$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) $(TEST_DIR)/test_ekf.c \
		-L$(ARM_LIB_DIR) -lekf -lm -o $(ARM_BUILD_DIR)/test_ekf_arm
	@echo "ARM 测试程序已创建（matrix + ekf，需在 ARM 设备 / QEMU 上运行）"

arm_all: arm_lib arm_test

arm_clean:
	rm -rf $(ARM_BUILD_DIR) $(ARM_LIB_DIR)
	@echo "ARM 构建清理完成"
