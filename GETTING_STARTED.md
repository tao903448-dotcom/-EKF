# 快速开始指南

## 前置条件

### 1. 安装GCC编译器

**Windows用户：**
1. 下载 MinGW-w64：https://www.mingw-w64.org/
2. 安装时选择：
   - Architecture: x86_64
   - Threads: posix
   - Exception: seh
3. 将 `C:\mingw64\bin` 添加到系统PATH环境变量

**验证安装：**
```bash
gcc --version
```

### 2. 安装Git（可选）
- 下载：https://git-scm.com/
- 用于版本控制

## 构建项目

### Linux / macOS / Windows(MinGW) 通用

```bash
make            # 构建静态库 + 单元测试 + 命令行 demo
make test       # 运行全部单元测试
make run-demo   # 运行诚实双场景对比
```

不想用 make 时，可直接 gcc 编译（单文件）：
```bash
gcc -I include examples/ekf_demo.c src/matrix.c src/ekf.c -lm -o ekf_demo
./ekf_demo
```

## 运行测试

### 矩阵运算测试
```bash
# Windows
build\test_matrix.exe

# Linux/Mac
./build/test_matrix
```

**预期输出：**
```
========== 矩阵运算库测试 ==========

PASS: 矩阵初始化测试
PASS: 单位矩阵测试
PASS: 矩阵加法测试
PASS: 矩阵乘法测试
PASS: 矩阵转置测试
PASS: 矩阵求逆测试
PASS: 矩阵行列式测试
PASS: Cholesky分解测试
PASS: 矩阵乘法别名安全测试
PASS: 矩阵转置别名安全测试
PASS: 大矩阵求逆（无越界）测试

========== 测试结果 ==========
通过: 11
失败: 0
总计: 11
```

### EKF 命令行演示
```bash
make run-demo            # 或 ./build/ekf_demo
./build/ekf_demo out.csv # 额外导出两个场景的轨迹 CSV
```

**预期输出（节选）：**
```
场景 A：常值模型追踪快速正弦（过程模型失配，测量无野值）
   标准EKF  (Standard)  RMSE =  15.716
   Joseph形式(Joseph)    RMSE =  15.716
   Student-t (鲁棒)      RMSE =  44.991
   自适应EKF (Adaptive)  RMSE =  47.592

场景 B：匀速模型 + 15% 脉冲野值（模型正确，测量含离群点）
   标准EKF  (Standard)  RMSE =   5.978
   Student-t (鲁棒)      RMSE =   0.636   ↓ 89.4%
```

## 项目结构说明

```
software-cup-ekf/
├── include/              # 头文件
│   ├── matrix.h         # 矩阵运算库接口
│   └── ekf.h            # EKF框架接口
├── src/                 # 源代码
│   ├── matrix.c         # 矩阵库（标量 + NEON，alias-safe）
│   └── ekf.c            # EKF框架实现
├── tests/               # 测试代码（单元 + 回归）
│   ├── test_matrix.c
│   ├── test_ekf.c
│   └── test_benchmark_fixed.c
├── examples/
│   ├── ekf_demo.c       # 跨平台命令行演示
│   └── imgui_demo/      # Windows GUI 演示
├── docs/                # 文档（技术文档 + 深度剖析与优化报告）
├── build/               # 编译输出（自动创建）
├── lib/                 # 库文件（自动创建）
├── Makefile             # 构建脚本（Linux/macOS/MinGW + ARM）
└── README.md            # 项目说明
```

## 下一步开发

### 1. 理解代码
- 阅读 `include/matrix.h` 了解矩阵运算接口
- 阅读 `include/ekf.h` 了解EKF框架接口
- 查看 `examples/ekf_demo.c` 学习使用方法

### 2. 修改和扩展
- 调整矩阵维度：修改 `matrix.h` 中的 `MATRIX_MAX_ROWS` 和 `MATRIX_MAX_COLS`
- 添加新的状态模型：在 `ekf_demo.c` 中修改状态转移函数
- 实现自定义观测模型：修改观测函数

### 3. 性能优化
- 启用ARM NEON优化（需要ARM平台）
- 调整编译器优化选项
- 减少临时矩阵使用

## 常见问题

### Q: 编译时出现"undefined reference to `matrix_init`"
**A:** 确保先编译库文件，再编译测试程序：
```bash
make lib
make tests
```

### Q: 运行测试时出现"FAIL"
**A:** 检查：
1. 编译器是否正确安装
2. 是否使用了正确的编译选项
3. 代码是否有修改

### Q: 如何添加新的矩阵运算？
**A:** 
1. 在 `include/matrix.h` 中添加函数声明
2. 在 `src/matrix.c` 中实现函数
3. 在 `tests/test_matrix.c` 中添加测试用例

### Q: 如何调整EKF参数？
**A:** 在 `ekf_demo.c` 中修改：
- 过程噪声 `Q`
- 观测噪声 `R`
- 初始状态 `x0`
- 初始协方差 `P0`

## 调试技巧

### 1. 启用调试信息
```bash
make debug
```

### 2. 使用printf调试
在关键位置添加打印语句：
```c
matrix_print(&mat, "Matrix Name");
```

### 3. 检查矩阵维度
确保矩阵运算维度匹配：
- 加法/减法：维度必须相同
- 乘法：A的列数 = B的行数
- 求逆：必须是方阵

## 性能基准

### 矩阵运算性能（参考）
- 3x3矩阵乘法：< 1μs
- 3x3矩阵求逆：< 5μs
- EKF更新（2维状态）：< 10μs

### 内存使用
- 矩阵结构体：~1KB
- EKF状态：~10KB
- 总内存：~50KB

## 获取帮助

### 文档
- `README.md` - 项目概述
- `docs/技术文档.md` - 技术说明
- `docs/深度剖析与优化报告.md` - 缺陷剖析与重构记录
- `GETTING_STARTED.md` - 本文档

### 代码注释
所有源文件都有详细的Doxygen风格注释

### 调试输出
使用 `matrix_print()` 函数打印矩阵内容

## 提交代码

### 1. 添加文件
```bash
git add .
```

### 2. 提交更改
```bash
git commit -m "描述你的更改"
```

### 3. 推送（如果配置了远程仓库）
```bash
git push origin main
```

## 联系方式

如有问题，请联系开发团队。

---

**最后更新：** 2026年6月11日