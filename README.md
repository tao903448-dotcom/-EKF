# 面向模型失配的自适应EKF系统

## 第十五届中国软件杯 · A8 四旋翼无人机位姿控制系统设计优化

> 重构版（2026-06-20）：修复了协方差坍缩、矩阵求逆越界、Student-t 方向写反、
> ARM/NEON 构建失败、演示算法标签错位等一系列缺陷，并用**诚实的双场景基准**
> 重新评估四种滤波方法。详见 [`docs/深度剖析与优化报告.md`](docs/深度剖析与优化报告.md)。

---

## 项目简介

针对四旋翼无人机位姿估计中的状态估计问题，实现了一套零动态分配、可在嵌入式
实时系统运行的扩展卡尔曼滤波（EKF）框架，并对比四种更新策略：

- **标准 EKF**：基础线性化更新
- **Joseph 形式**：数值稳定、保持协方差对称正定（对最优增益与标准 EKF 数学等价）
- **Student-t 鲁棒更新**：按马氏距离对测量野值降权
- **自适应更新**：按归一化新息平方（NIS）动态调整测量噪声

**旗舰应用——四旋翼姿态估计**：基于该框架实现了 7 状态（四元数 + 陀螺零偏）
姿态 EKF，融合陀螺 / 加速度计 / 磁力计；在振动野值、机动失配场景下，自适应 /
鲁棒更新把姿态 RMSE 相对标准 EKF **降低 75% / 39%** 并恢复滤波一致性
（详见 [`docs/姿态估计实验报告.md`](docs/姿态估计实验报告.md)）。

核心特性：零动态分配矩阵库、**alias-safe** 矩阵运算、可选 ARM NEON 加速
（与标量同处一份 `matrix.c`，无重复符号）、四元数约束状态的归一化钩子、
数值雅可比、GitHub Actions CI（构建 / 测试 / ASan / ARM 交叉编译）。

---

## 项目结构

```
software-cup-ekf/
├── include/
│   ├── matrix.h            # 矩阵运算库接口
│   ├── ekf.h               # EKF 框架接口
│   ├── quaternion.h        # 四元数姿态运算
│   ├── attitude.h          # 四旋翼姿态 EKF 模型(7 状态)
│   └── imu_sim.h           # 可复现 IMU 仿真器(header-only)
├── src/
│   ├── matrix.c            # 矩阵库（标量 + NEON 内核，alias-safe）
│   ├── ekf.c               # EKF 框架
│   ├── quaternion.c        # 四元数运算
│   └── attitude.c          # 姿态 EKF 模型(数值雅可比)
├── tests/
│   ├── test_matrix.c       # 矩阵单元 + 回归测试
│   ├── test_ekf.c          # EKF 单元 + 回归测试
│   ├── test_attitude.c     # 姿态 EKF / 四元数测试
│   └── test_benchmark_fixed.c  # 性能基准
├── examples/
│   ├── ekf_demo.c          # 1D/2D 四方法对比(无 GUI)
│   ├── attitude_demo.c     # 四旋翼姿态估计 蒙特卡洛评测
│   └── imgui_demo/main.cpp # Windows + DX11 + ImGui 图形仪表盘
├── docs/
│   ├── 技术文档.md
│   ├── 深度剖析与优化报告.md   # 缺陷剖析与重构记录
│   └── 姿态估计实验报告.md     # 四旋翼姿态估计实验
├── .github/workflows/ci.yml
├── Makefile
└── README.md
```

---

## 快速开始

```bash
make            # 构建静态库 + 单元测试 + 命令行 demo
make test       # 运行全部单元测试（matrix 11 + ekf 41 + attitude 11）
make run-demo   # 1D/2D 四方法诚实双场景对比
make run-attitude  # 四旋翼姿态估计 四方法×三场景 蒙特卡洛评测
make asan       # AddressSanitizer/UBSan 下跑测试
make bench      # 构建性能基准
make clean
```

单文件手动编译：
```bash
gcc -I include examples/ekf_demo.c src/matrix.c src/ekf.c -lm -o ekf_demo
./ekf_demo out.csv        # 运行并导出轨迹 CSV
```

ARM 交叉编译（NEON 由 `-mfpu=neon` 自动开启）：
```bash
make arm_all    # 需要 arm-linux-gnueabihf 工具链
```

---

## API 示例

```c
#include "matrix.h"
#include "ekf.h"

EKF_Config cfg;
ekf_config_init(&cfg, 2, 1);                 // 2 维状态，1 维观测
ekf_set_functions(&cfg, f, h, F_jac, H_jac); // 状态/观测函数及其雅可比
ekf_set_process_noise(&cfg, &Q);
ekf_set_measurement_noise(&cfg, &R);
ekf_set_update_method(&cfg, EKF_UPDATE_STUDENT_T);  // 鲁棒更新

EKF_State ekf;
ekf_state_init(&ekf, &cfg, &x0, &P0);

for (int k = 0; k < N; k++) {
    ekf_predict(&ekf, &cfg, &u, dt);
    ekf_update(&ekf, &cfg, &z);
    Matrix x; ekf_get_state(&ekf, &x);
}
```

> 矩阵运算 **alias-safe**：`matrix_mul(&P, &A, &P)` 这类 result 与输入同址的
> 写法结果正确（修复前会被清零，是协方差坍缩的根因）。

---

## 性能数据（x86-64，-O2，标量路径，仅供量级参考）

| 运算 | 4×4 | 8×8 |
|---|---|---|
| 加法 / 转置 | ~0.1 ms / 1000 次 | ~0.4 ms / 1000 次 |
| 乘法 | ~0.8 ms / 1000 次 | ~5 ms / 1000 次 |
| 求逆 | ~2 ms / 1000 次 | ~15 ms / 1000 次 |

> 具体数值随平台/编译器波动，请以本机 `make bench` 实测为准。
> ARM NEON 对加/减/数乘/乘法/转置有加速，量级 2–4×（依设备而定）。

---

## 算法对比（诚实评测）

通过 `make run-demo` 可复现：

- **场景 A（过程模型失配，无野值）**：标准 EKF 最优；Student-t / 自适应因压低增益反而更差。
- **场景 B（测量含 15% 脉冲野值，模型正确）**：Student-t / 自适应相对标准 EKF
  **RMSE 降低约 89% / 87%**——这是鲁棒滤波真正的用武之地。

**结论：没有"永远最优"的更新方法，应按噪声特性选型。**
（早期版本曾笼统宣称"自适应降低 65%"，经核查该结论受协方差坍缩 bug 与算法标签
错位影响，已在重构版中纠正。）

---

## 四旋翼姿态估计（旗舰结果，`make run-attitude` 可复现）

7 状态四元数姿态 EKF，融合陀螺/加速度计/磁力计，20 种子蒙特卡洛、200Hz/15s：

| 场景 | 标准EKF | Student-t | 自适应 |
|---|---|---|---|
| CLEAN（温和） | 0.42° | 0.43° | 0.41° |
| OUTLIER（振动野值） | 1.75° (NIS 73✗) | 0.44° **(↓75%)** | 0.42° **(↓76%, NIS 4✓)** |
| MANEUVER（机动失配） | 25.7° | 18.6° | 15.5° **(↓39%)** |

鲁棒/自适应不仅降低姿态 RMSE，更把新息一致性指标 NIS 从 73 拉回≈4——
即在野值/失配下**恢复滤波器可信度**。详见 `docs/姿态估计实验报告.md`。

---

## 测试结果

| 套件 | 结果 |
|---|---|
| `test_matrix` | 11 / 11 ✓（含别名安全、14×14 求逆越界回归） |
| `test_ekf` | 41 / 41 ✓（含协方差不坍缩、Standard≡Joseph、鲁棒抗野值、预测不混叠、枚举顺序守卫） |
| `test_attitude` | 11 / 11 ✓（四元数运算、姿态精度/一致性、鲁棒性回归） |

CI：提供 GitHub Actions 配置 `ci/ci.yml`（x86 构建+测试、ASan/UBSan、ARM(NEON)
交叉编译三作业）。启用：`mkdir -p .github/workflows && cp ci/ci.yml .github/workflows/`
后推送（推送 workflow 文件需 token 具备 `workflow` 权限，或经网页端添加）。

---

## 许可证

本项目仅供学习与比赛使用。

**版本**：2.0.0（重构）  **日期**：2026-06-20
