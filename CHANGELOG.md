# 更新日志

本项目遵循语义化版本思想；日期为 ISO 格式。

## [3.1.0] - 2026-06-21 —— 全项目锐评驱动的健壮性/数值/工程迭代

### 锐评
- 8-agent 全项目深度锐评（83 条缺陷/改进），落账于 `TODO.md`（P0/P1/P2 分级
  + 「服务器待执行」批处理清单），作为长线程迭代单一事实来源。

### 修复 / 健壮性
- **步长视图静默算错**：`matrix_add/sub/scale` 改行列索引（连续路径仍走 NEON），
  修复 `MatrixView(stride≠cols)` 读错（锐评新发现的真实潜伏缺陷）。
- **NaN/Inf 守卫**：新增 `matrix_is_finite`；`matrix_inverse/cholesky` 入口拦截、
  `ekf_update` 拦截非有限观测（丢坏量测、保预测），防野值毒化状态。

### 算法 / 数值
- **Cholesky 线性解**：新增 `matrix_cholesky_solve`；`ekf_gain` 解 S Kᵀ=(PH')ᵀ、
  `ekf_nis` 解 S w=y，取代对 S 的显式 Gauss-Jordan 求逆（更快更稳，结果等价），
  非正定时回退通用求逆。
- 加速度自适应门控（acceleration rejection）：MANEUVER 自适应 15.5°→12.5°。

### 工程 / CI / 测试
- CI 3→6 作业：新增 gcc `-Werror` 严格告警、Clang 构建、cppcheck 静态分析。
- 测试 +5：NaN 守卫、步长视图、Cholesky 解、加速度门控（matrix 14 + ekf 41 +
  attitude 13 = **68** 全过）；`-Werror` 零告警，ASan/UBSan 干净。
- 新增 `LICENSE`、`tools/plot_attitude.py`、`docs/姿态估计实验报告.md`。


## [3.0.0] - 2026-06-21 —— 国奖级深度迭代：四旋翼姿态估计

### 新增
- **四元数姿态运算库** `quaternion.{h,c}`（Hamilton, body→world）：乘法、归一化、
  旋转矩阵、欧拉/旋转向量互换、姿态夹角。
- **四旋翼姿态 EKF 模型** `attitude.{h,c}`：7 状态（四元数 + 陀螺零偏），融合
  陀螺/加速度计/磁力计，方向型观测，中心差分**数值雅可比**（免手推）。
- 通用框架新增 **`state_normalize` 钩子**，支持四元数等带约束状态（每步重归一化）。
- **可复现 IMU 仿真器** `imu_sim.h`（header-only）：轨迹 → 真值角速度 → 带噪/野值/
  线加速度失配的传感器合成；CLEAN / OUTLIER / MANEUVER 三场景。
- **姿态评测** `examples/attitude_demo.c`：四方法 × 三场景 × 20 种子蒙特卡洛，
  报告姿态 RMSE / 零偏 RMSE / 平均 NIS / NIS 越界比例，可导出轨迹 CSV。
- **测试** `tests/test_attitude.c`（四元数 + 姿态精度/一致性/鲁棒回归，11/11）。
- **CI** `.github/workflows/ci.yml`：x86 构建+测试、ASan/UBSan、ARM(NEON) 交叉编译。
- `make` 目标 `run-attitude` / `asan`；`docs/姿态估计实验报告.md`；精美化 README。

### 关键结果（`make run-attitude` 复现）
- CLEAN：四法 ≈0.42°，NIS≈4 一致，标准 ≡ Joseph。
- OUTLIER（振动野值）：标准 1.75°/NIS 73 → Student-t 0.44°（↓75%）、自适应 0.42°（↓76%，NIS→4）。
- MANEUVER（机动失配）：标准 25.7° → 自适应 15.5°（↓39%），并诚实说明物理上限。

## [2.0.0] - 2026-06-20 —— 深度剖析 + 大刀阔斧重构 + 仓库规范化

### 修复（致命/重要）
- **协方差坍缩**：`matrix_mul` 别名不安全，`matrix_mul(&P,&A,&P)` 先清零再读取，
  使标准 EKF 一步更新后协方差归零、增益失效。改为 alias-safe（根因修复）。
- **矩阵求逆越界**：增广矩阵 `n×2n` 越界写 `data[256]`，`n≥12` 栈溢出。改用独立缓冲。
- **预测步混叠**：状态函数输入/输出同址；新增 `x_pred` 缓冲分离。
- **雅可比线性化点**：F 应在先验 x（而非 f(x)）处求值。
- **Student-t 方向写反**：野值应膨胀 R 降权，原实现反向。
- **ARM/NEON 构建失败**：`matrix_neon.c` 重复定义又缺函数；合并进唯一 `matrix.c`。
- **演示算法标签错位**：UI 下标 → 枚举映射不一致。

### 变更
- 四个 update 函数去重为共用辅助函数；协方差更新改用预分配暂存，单步栈占用 ~8.6KB → ~2.4KB。
- 协方差每步对称化（SPD 数值保持）；自适应改多维 NIS。
- 新增可移植 CLI demo `examples/ekf_demo.c`（双场景诚实评测 + CSV）。
- 回归测试覆盖每个缺陷（matrix 11/11、ekf 41/41）；ASan/UBSan 零报错。
- **仓库规范化**：去除源码 zip，展开为标准目录布局，新增 `.gitignore`。
- 文档全面诚实化，纠正"自适应降低 65%"等受缺陷影响的失真结论。

## [1.0.0] - 初始提交

原始软件杯作品：矩阵库 + EKF 框架 + ImGui 演示（含上述待修复缺陷）。
