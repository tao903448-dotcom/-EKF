# TODO —— 长线程迭代与优化清单

> 本文件是 -EKF 项目的**全方位锐评 + 长期迭代账本**。逐项标注优先级、所属模块、
> 工作量(S/M/L)、是否依赖算力服务器。完成的条目打勾并在 CHANGELOG 记录。
>
> 维护约定：新缺陷追加到对应优先级；已完成移至文末「已完成」并保留结论。
> 自动化锐评(多 agent)结论会持续并入本表。

图例：`[ ]` 待办 `[~]` 进行中 `[x]` 完成 ｜ 工作量 S<半天 / M<两天 / L>两天
｜ 🖥️ = 需连算力服务器(`ssh -p 10085 luxliang@127.0.0.1`)

---

## P0 · 可信度与正确性边界（评委最敏感，优先）

- [ ] **GUI 假数据治理** (M, `examples/imgui_demo/main.cpp`)
  遥测页硬编码 `COV: 0.124`、`50.2 Hz`、`GNSS Precision Fix`、经纬度、新息均值/方差
  等**伪造为真实**的数值，演示给评委看的并非滤波器真实内部量 → 改为显示真实
  `P` 对角、NIS、卡尔曼增益、零偏估计；无法真实化的明确标注"示意/占位"。
- [ ] **GUI 跨平台/可验证** (L, GUI)
  仅 Windows + DX11，CI 与 Linux/Mac 无法构建，无法自动测试 → 评估抽象渲染后端
  或补一个跨平台版本（GLFW+OpenGL 或 Web/WASM），纳入 CI 至少做编译检查。
- [ ] 🖥️ **真实数据验证** (L, `examples/`+`tools/`)
  目前仅自仿真，存在"自证"风险 → 接入公开 IMU 数据集（EuRoC MAV / TUM-VI）
  回放，给出真实数据上的姿态 RMSE，显著增强说服力。
- [x] **LICENSE 文件** (S, 根目录)
  已补 `LICENSE`（学习与竞赛用途声明，与 README 意图一致）。

## P1 · 算法深化 + 工程完善

### 算法
- [ ] **MEKF 乘性误差状态 EKF 对照** (L, `src/`)
  现为加性四元数+重归一化；补 6 维误差状态乘性 EKF 作对照，协方差更严谨，
  作为"理论完备性"加分项与基线对比。
- [ ] **Cholesky-solve 取代逐步求逆** (M, `src/ekf.c`+`src/matrix.c`)
  每步对 S 做 Gauss-Jordan 求逆；S 对称正定 → 改 Cholesky 分解 + 前后代回代，
  更快更稳，并加迭代精化。
- [ ] **可选解析雅可比(姿态)** (M, `src/attitude.c`)
  数值雅可比每步 2n 次状态函数求值，嵌入式偏贵 → 提供解析 F/H 选项并与数值
  版交叉校验。
- [ ] **磁力计模型完善** (M, `src/attitude.c`)
  现用 `[1,0,0]` 单位参考，无倾角/磁偏角/硬铁软铁标定 → 加倾角与标定接口。
- [ ] **自适应同时调 Q** (M, `src/ekf.c`)
  自适应当前只放大 R；过程失配本应调 Q → 增加基于 NIS 的 Q 自适应分支。
- [ ] **数值守卫** (S, `src/ekf.c`+`src/matrix.c`)
  协方差对角下界、NaN/Inf 检测与安全回退，防长跑发散。

### 工程 / CI / 测试
- [ ] 🖥️ **覆盖率** (S, CI) gcov/lcov 报告，给出行/分支覆盖并入 CI。
- [ ] 🖥️ **静态分析** (S, CI) cppcheck + clang-tidy 全量扫描，清零告警。
- [ ] **多编译器** (S, CI) 增 clang 构建；逐步开启 `-Werror`。
- [ ] 🖥️ **valgrind/memcheck** (S, CI) 所有测试+demo 跑 valgrind（本地无，需服务器）。
- [ ] 🖥️ **ARM 真跑** (M, CI) 用 QEMU 运行 ARM 测试，而非仅交叉编译。
- [ ] **参考滤波对照测试** (M, `tests/`) 实现独立双精度参考 KF，断言一致。
- [ ] **属性/模糊测试** (M, `tests/`) 矩阵运算随机输入 + 边界(NaN/Inf/奇异)。
- [ ] **GUI 纳入构建检查** (S, CI) 至少在有 imgui 依赖时做编译。
- [ ] **CMake 构建** (M, 根目录) 在 Makefile 之外提供 CMake，便于更广集成。
- [ ] **Doxygen API 文档** (S, `docs/`) 头文件已有注释 → 生成 API 参考站点。

## P2 · 锦上添花 / 长期研究

- [ ] **双精度可选** (M) `typedef real_t`，编译期切换 float/double。
- [ ] 🖥️ **定点实现路径** (L) 面向无 FPU MCU 的定点矩阵/EKF。
- [ ] **MISRA-C 合规** (M) 静态扫描 + 整改，嵌入式认证友好。
- [ ] **编译期维度特化** (M) 减小固定 16×16 的内存/算力开销。
- [ ] **RTS 平滑器 / UD 分解滤波** (L) 离线平滑与更稳的协方差因子化。
- [ ] **GUI 真实可视化** (M) 姿态立方体动画 + 协方差椭球 + 实时 NIS 曲线。
- [ ] 🖥️ **大规模蒙特卡洛调参** (M) 1000+ 种子参数扫描，标定 Q/R/门控/Student-t ν。

---

## 🖥️ 服务器待执行（隧道恢复后批量跑）

> 当前 `ssh -p 10085 luxliang@127.0.0.1` 隧道端口可连但 banner 超时（上游服务器
> 未响应），需在服务器侧拉起隧道/服务后执行：

1. `make && make test && make asan`（真实 make 环境）
2. `valgrind --leak-check=full` 跑全部测试与两个 demo
3. `gcov`/`lcov` 覆盖率报告
4. `cppcheck --enable=all` + `clang-tidy` 全量
5. ARM：`make arm_all` + QEMU 运行
6. 大规模蒙特卡洛参数扫描（多核）
7. 真实数据集下载与回放评测

---

## 已完成

- [x] 修复协方差坍缩(matrix_mul 别名)、矩阵求逆越界、Student-t 方向、预测混叠、
  雅可比线性化点、NEON 构建失败、demo 枚举错位（见 CHANGELOG 2.0.0）
- [x] 仓库规范化（去 zip、标准布局、.gitignore）
- [x] 四旋翼姿态估计 EKF（四元数+零偏）+ 严格蒙特卡洛评测 + 报告（3.0.0）
- [x] 加速度自适应门控（acceleration rejection），MANEUVER 自适应 15.5°→12.5°
- [x] 精美化 README + GitHub Actions CI（x86/ASan/ARM，三作业全绿）+ CHANGELOG
- [x] 绘图工具 `tools/plot_attitude.py`
- [x] 两轮对抗式多 agent 复核（核心 4 确认并修复；姿态 0 问题）
