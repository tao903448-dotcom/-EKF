# TODO —— 全方位锐评 + 长线程迭代账本

> -EKF 项目的**缺陷/改进单一事实来源**。来自 8-agent 全项目锐评（83 条）+ 人工
> 复核去重。每项标注 `[严重度|工作量]`、所属文件、是否需算力服务器。
>
> 严重度 crit>high>medi>low ｜ 工作量 S<半天 / M<两天 / L>两天 ｜ 🖥️=需服务器
> 状态：`[ ]` 待办 `[~]` 进行中 `[x]` 完成（完成项移至文末并记 CHANGELOG）。

---

## P0 · 可信度 / 正确性（评委最敏感）

- [ ] `[crit|M]` **GUI 伪造遥测充当真实数据**（`examples/imgui_demo/main.cpp`）
  硬编码 `COV:0.124`/`50.2Hz`/`GNSS Precision Fix`/经纬度/新息均值方差/分析页
  fallback RMSE 与"硬件利用率 78%"等，均为假值却以真实示之 → 改为显示后端真实
  `P` 对角、NIS、增益、零偏；无法真实化者明确标注"示意"。**评委可信度风险。**
- [ ] `[crit|M]` **GUI 无视后端真实协方差/新息**（同上）后端已有真实量却不用 → 接真值。
- [ ] `[high|S]` **GUI 均匀噪声误标为高斯**（同上）查表均匀分布注释成 Gaussian → 改真高斯或正名。
- [ ] `[high|M]` **GUI 分析页硬编码 fallback RMSE / 假硬件利用率**（同上）→ 全部接真值或删。
- [ ] `[low|S]` **GUI 死控件**（同上）无功能按钮误导用户 → 接线或移除。
- [x] `[high|S]` **matrix_add/sub/scale 对 MatrixView(stride≠cols) 静默算错**（`src/matrix.c`）
  逐元素用扁平 `data[i]`，视图非连续时读错；其余 mul/transpose/copy 正确。→ 改 MATRIX_INDEX 行列索引。**真实潜伏 bug（锐评新发现）。**
- [x] `[high|S]` **全库无 NaN/Inf 守卫**（`src/matrix.c`,`src/ekf.c`）`NaN<=0` 为 false 致
  Cholesky/inverse 把 NaN 当成功返回，单次野值毒化整个状态 → inverse/cholesky 入口
  有限性扫描 + ekf_update 对 z、ekf 对 P/x 守卫。
- [ ] 🖥️ `[high|L]` **缺真实数据验证**（`examples/`,`tools/`）仅自仿真有"自证"风险 →
  EuRoC/TUM-VI 公开 IMU 数据集回放评测。
- [ ] `[low|S]` **LICENSE 用自定义措辞**（`LICENSE`）→ 可加 SPDX 标识便于识别（已建文件）。

## P1 · 算法深化

- [x] `[high|M]` **Cholesky-solve 取代 Gauss-Jordan 求逆**（`src/ekf.c`,`src/matrix.c`）
  S=HPH'+R 对称正定，每步全 GJ 求逆浪费~2×且稳定性差；库已有 cholesky 却没用 →
  加 `matrix_cholesky_solve`/`matrix_solve`，ekf_gain/ekf_nis 改 Cholesky 解。
- [ ] `[high|L]` **平方根/UD 协方差分解滤波**（`src/`）数值更稳，长跑不失正定。
- [ ] `[high|L]` **自适应仅调 R，未在线估计 Q**（`src/ekf.c`）过程失配应调 Q → 加基于 NIS 的 Q 自适应。
- [ ] `[high|M]` **核心缺量测门控/卡方野值拒绝**（`src/ekf.c`）→ 在核心层加 NIS 卡方门限。
- [ ] `[high|L]` **加性 7D 四元数协方差过参数化（vs MEKF）**（`src/attitude.c`）秩/一致性
  不严谨 → 实现 6 维误差状态乘性 EKF(MEKF) 作对照基线。
- [ ] `[high|M]` **数值雅可比穿过函数内归一化，扭曲线性化**（`src/attitude.c`）→ 归一化移出
  或提供解析雅可比并交叉校验。
- [ ] `[medi|M]` **API 无法逐量测设置 R**（`src/ekf.c`）R 锁定在 config（门控已临时绕过）→ 提供逐步 R 接口。
- [ ] `[medi|M]` **磁力计模型过简**（`src/attitude.c`）`[1,0,0]` 无倾角/磁偏/硬铁软铁 → 完善 + 标定。
- [ ] `[medi|L]` **缺加速度/陀螺/磁力计标定阶段**（`src/attitude.c`）→ 加标定接口。
- [ ] `[medi|M]` **缺静止检测与鲁棒初始化**（`src/attitude.c`）→ ZUPT/零速检测 + 自适应初始姿态。
- [ ] `[medi|M]` **无 SPD 投影助手**（`src/matrix.c`）轻微漂移即被 Cholesky 拒 → 加最近 SPD 投影。
- [ ] `[medi|S]` **matrix_determinant float 直乘 n≥10 溢出**（`src/matrix.c`）→ 对数域累加或行缩放。
- [ ] `[low|L]` **缺 RTS 平滑器**（`src/`）框架自称却无 → 离线平滑。
- [ ] `[low|M]` **缺状态+参数联合(增广)估计**（`src/ekf.c`）。
- [ ] `[low|S]` **四元数无符号连续性处理**（`src/quaternion.c`,attitude）日志/误差输入跳变。

## P1 · 工程 / CI / 测试

- [ ] `[crit|M]` 🖥️ **无覆盖率插桩**（CI）gcov/lcov 行/分支覆盖。
- [ ] `[crit|S]` **Cholesky/LU 失败路径无测试**（`tests/`）非正定/奇异输入未验证。
- [x] `[high|S]` **CI 未开 -Werror**（`.github/workflows/ci.yml`,`Makefile`）告警可累积。
- [ ] `[high|M]` 🖥️ **ARM CI 只交叉编译不运行**（CI）→ QEMU 运行。
- [ ] `[high|M]` 🖥️ **CI 无 valgrind**（CI）→ memcheck 全测试+demo。
- [x] `[high|M]` **EKF 对独立双精度参考滤波验证**（`tests/`）已实现，max|Δ|=6.5e-6。
- [ ] `[high|M]` **benchmark 无判定准则且用 clock()**（`tests/`）→ 加阈值/单调时钟。
- [ ] `[high|M]` **GUI 完全无测试**（`tests/`）已知映射 bug 未回归锁定 → 抽可测逻辑。
- [ ] `[high|M]` **矩阵运算无 NaN/Inf 边界测试**（`tests/`）。
- [x] `[medi|M]` **CI 无静态分析**（CI）cppcheck/clang-tidy。
- [x] `[medi|S]` **CI 仅 GCC 无 clang**（CI）。
- [ ] `[medi|L]` **GUI demo 从不在 CI 构建**（CI）。
- [ ] `[medi|S]` **Makefile 无 install/uninstall**。
- [ ] `[medi|M]` **无 pkg-config/.pc 或 CMake 包配置**。
- [ ] `[medi|M]` **TEST_ASSERT 失败后继续、各文件不一致**（`tests/`）→ 统一断言/早退。
- [ ] `[medi|L]` **矩阵运算无属性/模糊测试**（`tests/`）。
- [ ] `[medi|S]` **matrix_sub/scale 无正确性测试**（`tests/`）。
- [ ] `[medi|S]` **EKF 维度不匹配/未初始化错误路径无测试**（`tests/`）。
- [ ] `[medi|S]` **四元数测试仅happy-path**（`tests/`）缺万向锁/零向量/近对极。
- [ ] `[high|M]` **无 Doxygen API 文档**（`docs/`）→ 生成 API 站点。
- [ ] `[medi|M]` **文档内容重叠**（`docs/`）多文件重复 → 精简。
- [ ] `[medi|M]` **无 CONTRIBUTING.md**。
- [ ] `[low|*]` 版本 tag/发布、plot 脚本 CI 烟囱测试+依赖钉、NIS 阈值魔数说明、init 边界测试。

## P2 · 嵌入式 / 长期

- [ ] `[high|M]` **全程单精度 float 硬编码**（`src/`）→ `typedef real_t` 编译期切 float/double。
- [ ] `[high|M]` **EKF_State ~18KB 超 MCU SRAM**（`include/ekf.h`）→ 维度特化/精简暂存。
- [ ] `[high|M]` **matrix_inverse 2KB 栈帧**（`src/matrix.c`）热路径最大单帧 → Cholesky-solve 顺带缓解。
- [ ] `[medi|S]` **alias-safe 仅靠指针相等**（`src/matrix.c`）视图/部分重叠仍会静默错 → 文档+检测。
- [ ] `[medi|S]` **无 -ffast-math/FP 模型说明**（NEON FMA 与标量结果不同、NaN 行为未定义）。
- [ ] `[medi|S]` **无线程安全/可重入声明**（共享 EKF_State 暂存 → 非可重入）→ 文档化。
- [ ] `[medi|L]` **无 WCET/周期分析**；`[medi|L]` **无 MISRA-C 合规/静态零告警门**。
- [ ] `[medi|M]` **MATRIX_MAX 16×16 硬上限不可编译期配置**（`include/matrix.h`）→ `#ifndef` 包裹 + 文档化栈占用。
- [ ] `[medi|L]` **错误码单一 bool**（`src/matrix.c`）→ MatrixStatus 枚举或 errno 式。
- [x] `[medi|M]` **(部分) Cholesky 线性解**：已加 `matrix_cholesky_solve`；通用 LU `matrix_solve` 待补。
- [ ] `[medi|L]` **MatrixView 已实现却完全未用 + const 违反**（`src/matrix.c`）→ 用起来或删，修 const。
- [ ] `[medi|S]` **MATRIX_ALIGNMENT 定义但从未应用**（NEON 可能非对齐加载）→ 对齐 data 或文档化。
- [ ] `[low|S]` **无 AXPY 原语**；`[low|S]` **matrix_get 越界静默返 0** 伪装有效数据；
  `[low|S]` **控制库重度依赖 stdio**（`printf` 拉入）→ 调试输出可裁剪。
- [ ] `[low|L]` **跨平台 GUI**（Windows-only 限制复现）→ GLFW/OpenGL 或 Web/WASM。

---

## 🖥️ 服务器待执行（隧道恢复后批量；当前 banner 超时不可达）

`ssh -p 10085 luxliang@127.0.0.1`（上游未响应，需服务器侧拉起）：
1. `make && make test && make asan`（真实 make 环境复核）
2. `valgrind --leak-check=full --error-exitcode=1` 跑全部测试+两 demo
3. `gcov`/`lcov` 覆盖率报告（含分支）
4. `cppcheck --enable=all` + `clang-tidy` 全量清零
5. ARM：`make arm_all` + QEMU 运行测试
6. 大规模蒙特卡洛（多核 1000+ 种子）标定 Q/R/门控/Student-t ν
7. 真实数据集（EuRoC/TUM-VI）下载与回放评测

---

## 已完成

- [x] 修复协方差坍缩 / 求逆越界 / Student-t 方向 / 预测混叠 / 雅可比线性化点 /
  NEON 构建失败 / demo 枚举错位（CHANGELOG 2.0.0）
- [x] 仓库规范化（去 zip、标准布局、.gitignore、LICENSE）
- [x] 四旋翼姿态估计 EKF + 严格蒙特卡洛评测 + 报告（3.0.0）
- [x] 加速度自适应门控：MANEUVER 自适应 15.5°→12.5°
- [x] 精美化 README + GitHub Actions CI（x86/ASan/ARM 三作业全绿）+ CHANGELOG
- [x] 绘图工具 `tools/plot_attitude.py`
- [x] 三轮对抗式多 agent 复核（核心修复 + 姿态 0 问题 + 全项目 83 条锐评入账）
