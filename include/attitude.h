/**
 * @file attitude.h
 * @brief 四旋翼姿态估计 EKF 模型（四元数 + 陀螺零偏，IMU 融合）
 *
 * 把"四旋翼无人机位姿估计"建模为通用 EKF 框架的一个具体配置，从而让
 * 标准 / Joseph / Student-t / 自适应四种更新方法都能直接作用于真实问题。
 *
 * 状态 x (7 维)：
 *   x[0..3] = 姿态四元数 q (body→world, Hamilton [w,x,y,z])
 *   x[4..6] = 陀螺零偏 b (rad/s, body)
 *
 * 控制输入 u (3 维)：陀螺测量 ω_m (rad/s, body)
 * 观测 z (6 维)：
 *   z[0..2] = 加速度计给出的"重力方向"单位向量(body)  → 约束 roll/pitch
 *   z[3..5] = 磁力计给出的"参考磁场方向"单位向量(body) → 约束 yaw
 *
 * 设计取舍：
 *  - 采用四元数避免欧拉角万向锁，适合大机动；
 *  - 更新采用"加性 EKF + 每步重归一化"（通过框架的 state_normalize 钩子），
 *    实现简单且能复用框架的全部鲁棒/自适应方法；严格的乘性 EKF(MEKF)
 *    误差状态形式为后续可选增强。
 *  - 雅可比用中心差分数值求取（correct-by-construction，免手推易错），
 *    对 7/6 维小系统开销可忽略。
 *  - 观测使用"方向单位向量"：加速度计在机动(非重力比力)下会偏离重力方向，
 *    天然构成"模型失配"，正是自适应/鲁棒更新的用武之地。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ATT_STATE_DIM 7
#define ATT_MEAS_DIM  6

/** 世界系重力方向(单位向量, z 轴向上) */
extern const float ATT_G_DIR_W[3];
/** 世界系参考磁场方向(单位向量, 指北) */
extern const float ATT_M_DIR_W[3];

/* ---- 供 ekf_set_functions / ekf_set_state_normalize 使用的回调 ---- */
bool attitude_state_func(const Matrix *x, const Matrix *u, Matrix *x_new, float dt);
bool attitude_meas_func(const Matrix *x, Matrix *z);
bool attitude_state_jacobian(const Matrix *x, const Matrix *u, Matrix *F, float dt);
bool attitude_meas_jacobian(const Matrix *x, Matrix *H);
void attitude_normalize(Matrix *x);

/**
 * @brief 一键把 EKF 配置初始化为四旋翼姿态模型(7 状态/6 观测 + 回调全部挂好)。
 *        调用后用户仍需设置 Q、R、x0、P0、更新方法。
 * @return 成功返回 true
 */
bool attitude_config_init(EKF_Config *config);

/**
 * @brief 便捷构造姿态初始状态向量 x0(7x1) 与协方差 P0(7x7)。
 * @param x0     输出，7x1
 * @param P0     输出，7x7
 * @param q_init 初始四元数(4)
 * @param att_std    初始姿态(四元数分量)标准差
 * @param bias_std   初始零偏标准差(rad/s)
 */
bool attitude_init_state(Matrix *x0, Matrix *P0, const float q_init[4],
                         float att_std, float bias_std);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
