/**
 * @file mekf.h
 * @brief 乘性扩展卡尔曼滤波器（MEKF）—— 姿态估计的金标准误差状态形式
 *
 * 与本项目通用框架中的"加性 7 状态四元数 EKF"互为对照：MEKF 用 **6 维误差状态**
 * `δx = [δθ(3), δb(3)]`（3 轴姿态误差 + 3 轴零偏误差）表示不确定性，协方差 P 为
 * 6×6，从而**不对单位四元数过参数化**（加性 7 状态把 3 自由度旋转塞进 4 维，协方差
 * 秩/一致性欠严谨）。这是航天/无人机姿态估计的标准做法。
 *
 * 标称状态：四元数 q（body→world）+ 陀螺零偏 b；误差以"乘性"方式注入：
 *   预测：q ← q ⊗ Δq((ω_m−b)dt)；误差传播 P = F P Fᵀ + Q
 *   更新：由向量观测算 6×6 卡尔曼更新得 δx，注入 q ← q ⊗ δq(δθ)、b ← b + δb，误差归零
 *
 * 观测：加速度计"重力方向"+ 磁力计"参考方向"（与 attitude.c 一致的单位向量）。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#ifndef MEKF_H
#define MEKF_H

#include "matrix.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4];   /**< 标称姿态四元数 (body→world, Hamilton) */
    float b[3];   /**< 标称陀螺零偏 (rad/s) */
    Matrix P;     /**< 6×6 误差协方差 [δθ; δb] */
    Matrix Q;     /**< 6×6 过程噪声 */
    Matrix R;     /**< 6×6 观测噪声（加速度 3 + 磁 3） */
} MEKF;

/**
 * @brief 初始化 MEKF。
 * @param q0 初始四元数(4)；att_std 初始姿态误差标准差(rad)；bias_std 初始零偏标准差(rad/s)
 */
bool mekf_init(MEKF *f, const float q0[4], float att_std, float bias_std);

/** 设置 6×6 过程噪声 Q 与观测噪声 R（均为对角或满阵） */
bool mekf_set_noise(MEKF *f, const Matrix *Q6, const Matrix *R6);

/** 预测步：陀螺驱动姿态积分 + 误差协方差传播 */
bool mekf_predict(MEKF *f, const float gyro[3], float dt);

/** 更新步：加速度计 + 磁力计方向观测（各为单位向量, body 系） */
bool mekf_update(MEKF *f, const float accel_dir[3], const float mag_dir[3]);

void mekf_get_quat(const MEKF *f, float q[4]);
void mekf_get_bias(const MEKF *f, float b[3]);

#ifdef __cplusplus
}
#endif

#endif /* MEKF_H */
