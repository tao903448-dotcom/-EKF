/**
 * @file imu_sim.h
 * @brief 可复现的四旋翼 IMU 仿真器（header-only，供 demo 与测试共用）
 *
 * 生成一条飞行姿态轨迹，由轨迹反解出"真值"机体角速度，并合成带噪声的
 * 陀螺/加速度计/磁力计观测。三种场景：
 *   SIM_CLEAN     —— 温和机动 + 小噪声（理想）
 *   SIM_OUTLIER   —— 在 CLEAN 基础上注入加速度计脉冲野值（电机振动）
 *   SIM_MANEUVER  —— 大机动 + 显著线加速度（比力≠重力 → 加速度计"模型失配"）
 *
 * 关键物理：加速度计测比力 f = a_lin_world + g_reaction；机动时 a_lin 使
 * "重力方向"观测偏离真值，天然构成模型失配，正是自适应/鲁棒滤波的考点。
 *
 * 全部为确定性伪随机（LCG + Box-Muller），跨平台结果一致、可复现。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#ifndef IMU_SIM_H
#define IMU_SIM_H

#include "quaternion.h"
#include <math.h>
#include <stdint.h>

#define IMU_G 9.80665f
#ifndef IMU_PI
#define IMU_PI 3.14159265358979f
#endif

typedef enum {
    SIM_CLEAN = 0,
    SIM_OUTLIER = 1,
    SIM_MANEUVER = 2
} ImuScenario;

typedef struct {
    float dt;                 /* 采样周期 (s) */
    ImuScenario scenario;
    float gyro_noise_std;     /* 陀螺白噪声 (rad/s) */
    float gyro_bias_rw_std;   /* 陀螺零偏随机游走每步增量标准差 (rad/s) */
    float accel_noise_std;    /* 加速度方向噪声 (归一化前, 相对单位) */
    float mag_noise_std;      /* 磁方向噪声 */
    float outlier_prob;       /* 加速度野值概率 */
    float outlier_mag;        /* 野值幅度 */
    float maneuver_accel;     /* 线加速度幅度 (m/s^2) */
} ImuSimConfig;

typedef struct {
    uint32_t rng;
    int k;
    float bias_true[3];       /* 真值零偏(随机游走) */
} ImuSim;

typedef struct {
    float gyro[3];            /* 陀螺测量 (rad/s) */
    float accel_dir[3];       /* 归一化重力方向观测 (body) */
    float accel_mag;          /* 加速度计比力幅值 (m/s^2，归一化前)；静止≈g，机动时偏离 */
    float mag_dir[3];         /* 归一化磁方向观测 (body) */
    float q_true[4];          /* 真值姿态四元数(用于评估) */
    float bias_true[3];       /* 真值零偏(用于评估) */
} ImuSample;

/* ---- 内部：确定性 RNG ---- */
static inline float imu__u01(ImuSim *s) {
    s->rng = s->rng * 1103515245u + 12345u;
    return (float)((s->rng >> 16) & 0x7fff) / 32767.0f;
}
static inline float imu__randn(ImuSim *s) {
    float a = imu__u01(s), b = imu__u01(s);
    if (a < 1e-7f) a = 1e-7f;
    return sqrtf(-2.0f * logf(a)) * cosf(2.0f * IMU_PI * b);
}

/* ---- 默认配置 ---- */
static inline ImuSimConfig imu_sim_default(ImuScenario scenario) {
    ImuSimConfig c;
    c.dt = 0.005f;                 /* 200 Hz */
    c.scenario = scenario;
    c.gyro_noise_std = 0.01f;      /* ~0.57 deg/s */
    c.gyro_bias_rw_std = 2e-4f;
    c.accel_noise_std = 0.02f;
    c.mag_noise_std = 0.02f;
    c.outlier_prob = (scenario == SIM_OUTLIER) ? 0.12f : 0.0f;
    c.outlier_mag = 1.2f;
    c.maneuver_accel = (scenario == SIM_MANEUVER) ? 6.0f : 0.0f;
    return c;
}

static inline void imu_sim_init(ImuSim *s, uint32_t seed) {
    s->rng = seed ? seed : 1u;
    s->k = 0;
    s->bias_true[0] = 0.01f; s->bias_true[1] = -0.015f; s->bias_true[2] = 0.008f;
}

/* ---- 真值欧拉角轨迹(随场景) ---- */
static inline void imu__truth_euler(const ImuSimConfig *c, float t,
                                    float *roll, float *pitch, float *yaw) {
    float Ar, Ap, fr, fp, yr;
    if (c->scenario == SIM_MANEUVER) {
        Ar = 1.0f; Ap = 0.8f; fr = 0.6f; fp = 0.5f; yr = 0.8f;
    } else {
        Ar = 0.4f; Ap = 0.3f; fr = 0.25f; fp = 0.18f; yr = 0.3f;
    }
    *roll  = Ar * sinf(2.0f * IMU_PI * fr * t);
    *pitch = Ap * sinf(2.0f * IMU_PI * fp * t + 0.7f);
    *yaw   = yr * t;
}

static inline void imu__truth_quat(const ImuSimConfig *c, float t, float q[4]) {
    float r, p, y;
    imu__truth_euler(c, t, &r, &p, &y);
    quat_from_euler(q, r, p, y);
}

/* 世界系线加速度(仅 MANEUVER 非零) */
static inline void imu__lin_accel(const ImuSimConfig *c, float t, float a[3]) {
    if (c->maneuver_accel <= 0.0f) { a[0] = a[1] = a[2] = 0.0f; return; }
    float A = c->maneuver_accel;
    a[0] = A * sinf(2.0f * IMU_PI * 0.7f * t);
    a[1] = A * cosf(2.0f * IMU_PI * 0.5f * t);
    a[2] = 0.4f * A * sinf(2.0f * IMU_PI * 0.9f * t);
}

/* 由相邻真值四元数解出机体角速度(body, rad/s) */
static inline void imu__true_omega(const float q_k[4], const float q_n[4],
                                   float dt, float omega[3]) {
    float qc[4] = { q_k[0], -q_k[1], -q_k[2], -q_k[3] };  /* conj */
    float dq[4];
    quat_mul(dq, qc, q_n);                                 /* body 增量 */
    if (dq[0] < 0.0f) { dq[0] = -dq[0]; dq[1] = -dq[1]; dq[2] = -dq[2]; dq[3] = -dq[3]; }
    float vn = sqrtf(dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    float angle = 2.0f * atan2f(vn, dq[0]);
    float scale = (vn > 1e-9f) ? (angle / (vn * dt)) : (2.0f / dt);
    omega[0] = dq[1] * scale;
    omega[1] = dq[2] * scale;
    omega[2] = dq[3] * scale;
}

/**
 * @brief 推进一步，生成第 k 步的测量与（下一时刻的）真值。
 *
 * 约定：gyro 为区间 [k, k+1] 的角速度（用于 predict 到 k+1），
 *       accel/mag/q_true 为 k+1 时刻的量（用于 update 与评估）。
 */
static inline void imu_sim_step(ImuSim *s, const ImuSimConfig *c, ImuSample *out) {
    float t  = s->k * c->dt;
    float tn = (s->k + 1) * c->dt;

    float q_k[4], q_n[4];
    imu__truth_quat(c, t, q_k);
    imu__truth_quat(c, tn, q_n);

    /* 真值角速度 + 零偏随机游走 + 白噪声 → 陀螺测量 */
    float omega[3];
    imu__true_omega(q_k, q_n, c->dt, omega);
    for (int i = 0; i < 3; i++) {
        s->bias_true[i] += c->gyro_bias_rw_std * imu__randn(s);
        out->gyro[i] = omega[i] + s->bias_true[i] + c->gyro_noise_std * imu__randn(s);
        out->bias_true[i] = s->bias_true[i];
    }

    /* 比力(world) = 线加速度 + 重力反作用[0,0,G]，转到机体并归一化 */
    float a_lin[3]; imu__lin_accel(c, tn, a_lin);
    float f_w[3] = { a_lin[0], a_lin[1], a_lin[2] + IMU_G };
    float f_b[3];
    quat_rotate_inv(f_b, q_n, f_w);
    for (int i = 0; i < 3; i++) f_b[i] += c->accel_noise_std * IMU_G * imu__randn(s);
    /* 振动野值 */
    if (c->outlier_prob > 0.0f && imu__u01(s) < c->outlier_prob) {
        for (int i = 0; i < 3; i++) f_b[i] += c->outlier_mag * IMU_G * (imu__u01(s) - 0.5f);
    }
    float fn = sqrtf(f_b[0]*f_b[0] + f_b[1]*f_b[1] + f_b[2]*f_b[2]);
    if (fn < 1e-6f) fn = 1e-6f;
    out->accel_dir[0] = f_b[0]/fn; out->accel_dir[1] = f_b[1]/fn; out->accel_dir[2] = f_b[2]/fn;
    out->accel_mag = fn;   /* 比力幅值：静止≈g，机动(含线加速度/野值)时偏离 */

    /* 磁方向(world→body) + 噪声并归一化（参考方向须与 attitude 模型一致：指北[1,0,0]） */
    const float m_w_ref[3] = { 1.0f, 0.0f, 0.0f };
    float m_b[3];
    quat_rotate_inv(m_b, q_n, m_w_ref);
    for (int i = 0; i < 3; i++) m_b[i] += c->mag_noise_std * imu__randn(s);
    float mn = sqrtf(m_b[0]*m_b[0] + m_b[1]*m_b[1] + m_b[2]*m_b[2]);
    if (mn < 1e-6f) mn = 1e-6f;
    out->mag_dir[0] = m_b[0]/mn; out->mag_dir[1] = m_b[1]/mn; out->mag_dir[2] = m_b[2]/mn;

    out->q_true[0] = q_n[0]; out->q_true[1] = q_n[1];
    out->q_true[2] = q_n[2]; out->q_true[3] = q_n[3];

    s->k++;
}

#endif /* IMU_SIM_H */
