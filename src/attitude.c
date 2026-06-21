/**
 * @file attitude.c
 * @brief 四旋翼姿态估计 EKF 模型实现（四元数 + 陀螺零偏）
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "attitude.h"
#include "quaternion.h"
#include <string.h>
#include <math.h>

const float ATT_G_DIR_W[3] = {0.0f, 0.0f, 1.0f};   /* 重力反作用方向(向上) */
const float ATT_M_DIR_W[3] = {1.0f, 0.0f, 0.0f};   /* 参考磁场方向(指北)   */

/* ---- 预测：q ← q ⊗ Δq(ω·dt)，b 不变 ---- */
bool attitude_state_func(const Matrix *x, const Matrix *u, Matrix *x_new, float dt) {
    if (x == NULL || u == NULL || x_new == NULL) return false;

    float q[4]  = { x->data[0], x->data[1], x->data[2], x->data[3] };
    float b[3]  = { x->data[4], x->data[5], x->data[6] };
    float wm[3] = { u->data[0], u->data[1], u->data[2] };

    /* 零偏校正后的角速度 */
    float omega[3] = { wm[0] - b[0], wm[1] - b[1], wm[2] - b[2] };
    float rotvec[3] = { omega[0]*dt, omega[1]*dt, omega[2]*dt };

    float dq[4], qn[4];
    quat_from_rotvec(dq, rotvec);
    quat_mul(qn, q, dq);
    quat_normalize(qn);

    x_new->rows = ATT_STATE_DIM; x_new->cols = 1; x_new->stride = 1;
    x_new->data[0] = qn[0]; x_new->data[1] = qn[1];
    x_new->data[2] = qn[2]; x_new->data[3] = qn[3];
    x_new->data[4] = b[0];  x_new->data[5] = b[1]; x_new->data[6] = b[2];
    return true;
}

/* ---- 观测：z = [R(q)^T·ĝ_w ; R(q)^T·m̂_w]，均为单位方向 ---- */
bool attitude_meas_func(const Matrix *x, Matrix *z) {
    if (x == NULL || z == NULL) return false;
    float q[4] = { x->data[0], x->data[1], x->data[2], x->data[3] };

    float g_b[3], m_b[3];
    quat_rotate_inv(g_b, q, ATT_G_DIR_W);   /* 世界→机体 */
    quat_rotate_inv(m_b, q, ATT_M_DIR_W);

    z->rows = ATT_MEAS_DIM; z->cols = 1; z->stride = 1;
    z->data[0] = g_b[0]; z->data[1] = g_b[1]; z->data[2] = g_b[2];
    z->data[3] = m_b[0]; z->data[4] = m_b[1]; z->data[5] = m_b[2];
    return true;
}

/* ---- 数值雅可比：中心差分（correct-by-construction） ---- */
#define ATT_JAC_EPS 1e-4f

bool attitude_state_jacobian(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    if (x == NULL || u == NULL || F == NULL) return false;
    uint8_t n = ATT_STATE_DIM;
    F->rows = n; F->cols = n; F->stride = n;

    Matrix xp, xm, fp, fm;
    matrix_copy(&xp, x);
    matrix_copy(&xm, x);
    for (uint8_t j = 0; j < n; j++) {
        float orig = x->data[j];
        xp.data[j] = orig + ATT_JAC_EPS;
        xm.data[j] = orig - ATT_JAC_EPS;
        if (!attitude_state_func(&xp, u, &fp, dt)) return false;
        if (!attitude_state_func(&xm, u, &fm, dt)) return false;
        float inv2h = 1.0f / (2.0f * ATT_JAC_EPS);
        for (uint8_t i = 0; i < n; i++) {
            F->data[i * n + j] = (fp.data[i] - fm.data[i]) * inv2h;
        }
        xp.data[j] = orig;   /* 复位 */
        xm.data[j] = orig;
    }
    return true;
}

bool attitude_meas_jacobian(const Matrix *x, Matrix *H) {
    if (x == NULL || H == NULL) return false;
    uint8_t n = ATT_STATE_DIM, m = ATT_MEAS_DIM;
    H->rows = m; H->cols = n; H->stride = n;

    Matrix xp, xm, hp, hm;
    matrix_copy(&xp, x);
    matrix_copy(&xm, x);
    for (uint8_t j = 0; j < n; j++) {
        float orig = x->data[j];
        xp.data[j] = orig + ATT_JAC_EPS;
        xm.data[j] = orig - ATT_JAC_EPS;
        if (!attitude_meas_func(&xp, &hp)) return false;
        if (!attitude_meas_func(&xm, &hm)) return false;
        float inv2h = 1.0f / (2.0f * ATT_JAC_EPS);
        for (uint8_t i = 0; i < m; i++) {
            H->data[i * n + j] = (hp.data[i] - hm.data[i]) * inv2h;
        }
        xp.data[j] = orig;
        xm.data[j] = orig;
    }
    return true;
}

/* ---- 四元数重归一化(零偏不变) ---- */
void attitude_normalize(Matrix *x) {
    if (x == NULL || x->rows < 4) return;
    float q[4] = { x->data[0], x->data[1], x->data[2], x->data[3] };
    quat_normalize(q);
    x->data[0] = q[0]; x->data[1] = q[1]; x->data[2] = q[2]; x->data[3] = q[3];
}

bool attitude_config_init(EKF_Config *config) {
    if (!ekf_config_init(config, ATT_STATE_DIM, ATT_MEAS_DIM)) return false;
    ekf_set_functions(config, attitude_state_func, attitude_meas_func,
                      attitude_state_jacobian, attitude_meas_jacobian);
    ekf_set_state_normalize(config, attitude_normalize);
    return true;
}

bool attitude_init_state(Matrix *x0, Matrix *P0, const float q_init[4],
                         float att_std, float bias_std) {
    if (x0 == NULL || P0 == NULL || q_init == NULL) return false;
    matrix_zeros(x0, ATT_STATE_DIM, 1);
    float q[4] = { q_init[0], q_init[1], q_init[2], q_init[3] };
    quat_normalize(q);
    x0->data[0] = q[0]; x0->data[1] = q[1]; x0->data[2] = q[2]; x0->data[3] = q[3];
    /* 零偏初值 0 */

    matrix_zeros(P0, ATT_STATE_DIM, ATT_STATE_DIM);
    float av = att_std * att_std;
    float bv = bias_std * bias_std;
    for (uint8_t i = 0; i < 4; i++) P0->data[i * ATT_STATE_DIM + i] = av;
    for (uint8_t i = 4; i < 7; i++) P0->data[i * ATT_STATE_DIM + i] = bv;
    return true;
}
