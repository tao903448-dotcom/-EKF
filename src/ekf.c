/**
 * @file ekf.c
 * @brief 扩展卡尔曼滤波器（EKF）实现
 *
 * 实现面向模型失配的自适应 EKF：
 *  - 标准 EKF 预测/更新
 *  - Joseph 形式协方差更新（数值稳定，保持对称正定）
 *  - Student-t 鲁棒更新（按马氏距离对野值降权）
 *  - 自适应更新（基于归一化新息平方 NIS 调整观测噪声）
 *
 * 本次重构修复的关键缺陷：
 *  1. 预测步状态函数原地混叠：原 state_func(&x,u,&x,dt) 违反“输入/输出
 *     分离”约定，对一般非线性 f 会读到已被覆盖的状态。现引入独立的
 *     x_pred 缓冲。
 *  2. 协方差更新混叠：依赖 matrix_mul 的 alias-safe 修复，
 *     P=(I-KH)P / K R K' 等不再被误清零。
 *  3. Student-t 权重方向写反：原实现对野值“增信”而非降权，现改为
 *     R_eff = R * (ν + d²/δ²)/(ν + n)，野值 → R 膨胀 → 增益收缩。
 *  4. 自适应更新只取标量 .data[0]（仅 1 维有效），现用多维 NIS。
 *  5. 每步更新后对 P 做对称化，抑制浮点累积导致的非对称漂移。
 *
 * 四种更新方法共用 S、K、状态更新、Joseph 协方差等辅助函数，消除了
 * 原先 4 份近乎复制的代码。
 *
 * @author 软件杯团队（重构）
 * @date 2026-06-20
 */

#include "ekf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 内部辅助函数 */
static bool ekf_innovation_cov(EKF_State *s, const Matrix *R);
static bool ekf_gain(EKF_State *s);
static bool ekf_apply_state(EKF_State *s);
static bool ekf_cov_simple(EKF_State *s, const EKF_Config *cfg);
static bool ekf_cov_joseph(EKF_State *s, const EKF_Config *cfg, const Matrix *R_eff);
static void ekf_symmetrize(Matrix *P);
static float ekf_nis(EKF_State *s);

static bool ekf_standard_update(EKF_State *state, const EKF_Config *config);
static bool ekf_joseph_update(EKF_State *state, const EKF_Config *config);
static bool ekf_student_t_update(EKF_State *state, const EKF_Config *config);
static bool ekf_adaptive_update(EKF_State *state, const EKF_Config *config);

/* ========== 配置函数 ========== */

bool ekf_config_init(EKF_Config *config, uint8_t state_dim, uint8_t measurement_dim) {
    if (config == NULL || state_dim == 0 || measurement_dim == 0 ||
        state_dim > EKF_MAX_STATE_DIM || measurement_dim > EKF_MAX_MEASUREMENT_DIM) {
        return false;
    }

    config->state_dim = state_dim;
    config->measurement_dim = measurement_dim;
    config->update_method = EKF_UPDATE_STANDARD;

    config->state_func = NULL;
    config->measurement_func = NULL;
    config->state_jacobian_func = NULL;
    config->measurement_jacobian_func = NULL;
    config->state_normalize_func = NULL;

    matrix_zeros(&config->Q, state_dim, state_dim);
    matrix_zeros(&config->R, measurement_dim, measurement_dim);

    config->student_t_nu = 5.0f;
    config->student_t_delta = 1.0f;

    config->adaptive_window = 50.0f;
    config->adaptive_factor = 0.95f;

    config->nis_gate = 0.0f;   /* 默认关闭新息门控 */

    return true;
}

void ekf_set_functions(EKF_Config *config,
                      EKF_StateFunc state_func,
                      EKF_MeasurementFunc measurement_func,
                      EKF_StateJacobianFunc state_jacobian,
                      EKF_MeasurementJacobianFunc measurement_jacobian) {
    if (config != NULL) {
        config->state_func = state_func;
        config->measurement_func = measurement_func;
        config->state_jacobian_func = state_jacobian;
        config->measurement_jacobian_func = measurement_jacobian;
    }
}

bool ekf_set_process_noise(EKF_Config *config, const Matrix *Q) {
    if (config == NULL || Q == NULL) {
        return false;
    }
    if (Q->rows != config->state_dim || Q->cols != config->state_dim) {
        return false;
    }
    return matrix_copy(&config->Q, Q);
}

bool ekf_set_measurement_noise(EKF_Config *config, const Matrix *R) {
    if (config == NULL || R == NULL) {
        return false;
    }
    if (R->rows != config->measurement_dim || R->cols != config->measurement_dim) {
        return false;
    }
    return matrix_copy(&config->R, R);
}

void ekf_set_student_t_params(EKF_Config *config, float nu, float delta) {
    if (config != NULL) {
        config->student_t_nu = nu;
        config->student_t_delta = delta;
    }
}

void ekf_set_adaptive_params(EKF_Config *config, float window_size, float factor) {
    if (config != NULL) {
        config->adaptive_window = window_size;
        config->adaptive_factor = factor;
    }
}

void ekf_set_update_method(EKF_Config *config, EKF_UpdateMethod method) {
    if (config != NULL) {
        config->update_method = method;
    }
}

void ekf_set_state_normalize(EKF_Config *config, EKF_StateNormalizeFunc fn) {
    if (config != NULL) {
        config->state_normalize_func = fn;
    }
}

void ekf_set_nis_gate(EKF_Config *config, float nis_threshold) {
    if (config != NULL) {
        config->nis_gate = nis_threshold;
    }
}

uint32_t ekf_get_rejected_count(const EKF_State *state) {
    return (state != NULL) ? state->rejected_count : 0u;
}

/* ========== 状态函数 ========== */

bool ekf_state_init(EKF_State *state, const EKF_Config *config,
                   const Matrix *x0, const Matrix *P0) {
    if (state == NULL || config == NULL || x0 == NULL || P0 == NULL) {
        return false;
    }
    if (x0->rows != config->state_dim || x0->cols != 1) {
        return false;
    }
    if (P0->rows != config->state_dim || P0->cols != config->state_dim) {
        return false;
    }

    matrix_copy(&state->x, x0);
    matrix_copy(&state->P, P0);

    matrix_zeros(&state->x_pred, config->state_dim, 1);
    matrix_zeros(&state->S, config->measurement_dim, config->measurement_dim);
    matrix_zeros(&state->K, config->state_dim, config->measurement_dim);
    matrix_zeros(&state->y, config->measurement_dim, 1);
    matrix_zeros(&state->z, config->measurement_dim, 1);
    matrix_zeros(&state->z_pred, config->measurement_dim, 1);

    matrix_zeros(&state->F, config->state_dim, config->state_dim);
    matrix_zeros(&state->H, config->measurement_dim, config->state_dim);
    matrix_zeros(&state->P_pred, config->state_dim, config->state_dim);
    matrix_zeros(&state->temp1, config->state_dim, config->state_dim);
    matrix_zeros(&state->temp2, config->state_dim, config->state_dim);
    matrix_zeros(&state->temp3, config->state_dim, config->state_dim);
    matrix_zeros(&state->temp4, config->state_dim, config->state_dim);
    matrix_zeros(&state->temp5, config->state_dim, config->state_dim);
    matrix_zeros(&state->S_inv, config->measurement_dim, config->measurement_dim);
    matrix_zeros(&state->R_eff, config->measurement_dim, config->measurement_dim);

    state->initialized = true;
    state->step_count = 0;
    state->rejected_count = 0;

    return true;
}

bool ekf_predict(EKF_State *state, const EKF_Config *config,
                const Matrix *u, float dt) {
    if (state == NULL || config == NULL || !state->initialized) {
        return false;
    }
    if (config->state_func == NULL || config->state_jacobian_func == NULL) {
        return false;
    }

    /* 1. 状态预测：x_pred = f(x, u, dt)，输入/输出分离（不混叠） */
    if (!config->state_func(&state->x, u, &state->x_pred, dt)) {
        return false;
    }

    /* 2. 状态转移雅可比 F = df/dx，必须在【先验状态 x_{k-1}】处线性化
       （泰勒展开点 = f 的输入，而非 f 的输出 x_pred）。此时 state->x
       仍为先验（直到步骤 4 才被覆盖），故直接传 &state->x。 */
    if (!config->state_jacobian_func(&state->x, u, &state->F, dt)) {
        return false;
    }

    /* 3. 协方差预测：P = F P F' + Q */
    if (!matrix_mul(&state->temp1, &state->F, &state->P)) {       /* F P */
        return false;
    }
    if (!matrix_transpose(&state->temp2, &state->F)) {           /* F' */
        return false;
    }
    if (!matrix_mul(&state->P_pred, &state->temp1, &state->temp2)) { /* F P F' */
        return false;
    }
    if (!matrix_add(&state->P, &state->P_pred, &config->Q)) {     /* + Q */
        return false;
    }
    ekf_symmetrize(&state->P);

    /* 4. 提交预测状态，并按需投影回约束流形（如四元数归一化） */
    matrix_copy(&state->x, &state->x_pred);
    if (config->state_normalize_func != NULL) {
        config->state_normalize_func(&state->x);
    }

    state->step_count++;
    return true;
}

bool ekf_update(EKF_State *state, const EKF_Config *config, const Matrix *z) {
    if (state == NULL || config == NULL || z == NULL || !state->initialized) {
        return false;
    }
    if (config->measurement_func == NULL || config->measurement_jacobian_func == NULL) {
        return false;
    }
    /* 拦截非有限观测（NaN/Inf）：丢弃该量测、保留预测，避免毒化状态 */
    if (!matrix_is_finite(z)) {
        return false;
    }

    matrix_copy(&state->z, z);

    /* z_pred = h(x) */
    if (!config->measurement_func(&state->x, &state->z_pred)) {
        return false;
    }
    /* H = dh/dx */
    if (!config->measurement_jacobian_func(&state->x, &state->H)) {
        return false;
    }
    /* 新息 y = z - z_pred */
    if (!matrix_sub(&state->y, &state->z, &state->z_pred)) {
        return false;
    }

    /* 新息卡方门控：NIS=yᵀS⁻¹y 超阈值 → 判为野值，整步拒绝该量测、仅保留预测。
       对所有更新方法通用，是粗野值的硬性前置防线。 */
    if (config->nis_gate > 0.0f) {
        if (!ekf_innovation_cov(state, &config->R)) return false;  /* 基础 S */
        float nis = ekf_nis(state);
        if (nis > config->nis_gate) {
            state->rejected_count++;
            return true;   /* 拒绝量测：保留预测的 x、P */
        }
    }

    bool ok;
    switch (config->update_method) {
        case EKF_UPDATE_STANDARD:  ok = ekf_standard_update(state, config); break;
        case EKF_UPDATE_JOSEPH:    ok = ekf_joseph_update(state, config); break;
        case EKF_UPDATE_STUDENT_T: ok = ekf_student_t_update(state, config); break;
        case EKF_UPDATE_ADAPTIVE:  ok = ekf_adaptive_update(state, config); break;
        default:                   return false;
    }

    /* 更新后按需把状态投影回约束流形（如四元数归一化） */
    if (ok && config->state_normalize_func != NULL) {
        config->state_normalize_func(&state->x);
    }
    return ok;
}

/* ========== 共用辅助：新息协方差 / 增益 / 状态更新 / 协方差更新 ========== */

/* S = H P H' + R；副产物：temp3 = H'，temp2 = H P H'（供重标定复用） */
static bool ekf_innovation_cov(EKF_State *s, const Matrix *R) {
    if (!matrix_transpose(&s->temp3, &s->H)) return false;          /* H'   */
    if (!matrix_mul(&s->temp1, &s->H, &s->P)) return false;          /* H P  */
    if (!matrix_mul(&s->temp2, &s->temp1, &s->temp3)) return false;  /* HPH' */
    if (!matrix_add(&s->S, &s->temp2, R)) return false;              /* +R   */
    return true;
}

/* K = P H' S^-1（要求 temp3 = H' 已就绪）
   首选 Cholesky 解：S 对称正定，解 S Kᵀ = (P H')ᵀ 比显式求逆更快更稳；
   若 S 非正定（数值退化）则回退到通用 Gauss-Jordan 求逆，保证不退步。 */
static bool ekf_gain(EKF_State *s) {
    if (!matrix_mul(&s->temp1, &s->P, &s->temp3)) return false;   /* M = P H' (state×meas) */
    /* Cholesky 路径：L=chol(S) 暂存于 S_inv 槽 */
    if (matrix_cholesky(&s->S_inv, &s->S)) {
        if (matrix_transpose(&s->temp2, &s->temp1) &&                 /* Mᵀ (meas×state) */
            matrix_cholesky_solve(&s->temp4, &s->S_inv, &s->temp2) && /* Kᵀ = S⁻¹ Mᵀ     */
            matrix_transpose(&s->K, &s->temp4)) {                     /* K (state×meas)  */
            return true;
        }
    }
    /* 回退：显式求逆 */
    if (!matrix_inverse(&s->S_inv, &s->S)) return false;
    if (!matrix_mul(&s->K, &s->temp1, &s->S_inv)) return false;   /* K = M S⁻¹ */
    return true;
}

/* x = x + K y */
static bool ekf_apply_state(EKF_State *s) {
    if (!matrix_mul(&s->temp1, &s->K, &s->y)) return false;
    if (!matrix_add(&s->x, &s->x, &s->temp1)) return false;
    return true;
}

/* 简单形式：P = (I - K H) P（matrix_mul 已 alias-safe）
   全部使用预分配的状态暂存矩阵，避免在更新热路径上产生大栈帧。 */
static bool ekf_cov_simple(EKF_State *s, const EKF_Config *cfg) {
    matrix_eye(&s->temp4, cfg->state_dim);                     /* I       */
    if (!matrix_mul(&s->temp1, &s->K, &s->H)) return false;    /* K H     */
    if (!matrix_sub(&s->temp2, &s->temp4, &s->temp1)) return false; /* I - K H */
    if (!matrix_mul(&s->P, &s->temp2, &s->P)) return false;    /* (I-KH)P */
    ekf_symmetrize(&s->P);
    return true;
}

/* Joseph 形式：P = (I-KH) P (I-KH)' + K R_eff K'
   仅复用 temp1/temp2/temp3/temp4/temp5（temp3=H' 在协方差阶段已不需要），
   不在栈上分配任何 Matrix，单步更新栈占用从 ~6KB 降到接近 0。 */
static bool ekf_cov_joseph(EKF_State *s, const EKF_Config *cfg, const Matrix *R_eff) {
    matrix_eye(&s->temp4, cfg->state_dim);                        /* I              */
    if (!matrix_mul(&s->temp1, &s->K, &s->H)) return false;       /* K H            */
    if (!matrix_sub(&s->temp3, &s->temp4, &s->temp1)) return false;/* I-KH (借用temp3)*/
    if (!matrix_transpose(&s->temp5, &s->temp3)) return false;    /* (I-KH)'        */
    if (!matrix_mul(&s->temp1, &s->temp3, &s->P)) return false;   /* (I-KH)P        */
    if (!matrix_mul(&s->temp2, &s->temp1, &s->temp5)) return false;/* (I-KH)P(I-KH)' */

    if (!matrix_mul(&s->temp4, &s->K, R_eff)) return false;       /* K R_eff (复用 I 槽)*/
    if (!matrix_transpose(&s->temp5, &s->K)) return false;        /* K'             */
    if (!matrix_mul(&s->temp1, &s->temp4, &s->temp5)) return false;/* K R_eff K'     */

    if (!matrix_add(&s->P, &s->temp2, &s->temp1)) return false;
    ekf_symmetrize(&s->P);
    return true;
}

/* P ← (P + P') / 2，抑制浮点不对称漂移 */
static void ekf_symmetrize(Matrix *P) {
    for (uint8_t i = 0; i < P->rows; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < P->cols; j++) {
            float a = matrix_get(P, i, j);
            float b = matrix_get(P, j, i);
            float avg = 0.5f * (a + b);
            matrix_set(P, i, j, avg);
            matrix_set(P, j, i, avg);
        }
    }
}

/* 归一化新息平方 NIS = y' S^-1 y（任意观测维度）
   首选 Cholesky 解 S w = y（仅一个右端列，最省），回退到显式求逆。
   复用预分配 S_inv（暂存 L）与 temp1（此时 HP 已不需要），不占栈。 */
static float ekf_nis(EKF_State *s) {
    bool ok = false;
    if (matrix_cholesky(&s->S_inv, &s->S)) {                 /* L = chol(S) */
        ok = matrix_cholesky_solve(&s->temp1, &s->S_inv, &s->y);  /* w = S^-1 y */
    }
    if (!ok) {  /* 回退：显式求逆 */
        if (!matrix_inverse(&s->S_inv, &s->S)) return 0.0f;
        if (!matrix_mul(&s->temp1, &s->S_inv, &s->y)) return 0.0f;
    }
    float nis = 0.0f;
    for (uint8_t i = 0; i < s->y.rows; i++) {
        nis += s->y.data[i] * s->temp1.data[i];
    }
    return nis;
}

/* ========== 标准卡尔曼更新 ========== */

static bool ekf_standard_update(EKF_State *state, const EKF_Config *config) {
    if (!ekf_innovation_cov(state, &config->R)) return false;
    if (!ekf_gain(state)) return false;
    if (!ekf_apply_state(state)) return false;
    if (!ekf_cov_simple(state, config)) return false;
    return true;
}

/* ========== Joseph 形式更新 ========== */

static bool ekf_joseph_update(EKF_State *state, const EKF_Config *config) {
    if (!ekf_innovation_cov(state, &config->R)) return false;
    if (!ekf_gain(state)) return false;
    if (!ekf_apply_state(state)) return false;
    if (!ekf_cov_joseph(state, config, &config->R)) return false;
    return true;
}

/* ========== Student-t 鲁棒更新 ========== */

static bool ekf_student_t_update(EKF_State *state, const EKF_Config *config) {
    /* 1. 基础新息协方差 S = H P H' + R */
    if (!ekf_innovation_cov(state, &config->R)) return false;

    /* 2. 马氏距离平方（NIS） */
    float d2 = ekf_nis(state);

    /* 3. Student-t 重标定：野值 → R 膨胀 → 降权
          R_eff = R * (ν + d²/δ²) / (ν + n)
          —— 修正原实现方向写反的缺陷 */
    float nu = config->student_t_nu;
    float delta = config->student_t_delta;
    float n = (float)config->measurement_dim;
    float d2n = d2 / (delta * delta);
    float scale = (nu + d2n) / (nu + n);
    if (scale < 1e-3f) scale = 1e-3f;

    if (!matrix_scale(&state->R_eff, &config->R, scale)) return false;

    /* 4. 用 R_eff 重算 S = HPH' + R_eff（temp2 仍为 HPH'） */
    if (!matrix_add(&state->S, &state->temp2, &state->R_eff)) return false;

    /* 5. 增益、状态、Joseph 协方差更新 */
    if (!ekf_gain(state)) return false;
    if (!ekf_apply_state(state)) return false;
    if (!ekf_cov_joseph(state, config, &state->R_eff)) return false;
    return true;
}

/* ========== 自适应更新（基于 NIS 的观测噪声匹配） ========== */

static bool ekf_adaptive_update(EKF_State *state, const EKF_Config *config) {
    /* 1. 基础新息协方差 */
    if (!ekf_innovation_cov(state, &config->R)) return false;

    /* 2. 归一化新息平方，理想期望值为观测维度 n */
    float n = (float)config->measurement_dim;
    float nis = ekf_nis(state);
    float nis_ratio = nis / (n + 1e-10f);

    /* 3. 自适应因子：NIS 超出期望 → 膨胀 R（更信任模型）
          factor = 1 + α·max(0, NIS/n − 1)，并夹在 [0.1, window] */
    float alpha = config->adaptive_factor;
    float window = config->adaptive_window;
    float factor = 1.0f + alpha * (nis_ratio > 1.0f ? (nis_ratio - 1.0f) : 0.0f);
    if (factor > window) factor = window;
    if (factor < 0.1f) factor = 0.1f;

    if (!matrix_scale(&state->R_eff, &config->R, factor)) return false;

    /* 4. 重算 S、增益、状态、Joseph 协方差 */
    if (!matrix_add(&state->S, &state->temp2, &state->R_eff)) return false;
    if (!ekf_gain(state)) return false;
    if (!ekf_apply_state(state)) return false;
    if (!ekf_cov_joseph(state, config, &state->R_eff)) return false;
    return true;
}

/* ========== 状态获取函数 ========== */

bool ekf_get_state(const EKF_State *state, Matrix *x) {
    if (state == NULL || x == NULL || !state->initialized) {
        return false;
    }
    return matrix_copy(x, &state->x);
}

bool ekf_get_covariance(const EKF_State *state, Matrix *P) {
    if (state == NULL || P == NULL || !state->initialized) {
        return false;
    }
    return matrix_copy(P, &state->P);
}

bool ekf_get_innovation(const EKF_State *state, Matrix *y) {
    if (state == NULL || y == NULL || !state->initialized) {
        return false;
    }
    return matrix_copy(y, &state->y);
}

bool ekf_get_kalman_gain(const EKF_State *state, Matrix *K) {
    if (state == NULL || K == NULL || !state->initialized) {
        return false;
    }
    return matrix_copy(K, &state->K);
}

void ekf_reset(EKF_State *state) {
    if (state != NULL) {
        state->initialized = false;
        state->step_count = 0;
    }
}

void ekf_print_state(const EKF_State *state) {
    if (state == NULL) {
        return;
    }
    printf("EKF State:\n");
    printf("  Initialized: %s\n", state->initialized ? "true" : "false");
    printf("  Step count: %u\n", state->step_count);
    if (state->initialized) {
        matrix_print(&state->x, "State (x)");
        matrix_print(&state->P, "Covariance (P)");
    }
}
