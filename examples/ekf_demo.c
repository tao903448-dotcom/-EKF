/**
 * @file ekf_demo.c
 * @brief 自适应 EKF 跨平台命令行演示（无 GUI 依赖）
 *
 * 设计目标：用一个可在 Linux / macOS / Windows 直接编译运行的程序，
 * 诚实地展示四种 EKF 更新方法（标准 / Joseph / Student-t / 自适应）
 * 在两类场景下的真实表现，并导出 CSV 供绘图：
 *
 *   场景 A —— 过程模型失配（常值模型追踪快速正弦）：
 *     测量噪声适中、无明显野值。此时滤波误差主要来自“模型跟不上”，
 *     增益本应增大；而 Student-t / 自适应通过“膨胀 R”降增益，反而更差。
 *     结论：鲁棒/自适应方法对“过程失配”无能为力，标准 EKF 已最优。
 *
 *   场景 B —— 测量野值（匀速模型 + 15% 脉冲野值）：
 *     模型正确、观测含离群点。这正是 Student-t / 自适应的用武之地，
 *     它们对野值降权，RMSE 相对标准 EKF 大幅下降。
 *
 * 两个场景合起来说明：没有“万能最优”的滤波方法，要按噪声特性选型。
 * 这与单一“自适应永远最好”的宣传相反，但更接近工程真相。
 *
 * 编译：  gcc -I include examples/ekf_demo.c src/matrix.c src/ekf.c -lm -o ekf_demo
 * 运行：  ./ekf_demo            # 打印对比表
 *         ./ekf_demo out.csv    # 额外把两个场景的轨迹写入 CSV
 *
 * @author 软件杯团队（重构）
 * @date 2026-06-20
 */

#include "matrix.h"
#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ---------- 可复现的伪随机（LCG + Box-Muller），跨平台一致 ---------- */
static unsigned g_rng = 2026u;
static void rng_seed(unsigned s) { g_rng = s; }
static float u01(void) {
    g_rng = g_rng * 1103515245u + 12345u;
    return (float)((g_rng >> 16) & 0x7fff) / 32767.0f;
}
static float randn(void) {
    float a = u01(), b = u01();
    if (a < 1e-6f) a = 1e-6f;
    return sqrtf(-2.0f * logf(a)) * cosf(6.2831853f * b);
}

/* ---------- 1D 常值（随机游走）模型：x' = x ---------- */
static bool sf_const(const Matrix *x, const Matrix *u, Matrix *xn, float dt) {
    (void)u; (void)dt; return matrix_copy(xn, x);
}
static bool mf_pos1(const Matrix *x, Matrix *z) { return matrix_copy(z, x); }
static bool sj_const(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    (void)x; (void)u; (void)dt; return matrix_eye(F, 1);
}
static bool mj_pos1(const Matrix *x, Matrix *H) { (void)x; return matrix_eye(H, 1); }

/* ---------- 2D 匀速模型：state=[pos,vel]，测量 pos ---------- */
static bool sf_cv(const Matrix *x, const Matrix *u, Matrix *xn, float dt) {
    (void)u;
    float p = matrix_get(x, 0, 0), v = matrix_get(x, 1, 0);
    matrix_set(xn, 0, 0, p + v * dt);
    matrix_set(xn, 1, 0, v);
    return true;
}
static bool mf_cv(const Matrix *x, Matrix *z) {
    matrix_set(z, 0, 0, matrix_get(x, 0, 0));
    return true;
}
static bool sj_cv(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    (void)x; (void)u;
    matrix_set(F, 0, 0, 1); matrix_set(F, 0, 1, dt);
    matrix_set(F, 1, 0, 0); matrix_set(F, 1, 1, 1);
    return true;
}
static bool mj_cv(const Matrix *x, Matrix *H) {
    (void)x;
    matrix_set(H, 0, 0, 1); matrix_set(H, 0, 1, 0);
    return true;
}

static const char *method_name(EKF_UpdateMethod m) {
    switch (m) {
        case EKF_UPDATE_STANDARD:  return "标准EKF  (Standard)";
        case EKF_UPDATE_JOSEPH:    return "Joseph形式(Joseph)  ";
        case EKF_UPDATE_STUDENT_T: return "Student-t (鲁棒)    ";
        case EKF_UPDATE_ADAPTIVE:  return "自适应EKF (Adaptive)";
        default:                   return "未知";
    }
}

/* 运行一个 1D 场景，返回 RMSE；est_out 非空时写出估计轨迹 */
static float run_1d(EKF_UpdateMethod method, const float *truth, const float *obs,
                    int n, float Q, float R, float *est_out) {
    EKF_Config c; EKF_State s;
    memset(&c, 0, sizeof c); memset(&s, 0, sizeof s);
    ekf_config_init(&c, 1, 1);
    ekf_set_functions(&c, sf_const, mf_pos1, sj_const, mj_pos1);
    Matrix Qm, Rm; matrix_init(&Qm, 1, 1); matrix_init(&Rm, 1, 1);
    Qm.data[0] = Q; Rm.data[0] = R;
    ekf_set_process_noise(&c, &Qm); ekf_set_measurement_noise(&c, &Rm);
    ekf_set_update_method(&c, method);
    ekf_set_student_t_params(&c, 3.0f, 1.0f);
    ekf_set_adaptive_params(&c, 50.0f, 2.0f);
    Matrix x0, P0; matrix_init(&x0, 1, 1); matrix_init(&P0, 1, 1);
    x0.data[0] = truth[0]; P0.data[0] = 10.0f;
    ekf_state_init(&s, &c, &x0, &P0);

    Matrix u, z; matrix_init(&u, 1, 1); matrix_init(&z, 1, 1);
    double sse = 0.0;
    for (int i = 0; i < n; i++) {
        ekf_predict(&s, &c, &u, 0.1f);
        z.data[0] = obs[i];
        ekf_update(&s, &c, &z);
        float est = s.x.data[0];
        if (est_out) est_out[i] = est;
        double d = (double)est - truth[i];
        sse += d * d;
    }
    return (float)sqrt(sse / n);
}

/* 运行一个 2D 匀速场景，返回位置 RMSE；est_out 写位置估计 */
static float run_2d(EKF_UpdateMethod method, const float *truth, const float *obs,
                    int n, float *est_out) {
    EKF_Config c; EKF_State s;
    memset(&c, 0, sizeof c); memset(&s, 0, sizeof s);
    ekf_config_init(&c, 2, 1);
    ekf_set_functions(&c, sf_cv, mf_cv, sj_cv, mj_cv);
    Matrix Qm, Rm; matrix_init(&Qm, 2, 2); matrix_init(&Rm, 1, 1);
    matrix_set(&Qm, 0, 0, 0.01f); matrix_set(&Qm, 1, 1, 0.01f);
    matrix_set(&Rm, 0, 0, 1.0f);
    ekf_set_process_noise(&c, &Qm); ekf_set_measurement_noise(&c, &Rm);
    ekf_set_update_method(&c, method);
    ekf_set_student_t_params(&c, 3.0f, 1.0f);
    ekf_set_adaptive_params(&c, 50.0f, 2.0f);
    Matrix x0, P0; matrix_init(&x0, 2, 1); matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0); matrix_set(&x0, 1, 0, 1);
    matrix_set(&P0, 0, 0, 1); matrix_set(&P0, 1, 1, 1);
    ekf_state_init(&s, &c, &x0, &P0);

    Matrix u, z; matrix_init(&u, 1, 1); matrix_init(&z, 1, 1);
    double sse = 0.0;
    for (int i = 0; i < n; i++) {
        ekf_predict(&s, &c, &u, 1.0f);
        z.data[0] = obs[i];
        ekf_update(&s, &c, &z);
        float est = matrix_get(&s.x, 0, 0);
        if (est_out) est_out[i] = est;
        double d = (double)est - truth[i];
        sse += d * d;
    }
    return (float)sqrt(sse / n);
}

#define NA 200
#define NB 300

int main(int argc, char **argv) {
    static float a_truth[NA], a_obs[NA];
    static float b_truth[NB], b_obs[NB];
    static float est[NB];  /* 复用 */

    /* ---- 场景 A：常值模型追踪快速正弦（过程失配，无野值） ---- */
    rng_seed(2026u);
    for (int i = 0; i < NA; i++) {
        float t = i * 0.1f;
        a_truth[i] = 120.0f + sinf(t * 0.8f) * 60.0f + cosf(t * 0.3f) * 20.0f;
        a_obs[i]   = a_truth[i] + randn() * 5.5f;   /* 适中高斯噪声，无脉冲 */
    }

    /* ---- 场景 B：匀速模型 + 15% 脉冲野值（测量野值，模型正确） ---- */
    rng_seed(7u);
    for (int i = 0; i < NB; i++) {
        b_truth[i] = (float)i * 1.0f;               /* 速度 1 的匀速运动 */
        float noise = randn() * 1.0f;
        if (u01() < 0.15f) noise += (u01() > 0.5f ? 1.0f : -1.0f) * 25.0f;
        b_obs[i] = b_truth[i] + noise;
    }

    const EKF_UpdateMethod methods[4] = {
        EKF_UPDATE_STANDARD, EKF_UPDATE_JOSEPH,
        EKF_UPDATE_STUDENT_T, EKF_UPDATE_ADAPTIVE
    };
    float rmse_a[4], rmse_b[4];

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║      自适应 EKF —— 跨平台诚实对比演示                         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    printf("场景 A：常值模型追踪快速正弦（过程模型失配，测量无野值）\n");
    printf("──────────────────────────────────────────────────────────\n");
    for (int k = 0; k < 4; k++) {
        rmse_a[k] = run_1d(methods[k], a_truth, a_obs, NA, 1.0f, 30.0f, NULL);
        printf("   %s  RMSE = %7.3f\n", method_name(methods[k]), rmse_a[k]);
    }
    printf("   ↳ 标准≈Joseph（数学等价）；R 膨胀类方法因降增益而更差——\n");
    printf("     过程失配应增大 Q，而非靠鲁棒/自适应。\n\n");

    printf("场景 B：匀速模型 + 15%% 脉冲野值（模型正确，测量含离群点）\n");
    printf("──────────────────────────────────────────────────────────\n");
    for (int k = 0; k < 4; k++) {
        rmse_b[k] = run_2d(methods[k], b_truth, b_obs, NB, NULL);
        printf("   %s  RMSE = %7.3f\n", method_name(methods[k]), rmse_b[k]);
    }
    if (rmse_b[0] > 1e-6f) {
        printf("   ↳ Student-t 相对标准 EKF 降低 %.1f%%，自适应降低 %.1f%%——\n",
               (1.0f - rmse_b[2] / rmse_b[0]) * 100.0f,
               (1.0f - rmse_b[3] / rmse_b[0]) * 100.0f);
        printf("     这是鲁棒滤波真正的用武之地（抗野值）。\n");
    }

    printf("\n结论：不存在“永远最优”的更新方法，需按噪声特性选型。\n");

    /* ---- 可选：导出 CSV ---- */
    if (argc > 1) {
        FILE *f = fopen(argv[1], "w");
        if (!f) { fprintf(stderr, "无法写入 %s\n", argv[1]); return 1; }
        fprintf(f, "scenario,step,truth,obs,est_standard,est_joseph,est_studentt,est_adaptive\n");

        /* 场景 A 轨迹 */
        float ea[4][NA];
        for (int k = 0; k < 4; k++) run_1d(methods[k], a_truth, a_obs, NA, 1.0f, 30.0f, ea[k]);
        for (int i = 0; i < NA; i++) {
            fprintf(f, "A,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    i, a_truth[i], a_obs[i], ea[0][i], ea[1][i], ea[2][i], ea[3][i]);
        }
        /* 场景 B 轨迹 */
        float eb[4][NB];
        for (int k = 0; k < 4; k++) run_2d(methods[k], b_truth, b_obs, NB, eb[k]);
        for (int i = 0; i < NB; i++) {
            fprintf(f, "B,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    i, b_truth[i], b_obs[i], eb[0][i], eb[1][i], eb[2][i], eb[3][i]);
        }
        fclose(f);
        (void)est;
        printf("\n已导出轨迹 CSV：%s\n", argv[1]);
    }

    return 0;
}
