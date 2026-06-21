/**
 * @file test_attitude.c
 * @brief 四旋翼姿态 EKF + 四元数运算 单元/集成测试
 *
 * 覆盖：
 *  - 四元数基础运算（归一化、乘法单位元、欧拉互换、夹角）
 *  - 姿态 EKF 在 CLEAN 场景的精度与一致性（RMSE / NIS 边界）
 *  - 鲁棒性回归：OUTLIER 下 Student-t 显著优于标准；MANEUVER 下自适应优于标准
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "attitude.h"
#include "quaternion.h"
#include "imu_sim.h"
#include <stdio.h>
#include <math.h>

static int passed = 0, failed = 0;
#define CHECK(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", msg); passed++; } \
    else { printf("  FAIL: %s\n", msg); failed++; } } while (0)

#define DEG (180.0f / 3.14159265358979f)

/* ---- 四元数运算 ---- */
static void test_quaternion(void) {
    printf("\nTest: 四元数运算\n");

    float q[4] = {2, 0, 0, 0};
    quat_normalize(q);
    CHECK(fabsf(q[0] - 1.0f) < 1e-6f, "归一化 [2,0,0,0]->[1,0,0,0]");

    float id[4] = {1, 0, 0, 0}, a[4] = {0.5f, 0.5f, 0.5f, 0.5f}, out[4];
    quat_mul(out, a, id);
    CHECK(fabsf(out[0]-a[0]) < 1e-6f && fabsf(out[1]-a[1]) < 1e-6f &&
          fabsf(out[2]-a[2]) < 1e-6f && fabsf(out[3]-a[3]) < 1e-6f, "q ⊗ 单位元 = q");

    /* 欧拉 → 四元数 → 欧拉 往返 */
    float r0 = 0.3f, p0 = -0.2f, y0 = 1.1f, qe[4], r1, p1, y1;
    quat_from_euler(qe, r0, p0, y0);
    quat_to_euler(qe, &r1, &p1, &y1);
    CHECK(fabsf(r1-r0) < 1e-4f && fabsf(p1-p0) < 1e-4f && fabsf(y1-y0) < 1e-4f,
          "欧拉↔四元数 往返一致");

    /* 同一姿态夹角为 0；相反四元数夹角也为 0(双覆盖) */
    float qn[4] = {-qe[0], -qe[1], -qe[2], -qe[3]};
    CHECK(quat_angle_between(qe, qe) < 1e-4f, "同姿态夹角=0");
    CHECK(quat_angle_between(qe, qn) < 1e-4f, "±q 表示同一姿态夹角=0");

    /* 旋转一致性：world→body→world 复原 */
    float vw[3] = {0.3f, -0.7f, 0.5f}, vb[3], vw2[3];
    quat_rotate_inv(vb, qe, vw);
    quat_rotate(vw2, qe, vb);
    CHECK(fabsf(vw2[0]-vw[0]) < 1e-4f && fabsf(vw2[1]-vw[1]) < 1e-4f &&
          fabsf(vw2[2]-vw[2]) < 1e-4f, "旋转 R·R^T 复原向量");
}

/* 跑一次姿态滤波，返回稳态姿态 RMSE(deg) 与平均 NIS */
static void run_filter_g(ImuScenario scenario, EKF_UpdateMethod method, int gate,
                         uint32_t seed, double *rmse_deg, double *avg_nis) {
    ImuSimConfig sc = imu_sim_default(scenario);
    ImuSim sim; imu_sim_init(&sim, seed);
    EKF_Config cfg; attitude_config_init(&cfg);
    Matrix Q, R, Rg;
    matrix_zeros(&Q, 7, 7); matrix_zeros(&R, 6, 6); matrix_zeros(&Rg, 6, 6);
    for (int i = 0; i < 4; i++) Q.data[i*7+i] = 1e-6f;
    for (int i = 4; i < 7; i++) Q.data[i*7+i] = 1e-9f;
    for (int i = 0; i < 6; i++) R.data[i*6+i] = 4e-4f;
    ekf_set_process_noise(&cfg, &Q);
    ekf_set_measurement_noise(&cfg, &R);
    ekf_set_update_method(&cfg, method);
    ekf_set_student_t_params(&cfg, 4.0f, 0.5f);
    ekf_set_adaptive_params(&cfg, 100.0f, 8.0f);

    Matrix x0, P0; float qid[4] = {1, 0, 0, 0};
    attitude_init_state(&x0, &P0, qid, 0.5f, 0.05f);
    EKF_State st; ekf_state_init(&st, &cfg, &x0, &P0);

    Matrix u, z; matrix_init(&u, 3, 1); matrix_init(&z, 6, 1);
    int N = 2000, warm = N / 2; double sse = 0, nis_sum = 0; int cnt = 0;
    for (int k = 0; k < N; k++) {
        ImuSample s; imu_sim_step(&sim, &sc, &s);
        for (int i = 0; i < 3; i++) u.data[i] = s.gyro[i];
        ekf_predict(&st, &cfg, &u, sc.dt);
        for (int i = 0; i < 3; i++) { z.data[i] = s.accel_dir[i]; z.data[3+i] = s.mag_dir[i]; }
        if (gate) {
            float dev = fabsf(s.accel_mag / IMU_G - 1.0f);
            float gs = 1.0f + 800.0f * dev * dev; if (gs > 1000.0f) gs = 1000.0f;
            for (int i = 0; i < 3; i++) Rg.data[i*6+i] = 4e-4f * gs;
            for (int i = 3; i < 6; i++) Rg.data[i*6+i] = 4e-4f;
            ekf_set_measurement_noise(&cfg, &Rg);
        }
        ekf_update(&st, &cfg, &z);
        float qe[4] = {st.x.data[0], st.x.data[1], st.x.data[2], st.x.data[3]};
        float ang = quat_angle_between(qe, s.q_true) * DEG;
        Matrix Sinv, Sy; float nis = 0;
        if (matrix_inverse(&Sinv, &st.S) && matrix_mul(&Sy, &Sinv, &st.y))
            for (int i = 0; i < 6; i++) nis += st.y.data[i] * Sy.data[i];
        if (k >= warm) { sse += (double)ang*ang; nis_sum += nis; cnt++; }
    }
    *rmse_deg = sqrt(sse / cnt);
    *avg_nis  = nis_sum / cnt;
}

/* 无门控的便捷封装（保持既有测试调用不变） */
static void run_filter(ImuScenario scenario, EKF_UpdateMethod method, uint32_t seed,
                       double *rmse_deg, double *avg_nis) {
    run_filter_g(scenario, method, 0, seed, rmse_deg, avg_nis);
}

static void test_attitude_clean(void) {
    printf("\nTest: 姿态 EKF —— CLEAN 精度与一致性\n");
    double rmse, nis;
    run_filter(SIM_CLEAN, EKF_UPDATE_STANDARD, 42u, &rmse, &nis);
    printf("  (RMSE=%.3f deg, avgNIS=%.3f)\n", rmse, nis);
    CHECK(rmse < 1.0, "CLEAN 姿态 RMSE < 1.0°");
    CHECK(nis > 2.0 && nis < 7.0, "CLEAN 平均 NIS ∈ (2,7)（一致，有效自由度≈4）");
}

static void test_attitude_standard_equals_joseph(void) {
    printf("\nTest: 姿态 EKF —— Standard ≡ Joseph\n");
    double rs, rj, ns, nj;
    run_filter(SIM_CLEAN, EKF_UPDATE_STANDARD, 7u, &rs, &ns);
    run_filter(SIM_CLEAN, EKF_UPDATE_JOSEPH, 7u, &rj, &nj);
    CHECK(fabs(rs - rj) < 1e-3, "标准与 Joseph 姿态 RMSE 一致");
}

static void test_attitude_robust(void) {
    printf("\nTest: 姿态 EKF —— 鲁棒性回归\n");
    double r_std, r_stu, r_ada, n;
    run_filter(SIM_OUTLIER, EKF_UPDATE_STANDARD, 42u, &r_std, &n);
    run_filter(SIM_OUTLIER, EKF_UPDATE_STUDENT_T, 42u, &r_stu, &n);
    printf("  OUTLIER : std=%.3f  student-t=%.3f deg\n", r_std, r_stu);
    CHECK(r_stu < 0.6 * r_std, "OUTLIER 下 Student-t 较标准降低 >40%");

    run_filter(SIM_MANEUVER, EKF_UPDATE_STANDARD, 42u, &r_std, &n);
    run_filter(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 42u, &r_ada, &n);
    printf("  MANEUVER: std=%.3f  adaptive=%.3f deg\n", r_std, r_ada);
    CHECK(r_ada < r_std, "MANEUVER 下自适应较标准更优");
}

static void test_attitude_accel_gating(void) {
    printf("\nTest: 姿态 EKF —— 加速度自适应门控\n");
    double r_no, r_gate, r_clean_no, r_clean_gate, n;
    /* MANEUVER：门控应进一步降低误差 */
    run_filter_g(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 0, 42u, &r_no, &n);
    run_filter_g(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 1, 42u, &r_gate, &n);
    printf("  MANEUVER 自适应: 无门控=%.3f  +门控=%.3f deg\n", r_no, r_gate);
    CHECK(r_gate < r_no, "MANEUVER 下门控进一步降低误差");
    /* CLEAN：门控不应明显劣化（比力≈g，门控≈1） */
    run_filter_g(SIM_CLEAN, EKF_UPDATE_ADAPTIVE, 0, 42u, &r_clean_no, &n);
    run_filter_g(SIM_CLEAN, EKF_UPDATE_ADAPTIVE, 1, 42u, &r_clean_gate, &n);
    printf("  CLEAN 自适应: 无门控=%.3f  +门控=%.3f deg\n", r_clean_no, r_clean_gate);
    CHECK(r_clean_gate < r_clean_no * 1.1 + 1e-6, "CLEAN 下门控不明显劣化(<+10%)");
}

int main(void) {
    printf("========== 姿态 EKF / 四元数 测试 ==========\n");
    test_quaternion();
    test_attitude_clean();
    test_attitude_standard_equals_joseph();
    test_attitude_robust();
    test_attitude_accel_gating();
    printf("\n========== 结果 ==========\n通过: %d\n失败: %d\n总计: %d\n",
           passed, failed, passed + failed);
    return failed > 0 ? 1 : 0;
}
