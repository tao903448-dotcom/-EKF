/**
 * @file test_ekf.c
 * @brief EKF框架单元测试
 *
 * 测试EKF框架的各项功能：
 * - 配置初始化
 * - 状态初始化
 * - 预测步骤
 * - 更新步骤
 * - 状态获取
 *
 * @author 软件杯团队
 * @date 2026-06-16
 */

#include "matrix.h"
#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

/* 测试计数 */
static int test_passed = 0;
static int test_failed = 0;

/* 辅助宏 */
#define TEST_ASSERT(condition, msg) \
    do { \
        if (condition) { \
            printf("  PASS: %s\n", msg); \
            test_passed++; \
        } else { \
            printf("  FAIL: %s\n", msg); \
            test_failed++; \
        } \
    } while(0)

/* ========== 测试函数 ========== */

/**
 * @brief 简单的一维位置跟踪状态函数
 */
static bool test_state_func(const Matrix *x, const Matrix *u, Matrix *x_new, float dt) {
    (void)u;
    float pos = matrix_get(x, 0, 0);
    float vel = matrix_get(x, 1, 0);
    matrix_set(x_new, 0, 0, pos + vel * dt);
    matrix_set(x_new, 1, 0, vel);
    return true;
}

/**
 * @brief 简单的观测函数
 */
static bool test_meas_func(const Matrix *x, Matrix *z) {
    matrix_set(z, 0, 0, matrix_get(x, 0, 0));
    return true;
}

/**
 * @brief 简单的状态雅可比函数
 */
static bool test_state_jacob(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    (void)x; (void)u;
    matrix_set(F, 0, 0, 1.0f);
    matrix_set(F, 0, 1, dt);
    matrix_set(F, 1, 0, 0.0f);
    matrix_set(F, 1, 1, 1.0f);
    return true;
}

/**
 * @brief 简单的观测雅可比函数
 */
static bool test_meas_jacob(const Matrix *x, Matrix *H) {
    (void)x;
    matrix_set(H, 0, 0, 1.0f);
    matrix_set(H, 0, 1, 0.0f);
    return true;
}

/**
 * @brief 测试EKF配置初始化
 */
static void test_ekf_config_init(void) {
    printf("\nTest: EKF Config Init\n");

    EKF_Config config;
    bool result = ekf_config_init(&config, 2, 1);

    TEST_ASSERT(result == true, "Config init success");
    TEST_ASSERT(config.state_dim == 2, "State dimension = 2");
    TEST_ASSERT(config.measurement_dim == 1, "Measurement dimension = 1");
    TEST_ASSERT(config.update_method == EKF_UPDATE_STANDARD, "Default update method = STANDARD");
    TEST_ASSERT(config.student_t_nu == 5.0f, "Default Student-t nu = 5.0");
    TEST_ASSERT(config.adaptive_window == 50.0f, "Default adaptive window = 50.0");
}

/**
 * @brief 测试EKF函数设置
 */
static void test_ekf_set_functions(void) {
    printf("\nTest: EKF Set Functions\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);

    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    TEST_ASSERT(config.state_func == test_state_func, "State function set");
    TEST_ASSERT(config.measurement_func == test_meas_func, "Measurement function set");
    TEST_ASSERT(config.state_jacobian_func == test_state_jacob, "State Jacobian set");
    TEST_ASSERT(config.measurement_jacobian_func == test_meas_jacob, "Measurement Jacobian set");
}

/**
 * @brief 测试噪声设置
 */
static void test_ekf_noise(void) {
    printf("\nTest: EKF Noise Setting\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);

    Matrix Q, R;
    matrix_init(&Q, 2, 2);
    matrix_init(&R, 1, 1);

    matrix_set(&Q, 0, 0, 0.1f);
    matrix_set(&Q, 1, 1, 0.2f);
    matrix_set(&R, 0, 0, 1.0f);

    bool q_result = ekf_set_process_noise(&config, &Q);
    bool r_result = ekf_set_measurement_noise(&config, &R);

    TEST_ASSERT(q_result == true, "Process noise set successfully");
    TEST_ASSERT(r_result == true, "Measurement noise set successfully");
    TEST_ASSERT(config.Q.data[0] == 0.1f, "Q[0][0] = 0.1");
    TEST_ASSERT(config.R.data[0] == 1.0f, "R[0][0] = 1.0");
}

/**
 * @brief 测试EKF状态初始化
 */
static void test_ekf_state_init(void) {
    printf("\nTest: EKF State Init\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1);
    matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0.0f);
    matrix_set(&x0, 1, 0, 1.0f);
    matrix_set(&P0, 0, 0, 1.0f);
    matrix_set(&P0, 1, 1, 1.0f);

    EKF_State ekf;
    bool result = ekf_state_init(&ekf, &config, &x0, &P0);

    TEST_ASSERT(result == true, "EKF state init success");
    TEST_ASSERT(ekf.initialized == true, "EKF initialized flag set");
    TEST_ASSERT(ekf.step_count == 0, "Step count = 0");
}

/**
 * @brief 测试EKF预测步骤
 */
static void test_ekf_predict(void) {
    printf("\nTest: EKF Predict\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    Matrix Q, R;
    matrix_init(&Q, 2, 2);
    matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.1f);
    matrix_set(&Q, 1, 1, 0.1f);
    matrix_set(&R, 0, 0, 1.0f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1);
    matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0.0f);
    matrix_set(&x0, 1, 0, 1.0f);
    matrix_set(&P0, 0, 0, 1.0f);
    matrix_set(&P0, 1, 1, 1.0f);

    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);

    Matrix u;
    matrix_init(&u, 1, 1);

    bool result = ekf_predict(&ekf, &config, &u, 1.0f);

    TEST_ASSERT(result == true, "Predict step success");
    TEST_ASSERT(ekf.step_count == 1, "Step count incremented");
}

/**
 * @brief 测试EKF更新步骤
 */
static void test_ekf_update(void) {
    printf("\nTest: EKF Update\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    Matrix Q, R;
    matrix_init(&Q, 2, 2);
    matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.1f);
    matrix_set(&Q, 1, 1, 0.1f);
    matrix_set(&R, 0, 0, 1.0f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1);
    matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0.0f);
    matrix_set(&x0, 1, 0, 1.0f);
    matrix_set(&P0, 0, 0, 1.0f);
    matrix_set(&P0, 1, 1, 1.0f);

    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);

    Matrix u;
    matrix_init(&u, 1, 1);
    ekf_predict(&ekf, &config, &u, 1.0f);

    Matrix z;
    matrix_init(&z, 1, 1);
    matrix_set(&z, 0, 0, 1.5f);  /* 观测值 */

    bool result = ekf_update(&ekf, &config, &z);

    TEST_ASSERT(result == true, "Update step success");

    /* 获取估计值 */
    Matrix x_est;
    matrix_init(&x_est, 2, 1);
    ekf_get_state(&ekf, &x_est);

    float est_pos = matrix_get(&x_est, 0, 0);
    TEST_ASSERT(est_pos > 0.0f && est_pos < 2.0f, "Position estimate reasonable");
}

/**
 * @brief 测试EKF完整流程
 */
static void test_ekf_full_cycle(void) {
    printf("\nTest: EKF Full Cycle\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    Matrix Q, R;
    matrix_init(&Q, 2, 2);
    matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.1f);
    matrix_set(&Q, 1, 1, 0.1f);
    matrix_set(&R, 0, 0, 1.0f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1);
    matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0.0f);
    matrix_set(&x0, 1, 0, 1.0f);
    matrix_set(&P0, 0, 0, 1.0f);
    matrix_set(&P0, 1, 1, 1.0f);

    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);

    Matrix u;
    matrix_init(&u, 1, 1);

    /* 运行10步 */
    float true_pos = 0.0f;
    float velocity = 1.0f;
    float mse = 0.0f;

    for (int i = 0; i < 10; i++) {
        true_pos += velocity;
        float obs = true_pos + ((float)rand() / RAND_MAX - 0.5f) * 0.5f;

        ekf_predict(&ekf, &config, &u, 1.0f);

        Matrix z;
        matrix_init(&z, 1, 1);
        matrix_set(&z, 0, 0, obs);
        ekf_update(&ekf, &config, &z);

        Matrix x_est;
        matrix_init(&x_est, 2, 1);
        ekf_get_state(&ekf, &x_est);
        float est = matrix_get(&x_est, 0, 0);

        float error = est - true_pos;
        mse += error * error;
    }

    mse /= 10.0f;

    TEST_ASSERT(ekf.step_count == 10, "Completed 10 steps");
    TEST_ASSERT(mse < 1.0f, "MSE < 1.0 (converged)");
}

/**
 * @brief 测试不同更新方法
 */
static void test_ekf_update_methods(void) {
    printf("\nTest: EKF Update Methods\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);

    Matrix Q, R;
    matrix_init(&Q, 2, 2);
    matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.1f);
    matrix_set(&Q, 1, 1, 0.1f);
    matrix_set(&R, 0, 0, 1.0f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);

    /* 测试标准更新 */
    ekf_set_update_method(&config, EKF_UPDATE_STANDARD);
    TEST_ASSERT(config.update_method == EKF_UPDATE_STANDARD, "Standard update method set");

    /* 测试Joseph更新 */
    ekf_set_update_method(&config, EKF_UPDATE_JOSEPH);
    TEST_ASSERT(config.update_method == EKF_UPDATE_JOSEPH, "Joseph update method set");

    /* 测试Student-t更新 */
    ekf_set_update_method(&config, EKF_UPDATE_STUDENT_T);
    TEST_ASSERT(config.update_method == EKF_UPDATE_STUDENT_T, "Student-t update method set");

    /* 测试自适应更新 */
    ekf_set_update_method(&config, EKF_UPDATE_ADAPTIVE);
    TEST_ASSERT(config.update_method == EKF_UPDATE_ADAPTIVE, "Adaptive update method set");
}

/* ========== 回归测试：针对历史缺陷 ========== */

/**
 * @brief 协方差不坍缩（回归：标准更新 P=(I-KH)P 因别名被清零）
 *
 * 一步 predict + update 后，P 应为对称正定（迹>0、非对角对称），
 * 并与手算参考接近。旧实现这里 P 会变成全 0。
 */
static void test_ekf_cov_no_collapse(void) {
    printf("\nTest: Covariance does not collapse (regression)\n");

    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);
    Matrix Q, R;
    matrix_init(&Q, 2, 2); matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.01f); matrix_set(&Q, 1, 1, 0.01f); matrix_set(&R, 0, 0, 0.25f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1); matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0); matrix_set(&x0, 1, 0, 1);
    matrix_set(&P0, 0, 0, 1); matrix_set(&P0, 1, 1, 1);

    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);

    Matrix u, z;
    matrix_init(&u, 1, 1);
    matrix_init(&z, 1, 1); matrix_set(&z, 0, 0, 1.2f);
    ekf_predict(&ekf, &config, &u, 1.0f);
    ekf_update(&ekf, &config, &z);

    Matrix P;
    matrix_init(&P, 2, 2);
    ekf_get_covariance(&ekf, &P);
    float trace = matrix_get(&P, 0, 0) + matrix_get(&P, 1, 1);
    float asym  = fabsf(matrix_get(&P, 0, 1) - matrix_get(&P, 1, 0));

    TEST_ASSERT(trace > 0.1f, "trace(P) > 0 (未坍缩)");
    TEST_ASSERT(matrix_get(&P, 0, 0) > 0.0f && matrix_get(&P, 1, 1) > 0.0f, "对角元为正");
    TEST_ASSERT(asym < 1e-4f, "P 对称");
    /* 手算参考：[[0.222,0.111],[0.111,0.567]] */
    TEST_ASSERT(fabsf(matrix_get(&P, 0, 0) - 0.2223f) < 1e-3f, "P[0][0] 接近参考 0.222");
    TEST_ASSERT(fabsf(matrix_get(&P, 1, 1) - 0.5675f) < 1e-3f, "P[1][1] 接近参考 0.567");
}

/* 共用：跑 N 步随机游走/匀速场景，返回某状态分量的最终估计与 RMSE */
static float run_method_rmse(EKF_UpdateMethod method, int with_outliers) {
    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, test_state_func, test_meas_func,
                      test_state_jacob, test_meas_jacob);
    Matrix Q, R;
    matrix_init(&Q, 2, 2); matrix_init(&R, 1, 1);
    matrix_set(&Q, 0, 0, 0.01f); matrix_set(&Q, 1, 1, 0.01f); matrix_set(&R, 0, 0, 1.0f);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);
    ekf_set_update_method(&config, method);
    ekf_set_student_t_params(&config, 3.0f, 1.0f);
    ekf_set_adaptive_params(&config, 50.0f, 2.0f);

    Matrix x0, P0;
    matrix_init(&x0, 2, 1); matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 0); matrix_set(&x0, 1, 0, 1);
    matrix_set(&P0, 0, 0, 1); matrix_set(&P0, 1, 1, 1);
    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);

    Matrix u, z;
    matrix_init(&u, 1, 1); matrix_init(&z, 1, 1);
    /* 固定可复现的伪随机 */
    unsigned rng = 123u;
    double sse = 0.0;
    int N = 200;
    for (int i = 0; i < N; i++) {
        float truth = (float)i;             /* 速度=1 的匀速 */
        rng = rng * 1103515245u + 12345u;
        float r = (float)((rng >> 16) & 0x7fff) / 32767.0f - 0.5f;
        float obs = truth + r * 1.0f;
        if (with_outliers) {
            rng = rng * 1103515245u + 12345u;
            float p = (float)((rng >> 16) & 0x7fff) / 32767.0f;
            if (p < 0.15f) obs += (p < 0.075f ? -25.0f : 25.0f);
        }
        ekf_predict(&ekf, &config, &u, 1.0f);
        matrix_set(&z, 0, 0, obs);
        ekf_update(&ekf, &config, &z);
        float est = matrix_get(&ekf.x, 0, 0);
        double d = (double)est - truth;
        sse += d * d;
    }
    return (float)sqrt(sse / N);
}

/**
 * @brief 标准更新与 Joseph 更新对最优增益数学等价
 */
static void test_ekf_standard_equals_joseph(void) {
    printf("\nTest: Standard == Joseph (math identity)\n");
    float rs = run_method_rmse(EKF_UPDATE_STANDARD, 0);
    float rj = run_method_rmse(EKF_UPDATE_JOSEPH, 0);
    TEST_ASSERT(fabsf(rs - rj) < 1e-3f, "标准与 Joseph RMSE 一致");
}

/**
 * @brief Student-t / 自适应在野值场景下显著优于标准（回归：方向写反时会更差）
 */
static void test_ekf_robust_beats_standard(void) {
    printf("\nTest: Robust methods beat standard under outliers\n");
    float rstd = run_method_rmse(EKF_UPDATE_STANDARD, 1);
    float rstu = run_method_rmse(EKF_UPDATE_STUDENT_T, 1);
    float rada = run_method_rmse(EKF_UPDATE_ADAPTIVE, 1);
    printf("  (std=%.3f student-t=%.3f adaptive=%.3f)\n", rstd, rstu, rada);
    TEST_ASSERT(rstu < rstd, "Student-t 在野值下优于标准");
    TEST_ASSERT(rada < rstd, "自适应 在野值下优于标准");
}

/**
 * @brief 预测步状态函数输入/输出分离（回归：原 state_func(&x,..,&x,..) 混叠）
 *
 * 用一个“交换”状态函数：x_new=[x1, x0]。若输入输出共址且实现先写 x_new[0]
 * 再读 x[1]，结果会错。本框架使用独立 x_pred，应得到正确交换。
 */
static bool swap_state_func(const Matrix *x, const Matrix *u, Matrix *xn, float dt) {
    (void)u; (void)dt;
    matrix_set(xn, 0, 0, matrix_get(x, 1, 0));
    matrix_set(xn, 1, 0, matrix_get(x, 0, 0));
    return true;
}
static bool swap_state_jacob(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    (void)x; (void)u; (void)dt;
    matrix_set(F, 0, 0, 0); matrix_set(F, 0, 1, 1);
    matrix_set(F, 1, 0, 1); matrix_set(F, 1, 1, 0);
    return true;
}
static void test_ekf_predict_no_alias(void) {
    printf("\nTest: Predict input/output separation (regression)\n");
    EKF_Config config;
    ekf_config_init(&config, 2, 1);
    ekf_set_functions(&config, swap_state_func, test_meas_func,
                      swap_state_jacob, test_meas_jacob);
    Matrix Q, R;
    matrix_init(&Q, 2, 2); matrix_init(&R, 1, 1);
    ekf_set_process_noise(&config, &Q);
    ekf_set_measurement_noise(&config, &R);
    Matrix x0, P0;
    matrix_init(&x0, 2, 1); matrix_init(&P0, 2, 2);
    matrix_set(&x0, 0, 0, 3.0f); matrix_set(&x0, 1, 0, 7.0f);
    matrix_set(&P0, 0, 0, 1); matrix_set(&P0, 1, 1, 1);
    EKF_State ekf;
    ekf_state_init(&ekf, &config, &x0, &P0);
    Matrix u; matrix_init(&u, 1, 1);
    ekf_predict(&ekf, &config, &u, 1.0f);
    Matrix x; matrix_init(&x, 2, 1); ekf_get_state(&ekf, &x);
    TEST_ASSERT(fabsf(matrix_get(&x, 0, 0) - 7.0f) < 1e-6f, "predict 交换 x[0]<-7");
    TEST_ASSERT(fabsf(matrix_get(&x, 1, 0) - 3.0f) < 1e-6f, "predict 交换 x[1]<-3");
}

/**
 * @brief 枚举顺序守卫（回归：demo 把 UI 下标直接 cast 成枚举导致标签错乱）
 *
 * 任何依赖“下标==方法”的上层代码都必须遵守此顺序，否则对比图标签会乱。
 */
static void test_ekf_enum_order(void) {
    printf("\nTest: Update-method enum order guard\n");
    TEST_ASSERT(EKF_UPDATE_STANDARD  == 0, "STANDARD==0");
    TEST_ASSERT(EKF_UPDATE_JOSEPH    == 1, "JOSEPH==1");
    TEST_ASSERT(EKF_UPDATE_STUDENT_T == 2, "STUDENT_T==2");
    TEST_ASSERT(EKF_UPDATE_ADAPTIVE  == 3, "ADAPTIVE==3");
}

/* ========== 主函数 ========== */

int main(void) {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║              EKF框架单元测试                             ║\n");
    printf("║              面向模型失配的自适应EKF系统                   ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");

    /* 运行测试 */
    test_ekf_config_init();
    test_ekf_set_functions();
    test_ekf_noise();
    test_ekf_state_init();
    test_ekf_predict();
    test_ekf_update();
    test_ekf_full_cycle();
    test_ekf_update_methods();

    /* 回归测试（针对已修复的历史缺陷） */
    test_ekf_cov_no_collapse();
    test_ekf_standard_equals_joseph();
    test_ekf_robust_beats_standard();
    test_ekf_predict_no_alias();
    test_ekf_enum_order();

    /* 打印结果 */
    printf("\n══════════════════════════════════════════════════════════\n");
    printf("测试结果:\n");
    printf("  通过: %d\n", test_passed);
    printf("  失败: %d\n", test_failed);
    printf("  总计: %d\n", test_passed + test_failed);
    printf("══════════════════════════════════════════════════════════\n");

    return test_failed > 0 ? 1 : 0;
}
