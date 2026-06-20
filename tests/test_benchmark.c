/**
 * @file test_benchmark.c
 * @brief 矩阵运算性能基准测试
 *
 * 对比不同优化级别的矩阵运算性能：
 * - 标量版本（无优化）
 * - NEON版本（ARM SIMD优化）
 *
 * @author 软件杯团队
 * @date 2026-06-15
 */

#include "matrix.h"
#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

/* 测试配置 */
#define BENCHMARK_ITERATIONS  1000
#define MATRIX_SIZE_SMALL     4
#define MATRIX_SIZE_MEDIUM    8
#define MATRIX_SIZE_LARGE     16

/* 计时辅助函数 */
static double get_time_ms(void) {
    return (double)clock() / CLOCKS_PER_SEC * 1000.0;
}

/* 生成随机矩阵 */
static void fill_random(Matrix *mat) {
    for (int i = 0; i < mat->rows * mat->cols; i++) {
        mat->data[i] = (float)rand() / RAND_MAX * 10.0f - 5.0f;
    }
}

/* 测试矩阵加法性能 */
static void benchmark_matrix_add(uint8_t size) {
    Matrix a, b, c;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);
    matrix_init(&c, size, size);
    fill_random(&a);
    fill_random(&b);

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_add(&c, &a, &b);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵加法 (%dx%d): %.2f ms (%d次迭代)\n",
           size, size, elapsed, BENCHMARK_ITERATIONS);
}

/* 测试矩阵乘法性能 */
static void benchmark_matrix_mul(uint8_t size) {
    Matrix a, b, c;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);
    matrix_init(&c, size, size);
    fill_random(&a);
    fill_random(&b);

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_mul(&c, &a, &b);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵乘法 (%dx%d): %.2f ms (%d次迭代)\n",
           size, size, elapsed, BENCHMARK_ITERATIONS);
}

/* 测试矩阵转置性能 */
static void benchmark_matrix_transpose(uint8_t size) {
    Matrix a, b;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);
    fill_random(&a);

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_transpose(&b, &a);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵转置 (%dx%d): %.2f ms (%d次迭代)\n",
           size, size, elapsed, BENCHMARK_ITERATIONS);
}

/* 测试矩阵求逆性能 */
static void benchmark_matrix_inverse(uint8_t size) {
    Matrix a, b;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);

    /* 创建对角占优矩阵确保可逆 */
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i == j) {
                matrix_set(&a, i, j, 10.0f + (float)i);
            } else {
                matrix_set(&a, i, j, 0.1f * (float)(i + j));
            }
        }
    }

    int count = BENCHMARK_ITERATIONS / 10;  /* 求逆较慢，减少迭代次数 */
    double start = get_time_ms();
    for (int i = 0; i < count; i++) {
        matrix_inverse(&b, &a);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵求逆 (%dx%d): %.2f ms (%d次迭代)\n",
           size, size, elapsed, count);
}

int main(void) {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║           矩阵运算性能基准测试                            ║\n");
    printf("║           面向模型失配的自适应EKF系统                      ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    printf("✓ ARM NEON优化: 已启用\n\n");
#else
    printf("✗ ARM NEON优化: 未启用（当前为x86标量模式）\n");
    printf("  提示: 在ARM平台上编译可启用NEON优化\n\n");
#endif

    printf("测试配置:\n");
    printf("  迭代次数: %d\n", BENCHMARK_ITERATIONS);
    printf("  矩阵尺寸: %dx%d, %dx%d, %dx%d\n\n",
           MATRIX_SIZE_SMALL, MATRIX_SIZE_SMALL,
           MATRIX_SIZE_MEDIUM, MATRIX_SIZE_MEDIUM,
           MATRIX_SIZE_LARGE, MATRIX_SIZE_LARGE);

    /* 小矩阵测试 */
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("小矩阵性能测试 (%dx%d)\n", MATRIX_SIZE_SMALL, MATRIX_SIZE_SMALL);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    benchmark_matrix_add(MATRIX_SIZE_SMALL);
    benchmark_matrix_mul(MATRIX_SIZE_SMALL);
    benchmark_matrix_transpose(MATRIX_SIZE_SMALL);
    benchmark_matrix_inverse(MATRIX_SIZE_SMALL);

    /* 中等矩阵测试 */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("中等矩阵性能测试 (%dx%d)\n", MATRIX_SIZE_MEDIUM, MATRIX_SIZE_MEDIUM);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    benchmark_matrix_add(MATRIX_SIZE_MEDIUM);
    benchmark_matrix_mul(MATRIX_SIZE_MEDIUM);
    benchmark_matrix_transpose(MATRIX_SIZE_MEDIUM);
    benchmark_matrix_inverse(MATRIX_SIZE_MEDIUM);

    /* 大矩阵测试 */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("大矩阵性能测试 (%dx%d)\n", MATRIX_SIZE_LARGE, MATRIX_SIZE_LARGE);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    benchmark_matrix_add(MATRIX_SIZE_LARGE);
    benchmark_matrix_mul(MATRIX_SIZE_LARGE);
    benchmark_matrix_transpose(MATRIX_SIZE_LARGE);
    benchmark_matrix_inverse(MATRIX_SIZE_LARGE);

    printf("\n══════════════════════════════════════════════════════════\n");
    printf("基准测试完成！\n");
    printf("══════════════════════════════════════════════════════════\n");

    return 0;
}
