/**
 * @file test_benchmark_fixed.c
 * @brief 矩阵运算性能基准测试（修复版）
 *
 * 对比不同矩阵尺寸的运算性能
 *
 * @author 软件杯团队
 * @date 2026-06-16
 */

#include "matrix.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* 测试配置 - 减少迭代次数避免栈溢出 */
#define BENCHMARK_ITERATIONS  1000
#define MATRIX_SIZE_SMALL     4
#define MATRIX_SIZE_MEDIUM    8

/* 计时辅助函数 */
static double get_time_ms(void) {
    return (double)clock() / CLOCKS_PER_SEC * 1000.0;
}

/* 测试矩阵加法性能 */
static void benchmark_add(uint8_t size) {
    static Matrix a, b, c;  /* 使用静态变量避免栈溢出 */
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);
    matrix_init(&c, size, size);

    /* 填充测试数据 */
    for (int i = 0; i < size * size; i++) {
        a.data[i] = (float)(i % 10) * 0.1f;
        b.data[i] = (float)(i % 7) * 0.2f;
    }

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_add(&c, &a, &b);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵加法 (%dx%d): %.2f ms\n", size, size, elapsed);
}

/* 测试矩阵乘法性能 */
static void benchmark_mul(uint8_t size) {
    static Matrix a, b, c;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);
    matrix_init(&c, size, size);

    for (int i = 0; i < size * size; i++) {
        a.data[i] = (float)(i % 10) * 0.1f;
        b.data[i] = (float)(i % 7) * 0.2f;
    }

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_mul(&c, &a, &b);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵乘法 (%dx%d): %.2f ms\n", size, size, elapsed);
}

/* 测试矩阵转置性能 */
static void benchmark_transpose(uint8_t size) {
    static Matrix a, b;
    matrix_init(&a, size, size);
    matrix_init(&b, size, size);

    for (int i = 0; i < size * size; i++) {
        a.data[i] = (float)(i % 10) * 0.1f;
    }

    double start = get_time_ms();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        matrix_transpose(&b, &a);
    }
    double elapsed = get_time_ms() - start;

    printf("  矩阵转置 (%dx%d): %.2f ms\n", size, size, elapsed);
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
    printf("  迭代次数: %d\n\n", BENCHMARK_ITERATIONS);

    /* 小矩阵测试 */
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("小矩阵性能测试 (%dx%d)\n", MATRIX_SIZE_SMALL, MATRIX_SIZE_SMALL);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    benchmark_add(MATRIX_SIZE_SMALL);
    benchmark_mul(MATRIX_SIZE_SMALL);
    benchmark_transpose(MATRIX_SIZE_SMALL);

    /* 中等矩阵测试 */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("中等矩阵性能测试 (%dx%d)\n", MATRIX_SIZE_MEDIUM, MATRIX_SIZE_MEDIUM);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    benchmark_add(MATRIX_SIZE_MEDIUM);
    benchmark_mul(MATRIX_SIZE_MEDIUM);
    benchmark_transpose(MATRIX_SIZE_MEDIUM);

    printf("\n══════════════════════════════════════════════════════════\n");
    printf("基准测试完成！\n");
    printf("══════════════════════════════════════════════════════════\n");

    return 0;
}
