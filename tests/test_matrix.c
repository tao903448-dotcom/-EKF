/**
 * @file test_matrix.c
 * @brief 矩阵运算库单元测试
 *
 * 测试矩阵运算库的基本功能
 *
 * @author 软件杯团队
 * @date 2026-06-11
 */

#include "matrix.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

/* 测试辅助宏 */
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s\n", message); \
            return false; \
        } \
    } while(0)

#define TEST_PASS(message) \
    printf("PASS: %s\n", message)

/* ========== 测试用例 ========== */

/**
 * @brief 测试矩阵初始化
 */
bool test_matrix_init(void) {
    Matrix mat;

    /* 测试正常初始化 */
    TEST_ASSERT(matrix_init(&mat, 3, 3), "初始化3x3矩阵失败");
    TEST_ASSERT(mat.rows == 3, "行数不正确");
    TEST_ASSERT(mat.cols == 3, "列数不正确");

    /* 测试边界条件 */
    TEST_ASSERT(!matrix_init(NULL, 3, 3), "NULL指针应返回false");
    TEST_ASSERT(!matrix_init(&mat, 0, 3), "0行应返回false");
    TEST_ASSERT(!matrix_init(&mat, 3, 0), "0列应返回false");

    TEST_PASS("矩阵初始化测试");
    return true;
}

/**
 * @brief 测试单位矩阵
 */
bool test_matrix_eye(void) {
    Matrix mat;

    TEST_ASSERT(matrix_eye(&mat, 3), "创建3x3单位矩阵失败");

    /* 检查对角线元素 */
    for (uint8_t i = 0; i < 3; i++) {
        float val = matrix_get(&mat, i, i);
        TEST_ASSERT(fabsf(val - 1.0f) < 1e-6f, "对角线元素应为1");
    }

    /* 检查非对角线元素 */
    TEST_ASSERT(fabsf(matrix_get(&mat, 0, 1)) < 1e-6f, "非对角线元素应为0");
    TEST_ASSERT(fabsf(matrix_get(&mat, 1, 0)) < 1e-6f, "非对角线元素应为0");

    TEST_PASS("单位矩阵测试");
    return true;
}

/**
 * @brief 测试矩阵加法
 */
bool test_matrix_add(void) {
    Matrix a, b, result;

    /* 初始化矩阵A */
    float data_a[] = {1, 2, 3, 4};
    matrix_from_array(&a, 2, 2, data_a);

    /* 初始化矩阵B */
    float data_b[] = {5, 6, 7, 8};
    matrix_from_array(&b, 2, 2, data_b);

    /* 测试加法 */
    TEST_ASSERT(matrix_add(&result, &a, &b), "矩阵加法失败");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 0) - 6.0f) < 1e-6f, "加法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 1) - 8.0f) < 1e-6f, "加法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 0) - 10.0f) < 1e-6f, "加法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 1) - 12.0f) < 1e-6f, "加法结果错误");

    TEST_PASS("矩阵加法测试");
    return true;
}

/**
 * @brief 测试矩阵乘法
 */
bool test_matrix_mul(void) {
    Matrix a, b, result;

    /* 初始化矩阵A (2x3) */
    float data_a[] = {1, 2, 3, 4, 5, 6};
    matrix_from_array(&a, 2, 3, data_a);

    /* 初始化矩阵B (3x2) */
    float data_b[] = {7, 8, 9, 10, 11, 12};
    matrix_from_array(&b, 3, 2, data_b);

    /* 测试乘法 */
    TEST_ASSERT(matrix_mul(&result, &a, &b), "矩阵乘法失败");
    TEST_ASSERT(result.rows == 2, "结果行数错误");
    TEST_ASSERT(result.cols == 2, "结果列数错误");

    /* 验证结果 */
    /* [1 2 3] * [7 8]   = [58 64]
       [4 5 6]   [9 10]    [139 154]
                 [11 12] */
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 0) - 58.0f) < 1e-6f, "乘法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 1) - 64.0f) < 1e-6f, "乘法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 0) - 139.0f) < 1e-6f, "乘法结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 1) - 154.0f) < 1e-6f, "乘法结果错误");

    TEST_PASS("矩阵乘法测试");
    return true;
}

/**
 * @brief 测试矩阵转置
 */
bool test_matrix_transpose(void) {
    Matrix mat, result;

    /* 初始化矩阵 */
    float data[] = {1, 2, 3, 4, 5, 6};
    matrix_from_array(&mat, 2, 3, data);

    /* 测试转置 */
    TEST_ASSERT(matrix_transpose(&result, &mat), "矩阵转置失败");
    TEST_ASSERT(result.rows == 3, "转置后行数错误");
    TEST_ASSERT(result.cols == 2, "转置后列数错误");

    /* 验证结果 */
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 0) - 1.0f) < 1e-6f, "转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 1) - 4.0f) < 1e-6f, "转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 0) - 2.0f) < 1e-6f, "转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 1) - 5.0f) < 1e-6f, "转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 2, 0) - 3.0f) < 1e-6f, "转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&result, 2, 1) - 6.0f) < 1e-6f, "转置结果错误");

    TEST_PASS("矩阵转置测试");
    return true;
}

/**
 * @brief 测试矩阵求逆
 */
bool test_matrix_inverse(void) {
    Matrix mat, result, identity;

    /* 初始化矩阵 */
    float data[] = {4, 7, 2, 6};
    matrix_from_array(&mat, 2, 2, data);

    /* 测试求逆 */
    TEST_ASSERT(matrix_inverse(&result, &mat), "矩阵求逆失败");

    /* 验证 A * A^(-1) = I */
    TEST_ASSERT(matrix_mul(&identity, &mat, &result), "矩阵乘法失败");

    /* 检查是否接近单位矩阵 */
    TEST_ASSERT(fabsf(matrix_get(&identity, 0, 0) - 1.0f) < 1e-4f, "逆矩阵验证失败");
    TEST_ASSERT(fabsf(matrix_get(&identity, 0, 1)) < 1e-4f, "逆矩阵验证失败");
    TEST_ASSERT(fabsf(matrix_get(&identity, 1, 0)) < 1e-4f, "逆矩阵验证失败");
    TEST_ASSERT(fabsf(matrix_get(&identity, 1, 1) - 1.0f) < 1e-4f, "逆矩阵验证失败");

    TEST_PASS("矩阵求逆测试");
    return true;
}

/**
 * @brief 测试矩阵行列式
 */
bool test_matrix_determinant(void) {
    Matrix mat;
    float det;

    /* 测试2x2矩阵 */
    float data_2x2[] = {1, 2, 3, 4};
    matrix_from_array(&mat, 2, 2, data_2x2);
    TEST_ASSERT(matrix_determinant(&det, &mat), "计算行列式失败");
    TEST_ASSERT(fabsf(det - (-2.0f)) < 1e-6f, "2x2行列式计算错误");

    /* 测试3x3矩阵 */
    float data_3x3[] = {1, 2, 3, 0, 1, 4, 5, 6, 0};
    matrix_from_array(&mat, 3, 3, data_3x3);
    TEST_ASSERT(matrix_determinant(&det, &mat), "计算行列式失败");
    TEST_ASSERT(fabsf(det - 1.0f) < 1e-6f, "3x3行列式计算错误");

    TEST_PASS("矩阵行列式测试");
    return true;
}

/**
 * @brief 测试Cholesky分解
 */
bool test_matrix_cholesky(void) {
    Matrix mat, L;

    /* 对称正定矩阵 */
    float data[] = {4, 2, 2, 3};
    matrix_from_array(&mat, 2, 2, data);

    TEST_ASSERT(matrix_cholesky(&L, &mat), "Cholesky分解失败");

    /* 验证 L * L' = A */
    Matrix L_T, result;
    matrix_transpose(&L_T, &L);
    matrix_mul(&result, &L, &L_T);

    TEST_ASSERT(fabsf(matrix_get(&result, 0, 0) - 4.0f) < 1e-4f, "Cholesky验证失败");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 1) - 2.0f) < 1e-4f, "Cholesky验证失败");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 0) - 2.0f) < 1e-4f, "Cholesky验证失败");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 1) - 3.0f) < 1e-4f, "Cholesky验证失败");

    TEST_PASS("Cholesky分解测试");
    return true;
}

/* ========== 回归测试：针对历史缺陷 ========== */

/**
 * @brief 别名安全的矩阵乘法（回归：result 与输入同址会被误清零）
 *
 * 旧实现先 memset(result) 再读取，当 result==a 或 result==b 时结果归零，
 * 正是 EKF 协方差坍缩的根因。这里显式构造别名场景。
 */
bool test_matrix_mul_aliasing(void) {
    float da[] = {1, 2, 3, 4};
    float db[] = {5, 6, 7, 8};
    /* 参考：A*B = [[19,22],[43,50]] */
    Matrix a, b, ref;
    matrix_from_array(&a, 2, 2, da);
    matrix_from_array(&b, 2, 2, db);
    matrix_from_array(&ref, 2, 2, da);  /* 占位，随即覆盖 */
    matrix_mul(&ref, &a, &b);

    /* 情形 1：result == a */
    Matrix a1; matrix_from_array(&a1, 2, 2, da);
    Matrix b1; matrix_from_array(&b1, 2, 2, db);
    matrix_mul(&a1, &a1, &b1);
    for (int i = 0; i < 4; i++)
        TEST_ASSERT(fabsf(a1.data[i] - ref.data[i]) < 1e-4f, "mul 别名(result==a)结果错误");

    /* 情形 2：result == b */
    Matrix a2; matrix_from_array(&a2, 2, 2, da);
    Matrix b2; matrix_from_array(&b2, 2, 2, db);
    matrix_mul(&b2, &a2, &b2);
    for (int i = 0; i < 4; i++)
        TEST_ASSERT(fabsf(b2.data[i] - ref.data[i]) < 1e-4f, "mul 别名(result==b)结果错误");

    TEST_PASS("矩阵乘法别名安全测试");
    return true;
}

/**
 * @brief 别名安全的矩阵转置（回归：原地转置非方阵会破坏数据）
 */
bool test_matrix_transpose_aliasing(void) {
    float d[] = {1, 2, 3, 4, 5, 6};  /* 2x3 */
    Matrix m; matrix_from_array(&m, 2, 3, d);
    matrix_transpose(&m, &m);  /* 原地转置 -> 3x2 */

    TEST_ASSERT(m.rows == 3 && m.cols == 2, "原地转置维度错误");
    TEST_ASSERT(fabsf(matrix_get(&m, 0, 0) - 1.0f) < 1e-6f, "原地转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&m, 0, 1) - 4.0f) < 1e-6f, "原地转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&m, 2, 0) - 3.0f) < 1e-6f, "原地转置结果错误");
    TEST_ASSERT(fabsf(matrix_get(&m, 2, 1) - 6.0f) < 1e-6f, "原地转置结果错误");
    TEST_PASS("矩阵转置别名安全测试");
    return true;
}

/**
 * @brief 大矩阵求逆（回归：增广矩阵 n*2n 越界写 data[256]，n>=12 崩溃）
 *
 * 取 n=14 的对角占优矩阵，验证不越界且 A*A^-1 ≈ I。
 */
bool test_matrix_inverse_large(void) {
    const uint8_t n = 14;
    Matrix a; matrix_init(&a, n, n);
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            matrix_set(&a, i, j, (i == j) ? (float)(n + 1) : 0.5f);
        }
    }
    Matrix inv, prod;
    TEST_ASSERT(matrix_inverse(&inv, &a), "14x14 求逆失败");
    TEST_ASSERT(matrix_mul(&prod, &a, &inv), "14x14 验证乘法失败");
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            float expect = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT(fabsf(matrix_get(&prod, i, j) - expect) < 1e-3f,
                        "14x14 A*A^-1 偏离单位矩阵");
        }
    }
    TEST_PASS("大矩阵求逆（无越界）测试");
    return true;
}

/**
 * @brief NaN/Inf 守卫（回归：NaN<=0 恒 false 会被误判为成功）
 */
bool test_matrix_finite_guard(void) {
    float ok[] = {4, 1, 1, 3};
    Matrix m; matrix_from_array(&m, 2, 2, ok);
    TEST_ASSERT(matrix_is_finite(&m), "有限矩阵 matrix_is_finite=true");

    Matrix bad; matrix_from_array(&bad, 2, 2, ok);
    matrix_set(&bad, 0, 0, NAN);
    TEST_ASSERT(!matrix_is_finite(&bad), "含 NaN 矩阵 matrix_is_finite=false");

    Matrix inv, L;
    TEST_ASSERT(!matrix_inverse(&inv, &bad), "含 NaN 求逆应返回 false");
    TEST_ASSERT(!matrix_cholesky(&L, &bad), "含 NaN Cholesky 应返回 false");

    matrix_set(&bad, 0, 0, INFINITY);
    TEST_ASSERT(!matrix_is_finite(&bad), "含 Inf 矩阵 matrix_is_finite=false");

    TEST_PASS("NaN/Inf 守卫测试");
    return true;
}

/**
 * @brief 逐元素运算的步长视图正确性（回归：扁平索引会读错非连续视图）
 */
bool test_matrix_strided_view(void) {
    /* 2x4 缓冲: 行0=[1,2,|99,99] 行1=[3,4,|99,99]，stride=4 的 2x2 视图 */
    Matrix view;
    float buf[8] = {1, 2, 99, 99, 3, 4, 99, 99};
    memcpy(view.data, buf, sizeof buf);
    view.rows = 2; view.cols = 2; view.stride = 4;

    Matrix result;
    TEST_ASSERT(matrix_add(&result, &view, &view), "视图加法返回 true");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 0) - 2.0f) < 1e-6f, "视图加法 [0][0]=2");
    TEST_ASSERT(fabsf(matrix_get(&result, 0, 1) - 4.0f) < 1e-6f, "视图加法 [0][1]=4");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 0) - 6.0f) < 1e-6f, "视图加法 [1][0]=6");
    TEST_ASSERT(fabsf(matrix_get(&result, 1, 1) - 8.0f) < 1e-6f, "视图加法 [1][1]=8");

    Matrix sc;
    TEST_ASSERT(matrix_scale(&sc, &view, 10.0f), "视图数乘返回 true");
    TEST_ASSERT(fabsf(matrix_get(&sc, 1, 1) - 40.0f) < 1e-6f, "视图数乘 [1][1]=40");

    TEST_PASS("步长视图逐元素运算测试");
    return true;
}

/* ========== 主测试函数 ========== */

int main(void) {
    printf("========== 矩阵运算库测试 ==========\n\n");

    int passed = 0;
    int failed = 0;

    /* 运行所有测试 */
    if (test_matrix_init()) passed++; else failed++;
    if (test_matrix_eye()) passed++; else failed++;
    if (test_matrix_add()) passed++; else failed++;
    if (test_matrix_mul()) passed++; else failed++;
    if (test_matrix_transpose()) passed++; else failed++;
    if (test_matrix_inverse()) passed++; else failed++;
    if (test_matrix_determinant()) passed++; else failed++;
    if (test_matrix_cholesky()) passed++; else failed++;
    if (test_matrix_mul_aliasing()) passed++; else failed++;
    if (test_matrix_transpose_aliasing()) passed++; else failed++;
    if (test_matrix_inverse_large()) passed++; else failed++;
    if (test_matrix_finite_guard()) passed++; else failed++;
    if (test_matrix_strided_view()) passed++; else failed++;

    printf("\n========== 测试结果 ==========\n");
    printf("通过: %d\n", passed);
    printf("失败: %d\n", failed);
    printf("总计: %d\n", passed + failed);

    return (failed > 0) ? 1 : 0;
}