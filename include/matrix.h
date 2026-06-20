/**
 * @file matrix.h
 * @brief 矩阵运算库 - 面向嵌入式环境的零动态分配矩阵运算
 *
 * 本库实现了高性能的矩阵运算，专为嵌入式实时系统设计：
 * - 零动态分配：所有内存在编译时预分配
 * - ARM NEON优化：定义 __ARM_NEON 时核心运算自动走 SIMD 路径，
 *   否则使用标量回退；标量与 NEON 同处一份 matrix.c，无重复符号。
 * - alias-safe：matrix_mul / matrix_transpose / matrix_inverse 允许
 *   result 与输入指向同一对象（如 matrix_mul(&P, &A, &P)），结果正确。
 *
 * @author 软件杯团队（重构）
 * @date 2026-06-20
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 矩阵维度配置 - 根据实际需求调整 */
#define MATRIX_MAX_ROWS     16
#define MATRIX_MAX_COLS     16
#define MATRIX_MAX_SIZE     (MATRIX_MAX_ROWS * MATRIX_MAX_COLS)

/* 内存对齐要求（字节） */
#define MATRIX_ALIGNMENT    16

/**
 * @brief 矩阵结构体
 *
 * 使用固定大小数组，避免动态内存分配
 * 数据按行优先存储
 */
typedef struct {
    float data[MATRIX_MAX_SIZE];    /**< 矩阵数据 */
    uint8_t rows;                   /**< 行数 */
    uint8_t cols;                   /**< 列数 */
    uint8_t stride;                 /**< 行步长（用于子矩阵） */
} Matrix;

/**
 * @brief 矩阵视图（不拥有数据）
 */
typedef struct {
    float *data;                    /**< 数据指针 */
    uint8_t rows;                   /**< 行数 */
    uint8_t cols;                   /**< 列数 */
    uint8_t stride;                 /**< 行步长 */
} MatrixView;

/* ========== 矩阵创建和初始化 ========== */

/**
 * @brief 初始化矩阵
 * @param mat 矩阵指针
 * @param rows 行数
 * @param cols 列数
 * @return 成功返回true，失败返回false
 */
bool matrix_init(Matrix *mat, uint8_t rows, uint8_t cols);

/**
 * @brief 创建零矩阵
 * @param mat 矩阵指针
 * @param rows 行数
 * @param cols 列数
 * @return 成功返回true，失败返回false
 */
bool matrix_zeros(Matrix *mat, uint8_t rows, uint8_t cols);

/**
 * @brief 创建单位矩阵
 * @param mat 矩阵指针
 * @param size 矩阵大小（方阵）
 * @return 成功返回true，失败返回false
 */
bool matrix_eye(Matrix *mat, uint8_t size);

/**
 * @brief 从数组初始化矩阵
 * @param mat 矩阵指针
 * @param rows 行数
 * @param cols 列数
 * @param data 数据数组
 * @return 成功返回true，失败返回false
 */
bool matrix_from_array(Matrix *mat, uint8_t rows, uint8_t cols, const float *data);

/**
 * @brief 复制矩阵
 * @param dst 目标矩阵
 * @param src 源矩阵
 * @return 成功返回true，失败返回false
 */
bool matrix_copy(Matrix *dst, const Matrix *src);

/* ========== 基本矩阵运算 ========== */

/**
 * @brief 矩阵加法
 * @param result 结果矩阵
 * @param a 矩阵A
 * @param b 矩阵B
 * @return 成功返回true，失败返回false
 */
bool matrix_add(Matrix *result, const Matrix *a, const Matrix *b);

/**
 * @brief 矩阵减法
 * @param result 结果矩阵
 * @param a 矩阵A
 * @param b 矩阵B
 * @return 成功返回true，失败返回false
 */
bool matrix_sub(Matrix *result, const Matrix *a, const Matrix *b);

/**
 * @brief 矩阵乘法
 * @param result 结果矩阵
 * @param a 矩阵A
 * @param b 矩阵B
 * @return 成功返回true，失败返回false
 */
bool matrix_mul(Matrix *result, const Matrix *a, const Matrix *b);

/**
 * @brief 矩阵标量乘法
 * @param result 结果矩阵
 * @param mat 输入矩阵
 * @param scalar 标量值
 * @return 成功返回true，失败返回false
 */
bool matrix_scale(Matrix *result, const Matrix *mat, float scalar);

/**
 * @brief 矩阵转置
 * @param result 结果矩阵
 * @param mat 输入矩阵
 * @return 成功返回true，失败返回false
 */
bool matrix_transpose(Matrix *result, const Matrix *mat);

/* ========== 高级矩阵运算 ========== */

/**
 * @brief 矩阵求逆（使用LU分解）
 * @param result 结果矩阵
 * @param mat 输入矩阵（必须是方阵）
 * @return 成功返回true，失败返回false（矩阵奇异）
 */
bool matrix_inverse(Matrix *result, const Matrix *mat);

/**
 * @brief 矩阵行列式
 * @param result 结果值
 * @param mat 输入矩阵（必须是方阵）
 * @return 成功返回true，失败返回false
 */
bool matrix_determinant(float *result, const Matrix *mat);

/**
 * @brief 矩阵迹（对角线元素之和）
 * @param result 结果值
 * @param mat 输入矩阵（必须是方阵）
 * @return 成功返回true，失败返回false
 */
bool matrix_trace(float *result, const Matrix *mat);

/* ========== 特殊矩阵运算 ========== */

/**
 * @brief 矩阵Cholesky分解
 * @param L 下三角矩阵结果
 * @param mat 输入对称正定矩阵
 * @return 成功返回true，失败返回false
 */
bool matrix_cholesky(Matrix *L, const Matrix *mat);

/**
 * @brief 矩阵LU分解
 * @param L 下三角矩阵
 * @param U 上三角矩阵
 * @param P 置换矩阵
 * @param mat 输入矩阵
 * @return 成功返回true，失败返回false
 */
bool matrix_lu(Matrix *L, Matrix *U, Matrix *P, const Matrix *mat);

/* ========== 矩阵元素访问 ========== */

/**
 * @brief 获取矩阵元素
 * @param mat 矩阵指针
 * @param row 行索引
 * @param col 列索引
 * @return 元素值
 */
float matrix_get(const Matrix *mat, uint8_t row, uint8_t col);

/**
 * @brief 设置矩阵元素
 * @param mat 矩阵指针
 * @param row 行索引
 * @param col 列索引
 * @param value 元素值
 */
void matrix_set(Matrix *mat, uint8_t row, uint8_t col, float value);

/**
 * @brief 获取矩阵对角线元素
 * @param result 结果向量
 * @param mat 输入矩阵
 * @return 成功返回true，失败返回false
 */
bool matrix_diag(float *result, const Matrix *mat);

/* ========== 矩阵视图操作 ========== */

/**
 * @brief 创建矩阵视图
 * @param view 视图指针
 * @param mat 源矩阵
 * @param row_start 起始行
 * @param col_start 起始列
 * @param rows 行数
 * @param cols 列数
 * @return 成功返回true，失败返回false
 */
bool matrix_view_create(MatrixView *view, const Matrix *mat,
                       uint8_t row_start, uint8_t col_start,
                       uint8_t rows, uint8_t cols);

/* ========== 工具函数 ========== */

/**
 * @brief 打印矩阵（调试用）
 * @param mat 矩阵指针
 * @param name 矩阵名称
 */
void matrix_print(const Matrix *mat, const char *name);

/**
 * @brief 检查矩阵是否为方阵
 * @param mat 矩阵指针
 * @return 是方阵返回true，否则返回false
 */
bool matrix_is_square(const Matrix *mat);

/**
 * @brief 检查矩阵是否对称
 * @param mat 矩阵指针
 * @return 是对称矩阵返回true，否则返回false
 */
bool matrix_is_symmetric(const Matrix *mat);

/**
 * @brief 检查矩阵维度是否匹配
 * @param a 矩阵A
 * @param b 矩阵B
 * @return 匹配返回true，否则返回false
 */
bool matrix_dims_match(const Matrix *a, const Matrix *b);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_H */