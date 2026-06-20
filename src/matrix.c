/**
 * @file matrix.c
 * @brief 矩阵运算库实现（零动态分配，alias-safe，可选 ARM NEON 加速）
 *
 * 设计要点：
 *  - 零动态分配：所有矩阵使用编译期固定大小数组（见 matrix.h）。
 *  - alias-safe：matrix_mul / matrix_transpose / matrix_inverse 允许
 *    result 与输入指向同一对象，内部自动使用临时缓冲，避免数据竞争。
 *    （旧实现中 matrix_mul(&P, ..., &P) 会先 memset(P) 再读取 P，
 *      导致结果被清零，是 EKF 协方差坍缩的根因，现已修复。）
 *  - NEON 加速：核心逐元素 / 乘法 / 转置内核在定义 __ARM_NEON 时
 *    自动启用；标量实现始终作为可验证的回退路径。整个库只有一份
 *    matrix.c，不再有 matrix_neon.c 与之产生重复符号 / 缺函数问题。
 *
 * @author 软件杯团队（重构）
 * @date 2026-06-20
 */

#include "matrix.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* NEON 支持检测 */
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #include <arm_neon.h>
    #define HAS_NEON 1
#else
    #define HAS_NEON 0
#endif

/* 内部辅助宏：对 Matrix（行优先、stride==cols 的不变量）取索引 */
#define MATRIX_INDEX(mat, row, col) ((row) * (mat)->stride + (col))
#define MATRIX_VALID_INDEX(mat, row, col) \
    ((row) < (mat)->rows && (col) < (mat)->cols)

/* 数值阈值集中管理，便于审阅与调参 */
#define MATRIX_PIVOT_EPS    1e-10f   /* 主元/奇异判定 */
#define MATRIX_SYM_EPS      1e-5f    /* 对称性判定 */

/* ========== 矩阵创建和初始化 ========== */

bool matrix_init(Matrix *mat, uint8_t rows, uint8_t cols) {
    if (mat == NULL || rows == 0 || cols == 0 ||
        rows > MATRIX_MAX_ROWS || cols > MATRIX_MAX_COLS) {
        return false;
    }

    mat->rows = rows;
    mat->cols = cols;
    mat->stride = cols;  /* 不变量：普通 Matrix 始终连续存储 */
    memset(mat->data, 0, sizeof(float) * (size_t)rows * cols);

    return true;
}

bool matrix_zeros(Matrix *mat, uint8_t rows, uint8_t cols) {
    return matrix_init(mat, rows, cols);
}

bool matrix_eye(Matrix *mat, uint8_t size) {
    if (!matrix_init(mat, size, size)) {
        return false;
    }

    for (uint8_t i = 0; i < size; i++) {
        mat->data[MATRIX_INDEX(mat, i, i)] = 1.0f;
    }

    return true;
}

bool matrix_from_array(Matrix *mat, uint8_t rows, uint8_t cols, const float *data) {
    if (mat == NULL || data == NULL || rows == 0 || cols == 0 ||
        rows > MATRIX_MAX_ROWS || cols > MATRIX_MAX_COLS) {
        return false;
    }

    mat->rows = rows;
    mat->cols = cols;
    mat->stride = cols;
    memcpy(mat->data, data, sizeof(float) * (size_t)rows * cols);

    return true;
}

bool matrix_copy(Matrix *dst, const Matrix *src) {
    if (dst == NULL || src == NULL) {
        return false;
    }
    if (dst == src) {
        return true;  /* 自拷贝无操作 */
    }

    dst->rows = src->rows;
    dst->cols = src->cols;
    dst->stride = src->cols;  /* 规整为连续存储，维持不变量 */

    /* 逐行拷贝以兼容 src 可能是子矩阵视图（stride != cols）的情形 */
    for (uint8_t i = 0; i < src->rows; i++) {
        memcpy(&dst->data[(size_t)i * dst->stride],
               &src->data[(size_t)i * src->stride],
               sizeof(float) * src->cols);
    }

    return true;
}

/* ========== 基本矩阵运算（逐元素，result 可与输入别名） ========== */

bool matrix_add(Matrix *result, const Matrix *a, const Matrix *b) {
    if (result == NULL || a == NULL || b == NULL) {
        return false;
    }
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }

    result->rows = a->rows;
    result->cols = a->cols;
    result->stride = a->cols;

    uint32_t total = (uint32_t)a->rows * a->cols;
    uint32_t i = 0;

#if HAS_NEON
    uint32_t neon_count = total & ~3u;
    for (; i < neon_count; i += 4) {
        float32x4_t va = vld1q_f32(a->data + i);
        float32x4_t vb = vld1q_f32(b->data + i);
        vst1q_f32(result->data + i, vaddq_f32(va, vb));
    }
#endif
    for (; i < total; i++) {
        result->data[i] = a->data[i] + b->data[i];
    }
    return true;
}

bool matrix_sub(Matrix *result, const Matrix *a, const Matrix *b) {
    if (result == NULL || a == NULL || b == NULL) {
        return false;
    }
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }

    result->rows = a->rows;
    result->cols = a->cols;
    result->stride = a->cols;

    uint32_t total = (uint32_t)a->rows * a->cols;
    uint32_t i = 0;

#if HAS_NEON
    uint32_t neon_count = total & ~3u;
    for (; i < neon_count; i += 4) {
        float32x4_t va = vld1q_f32(a->data + i);
        float32x4_t vb = vld1q_f32(b->data + i);
        vst1q_f32(result->data + i, vsubq_f32(va, vb));
    }
#endif
    for (; i < total; i++) {
        result->data[i] = a->data[i] - b->data[i];
    }
    return true;
}

bool matrix_scale(Matrix *result, const Matrix *mat, float scalar) {
    if (result == NULL || mat == NULL) {
        return false;
    }

    result->rows = mat->rows;
    result->cols = mat->cols;
    result->stride = mat->cols;

    uint32_t total = (uint32_t)mat->rows * mat->cols;
    uint32_t i = 0;

#if HAS_NEON
    float32x4_t vs = vdupq_n_f32(scalar);
    uint32_t neon_count = total & ~3u;
    for (; i < neon_count; i += 4) {
        float32x4_t vm = vld1q_f32(mat->data + i);
        vst1q_f32(result->data + i, vmulq_f32(vm, vs));
    }
#endif
    for (; i < total; i++) {
        result->data[i] = mat->data[i] * scalar;
    }
    return true;
}

/* ---- 矩阵乘法内核：要求 dst 与 a/b 不别名，dst 为连续存储 ---- */
static void matrix_mul_kernel(float *dst, const Matrix *a, const Matrix *b) {
    uint8_t m = a->rows, k = a->cols, n = b->cols;
    memset(dst, 0, sizeof(float) * (size_t)m * n);

    for (uint8_t i = 0; i < m; i++) {
        float *drow = dst + (size_t)i * n;
        for (uint8_t kk = 0; kk < k; kk++) {
            float a_ik = a->data[MATRIX_INDEX(a, i, kk)];
            const float *brow = &b->data[MATRIX_INDEX(b, kk, 0)];
            uint8_t j = 0;
#if HAS_NEON
            float32x4_t va = vdupq_n_f32(a_ik);
            for (; j + 4 <= n; j += 4) {
                float32x4_t vb = vld1q_f32(brow + j);
                float32x4_t vr = vld1q_f32(drow + j);
                vst1q_f32(drow + j, vmlaq_f32(vr, va, vb));
            }
#endif
            for (; j < n; j++) {
                drow[j] += a_ik * brow[j];
            }
        }
    }
}

bool matrix_mul(Matrix *result, const Matrix *a, const Matrix *b) {
    if (result == NULL || a == NULL || b == NULL) {
        return false;
    }
    if (a->cols != b->rows) {
        return false;
    }

    uint8_t m = a->rows, n = b->cols;

    /* alias-safe：result 与任一输入同址时，先算到临时缓冲再拷回。
       这是修复 EKF 协方差坍缩 (P=(I-KH)P) 的根因所在。 */
    if (result == a || result == b) {
        float tmp[MATRIX_MAX_SIZE];
        matrix_mul_kernel(tmp, a, b);
        result->rows = m;
        result->cols = n;
        result->stride = n;
        memcpy(result->data, tmp, sizeof(float) * (size_t)m * n);
    } else {
        result->rows = m;
        result->cols = n;
        result->stride = n;
        matrix_mul_kernel(result->data, a, b);
    }
    return true;
}

bool matrix_transpose(Matrix *result, const Matrix *mat) {
    if (result == NULL || mat == NULL) {
        return false;
    }

    uint8_t r = mat->rows, c = mat->cols;

    /* alias-safe：原地转置非方阵会破坏数据，统一走临时缓冲 */
    if (result == mat) {
        float tmp[MATRIX_MAX_SIZE];
        for (uint8_t i = 0; i < r; i++) {
            for (uint8_t j = 0; j < c; j++) {
                tmp[(size_t)j * r + i] = mat->data[MATRIX_INDEX(mat, i, j)];
            }
        }
        result->rows = c;
        result->cols = r;
        result->stride = r;
        memcpy(result->data, tmp, sizeof(float) * (size_t)r * c);
        return true;
    }

    result->rows = c;
    result->cols = r;
    result->stride = r;
    for (uint8_t i = 0; i < r; i++) {
        for (uint8_t j = 0; j < c; j++) {
            result->data[MATRIX_INDEX(result, j, i)] =
                mat->data[MATRIX_INDEX(mat, i, j)];
        }
    }
    return true;
}

/* ========== 高级矩阵运算 ========== */

bool matrix_inverse(Matrix *result, const Matrix *mat) {
    if (result == NULL || mat == NULL || !matrix_is_square(mat)) {
        return false;
    }

    uint8_t n = mat->rows;
    uint8_t w = (uint8_t)(2 * n);

    /* 增广矩阵 [A | I] 存于独立缓冲：n*2n ≤ 16*32 = 512 floats。
       旧实现把它塞进 Matrix.data[256]，n≥12 时越界写栈——已修复。 */
    float aug[MATRIX_MAX_ROWS * 2 * MATRIX_MAX_COLS];

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug[(size_t)i * w + j] = mat->data[MATRIX_INDEX(mat, i, j)];
        }
        for (uint8_t j = n; j < w; j++) {
            aug[(size_t)i * w + j] = 0.0f;
        }
        aug[(size_t)i * w + (n + i)] = 1.0f;
    }

    /* 部分主元高斯-约旦消元 */
    for (uint8_t i = 0; i < n; i++) {
        float max_val = fabsf(aug[(size_t)i * w + i]);
        uint8_t max_row = i;
        for (uint8_t k = i + 1; k < n; k++) {
            float val = fabsf(aug[(size_t)k * w + i]);
            if (val > max_val) { max_val = val; max_row = k; }
        }

        if (max_val < MATRIX_PIVOT_EPS) {
            return false;  /* 奇异 */
        }

        if (max_row != i) {
            for (uint8_t j = 0; j < w; j++) {
                float t = aug[(size_t)i * w + j];
                aug[(size_t)i * w + j] = aug[(size_t)max_row * w + j];
                aug[(size_t)max_row * w + j] = t;
            }
        }

        float pivot = aug[(size_t)i * w + i];
        float inv_pivot = 1.0f / pivot;
        for (uint8_t j = 0; j < w; j++) {
            aug[(size_t)i * w + j] *= inv_pivot;
        }

        for (uint8_t k = 0; k < n; k++) {
            if (k != i) {
                float factor = aug[(size_t)k * w + i];
                if (factor != 0.0f) {
                    for (uint8_t j = 0; j < w; j++) {
                        aug[(size_t)k * w + j] -= factor * aug[(size_t)i * w + j];
                    }
                }
            }
        }
    }

    /* 提取逆矩阵（最后写 result，故 result==mat 也安全） */
    result->rows = n;
    result->cols = n;
    result->stride = n;
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            result->data[MATRIX_INDEX(result, i, j)] = aug[(size_t)i * w + (n + j)];
        }
    }
    return true;
}

bool matrix_determinant(float *result, const Matrix *mat) {
    if (result == NULL || mat == NULL || !matrix_is_square(mat)) {
        return false;
    }

    uint8_t n = mat->rows;

    if (n == 1) {
        *result = mat->data[0];
        return true;
    }
    if (n == 2) {
        *result = mat->data[0] * mat->data[3] - mat->data[1] * mat->data[2];
        return true;
    }

    /* 就地 LU（部分主元），用行交换次数定符号——O(n²)，无需查置换矩阵 */
    float a[MATRIX_MAX_SIZE];
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            a[(size_t)i * n + j] = mat->data[MATRIX_INDEX(mat, i, j)];
        }
    }

    float det = 1.0f;
    for (uint8_t i = 0; i < n; i++) {
        float max_val = fabsf(a[(size_t)i * n + i]);
        uint8_t max_row = i;
        for (uint8_t k = i + 1; k < n; k++) {
            float v = fabsf(a[(size_t)k * n + i]);
            if (v > max_val) { max_val = v; max_row = k; }
        }
        if (max_val < MATRIX_PIVOT_EPS) {
            *result = 0.0f;
            return true;  /* 奇异，行列式为 0 */
        }
        if (max_row != i) {
            for (uint8_t j = 0; j < n; j++) {
                float t = a[(size_t)i * n + j];
                a[(size_t)i * n + j] = a[(size_t)max_row * n + j];
                a[(size_t)max_row * n + j] = t;
            }
            det = -det;
        }
        float pivot = a[(size_t)i * n + i];
        det *= pivot;
        for (uint8_t k = i + 1; k < n; k++) {
            float factor = a[(size_t)k * n + i] / pivot;
            for (uint8_t j = i; j < n; j++) {
                a[(size_t)k * n + j] -= factor * a[(size_t)i * n + j];
            }
        }
    }

    *result = det;
    return true;
}

bool matrix_trace(float *result, const Matrix *mat) {
    if (result == NULL || mat == NULL || !matrix_is_square(mat)) {
        return false;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < mat->rows; i++) {
        sum += mat->data[MATRIX_INDEX(mat, i, i)];
    }
    *result = sum;
    return true;
}

/* ========== 特殊矩阵运算 ========== */

bool matrix_cholesky(Matrix *L, const Matrix *mat) {
    if (L == NULL || mat == NULL || !matrix_is_square(mat)) {
        return false;
    }
    if (!matrix_is_symmetric(mat)) {
        return false;
    }

    uint8_t n = mat->rows;
    matrix_zeros(L, n, n);

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            float sum = 0.0f;
            if (i == j) {
                for (uint8_t k = 0; k < j; k++) {
                    float l_jk = L->data[MATRIX_INDEX(L, j, k)];
                    sum += l_jk * l_jk;
                }
                float val = mat->data[MATRIX_INDEX(mat, j, j)] - sum;
                if (val <= 0.0f) {
                    return false;  /* 非正定 */
                }
                L->data[MATRIX_INDEX(L, j, j)] = sqrtf(val);
            } else {
                for (uint8_t k = 0; k < j; k++) {
                    sum += L->data[MATRIX_INDEX(L, i, k)] *
                           L->data[MATRIX_INDEX(L, j, k)];
                }
                L->data[MATRIX_INDEX(L, i, j)] =
                    (mat->data[MATRIX_INDEX(mat, i, j)] - sum) /
                    L->data[MATRIX_INDEX(L, j, j)];
            }
        }
    }
    return true;
}

bool matrix_lu(Matrix *L, Matrix *U, Matrix *P, const Matrix *mat) {
    if (L == NULL || U == NULL || P == NULL || mat == NULL ||
        !matrix_is_square(mat)) {
        return false;
    }

    uint8_t n = mat->rows;
    matrix_eye(P, n);
    matrix_copy(U, mat);
    matrix_eye(L, n);

    for (uint8_t i = 0; i < n; i++) {
        float max_val = fabsf(U->data[MATRIX_INDEX(U, i, i)]);
        uint8_t max_row = i;
        for (uint8_t k = i + 1; k < n; k++) {
            float val = fabsf(U->data[MATRIX_INDEX(U, k, i)]);
            if (val > max_val) { max_val = val; max_row = k; }
        }

        if (max_row != i) {
            for (uint8_t j = 0; j < n; j++) {
                float t = U->data[MATRIX_INDEX(U, i, j)];
                U->data[MATRIX_INDEX(U, i, j)] = U->data[MATRIX_INDEX(U, max_row, j)];
                U->data[MATRIX_INDEX(U, max_row, j)] = t;
            }
            for (uint8_t j = 0; j < i; j++) {
                float t = L->data[MATRIX_INDEX(L, i, j)];
                L->data[MATRIX_INDEX(L, i, j)] = L->data[MATRIX_INDEX(L, max_row, j)];
                L->data[MATRIX_INDEX(L, max_row, j)] = t;
            }
            for (uint8_t j = 0; j < n; j++) {
                float t = P->data[MATRIX_INDEX(P, i, j)];
                P->data[MATRIX_INDEX(P, i, j)] = P->data[MATRIX_INDEX(P, max_row, j)];
                P->data[MATRIX_INDEX(P, max_row, j)] = t;
            }
        }

        if (fabsf(U->data[MATRIX_INDEX(U, i, i)]) < MATRIX_PIVOT_EPS) {
            return false;
        }

        for (uint8_t j = i + 1; j < n; j++) {
            L->data[MATRIX_INDEX(L, j, i)] =
                U->data[MATRIX_INDEX(U, j, i)] / U->data[MATRIX_INDEX(U, i, i)];
            for (uint8_t k = i; k < n; k++) {
                U->data[MATRIX_INDEX(U, j, k)] -=
                    L->data[MATRIX_INDEX(L, j, i)] * U->data[MATRIX_INDEX(U, i, k)];
            }
        }
    }
    return true;
}

/* ========== 矩阵元素访问 ========== */

float matrix_get(const Matrix *mat, uint8_t row, uint8_t col) {
    if (mat == NULL || !MATRIX_VALID_INDEX(mat, row, col)) {
        return 0.0f;
    }
    return mat->data[MATRIX_INDEX(mat, row, col)];
}

void matrix_set(Matrix *mat, uint8_t row, uint8_t col, float value) {
    if (mat != NULL && MATRIX_VALID_INDEX(mat, row, col)) {
        mat->data[MATRIX_INDEX(mat, row, col)] = value;
    }
}

bool matrix_diag(float *result, const Matrix *mat) {
    if (result == NULL || mat == NULL || !matrix_is_square(mat)) {
        return false;
    }
    for (uint8_t i = 0; i < mat->rows; i++) {
        result[i] = mat->data[MATRIX_INDEX(mat, i, i)];
    }
    return true;
}

/* ========== 矩阵视图操作 ========== */

bool matrix_view_create(MatrixView *view, const Matrix *mat,
                       uint8_t row_start, uint8_t col_start,
                       uint8_t rows, uint8_t cols) {
    if (view == NULL || mat == NULL) {
        return false;
    }
    if ((uint16_t)row_start + rows > mat->rows ||
        (uint16_t)col_start + cols > mat->cols) {
        return false;
    }

    view->data = (float *)&mat->data[MATRIX_INDEX(mat, row_start, col_start)];
    view->rows = rows;
    view->cols = cols;
    view->stride = mat->stride;
    return true;
}

/* ========== 工具函数 ========== */

void matrix_print(const Matrix *mat, const char *name) {
    if (mat == NULL) {
        return;
    }
    if (name != NULL) {
        printf("%s (%dx%d):\n", name, mat->rows, mat->cols);
    }
    for (uint8_t i = 0; i < mat->rows; i++) {
        printf("[");
        for (uint8_t j = 0; j < mat->cols; j++) {
            printf("%8.4f", mat->data[MATRIX_INDEX(mat, i, j)]);
            if (j < mat->cols - 1) {
                printf(", ");
            }
        }
        printf("]\n");
    }
    printf("\n");
}

bool matrix_is_square(const Matrix *mat) {
    if (mat == NULL) {
        return false;
    }
    return mat->rows == mat->cols;
}

bool matrix_is_symmetric(const Matrix *mat) {
    if (mat == NULL || !matrix_is_square(mat)) {
        return false;
    }
    for (uint8_t i = 0; i < mat->rows; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < mat->cols; j++) {
            float diff = fabsf(mat->data[MATRIX_INDEX(mat, i, j)] -
                              mat->data[MATRIX_INDEX(mat, j, i)]);
            if (diff > MATRIX_SYM_EPS) {
                return false;
            }
        }
    }
    return true;
}

bool matrix_dims_match(const Matrix *a, const Matrix *b) {
    if (a == NULL || b == NULL) {
        return false;
    }
    return (a->rows == b->rows) && (a->cols == b->cols);
}
