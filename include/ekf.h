/**
 * @file ekf.h
 * @brief 扩展卡尔曼滤波器（EKF）框架
 *
 * 实现面向模型失配的自适应EKF系统：
 * - 标准EKF预测和更新
 * - Student-t鲁棒更新
 * - Joseph形式协方差更新
 * - 自适应噪声估计
 *
 * @author 软件杯团队
 * @date 2026-06-11
 */

#ifndef EKF_H
#define EKF_H

#include "matrix.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* EKF维度配置 */
#define EKF_MAX_STATE_DIM       16
#define EKF_MAX_MEASUREMENT_DIM 16

/**
 * @brief EKF状态函数类型
 * @param x 状态向量
 * @param u 控制输入
 * @param x_new 预测状态
 * @param dt 时间步长
 */
typedef bool (*EKF_StateFunc)(const Matrix *x, const Matrix *u,
                             Matrix *x_new, float dt);

/**
 * @brief EKF观测函数类型
 * @param x 状态向量
 * @param z 观测向量
 */
typedef bool (*EKF_MeasurementFunc)(const Matrix *x, Matrix *z);

/**
 * @brief EKF雅可比矩阵函数类型
 * @param x 状态向量
 * @param u 控制输入
 * @param F 状态转移雅可比矩阵
 * @param dt 时间步长
 */
typedef bool (*EKF_StateJacobianFunc)(const Matrix *x, const Matrix *u,
                                     Matrix *F, float dt);

/**
 * @brief EKF观测雅可比矩阵函数类型
 * @param x 状态向量
 * @param H 观测雅可比矩阵
 */
typedef bool (*EKF_MeasurementJacobianFunc)(const Matrix *x, Matrix *H);

/**
 * @brief 状态归一化函数类型（可选）
 *
 * 用于带约束的状态（如单位四元数姿态）：每次 predict / update 之后，
 * 框架会调用它把状态投影回约束流形（例如把四元数重新归一化）。
 * 若为 NULL 则不做任何处理（普通无约束 EKF）。
 *
 * @param x 状态向量（原地修改）
 */
typedef void (*EKF_StateNormalizeFunc)(Matrix *x);

/**
 * @brief 鲁棒更新方法
 */
typedef enum {
    EKF_UPDATE_STANDARD = 0,    /**< 标准卡尔曼更新 */
    EKF_UPDATE_JOSEPH,          /**< Joseph形式更新 */
    EKF_UPDATE_STUDENT_T,       /**< Student-t鲁棒更新 */
    EKF_UPDATE_ADAPTIVE         /**< 自适应更新 */
} EKF_UpdateMethod;

/**
 * @brief EKF配置结构体
 */
typedef struct {
    uint8_t state_dim;              /**< 状态维度 */
    uint8_t measurement_dim;        /**< 观测维度 */
    EKF_UpdateMethod update_method; /**< 更新方法 */

    /* 函数指针 */
    EKF_StateFunc state_func;                       /**< 状态转移函数 */
    EKF_MeasurementFunc measurement_func;           /**< 观测函数 */
    EKF_StateJacobianFunc state_jacobian_func;      /**< 状态雅可比函数 */
    EKF_MeasurementJacobianFunc measurement_jacobian_func; /**< 观测雅可比函数 */
    EKF_StateNormalizeFunc state_normalize_func;    /**< 状态归一化函数（可选，如四元数） */

    /* 噪声参数 */
    Matrix Q;   /**< 过程噪声协方差 */
    Matrix R;   /**< 观测噪声协方差 */

    /* Student-t参数 */
    float student_t_nu;     /**< 自由度参数 */
    float student_t_delta;  /**< 尺度参数 */

    /* 自适应参数 */
    float adaptive_window;  /**< 自适应窗口大小 */
    float adaptive_factor;  /**< 自适应因子 */
} EKF_Config;

/**
 * @brief EKF状态结构体
 */
typedef struct {
    Matrix x;   /**< 状态向量 */
    Matrix x_pred; /**< 预测状态向量（与 x 分离，避免状态函数原地混叠） */
    Matrix P;   /**< 状态协方差矩阵 */
    Matrix S;   /**< 新息协方差矩阵 */
    Matrix K;   /**< 卡尔曼增益矩阵 */
    Matrix y;   /**< 新息向量 */
    Matrix z;   /**< 观测向量 */
    Matrix z_pred; /**< 预测观测向量 */

    /* 临时矩阵（用于计算） */
    Matrix F;   /**< 状态转移雅可比矩阵 */
    Matrix H;   /**< 观测雅可比矩阵 */
    Matrix P_pred; /**< 预测协方差矩阵 */
    Matrix temp1;  /**< 临时矩阵1 */
    Matrix temp2;  /**< 临时矩阵2 */
    Matrix temp3;  /**< 临时矩阵3 */
    Matrix temp4;  /**< 临时矩阵4（协方差更新用，避免大栈帧） */
    Matrix temp5;  /**< 临时矩阵5（协方差更新用，避免大栈帧） */
    Matrix S_inv;  /**< 新息协方差之逆（预分配，避免栈上 1KB 局部） */
    Matrix R_eff;  /**< 重标定后的观测噪声（鲁棒/自适应用） */

    /* 状态标志 */
    bool initialized;   /**< 是否已初始化 */
    uint32_t step_count; /**< 步数计数器 */
} EKF_State;

/**
 * @brief 初始化EKF配置
 * @param config 配置指针
 * @param state_dim 状态维度
 * @param measurement_dim 观测维度
 * @return 成功返回true，失败返回false
 */
bool ekf_config_init(EKF_Config *config, uint8_t state_dim, uint8_t measurement_dim);

/**
 * @brief 设置EKF函数
 * @param config 配置指针
 * @param state_func 状态转移函数
 * @param measurement_func 观测函数
 * @param state_jacobian 状态雅可比函数
 * @param measurement_jacobian 观测雅可比函数
 */
void ekf_set_functions(EKF_Config *config,
                      EKF_StateFunc state_func,
                      EKF_MeasurementFunc measurement_func,
                      EKF_StateJacobianFunc state_jacobian,
                      EKF_MeasurementJacobianFunc measurement_jacobian);

/**
 * @brief 设置过程噪声协方差
 * @param config 配置指针
 * @param Q 过程噪声协方差矩阵
 * @return 成功返回true，失败返回false
 */
bool ekf_set_process_noise(EKF_Config *config, const Matrix *Q);

/**
 * @brief 设置观测噪声协方差
 * @param config 配置指针
 * @param R 观测噪声协方差矩阵
 * @return 成功返回true，失败返回false
 */
bool ekf_set_measurement_noise(EKF_Config *config, const Matrix *R);

/**
 * @brief 设置Student-t参数
 * @param config 配置指针
 * @param nu 自由度参数
 * @param delta 尺度参数
 */
void ekf_set_student_t_params(EKF_Config *config, float nu, float delta);

/**
 * @brief 设置自适应参数
 * @param config 配置指针
 * @param window_size 窗口大小
 * @param factor 自适应因子
 */
void ekf_set_adaptive_params(EKF_Config *config, float window_size, float factor);

/**
 * @brief 设置更新方法
 * @param config 配置指针
 * @param method 更新方法
 */
void ekf_set_update_method(EKF_Config *config, EKF_UpdateMethod method);

/**
 * @brief 设置状态归一化函数（可选，用于四元数等带约束状态）
 * @param config 配置指针
 * @param fn 归一化回调；传 NULL 取消
 */
void ekf_set_state_normalize(EKF_Config *config, EKF_StateNormalizeFunc fn);

/**
 * @brief 初始化EKF状态
 * @param state 状态指针
 * @param config 配置指针
 * @param x0 初始状态向量
 * @param P0 初始协方差矩阵
 * @return 成功返回true，失败返回false
 */
bool ekf_state_init(EKF_State *state, const EKF_Config *config,
                   const Matrix *x0, const Matrix *P0);

/**
 * @brief EKF预测步骤
 * @param state 状态指针
 * @param config 配置指针
 * @param u 控制输入
 * @param dt 时间步长
 * @return 成功返回true，失败返回false
 */
bool ekf_predict(EKF_State *state, const EKF_Config *config,
                const Matrix *u, float dt);

/**
 * @brief EKF更新步骤
 * @param state 状态指针
 * @param config 配置指针
 * @param z 观测向量
 * @return 成功返回true，失败返回false
 */
bool ekf_update(EKF_State *state, const EKF_Config *config, const Matrix *z);

/**
 * @brief 获取EKF状态估计
 * @param state 状态指针
 * @param x 状态向量输出
 * @return 成功返回true，失败返回false
 */
bool ekf_get_state(const EKF_State *state, Matrix *x);

/**
 * @brief 获取EKF协方差
 * @param state 状态指针
 * @param P 协方差矩阵输出
 * @return 成功返回true，失败返回false
 */
bool ekf_get_covariance(const EKF_State *state, Matrix *P);

/**
 * @brief 获取新息向量
 * @param state 状态指针
 * @param y 新息向量输出
 * @return 成功返回true，失败返回false
 */
bool ekf_get_innovation(const EKF_State *state, Matrix *y);

/**
 * @brief 获取卡尔曼增益
 * @param state 状态指针
 * @param K 卡尔曼增益输出
 * @return 成功返回true，失败返回false
 */
bool ekf_get_kalman_gain(const EKF_State *state, Matrix *K);

/**
 * @brief 重置EKF状态
 * @param state 状态指针
 */
void ekf_reset(EKF_State *state);

/**
 * @brief 打印EKF状态（调试用）
 * @param state 状态指针
 */
void ekf_print_state(const EKF_State *state);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */