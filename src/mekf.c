/**
 * @file mekf.c
 * @brief 乘性误差状态 EKF（MEKF）实现 —— 姿态估计金标准对照
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "mekf.h"
#include "quaternion.h"
#include <string.h>
#include <math.h>

/* 与 attitude 模型一致的世界参考方向（单位向量） */
static const float G_W[3] = {0.0f, 0.0f, 1.0f};   /* 重力反作用方向(上) */
static const float M_W[3] = {1.0f, 0.0f, 0.0f};   /* 参考磁场方向(北)   */

/* 把 s·[v]× （3 向量的反对称/叉乘矩阵）写入 M 的 (r0,c0) 起 3×3 子块。
   [v]× = [[0,-vz,vy],[vz,0,-vx],[-vy,vx,0]] */
static void skew_into(Matrix *M, uint8_t r0, uint8_t c0, const float v[3], float s) {
    float *d = M->data; uint8_t st = M->stride;
    d[(r0+0)*st + c0+0] = 0;        d[(r0+0)*st + c0+1] = -s*v[2];  d[(r0+0)*st + c0+2] =  s*v[1];
    d[(r0+1)*st + c0+0] =  s*v[2];  d[(r0+1)*st + c0+1] = 0;        d[(r0+1)*st + c0+2] = -s*v[0];
    d[(r0+2)*st + c0+0] = -s*v[1];  d[(r0+2)*st + c0+1] =  s*v[0];  d[(r0+2)*st + c0+2] = 0;
}

bool mekf_init(MEKF *f, const float q0[4], float att_std, float bias_std) {
    if (f == NULL || q0 == NULL) return false;
    f->q[0]=q0[0]; f->q[1]=q0[1]; f->q[2]=q0[2]; f->q[3]=q0[3];
    quat_normalize(f->q);
    f->b[0]=f->b[1]=f->b[2]=0.0f;
    matrix_zeros(&f->P, 6, 6);
    float a2 = att_std*att_std, b2 = bias_std*bias_std;
    for (int i=0;i<3;i++) f->P.data[i*6+i] = a2;
    for (int i=3;i<6;i++) f->P.data[i*6+i] = b2;
    matrix_zeros(&f->Q, 6, 6);
    matrix_zeros(&f->R, 6, 6);
    return true;
}

bool mekf_set_noise(MEKF *f, const Matrix *Q6, const Matrix *R6) {
    if (f == NULL || Q6 == NULL || R6 == NULL) return false;
    if (Q6->rows!=6||Q6->cols!=6||R6->rows!=6||R6->cols!=6) return false;
    matrix_copy(&f->Q, Q6);
    matrix_copy(&f->R, R6);
    return true;
}

bool mekf_predict(MEKF *f, const float gyro[3], float dt) {
    if (f == NULL || gyro == NULL) return false;
    float omega[3] = { gyro[0]-f->b[0], gyro[1]-f->b[1], gyro[2]-f->b[2] };

    /* 标称姿态积分 */
    float rotvec[3] = { omega[0]*dt, omega[1]*dt, omega[2]*dt };
    float dq[4], qn[4];
    quat_from_rotvec(dq, rotvec);
    quat_mul(qn, f->q, dq);
    quat_normalize(qn);
    f->q[0]=qn[0]; f->q[1]=qn[1]; f->q[2]=qn[2]; f->q[3]=qn[3];

    /* 误差状态转移 F (6×6):
       [ I - [ω]×·dt   -I·dt ]
       [      0          I    ]  */
    Matrix F; matrix_eye(&F, 6);
    skew_into(&F, 0, 0, omega, -dt);              /* 先写 -[ω]×dt 到左上 */
    F.data[0*6+0]=1; F.data[1*6+1]=1; F.data[2*6+2]=1;  /* 加回对角 I（skew 对角为 0） */
    F.data[0*6+3]=-dt; F.data[1*6+4]=-dt; F.data[2*6+5]=-dt;   /* 右上 -I·dt */

    /* P = F P Fᵀ + Q */
    Matrix FP, Ft, FPFt;
    if (!matrix_mul(&FP, &F, &f->P)) return false;
    if (!matrix_transpose(&Ft, &F)) return false;
    if (!matrix_mul(&FPFt, &FP, &Ft)) return false;
    if (!matrix_add(&f->P, &FPFt, &f->Q)) return false;
    /* 对称化 */
    for (uint8_t i=0;i<6;i++) for (uint8_t j=(uint8_t)(i+1);j<6;j++){
        float a=0.5f*(f->P.data[i*6+j]+f->P.data[j*6+i]);
        f->P.data[i*6+j]=a; f->P.data[j*6+i]=a;
    }
    return true;
}

bool mekf_update(MEKF *f, const float accel_dir[3], const float mag_dir[3]) {
    if (f == NULL || accel_dir == NULL || mag_dir == NULL) return false;

    /* 预测观测：h = R(q)^T·ref（世界→机体） */
    float h_acc[3], h_mag[3];
    quat_rotate_inv(h_acc, f->q, G_W);
    quat_rotate_inv(h_mag, f->q, M_W);

    /* 观测雅可比 H (6×6)：对 δθ 为 +[h]×，对 δb 为 0
       （q←q⊗δq 时 h'≈h+[h]×δθ，故 ∂h/∂δθ = [h]×） */
    Matrix H; matrix_zeros(&H, 6, 6);
    skew_into(&H, 0, 0, h_acc, 1.0f);
    skew_into(&H, 3, 0, h_mag, 1.0f);

    /* 新息 y = z - h (6) */
    Matrix y; matrix_init(&y, 6, 1);
    y.data[0]=accel_dir[0]-h_acc[0]; y.data[1]=accel_dir[1]-h_acc[1]; y.data[2]=accel_dir[2]-h_acc[2];
    y.data[3]=mag_dir[0]-h_mag[0];   y.data[4]=mag_dir[1]-h_mag[1];   y.data[5]=mag_dir[2]-h_mag[2];

    /* S = H P Hᵀ + R */
    Matrix Ht, HP, S;
    if (!matrix_transpose(&Ht, &H)) return false;
    if (!matrix_mul(&HP, &H, &f->P)) return false;
    if (!matrix_mul(&S, &HP, &Ht)) return false;
    if (!matrix_add(&S, &S, &f->R)) return false;

    /* K = P Hᵀ S⁻¹（Cholesky 解 S Kᵀ = (P Hᵀ)ᵀ，退化回退求逆） */
    Matrix PHt, K;
    if (!matrix_mul(&PHt, &f->P, &Ht)) return false;     /* 6×6 */
    Matrix L;
    if (matrix_cholesky(&L, &S)) {
        Matrix PHt_t, Kt;
        if (!matrix_transpose(&PHt_t, &PHt)) return false;     /* (P Hᵀ)ᵀ */
        if (!matrix_cholesky_solve(&Kt, &L, &PHt_t)) return false;
        if (!matrix_transpose(&K, &Kt)) return false;
    } else {
        Matrix Sinv;
        if (!matrix_inverse(&Sinv, &S)) return false;
        if (!matrix_mul(&K, &PHt, &Sinv)) return false;
    }

    /* δx = K y (6) */
    Matrix dx;
    if (!matrix_mul(&dx, &K, &y)) return false;

    /* 注入：q ← q ⊗ δq(δθ)，b ← b + δb，误差归零 */
    float dtheta[3] = { dx.data[0], dx.data[1], dx.data[2] };
    float dqx[4], qn[4];
    quat_from_rotvec(dqx, dtheta);
    quat_mul(qn, f->q, dqx);
    quat_normalize(qn);
    f->q[0]=qn[0]; f->q[1]=qn[1]; f->q[2]=qn[2]; f->q[3]=qn[3];
    f->b[0]+=dx.data[3]; f->b[1]+=dx.data[4]; f->b[2]+=dx.data[5];

    /* P ← (I - K H) P，再对称化 */
    Matrix KH, I, IKH, Pn;
    if (!matrix_mul(&KH, &K, &H)) return false;
    matrix_eye(&I, 6);
    if (!matrix_sub(&IKH, &I, &KH)) return false;
    if (!matrix_mul(&Pn, &IKH, &f->P)) return false;
    matrix_copy(&f->P, &Pn);
    for (uint8_t i=0;i<6;i++) for (uint8_t j=(uint8_t)(i+1);j<6;j++){
        float a=0.5f*(f->P.data[i*6+j]+f->P.data[j*6+i]);
        f->P.data[i*6+j]=a; f->P.data[j*6+i]=a;
    }
    return true;
}

void mekf_get_quat(const MEKF *f, float q[4]) {
    q[0]=f->q[0]; q[1]=f->q[1]; q[2]=f->q[2]; q[3]=f->q[3];
}
void mekf_get_bias(const MEKF *f, float b[3]) {
    b[0]=f->b[0]; b[1]=f->b[1]; b[2]=f->b[2];
}
