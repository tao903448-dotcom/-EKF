/**
 * @file quaternion.h
 * @brief 单位四元数姿态运算（Hamilton 约定，[w,x,y,z]，body→world）
 *
 * 为四旋翼姿态 EKF 提供四元数基础运算：乘法、归一化、与旋转矩阵互换、
 * 向量旋转、小角度增量四元数、四元数间夹角（用于姿态误差度量）。
 *
 * 约定：
 *  - 四元数 q = [w, x, y, z]，表示从机体系(body)到导航系(world)的旋转 q_wb；
 *  - 旋转矩阵 R(q) 把机体系向量变换到世界系：v_world = R(q) · v_body；
 *  - 因此世界系向量到机体系：v_body = R(q)^T · v_world。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

/** 四元数 q ⊗ p（Hamilton 乘法），out 可与 a/b 别名 */
void quat_mul(float out[4], const float a[4], const float b[4]);

/** 归一化为单位四元数；零四元数时退化为 [1,0,0,0] */
void quat_normalize(float q[4]);

/** 由四元数构造旋转矩阵 R(q)（body→world），R 为行优先 3x3（9 元素） */
void quat_to_rotmat(float R[9], const float q[4]);

/** 用 q 把机体系向量旋到世界系：v_world = R(q)·v_body */
void quat_rotate(float v_world[3], const float q[4], const float v_body[3]);

/** 用 q 把世界系向量旋到机体系：v_body = R(q)^T·v_world */
void quat_rotate_inv(float v_body[3], const float q[4], const float v_world[3]);

/** 由旋转向量 θ（轴*角，rad）构造增量四元数（精确指数映射） */
void quat_from_rotvec(float dq[4], const float rotvec[3]);

/** 由 ZYX 欧拉角(roll φ, pitch θ, yaw ψ，rad)构造四元数 */
void quat_from_euler(float q[4], float roll, float pitch, float yaw);

/** 四元数转 ZYX 欧拉角(roll,pitch,yaw，rad) */
void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);

/**
 * 两个单位四元数所代表姿态之间的夹角（rad，∈[0,π]）。
 * 用于姿态估计误差度量：angle = 2·acos(|<a,b>|)。
 */
float quat_angle_between(const float a[4], const float b[4]);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H */
