/**
 * @file quaternion.c
 * @brief 单位四元数姿态运算实现（Hamilton 约定，body→world）
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "quaternion.h"
#include <math.h>

void quat_mul(float out[4], const float a[4], const float b[4]) {
    float aw = a[0], ax = a[1], ay = a[2], az = a[3];
    float bw = b[0], bx = b[1], by = b[2], bz = b[3];
    float w = aw * bw - ax * bx - ay * by - az * bz;
    float x = aw * bx + ax * bw + ay * bz - az * by;
    float y = aw * by - ax * bz + ay * bw + az * bx;
    float z = aw * bz + ax * by - ay * bx + az * bw;
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void quat_normalize(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12f) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void quat_to_rotmat(float R[9], const float q[4]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    /* R(q) : body -> world，行优先 */
    R[0] = 1.0f - 2.0f*(yy + zz);
    R[1] = 2.0f*(xy - wz);
    R[2] = 2.0f*(xz + wy);
    R[3] = 2.0f*(xy + wz);
    R[4] = 1.0f - 2.0f*(xx + zz);
    R[5] = 2.0f*(yz - wx);
    R[6] = 2.0f*(xz - wy);
    R[7] = 2.0f*(yz + wx);
    R[8] = 1.0f - 2.0f*(xx + yy);
}

void quat_rotate(float v_world[3], const float q[4], const float v_body[3]) {
    float R[9];
    quat_to_rotmat(R, q);
    v_world[0] = R[0]*v_body[0] + R[1]*v_body[1] + R[2]*v_body[2];
    v_world[1] = R[3]*v_body[0] + R[4]*v_body[1] + R[5]*v_body[2];
    v_world[2] = R[6]*v_body[0] + R[7]*v_body[1] + R[8]*v_body[2];
}

void quat_rotate_inv(float v_body[3], const float q[4], const float v_world[3]) {
    float R[9];
    quat_to_rotmat(R, q);
    /* v_body = R^T · v_world */
    v_body[0] = R[0]*v_world[0] + R[3]*v_world[1] + R[6]*v_world[2];
    v_body[1] = R[1]*v_world[0] + R[4]*v_world[1] + R[7]*v_world[2];
    v_body[2] = R[2]*v_world[0] + R[5]*v_world[1] + R[8]*v_world[2];
}

void quat_from_rotvec(float dq[4], const float rotvec[3]) {
    float angle = sqrtf(rotvec[0]*rotvec[0] + rotvec[1]*rotvec[1] + rotvec[2]*rotvec[2]);
    if (angle < 1e-9f) {
        /* 小角度：dq ≈ [1, 0.5θ] 并归一化 */
        dq[0] = 1.0f;
        dq[1] = 0.5f * rotvec[0];
        dq[2] = 0.5f * rotvec[1];
        dq[3] = 0.5f * rotvec[2];
        quat_normalize(dq);
        return;
    }
    float half = 0.5f * angle;
    float s = sinf(half) / angle;
    dq[0] = cosf(half);
    dq[1] = rotvec[0] * s;
    dq[2] = rotvec[1] * s;
    dq[3] = rotvec[2] * s;
}

void quat_from_euler(float q[4], float roll, float pitch, float yaw) {
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);
    /* ZYX：q = qz(yaw) ⊗ qy(pitch) ⊗ qx(roll) */
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
    quat_normalize(q);
}

void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    /* roll (x) */
    float sinr = 2.0f*(w*x + y*z);
    float cosr = 1.0f - 2.0f*(x*x + y*y);
    if (roll)  *roll = atan2f(sinr, cosr);
    /* pitch (y)，含 ±90° 边界保护 */
    float sinp = 2.0f*(w*y - z*x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    if (pitch) *pitch = asinf(sinp);
    /* yaw (z) */
    float siny = 2.0f*(w*z + x*y);
    float cosy = 1.0f - 2.0f*(y*y + z*z);
    if (yaw)   *yaw = atan2f(siny, cosy);
}

float quat_angle_between(const float a[4], const float b[4]) {
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    dot = fabsf(dot);
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * acosf(dot);
}
