/**
 * @file test_mekf.c
 * @brief 乘性误差状态 EKF（MEKF）测试：精度、与加性 EKF 等价、协方差一致性(NEES)
 *
 * MEKF 用 6 维误差状态，协方差不对四元数过参数化，因此可用标准 **NEES**
 * （归一化估计误差平方，需真值）严格检验一致性：一致滤波的平均 NEES 应 ≈ 状态维 6。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "mekf.h"
#include "attitude.h"
#include "quaternion.h"
#include "imu_sim.h"
#include <stdio.h>
#include <math.h>

static int passed = 0, failed = 0;
#define CHECK(c,m) do{ if(c){printf("  PASS: %s\n",m);passed++;} else {printf("  FAIL: %s\n",m);failed++;} }while(0)
#define DEG (180.0f/3.14159265358979f)

/* 运行 MEKF，返回稳态姿态 RMSE(deg) 与平均 NEES */
static void run_mekf(ImuScenario sc_id, uint32_t seed, double *rmse_deg, double *avg_nees) {
    ImuSimConfig sc = imu_sim_default(sc_id);
    ImuSim sim; imu_sim_init(&sim, seed);
    MEKF f; float q0[4] = {1,0,0,0}; mekf_init(&f, q0, 0.5f, 0.05f);
    Matrix Q, R; matrix_zeros(&Q, 6, 6); matrix_zeros(&R, 6, 6);
    for (int i=0;i<3;i++) Q.data[i*6+i] = 4e-6f;
    for (int i=3;i<6;i++) Q.data[i*6+i] = 1e-9f;
    for (int i=0;i<6;i++) R.data[i*6+i] = 4e-4f;
    mekf_set_noise(&f, &Q, &R);

    int N = 3000, warm = N/3; double sse = 0, nees_sum = 0; int cnt = 0;
    for (int k = 0; k < N; k++) {
        ImuSample s; imu_sim_step(&sim, &sc, &s);
        mekf_predict(&f, s.gyro, sc.dt);
        mekf_update(&f, s.accel_dir, s.mag_dir);

        float qe[4]; mekf_get_quat(&f, qe);
        float be[3]; mekf_get_bias(&f, be);
        float ang = quat_angle_between(qe, s.q_true) * DEG;

        /* 误差向量 e = [δθ; δb]，δθ = 2·vec(conj(q_est)⊗q_true) */
        float qc[4] = { qe[0], -qe[1], -qe[2], -qe[3] }, dq[4];
        quat_mul(dq, qc, s.q_true);
        if (dq[0] < 0) { dq[1]=-dq[1]; dq[2]=-dq[2]; dq[3]=-dq[3]; }
        Matrix e; matrix_init(&e, 6, 1);
        e.data[0]=2*dq[1]; e.data[1]=2*dq[2]; e.data[2]=2*dq[3];
        e.data[3]=s.bias_true[0]-be[0]; e.data[4]=s.bias_true[1]-be[1]; e.data[5]=s.bias_true[2]-be[2];

        Matrix Pinv, Pe; float nees = 0;
        if (matrix_inverse(&Pinv, &f.P) && matrix_mul(&Pe, &Pinv, &e)) {
            for (int i=0;i<6;i++) nees += e.data[i]*Pe.data[i];
        }
        if (k >= warm) { sse += (double)ang*ang; nees_sum += nees; cnt++; }
    }
    *rmse_deg = sqrt(sse/cnt);
    *avg_nees = nees_sum/cnt;
}

/* 加性 7 状态 EKF（标准）做对照，返回 CLEAN 稳态 RMSE */
static double run_additive_clean(uint32_t seed) {
    ImuSimConfig sc = imu_sim_default(SIM_CLEAN);
    ImuSim sim; imu_sim_init(&sim, seed);
    EKF_Config cfg; attitude_config_init(&cfg);
    Matrix Q, R; matrix_zeros(&Q,7,7); matrix_zeros(&R,6,6);
    for (int i=0;i<4;i++) Q.data[i*7+i]=1e-6f;
    for (int i=4;i<7;i++) Q.data[i*7+i]=1e-9f;
    for (int i=0;i<6;i++) R.data[i*6+i]=4e-4f;
    ekf_set_process_noise(&cfg,&Q); ekf_set_measurement_noise(&cfg,&R);
    ekf_set_update_method(&cfg, EKF_UPDATE_STANDARD);
    Matrix x0,P0; float q0[4]={1,0,0,0};
    attitude_init_state(&x0,&P0,q0,0.5f,0.05f);
    EKF_State st; ekf_state_init(&st,&cfg,&x0,&P0);
    Matrix u,z; matrix_init(&u,3,1); matrix_init(&z,6,1);
    int N=3000, warm=N/3; double sse=0; int cnt=0;
    for (int k=0;k<N;k++){ ImuSample s; imu_sim_step(&sim,&sc,&s);
        for(int i=0;i<3;i++){ u.data[i]=s.gyro[i]; }
        ekf_predict(&st,&cfg,&u,sc.dt);
        for(int i=0;i<3;i++){z.data[i]=s.accel_dir[i];z.data[3+i]=s.mag_dir[i];}
        ekf_update(&st,&cfg,&z);
        float qe[4]={st.x.data[0],st.x.data[1],st.x.data[2],st.x.data[3]};
        float ang=quat_angle_between(qe,s.q_true)*DEG;
        if(k>=warm){sse+=(double)ang*ang;cnt++;} }
    return sqrt(sse/cnt);
}

int main(void) {
    printf("========== MEKF（乘性误差状态 EKF）测试 ==========\n");

    double rmse, nees;
    run_mekf(SIM_CLEAN, 42u, &rmse, &nees);
    printf("\nCLEAN: RMSE=%.3f deg, 平均NEES=%.2f (理想≈6)\n", rmse, nees);
    CHECK(rmse < 1.0, "MEKF CLEAN 姿态 RMSE < 1.0°");
    CHECK(nees > 2.0 && nees < 12.0, "MEKF 平均 NEES ∈ (2,12)（6 维误差状态一致）");

    double r_out, n_out;
    run_mekf(SIM_OUTLIER, 42u, &r_out, &n_out);
    printf("OUTLIER: RMSE=%.3f deg\n", r_out);
    CHECK(r_out < 5.0, "MEKF OUTLIER 不发散");

    /* 与加性 7 状态 EKF 点估计等价（同标准更新） */
    double r_mekf, n2, r_add = run_additive_clean(42u);
    run_mekf(SIM_CLEAN, 42u, &r_mekf, &n2);
    printf("等价性: MEKF=%.3f vs 加性EKF=%.3f deg\n", r_mekf, r_add);
    CHECK(fabs(r_mekf - r_add) < 0.05, "MEKF 与加性 EKF 点估计等价(|Δ|<0.05°)");

    printf("\n========== 结果 ==========\n通过: %d\n失败: %d\n总计: %d\n",
           passed, failed, passed+failed);
    return failed > 0 ? 1 : 0;
}
