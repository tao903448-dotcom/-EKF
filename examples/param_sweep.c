/**
 * @file param_sweep.c
 * @brief 四旋翼姿态 EKF 参数蒙特卡洛网格寻优（OpenMP 多核并行）
 *
 * 在算力服务器上用多核对关键参数做大规模多种子蒙特卡洛网格搜索，寻找
 * 使各场景姿态 RMSE 最小的参数，给出比手调更优、且统计稳健的默认值。
 *
 * 两个寻优任务：
 *   OUTLIER  : 扫 Student-t 的 (ν, δ)，最小化振动野值下 RMSE
 *   MANEUVER : 扫 自适应因子 α 与加速度门控强度 K（自适应+门控），最小化机动失配 RMSE
 *
 * 编译：  gcc -O2 -fopenmp -I include examples/param_sweep.c \
 *             src/quaternion.c src/attitude.c src/matrix.c src/ekf.c -lm -o param_sweep
 * 运行：  OMP_NUM_THREADS=64 ./param_sweep <nseeds>
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "attitude.h"
#include "quaternion.h"
#include "imu_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define DEG (180.0f / 3.14159265358979f)
#define SIM_STEPS 3000
#define R_DIAG 4e-4f

/* 单次运行：返回稳态姿态 RMSE(deg)。gate!=0 启用加速度门控(强度 gateK)。 */
static double run_once(ImuScenario scen, EKF_UpdateMethod method,
                       float nu, float delta, float adaptF, float adaptW,
                       int gate, float gateK, uint32_t seed) {
    ImuSimConfig sc = imu_sim_default(scen);
    ImuSim sim; imu_sim_init(&sim, seed);
    EKF_Config cfg; attitude_config_init(&cfg);
    Matrix Q, R, Rg;
    matrix_zeros(&Q, 7, 7); matrix_zeros(&R, 6, 6); matrix_zeros(&Rg, 6, 6);
    for (int i = 0; i < 4; i++) Q.data[i*7+i] = 1e-6f;
    for (int i = 4; i < 7; i++) Q.data[i*7+i] = 1e-9f;
    for (int i = 0; i < 6; i++) R.data[i*6+i] = R_DIAG;
    ekf_set_process_noise(&cfg, &Q); ekf_set_measurement_noise(&cfg, &R);
    ekf_set_update_method(&cfg, method);
    ekf_set_student_t_params(&cfg, nu, delta);
    ekf_set_adaptive_params(&cfg, adaptW, adaptF);
    Matrix x0, P0; float qid[4] = {1,0,0,0};
    attitude_init_state(&x0, &P0, qid, 0.5f, 0.05f);
    EKF_State st; ekf_state_init(&st, &cfg, &x0, &P0);
    Matrix u, z; matrix_init(&u,3,1); matrix_init(&z,6,1);
    double sse = 0; int cnt = 0, warm = SIM_STEPS/3;
    for (int k = 0; k < SIM_STEPS; k++) {
        ImuSample s; imu_sim_step(&sim, &sc, &s);
        for (int i=0;i<3;i++) u.data[i]=s.gyro[i];
        ekf_predict(&st,&cfg,&u,sc.dt);
        for (int i=0;i<3;i++){z.data[i]=s.accel_dir[i]; z.data[3+i]=s.mag_dir[i];}
        if (gate) {
            float dev = fabsf(s.accel_mag/IMU_G - 1.0f);
            float gs = 1.0f + gateK*dev*dev; if (gs>500.0f) gs=500.0f;
            for (int i=0;i<3;i++) Rg.data[i*6+i]=R_DIAG*gs;
            for (int i=3;i<6;i++) Rg.data[i*6+i]=R_DIAG;
            ekf_set_measurement_noise(&cfg,&Rg);
        }
        ekf_update(&st,&cfg,&z);
        float qe[4]={st.x.data[0],st.x.data[1],st.x.data[2],st.x.data[3]};
        float ang = quat_angle_between(qe, s.q_true)*DEG;
        if (k>=warm){ sse += (double)ang*ang; cnt++; }
    }
    return sqrt(sse/cnt);
}

static double mc(ImuScenario scen, EKF_UpdateMethod method, float nu, float delta,
                 float adaptF, float adaptW, int gate, float gateK, int nseeds) {
    double acc = 0;
    for (int s = 0; s < nseeds; s++)
        acc += run_once(scen, method, nu, delta, adaptF, adaptW, gate, gateK,
                        (uint32_t)(1000 + 7*s));
    return acc / nseeds;
}

int main(int argc, char **argv) {
    int nseeds = (argc > 1) ? atoi(argv[1]) : 30;
    printf("# param_sweep nseeds=%d steps=%d\n", nseeds, SIM_STEPS);

    /* ---- 任务1: OUTLIER 下 Student-t 扫 (nu, delta) ---- */
    const float NUS[]   = {2,3,4,5,6,8,10};
    const float DELTAS[]= {0.5f,0.75f,1.0f,1.5f,2.0f,3.0f};
    int n1 = (int)(sizeof(NUS)/sizeof*NUS), m1 = (int)(sizeof(DELTAS)/sizeof*DELTAS);
    double *res1 = calloc((size_t)n1*m1, sizeof(double));
    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int i = 0; i < n1; i++)
        for (int j = 0; j < m1; j++)
            res1[i*m1+j] = mc(SIM_OUTLIER, EKF_UPDATE_STUDENT_T, NUS[i], DELTAS[j],
                              1.0f, 100.0f, 0, 0.0f, nseeds);
    double best1 = 1e9; int bi=0,bj=0;
    for (int i=0;i<n1;i++) for (int j=0;j<m1;j++)
        if (res1[i*m1+j]<best1){best1=res1[i*m1+j];bi=i;bj=j;}
    printf("\n[OUTLIER / Student-t] 扫 (nu,delta), RMSE(deg):\n     ");
    for (int j=0;j<m1;j++) printf("  d=%.2f", DELTAS[j]); printf("\n");
    for (int i=0;i<n1;i++){ printf("nu=%4.1f", NUS[i]);
        for (int j=0;j<m1;j++) printf("  %.3f", res1[i*m1+j]); printf("\n"); }
    printf(">>> 最优: nu=%.1f delta=%.2f -> RMSE=%.3f (当前默认 nu=5,delta=1)\n",
           NUS[bi], DELTAS[bj], best1);
    free(res1);

    /* ---- 任务2: MANEUVER 下 自适应+门控 扫 (adaptF, gateK)（扩大网格找平台） ---- */
    const float AF[]   = {1.0f,2.0f,4.0f,8.0f,16.0f,32.0f};
    const float GK[]   = {200,400,800,1600,3200};
    int n2=(int)(sizeof(AF)/sizeof*AF), m2=(int)(sizeof(GK)/sizeof*GK);
    double *res2 = calloc((size_t)n2*m2, sizeof(double));
    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int i=0;i<n2;i++)
        for (int j=0;j<m2;j++)
            res2[i*m2+j] = mc(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 5.0f, 1.0f,
                              AF[i], 100.0f, 1, GK[j], nseeds);
    double best2=1e9; int ci=0,cj=0;
    for (int i=0;i<n2;i++) for (int j=0;j<m2;j++)
        if (res2[i*m2+j]<best2){best2=res2[i*m2+j];ci=i;cj=j;}
    printf("\n[MANEUVER / 自适应+门控] 扫 (adaptF,gateK), RMSE(deg):\n      ");
    for (int j=0;j<m2;j++) printf("  K=%4.0f", GK[j]); printf("\n");
    for (int i=0;i<n2;i++){ printf("aF=%4.1f", AF[i]);
        for (int j=0;j<m2;j++) printf("  %.2f", res2[i*m2+j]); printf("\n"); }
    printf(">>> 最优: adaptF=%.1f gateK=%.0f -> RMSE=%.2f (当前默认 adaptF=1,gateK=200)\n",
           AF[ci], GK[cj], best2);
    free(res2);

    /* ---- 任务3: 跨场景权衡——激进降权是否伤害良性场景？ ---- */
    struct { const char *name; float aF; float gK; } cand[] = {
        {"current  (aF=1, K=200) ", 1.0f, 200.0f},
        {"balanced (aF=4, K=400) ", 4.0f, 400.0f},
        {"aggr     (aF=8, K=800) ", 8.0f, 800.0f},
        {"extreme  (aF=16,K=1600)",16.0f,1600.0f},
    };
    printf("\n[跨场景权衡] 自适应+门控 各候选默认在三场景 RMSE(deg):\n");
    printf("  config                   CLEAN   OUTLIER  MANEUVER\n");
    for (int c = 0; c < 4; c++) {
        double rc = mc(SIM_CLEAN,    EKF_UPDATE_ADAPTIVE, 5,1, cand[c].aF,100, 1, cand[c].gK, nseeds);
        double ro = mc(SIM_OUTLIER,  EKF_UPDATE_ADAPTIVE, 5,1, cand[c].aF,100, 1, cand[c].gK, nseeds);
        double rm = mc(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 5,1, cand[c].aF,100, 1, cand[c].gK, nseeds);
        printf("  %s  %6.3f  %6.3f  %7.2f\n", cand[c].name, rc, ro, rm);
    }
    printf("  ↳ 选 CLEAN/OUTLIER 不明显劣化、MANEUVER 明显改善的折中作默认。\n");
    return 0;
}
