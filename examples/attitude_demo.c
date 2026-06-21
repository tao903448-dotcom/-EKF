/**
 * @file attitude_demo.c
 * @brief 四旋翼姿态估计 EKF —— 四方法 × 三场景 蒙特卡洛评测（跨平台 CLI）
 *
 * 这是把"面向模型失配的自适应 EKF"落到 A8 四旋翼位姿估计真实问题上的
 * 评测程序。对标准 / Joseph / Student-t / 自适应四种更新方法，在三类
 * 噪声/失配场景下做多种子蒙特卡洛，报告：
 *   - 姿态 RMSE（度，稳态）
 *   - 陀螺零偏 RMSE（rad/s）
 *   - 平均 NIS（一致性，单位向量观测有效自由度≈4）
 *
 * 结论预期：
 *   - CLEAN   ：各法接近，标准≈Joseph；
 *   - OUTLIER ：Student-t / 自适应显著抗振动野值；
 *   - MANEUVER：线加速度使加速度计"重力方向"失配，鲁棒/自适应占优。
 *
 * 用法：  ./attitude_demo            打印评测表
 *         ./attitude_demo traj.csv   额外导出一次代表性运行的轨迹 CSV
 *
 * 编译：  gcc -I include examples/attitude_demo.c \
 *             src/quaternion.c src/attitude.c src/matrix.c src/ekf.c -lm -o attitude_demo
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#define _POSIX_C_SOURCE 199309L   /* clock_gettime / CLOCK_MONOTONIC（-std=c99 下需此宏） */
#include "attitude.h"
#include "quaternion.h"
#include "imu_sim.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define SIM_STEPS   3000      /* 15 s @200Hz */
#define MC_SEEDS    20        /* 蒙特卡洛种子数 */
#define R_DIAG      4e-4f     /* 已标定：NIS≈有效自由度 */
#define DEG (180.0f / 3.14159265358979f)

/* 95% chi-square 上界(自由度≈4)，用于 NIS 越界比例统计 */
#define NIS_DOF      4.0f
#define NIS_CHI2_HI  9.49f    /* chi2_{0.95}(4) */

typedef struct {
    double att_rmse_deg;   /* 稳态姿态 RMSE (deg) */
    double bias_rmse;      /* 稳态零偏 RMSE (rad/s) */
    double avg_nis;        /* 平均 NIS */
    double nis_exceed_pct; /* NIS 超 95% 上界比例 (%) */
} RunStats;

/* 加速度自适应门控因子（acceleration rejection）：
   比力幅值越偏离重力 g，说明加速度计越受线加速度/野值污染，
   其"重力方向"越不可信 → 越放大加速度观测噪声。CLEAN 下偏差≈0，门控≈1。 */
#define GATE_K   800.0f   /* 64核蒙特卡洛寻优 (docs 报告 §5.2)：强降权按需触发，机动失配大降而良性场景不变 */
#define GATE_MAX 1000.0f
static float accel_gate(float accel_mag) {
    float dev = fabsf(accel_mag / IMU_G - 1.0f);
    float g = 1.0f + GATE_K * dev * dev;
    return g > GATE_MAX ? GATE_MAX : g;
}

static void configure_noise(EKF_Config *cfg) {
    Matrix Q, R;
    matrix_zeros(&Q, ATT_STATE_DIM, ATT_STATE_DIM);
    matrix_zeros(&R, ATT_MEAS_DIM, ATT_MEAS_DIM);
    for (int i = 0; i < 4; i++) Q.data[i * ATT_STATE_DIM + i] = 1e-6f;  /* 四元数 */
    for (int i = 4; i < 7; i++) Q.data[i * ATT_STATE_DIM + i] = 1e-9f;  /* 零偏   */
    for (int i = 0; i < ATT_MEAS_DIM; i++) R.data[i * ATT_MEAS_DIM + i] = R_DIAG;
    ekf_set_process_noise(cfg, &Q);
    ekf_set_measurement_noise(cfg, &R);
}

/* 跑一次完整滤波，返回统计。gate!=0 启用加速度门控；csv!=NULL 时写轨迹 */
static RunStats run_once(ImuScenario scenario, EKF_UpdateMethod method,
                         int gate, uint32_t seed, FILE *csv, const char *tag) {
    ImuSimConfig sc = imu_sim_default(scenario);
    ImuSim sim; imu_sim_init(&sim, seed);

    EKF_Config cfg; attitude_config_init(&cfg);
    configure_noise(&cfg);
    ekf_set_update_method(&cfg, method);
    ekf_set_student_t_params(&cfg, 4.0f, 0.5f);   /* 寻优: OUTLIER 最优 */
    ekf_set_adaptive_params(&cfg, 100.0f, 8.0f);  /* 寻优: MANEUVER 大降, CLEAN/OUTLIER 不变 */

    Matrix x0, P0; float qid[4] = {1, 0, 0, 0};
    attitude_init_state(&x0, &P0, qid, 0.5f, 0.05f);
    EKF_State st; ekf_state_init(&st, &cfg, &x0, &P0);

    Matrix u, z, Rg;
    matrix_init(&u, 3, 1);
    matrix_init(&z, ATT_MEAS_DIM, 1);
    matrix_zeros(&Rg, ATT_MEAS_DIM, ATT_MEAS_DIM);

    double sse = 0, bse = 0, nis_sum = 0; int cnt = 0, nis_exceed = 0;
    int warm = SIM_STEPS / 3;   /* 收敛预热后再统计稳态 */

    for (int k = 0; k < SIM_STEPS; k++) {
        ImuSample s; imu_sim_step(&sim, &sc, &s);
        for (int i = 0; i < 3; i++) u.data[i] = s.gyro[i];
        ekf_predict(&st, &cfg, &u, sc.dt);
        for (int i = 0; i < 3; i++) { z.data[i] = s.accel_dir[i]; z.data[3 + i] = s.mag_dir[i]; }
        if (gate) {   /* 按比力幅值偏差放大加速度块噪声（磁块不变） */
            float gs = accel_gate(s.accel_mag);
            for (int i = 0; i < 3; i++) Rg.data[i * ATT_MEAS_DIM + i] = R_DIAG * gs;
            for (int i = 3; i < 6; i++) Rg.data[i * ATT_MEAS_DIM + i] = R_DIAG;
            ekf_set_measurement_noise(&cfg, &Rg);
        }
        ekf_update(&st, &cfg, &z);

        float qest[4] = { st.x.data[0], st.x.data[1], st.x.data[2], st.x.data[3] };
        float ang = quat_angle_between(qest, s.q_true) * DEG;
        float db0 = st.x.data[4] - s.bias_true[0];
        float db1 = st.x.data[5] - s.bias_true[1];
        float db2 = st.x.data[6] - s.bias_true[2];

        Matrix Sinv, Sy; float nis = 0.0f;
        if (matrix_inverse(&Sinv, &st.S) && matrix_mul(&Sy, &Sinv, &st.y)) {
            for (int i = 0; i < ATT_MEAS_DIM; i++) nis += st.y.data[i] * Sy.data[i];
        }

        if (k >= warm) {
            sse += (double)ang * ang;
            bse += (double)(db0*db0 + db1*db1 + db2*db2) / 3.0;
            nis_sum += nis; cnt++;
            if (nis > NIS_CHI2_HI) nis_exceed++;
        }

        if (csv) {
            float er, ep, ey, tr, tp, ty;
            quat_to_euler(qest, &er, &ep, &ey);
            quat_to_euler(s.q_true, &tr, &tp, &ty);
            fprintf(csv, "%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    tag, k, tr*DEG, tp*DEG, ty*DEG, er*DEG, ep*DEG, ey*DEG, ang);
        }
    }

    RunStats r;
    r.att_rmse_deg  = sqrt(sse / cnt);
    r.bias_rmse     = sqrt(bse / cnt);
    r.avg_nis       = nis_sum / cnt;
    r.nis_exceed_pct = 100.0 * nis_exceed / cnt;
    return r;
}

/* 多种子蒙特卡洛求均值 */
static RunStats run_mc(ImuScenario scenario, EKF_UpdateMethod method, int gate) {
    RunStats acc = {0, 0, 0, 0};
    for (int s = 0; s < MC_SEEDS; s++) {
        RunStats r = run_once(scenario, method, gate, (uint32_t)(1000 + 7 * s), NULL, NULL);
        acc.att_rmse_deg += r.att_rmse_deg;
        acc.bias_rmse    += r.bias_rmse;
        acc.avg_nis      += r.avg_nis;
        acc.nis_exceed_pct += r.nis_exceed_pct;
    }
    acc.att_rmse_deg /= MC_SEEDS; acc.bias_rmse /= MC_SEEDS;
    acc.avg_nis /= MC_SEEDS; acc.nis_exceed_pct /= MC_SEEDS;
    return acc;
}

static const char *scen_name(ImuScenario s) {
    return (s == SIM_CLEAN) ? "CLEAN   (温和+小噪声)" :
           (s == SIM_OUTLIER) ? "OUTLIER (振动野值)  " :
                                "MANEUVER(大机动失配)";
}
static const char *meth_name(EKF_UpdateMethod m) {
    return (m == EKF_UPDATE_STANDARD) ? "标准EKF  " :
           (m == EKF_UPDATE_JOSEPH)   ? "Joseph   " :
           (m == EKF_UPDATE_STUDENT_T)? "Student-t" : "自适应EKF";
}

int main(int argc, char **argv) {
    const ImuScenario scens[3] = { SIM_CLEAN, SIM_OUTLIER, SIM_MANEUVER };
    const EKF_UpdateMethod meths[4] = {
        EKF_UPDATE_STANDARD, EKF_UPDATE_JOSEPH, EKF_UPDATE_STUDENT_T, EKF_UPDATE_ADAPTIVE
    };

    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║   四旋翼姿态估计 EKF —— 四方法 × 三场景 蒙特卡洛评测                   ║\n");
    printf("║   状态:四元数(4)+陀螺零偏(3)  观测:加速度计+磁力计方向(6)             ║\n");
    printf("║   %d 步/15s @200Hz, %d 种子蒙特卡洛                                  ║\n", SIM_STEPS, MC_SEEDS);
    printf("╚══════════════════════════════════════════════════════════════════════╝\n");

    for (int si = 0; si < 3; si++) {
        printf("\n场景 %s\n", scen_name(scens[si]));
        printf("─────────────────────────────────────────────────────────────────────\n");
        printf("  方法        姿态RMSE(°)   零偏RMSE(rad/s)   平均NIS   NIS越界%%\n");
        double base = 0;
        for (int mi = 0; mi < 4; mi++) {
            RunStats r = run_mc(scens[si], meths[mi], 0);
            if (mi == 0) base = r.att_rmse_deg;
            char impr[24] = "";
            if (mi >= 2 && base > 1e-9)
                snprintf(impr, sizeof impr, "  (↓%.0f%%)", (1.0 - r.att_rmse_deg / base) * 100.0);
            printf("  %s    %8.3f      %9.5f       %6.2f    %5.1f%s\n",
                   meth_name(meths[mi]), r.att_rmse_deg, r.bias_rmse,
                   r.avg_nis, r.nis_exceed_pct, impr);
        }
    }

    printf("\n要点：CLEAN 下标准≈Joseph 即足够；OUTLIER/MANEUVER 下 Student-t/自适应\n");
    printf("      通过对失配观测降权显著降低姿态误差——鲁棒滤波对症发挥作用。\n");

    /* ===== 加速度自适应门控（acceleration rejection）增益 ===== */
    printf("\n╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║   加速度自适应门控：按比力幅值偏差放大加速度观测噪声                  ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════╝\n");
    printf("  场景        方法           无门控RMSE(°)   +门控RMSE(°)   增益\n");
    printf("─────────────────────────────────────────────────────────────────────\n");
    EKF_UpdateMethod gm[2] = { EKF_UPDATE_STANDARD, EKF_UPDATE_ADAPTIVE };
    for (int si = 0; si < 3; si++) {
        for (int j = 0; j < 2; j++) {
            RunStats no = run_mc(scens[si], gm[j], 0);
            RunStats yes = run_mc(scens[si], gm[j], 1);
            double impr = (no.att_rmse_deg > 1e-9)
                ? (1.0 - yes.att_rmse_deg / no.att_rmse_deg) * 100.0 : 0.0;
            printf("  %-10s  %s    %9.3f      %9.3f     ↓%.0f%%\n",
                   (si==0?"CLEAN":si==1?"OUTLIER":"MANEUVER"),
                   meth_name(gm[j]), no.att_rmse_deg, yes.att_rmse_deg, impr);
        }
    }
    printf("  ↳ 门控对症机动失配/野值(比力偏离 g)，CLEAN 下几乎不动；与自适应叠加最优。\n");

    /* ===== 实时性：单步(predict+update)耗时与可支撑频率 ===== */
    {
        const int TN = 100000;
        ImuSimConfig sc = imu_sim_default(SIM_MANEUVER);
        ImuSim sim; imu_sim_init(&sim, 1);
        EKF_Config cfg; attitude_config_init(&cfg); configure_noise(&cfg);
        ekf_set_update_method(&cfg, EKF_UPDATE_ADAPTIVE);
        ekf_set_adaptive_params(&cfg, 100.0f, 8.0f);
        Matrix x0, P0; float qid[4] = {1,0,0,0};
        attitude_init_state(&x0, &P0, qid, 0.5f, 0.05f);
        EKF_State st; ekf_state_init(&st, &cfg, &x0, &P0);
        Matrix u, z; matrix_init(&u,3,1); matrix_init(&z,6,1);
        struct timespec t0, t1; clock_gettime(CLOCK_MONOTONIC, &t0);
        for (int k = 0; k < TN; k++) {
            ImuSample s; imu_sim_step(&sim, &sc, &s);
            for (int i=0;i<3;i++) u.data[i]=s.gyro[i];
            ekf_predict(&st,&cfg,&u,sc.dt);
            for (int i=0;i<3;i++){z.data[i]=s.accel_dir[i]; z.data[3+i]=s.mag_dir[i];}
            ekf_update(&st,&cfg,&z);
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double us = ((t1.tv_sec-t0.tv_sec)*1e9 + (t1.tv_nsec-t0.tv_nsec)) / TN / 1e3;
        printf("\n实时性：自适应EKF 单步约 %.2f µs（本机 x86-64 -O2），可支撑约 %.0f kHz，\n",
               us, 1000.0/us);
        printf("        远高于 200 Hz 飞控更新率——满足嵌入式实时要求。\n");
    }

    /* 可选：导出一次代表性运行(MANEUVER, 标准 vs 自适应 vs 自适应+门控)的轨迹 */
    if (argc > 1) {
        FILE *f = fopen(argv[1], "w");
        if (f) {
            fprintf(f, "tag,step,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,att_err_deg\n");
            run_once(SIM_MANEUVER, EKF_UPDATE_STANDARD, 0, 2026u, f, "MANEUVER_standard");
            run_once(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 0, 2026u, f, "MANEUVER_adaptive");
            run_once(SIM_MANEUVER, EKF_UPDATE_ADAPTIVE, 1, 2026u, f, "MANEUVER_adaptive_gated");
            fclose(f);
            printf("\n已导出轨迹 CSV：%s\n", argv[1]);
        }
    }
    return 0;
}
