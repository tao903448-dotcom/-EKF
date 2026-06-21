/**
 * @file main.cpp
 * @brief 自适应 EKF 系统 - 图形仪表盘（Windows + DX11 + ImGui）
 *
 * 真实 EKF 后端（调用本项目 ekf/matrix 库）+ ImGui 可视化。轨迹、估计、新息
 * 序列与四算法 RMSE 对比柱均来自真实滤波结果；仪表盘上的少量装饰性指示
 * （运行模式/频率等）为示意标注，不作为评测依据。
 *
 * 权威、可复现的定量评测请以命令行 `attitude_demo` / `ekf_demo` 为准
 * （多场景蒙特卡洛 + 一致性指标，跨平台、纳入 CI）。
 *
 * @author 软件杯团队
 * @date 2026-06-21
 */

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>
#include <vector>
#include <cmath>
#include <string>
#include <cstdio>

extern "C" {
    #include "matrix.h"
    #include "ekf.h"
}

// ============================================================
// EKF Backend
// ============================================================

static EKF_Config g_ekfConfig;
static EKF_State  g_ekfState;
static bool g_ekfInitialized = false;

static bool state_func_1d(const Matrix *x, const Matrix *u, Matrix *x_new, float dt) {
    (void)u; (void)dt;
    matrix_copy(x_new, x);
    return true;
}
static bool meas_func_1d(const Matrix *x, Matrix *z) {
    matrix_copy(z, x);
    return true;
}
static bool state_jac_1d(const Matrix *x, const Matrix *u, Matrix *F, float dt) {
    (void)x; (void)u; (void)dt;
    matrix_eye(F, 1);
    return true;
}
static bool meas_jac_1d(const Matrix *x, Matrix *H) {
    (void)x;
    matrix_eye(H, 1);
    return true;
}

// UI 算法下标 → 枚举的唯一映射。
// UI 顺序为 0=标准, 1=Student-t, 2=自适应, 3=Joseph（见 AlgoName / 单选按钮），
// 它与库枚举顺序(STANDARD=0,JOSEPH=1,STUDENT_T=2,ADAPTIVE=3)不同，
// 因此 InitEKF 与 run_ekf_algo 必须统一经此函数转换，
// 否则性能对比柱的标签会与实际算法错位（历史 bug）。
static EKF_UpdateMethod UiToMethod(int ui) {
    switch (ui) {
        case 0:  return EKF_UPDATE_STANDARD;
        case 1:  return EKF_UPDATE_STUDENT_T;
        case 2:  return EKF_UPDATE_ADAPTIVE;
        default: return EKF_UPDATE_JOSEPH;   // 3
    }
}

static void InitEKF(float Q, float R, int method) {
    ekf_config_init(&g_ekfConfig, 1, 1);
    ekf_set_functions(&g_ekfConfig, state_func_1d, meas_func_1d, state_jac_1d, meas_jac_1d);

    Matrix Qm, Rm;
    matrix_init(&Qm, 1, 1); matrix_init(&Rm, 1, 1);
    Qm.data[0] = Q; Rm.data[0] = R;
    ekf_set_process_noise(&g_ekfConfig, &Qm);
    ekf_set_measurement_noise(&g_ekfConfig, &Rm);

    ekf_set_update_method(&g_ekfConfig, UiToMethod(method));
    ekf_set_student_t_params(&g_ekfConfig, 3.0f, 1.0f);
    ekf_set_adaptive_params(&g_ekfConfig, 20.0f, 1.5f);

    Matrix x0, P0;
    matrix_init(&x0, 1, 1); matrix_init(&P0, 1, 1);
    x0.data[0] = 120.0f; P0.data[0] = 10.0f;
    ekf_state_init(&g_ekfState, &g_ekfConfig, &x0, &P0);
    g_ekfInitialized = true;
}

static void RunEKFStep(float obs) {
    if (!g_ekfInitialized) return;
    Matrix u, z;
    matrix_init(&u, 1, 1); matrix_init(&z, 1, 1);
    u.data[0] = 0.0f;
    ekf_predict(&g_ekfState, &g_ekfConfig, &u, 0.1f);
    z.data[0] = obs;
    ekf_update(&g_ekfState, &g_ekfConfig, &z);
}

static float GetEKFEstimate() {
    if (!g_ekfInitialized) return 0.0f;
    Matrix x;
    matrix_init(&x, 1, 1);
    ekf_get_state(&g_ekfState, &x);
    return x.data[0];
}

// ============================================================
// Global State
// ============================================================

static ID3D11Device* g_pd3dDevice = NULL;
static ID3D11DeviceContext* g_pd3dDeviceContext = NULL;
static IDXGISwapChain* g_pSwapChain = NULL;
static ID3D11RenderTargetView* g_mainRenderTargetView = NULL;

struct SimData {
    std::vector<float> true_vals, obs_vals, est_vals;
    float rmse = 0, max_err = 0;
    int step = 0;
    bool running = false;
};

// 飞行场景
enum FlightScenario {
    SCENARIO_SINE = 0,      // 正弦波（简单）
    SCENARIO_HELIX = 1,     // 螺旋爬升
    SCENARIO_FIGURE8 = 2,   // 8字形
    SCENARIO_HOVER = 3      // 悬停
};

static SimData g_sim;

// Settings
static float g_q = 1.0f;
static float g_r = 30.0f;
static int   g_algo = 2; // 0=Standard, 1=Student-t, 2=Adaptive, 3=Joseph
static bool  g_paused = false;
static int   g_scenario = 0; // 0=Sine, 1=Helix, 2=Figure8, 3=Hover
static int   g_current_page = -1; // -1=Home, 0=Dashboard, 1=Telemetry, 2=Missions, 3=Analysis

// Perf comparison
static float g_rmse_standard = 0;
static float g_rmse_student_t = 0;
static float g_rmse_adaptive = 0;
static float g_rmse_joseph = 0;

static int g_outlier_count = 0;

static const char* AlgoName(int i) {
    return (i == 0) ? "标准EKF" :
           (i == 1) ? "Student-t" :
           (i == 2) ? "自适应EKF" : "Joseph";
}

// ============================================================
// Theme
// ============================================================

struct T {
    static ImVec4 Bg, Card, Hover, Pri, PriH, PriC, Sec, Ter, Err;
    static ImVec4 Txt, TxtV, Out, OutV;
    static ImVec4 Blue, Green, Red;
};

ImVec4 T::Bg = {1,1,1,1};
ImVec4 T::Card = {1,1,1,1};
ImVec4 T::Hover = {.925f,.925f,.925f,1};
ImVec4 T::Pri = {.133f,.533f,.800f,1};
ImVec4 T::PriH = {.102f,.427f,.686f,1};
ImVec4 T::PriC = {.706f,.878f,1,1};
ImVec4 T::Sec = {.180f,.737f,.408f,1};
ImVec4 T::Ter = {.902f,.502f,.149f,1};
ImVec4 T::Err = {.937f,.200f,.200f,1};
ImVec4 T::Txt = {.133f,.133f,.133f,1};
ImVec4 T::TxtV = {.467f,.467f,.467f,1};
ImVec4 T::Out = {.733f,.733f,.733f,1};
ImVec4 T::OutV = {.867f,.867f,.867f,1};
ImVec4 T::Blue = {.133f,.533f,.800f,1};
ImVec4 T::Green = {.180f,.737f,.408f,1};
ImVec4 T::Red = {.937f,.200f,.200f,1};

void SetupTheme() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.Colors[ImGuiCol_WindowBg] = T::Bg;
    s.Colors[ImGuiCol_ChildBg] = {0,0,0,0};
    s.Colors[ImGuiCol_Border] = T::OutV;
    s.Colors[ImGuiCol_FrameBg] = T::Card;
    s.Colors[ImGuiCol_FrameBgHovered] = T::Hover;
    s.Colors[ImGuiCol_Button] = T::Pri;
    s.Colors[ImGuiCol_ButtonHovered] = T::PriH;
    s.Colors[ImGuiCol_ButtonActive] = T::PriH;
    s.Colors[ImGuiCol_CheckMark] = T::Pri;
    s.Colors[ImGuiCol_SliderGrab] = T::Pri;
    s.Colors[ImGuiCol_SliderGrabActive] = T::PriH;
    s.Colors[ImGuiCol_Text] = T::Txt;
    s.Colors[ImGuiCol_TextDisabled] = T::Out;
    s.WindowRounding = 12; s.ChildRounding = 12;
    s.FrameRounding = 8; s.GrabRounding = 8;
    s.WindowPadding = {0,0}; s.FramePadding = {12,8};
    s.ItemSpacing = {8,8}; s.WindowBorderSize = 0;
    s.ChildBorderSize = 0;
}

// ============================================================
// Data Generation & EKF Run
// ============================================================

// 确定性随机数（跨平台一致，用查表法避免整数溢出问题）
static const float g_rand_table[256] = {
    0.17f, 0.83f, 0.42f, 0.91f, 0.28f, 0.67f, 0.05f, 0.73f,
    0.36f, 0.89f, 0.14f, 0.62f, 0.48f, 0.95f, 0.23f, 0.71f,
    0.38f, 0.84f, 0.11f, 0.59f, 0.47f, 0.93f, 0.26f, 0.74f,
    0.33f, 0.88f, 0.15f, 0.63f, 0.44f, 0.97f, 0.21f, 0.69f,
    0.37f, 0.82f, 0.19f, 0.66f, 0.43f, 0.92f, 0.27f, 0.75f,
    0.31f, 0.87f, 0.12f, 0.61f, 0.46f, 0.94f, 0.24f, 0.72f,
    0.35f, 0.86f, 0.16f, 0.64f, 0.41f, 0.96f, 0.22f, 0.70f,
    0.39f, 0.81f, 0.13f, 0.65f, 0.45f, 0.90f, 0.29f, 0.78f,
    0.34f, 0.85f, 0.18f, 0.60f, 0.49f, 0.91f, 0.25f, 0.77f,
    0.32f, 0.88f, 0.10f, 0.58f, 0.47f, 0.93f, 0.20f, 0.68f,
    0.36f, 0.84f, 0.14f, 0.62f, 0.44f, 0.95f, 0.23f, 0.71f,
    0.38f, 0.82f, 0.11f, 0.59f, 0.48f, 0.92f, 0.26f, 0.74f,
    0.33f, 0.87f, 0.15f, 0.63f, 0.42f, 0.96f, 0.21f, 0.69f,
    0.37f, 0.83f, 0.19f, 0.66f, 0.45f, 0.94f, 0.27f, 0.75f,
    0.31f, 0.86f, 0.12f, 0.61f, 0.43f, 0.90f, 0.24f, 0.72f,
    0.35f, 0.81f, 0.16f, 0.64f, 0.46f, 0.93f, 0.22f, 0.70f,
    0.39f, 0.85f, 0.13f, 0.65f, 0.41f, 0.97f, 0.29f, 0.78f,
    0.34f, 0.88f, 0.18f, 0.60f, 0.49f, 0.91f, 0.25f, 0.77f,
    0.32f, 0.82f, 0.10f, 0.58f, 0.47f, 0.93f, 0.20f, 0.68f,
    0.36f, 0.84f, 0.14f, 0.62f, 0.44f, 0.95f, 0.23f, 0.71f,
    0.38f, 0.86f, 0.11f, 0.59f, 0.48f, 0.92f, 0.26f, 0.74f,
    0.33f, 0.83f, 0.15f, 0.63f, 0.42f, 0.96f, 0.21f, 0.69f,
    0.37f, 0.87f, 0.19f, 0.66f, 0.45f, 0.94f, 0.27f, 0.75f,
    0.31f, 0.81f, 0.12f, 0.61f, 0.43f, 0.90f, 0.24f, 0.72f,
    0.35f, 0.85f, 0.16f, 0.64f, 0.46f, 0.93f, 0.22f, 0.70f,
    0.39f, 0.88f, 0.13f, 0.65f, 0.41f, 0.97f, 0.29f, 0.78f,
    0.34f, 0.82f, 0.18f, 0.60f, 0.49f, 0.91f, 0.25f, 0.77f,
    0.32f, 0.86f, 0.10f, 0.58f, 0.47f, 0.93f, 0.20f, 0.68f,
    0.36f, 0.83f, 0.14f, 0.62f, 0.44f, 0.95f, 0.23f, 0.71f,
    0.38f, 0.87f, 0.11f, 0.59f, 0.48f, 0.92f, 0.26f, 0.74f,
    0.33f, 0.81f, 0.15f, 0.63f, 0.42f, 0.96f, 0.21f, 0.69f,
    0.37f, 0.85f, 0.19f, 0.66f, 0.45f, 0.94f, 0.27f, 0.75f
};
static int g_didx = 0;
static float det_rand() {
    float v = g_rand_table[g_didx & 255];
    g_didx++;
    return v;
}

void RunAllAlgorithms() {
    g_didx = 0;
    g_sim.true_vals.resize(200);
    g_sim.obs_vals.resize(200);

    for (int i = 0; i < 200; i++) {
        float t = i * 0.1f;
        float true_val = 0;

        switch (g_scenario) {
            case 0: // 正弦波
                true_val = 120.0f + sinf(t*0.8f)*60.0f + cosf(t*0.3f)*20.0f;
                break;
            case 1: // 螺旋爬升
                true_val = 100.0f + t * 2.0f + sinf(t*2.0f)*15.0f;
                break;
            case 2: // 8字形
                true_val = 150.0f + sinf(t*0.5f)*50.0f * cosf(t*0.25f);
                break;
            case 3: // 悬停
                true_val = 120.0f + sinf(t*0.1f)*5.0f;
                break;
        }

        float noise = (det_rand() - 0.5f) * 60.0f;

        // 添加野值（约10%的概率出现大幅异常观测）
        if (det_rand() < 0.1f) {
            noise += (det_rand() > 0.5f ? 1.0f : -1.0f) * 40.0f;
        }

        g_sim.true_vals[i] = true_val;
        g_sim.obs_vals[i] = true_val + noise;
    }

    // 用EKF库分别跑4种算法，每种用不同参数
    // 用独立的局部EKF状态，避免全局状态污染
    auto run_ekf_algo = [&](float Q, float R, int method) -> float {
        EKF_Config local_cfg;
        EKF_State local_state;
        memset(&local_cfg, 0, sizeof(local_cfg));
        memset(&local_state, 0, sizeof(local_state));
        ekf_config_init(&local_cfg, 1, 1);
        ekf_set_functions(&local_cfg, state_func_1d, meas_func_1d, state_jac_1d, meas_jac_1d);
        Matrix Qm, Rm;
        matrix_init(&Qm, 1, 1);
        matrix_init(&Rm, 1, 1);
        Qm.data[0] = Q;
        Rm.data[0] = R;
        ekf_set_process_noise(&local_cfg, &Qm);
        ekf_set_measurement_noise(&local_cfg, &Rm);
        ekf_set_update_method(&local_cfg, UiToMethod(method));
        ekf_set_student_t_params(&local_cfg, 3.0f, 1.0f);
        ekf_set_adaptive_params(&local_cfg, 20.0f, 1.5f);
        Matrix x0, P0;
        matrix_init(&x0, 1, 1);
        matrix_init(&P0, 1, 1);
        x0.data[0] = 120;
        P0.data[0] = 10;
        ekf_state_init(&local_state, &local_cfg, &x0, &P0);
        float sum = 0;
        for (int i = 0; i < 200; i++) {
            Matrix u, z;
            matrix_init(&u, 1, 1);
            matrix_init(&z, 1, 1);
            u.data[0] = 0;
            ekf_predict(&local_state, &local_cfg, &u, 0.1f);
            z.data[0] = g_sim.obs_vals[i];
            ekf_update(&local_state, &local_cfg, &z);
            float d = local_state.x.data[0] - g_sim.true_vals[i];
            sum += d * d;
        }
        return sqrtf(sum / 200.0f);
    };

    g_rmse_standard  = run_ekf_algo(g_q,       g_r,       0);  // 标准
    g_rmse_student_t = run_ekf_algo(g_q,       g_r,       1);  // Student-t
    g_rmse_adaptive  = run_ekf_algo(g_q,       g_r,       2);  // 自适应
    g_rmse_joseph    = run_ekf_algo(g_q,       g_r,       3);  // Joseph

    // 用选中的算法运行显示轨迹（与状态栏 / 性能柱一致）
    InitEKF(g_q, g_r, g_algo);
    g_sim.est_vals.resize(200);
    for (int i = 0; i < 200; i++) {
        RunEKFStep(g_sim.obs_vals[i]);
        g_sim.est_vals[i] = GetEKFEstimate();
    }

    g_sim.step = 0;
    g_sim.running = true;
    g_paused = false;
}

void RecalcWithParams() {
    RunAllAlgorithms();
}

float CalcRMSE() {
    if (g_sim.step == 0) return 0;
    // 直接从对应算法的预计算RMSE中获取，保证和性能对比一致
    switch (g_algo) {
        case 0: return g_rmse_standard;
        case 1: return g_rmse_student_t;
        case 2: return g_rmse_adaptive;
        case 3: return g_rmse_joseph;
        default: return g_rmse_standard;
    }
}

float CalcMaxErr() {
    float m = 0;
    for (int i = 0; i < g_sim.step; i++) {
        float e = fabsf(g_sim.est_vals[i] - g_sim.true_vals[i]);
        if (e > m) m = e;
    }
    return m;
}

// ============================================================
// DirectX
// ============================================================

bool CreateDeviceD3D(HWND);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND, UINT, WPARAM, LPARAM);

// ============================================================
// Home Page (目录页)
// ============================================================

void DrawNavBar() {
    float wh = ImGui::GetIO().DisplaySize.y;
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    float nav_w = 220;

    // 创建一个覆盖导航栏区域的隐藏窗口，让SetCursorPos能工作
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({nav_w, wh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
    ImGui::Begin("##NavBarHitbox", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleColor();

    // 导航栏背景
    dl->AddRectFilled({0, 0}, {nav_w, wh}, IM_COL32(255, 255, 255, 255));
    dl->AddLine({nav_w, 0}, {nav_w, wh}, IM_COL32(220, 220, 230, 200));

    // 标题
    dl->AddText({20, 30}, IM_COL32(0x00, 0x63, 0x9A, 255), "EKF Sentinel");
    dl->AddText({20, 52}, IM_COL32(0x99, 0x99, 0x99, 255), "Drone EKF");
    dl->AddText({20, 68}, IM_COL32(0x00, 0x96, 0x4A, 255), "ADAPTIVE FILTER ACTIVE");

    // 菜单项
    const char* nav_items[] = {"Dashboard", "Telemetry", "Mission", "Analysis"};
    int nav_pages[] = {0, 1, 2, 3};
    float nav_y = 140;

    for (int i = 0; i < 4; i++) {
        float ny = nav_y + i * 50;
        bool active = (g_current_page == nav_pages[i]);

        if (active) {
            dl->AddRectFilled({0, ny}, {nav_w, ny + 40}, IM_COL32(0xE8, 0xF0, 0xFE, 255));
            dl->AddRectFilled({0, ny}, {3, ny + 40}, IM_COL32(0x00, 0x63, 0x9A, 255), 2);
        }

        ImVec4 nc = active ? ImVec4(0.0f, 0.39f, 0.77f, 1) : ImVec4(0.25f, 0.29f, 0.32f, 1);
        dl->AddText({30, ny + 12}, IM_COL32((int)(nc.x*255), (int)(nc.y*255), (int)(nc.z*255), 255), nav_items[i]);

        // 导航项可点击（用窗口内坐标）
        ImGui::SetCursorPos({0, ny});
        ImGui::PushID(500 + i);
        if (ImGui::InvisibleButton("##nav", {nav_w, 40})) {
            g_current_page = nav_pages[i];
        }
        ImGui::PopID();
    }

    // 底部
    dl->AddText({20, wh - 60}, IM_COL32(0x99, 0x99, 0x99, 255), "System");
    dl->AddText({20, wh - 40}, IM_COL32(0x99, 0x99, 0x99, 255), "Support");

    ImGui::End(); // NavBarHitbox
}

void DrawHomePage() {
    float ww = ImGui::GetIO().DisplaySize.x;
    float wh = ImGui::GetIO().DisplaySize.y;
    float nav_w = 220;
    ImDrawList* dl = ImGui::GetForegroundDrawList();

    // 隐藏窗口让SetCursorPos工作
    ImGui::SetNextWindowPos({nav_w, 0});
    ImGui::SetNextWindowSize({ww - nav_w, wh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
    ImGui::Begin("##HomeHitbox", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleColor();

    // 右侧内容区
    float content_x = nav_w + 40;
    float content_w = ww - nav_w - 80;

    // 顶部标题
    dl->AddText({content_x, 80}, IM_COL32(0x17, 0x1C, 0x20, 255), "自适应EKF系统");
    dl->AddText({content_x, 120}, IM_COL32(0x64, 0x74, 0x8B, 255),
        "面向模型失配的无人机位姿控制");

    // 4个功能卡片
    float card_w = (content_w - 48) / 4.0f;
    float card_h = 250;
    float card_y = wh * 0.35f;

    struct PageCard { const char* icon; const char* title; const char* desc; int page; ImVec4 color; };
    PageCard cards[] = {
        {"D", "仪表盘",   "轨迹跟踪 / 性能对比",         0, ImVec4(0.133f,0.533f,0.800f,1)},
        {"T", "遥测数据", "实时滤波器内部状态",           1, ImVec4(0.180f,0.737f,0.408f,1)},
        {"M", "任务配置", "飞行场景 / 播放控制",          2, ImVec4(0.902f,0.502f,0.149f,1)},
        {"A", "算法分析", "四种算法对比 / 特性详情",      3, ImVec4(0.467f,0.467f,0.467f,1)},
    };

    for (int i = 0; i < 4; i++) {
        float x = content_x + i * (card_w + 16);
        float y = card_y;

        dl->AddRectFilled({x, y}, {x + card_w, y + card_h},
            IM_COL32(255, 255, 255, 255), 12.0f);
        dl->AddRect({x, y}, {x + card_w, y + card_h},
            IM_COL32(220, 220, 230, 200), 12.0f);

        // 可点击区域（SetCursorPos需要窗口内坐标）
        ImGui::SetCursorPos({x - nav_w, y});
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("##card", {card_w, card_h})) {
            g_current_page = cards[i].page;
        }
        ImGui::PopID();

        if (ImGui::IsItemHovered()) {
            dl->AddRect({x, y}, {x + card_w, y + card_h},
                IM_COL32((int)(cards[i].color.x*255), (int)(cards[i].color.y*255),
                         (int)(cards[i].color.z*255), 255), 12.0f);
        }

        float icon_bg_r = cards[i].color.x * 0.15f + 0.85f;
        float icon_bg_g = cards[i].color.y * 0.15f + 0.85f;
        float icon_bg_b = cards[i].color.z * 0.15f + 0.85f;
        dl->AddRectFilled({x + 20, y + 24}, {x + 56, y + 60},
            IM_COL32((int)(icon_bg_r*255), (int)(icon_bg_g*255), (int)(icon_bg_b*255), 255), 10.0f);

        ImVec2 isz = ImGui::CalcTextSize(cards[i].icon);
        dl->AddText({x + 38 - isz.x/2, y + 42 - isz.y/2},
            IM_COL32((int)(cards[i].color.x*255), (int)(cards[i].color.y*255),
                     (int)(cards[i].color.z*255), 255), cards[i].icon);

        dl->AddText({x + 20, y + 72}, IM_COL32(0x17, 0x1C, 0x20, 255), cards[i].title);
        dl->AddText({x + 20, y + 100}, IM_COL32(0x64, 0x74, 0x8B, 255), cards[i].desc);
        dl->AddText({x + card_w - 30, y + card_h - 30}, IM_COL32(0x22, 0x88, 0xCC, 255), ">");
    }

    // 底部信息
    dl->AddText({content_x, wh - 60}, IM_COL32(0x99, 0x99, 0x99, 255),
        "第十五届中国软件杯 - A8四旋翼无人机位姿控制系统设计优化");

    ImGui::End(); // HomeHitbox
}

void DrawSidebar() {
    ImGui::SetNextWindowPos({0,0});
    ImGui::SetNextWindowSize({240, ImGui::GetIO().DisplaySize.y});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, {0.957f,0.957f,0.957f,1});
    ImGui::Begin("##Sidebar", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoScrollWithMouse|ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    float sw = ImGui::GetContentRegionAvail().x;
    float pad = 10;
    float cw = sw - pad*2;

    // Header
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {pad,16});
    ImGui::TextColored(T::Txt, "Drone EKF v2.4");
    ImGui::TextColored(T::Out, "EKF-Slam");
    ImGui::PopStyleVar();

    ImGui::Separator();
    ImGui::Spacing();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {pad, pad});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {0, 8});

    // Algorithm Controls
    ImGui::PushStyleColor(ImGuiCol_ChildBg, T::Card);
    ImGui::BeginChild("##Algo", {cw, 130}, true);
    ImGui::PopStyleColor();
    ImGui::TextColored(T::Out, "算法控制");
    ImGui::Spacing();
    const char* algos[] = {"标准EKF", "Student-t", "自适应EKF", "Joseph"};
    for (int i = 0; i < 4; i++) {
        if (ImGui::RadioButton(algos[i], g_algo == i)) {
            g_algo = i;
            RecalcWithParams();
        }
    }
    ImGui::EndChild();
    ImGui::Spacing();

    // Parameters
    ImGui::PushStyleColor(ImGuiCol_ChildBg, T::Card);
    ImGui::BeginChild("##Param", {cw, 110}, true);
    ImGui::PopStyleColor();
    ImGui::TextColored(T::Out, "参数设置");
    ImGui::Spacing();
    ImGui::Text("Q协方差");
    ImGui::SameLine(cw - 60);
    ImGui::TextColored(T::Pri, "%.2f", g_q);
    ImGui::SetNextItemWidth(cw);
    if (ImGui::SliderFloat("##Q", &g_q, 0.01f, 2.0f, "%.2f")) RecalcWithParams();
    ImGui::Text("R协方差");
    ImGui::SameLine(cw - 60);
    ImGui::TextColored(T::Pri, "%.1f", g_r);
    ImGui::SetNextItemWidth(cw);
    if (ImGui::SliderFloat("##R", &g_r, 1.0f, 100.0f, "%.1f")) RecalcWithParams();
    ImGui::EndChild();
    ImGui::Spacing();

    // Playback
    ImGui::PushStyleColor(ImGuiCol_ChildBg, T::Card);
    ImGui::BeginChild("##Play", {cw, 65}, true);
    ImGui::PopStyleColor();
    ImGui::TextColored(T::Out, "播放控制");
    ImGui::Spacing();
    float bw = (cw - 8) / 3.0f;
    ImGui::PushStyleColor(ImGuiCol_Button, T::Pri);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, T::PriH);
    if (ImGui::Button(g_paused ? "继续" : "播放", {bw,28})) {
        g_paused = false;
        g_sim.running = true;
    }
    ImGui::PopStyleColor(2);
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, {0.957f,0.957f,0.957f,1});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, T::Hover);
    if (ImGui::Button("暂停", {bw,28})) {
        g_paused = true;
    }
    ImGui::PopStyleColor(2);
    ImGui::SameLine();
    if (ImGui::Button("重置", {bw,28})) {
        g_sim.step = 0;
        g_sim.running = false;
        g_paused = false;
    }
    ImGui::EndChild();

    ImGui::PopStyleVar(2);

    // Deploy at bottom
    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50);
    ImGui::PushStyleColor(ImGuiCol_Button, T::Pri);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, T::PriH);
    if (ImGui::Button("部署固件", {cw, 36})) {
    }
    ImGui::PopStyleColor(2);

    ImGui::End();
}

// ============================================================
// Top Header
// ============================================================

void DrawTopHeader() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    ImGui::SetNextWindowPos({300,0});
    ImGui::SetNextWindowSize({ww, 60});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, T::Card);
    ImGui::Begin("##Header", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {24,12});

    if (ImGui::Button("<", {30, 30})) g_current_page = -1;
    ImGui::SameLine(40);
    const char* page_names[] = {"仪表盘", "遥测数据", "任务配置", "算法分析"};
    ImGui::TextColored(T::Pri, "%s", (g_current_page >= 0 && g_current_page < 4) ? page_names[g_current_page] : "自适应EKF系统");
    ImGui::SameLine(200);

    const char* tabs[] = {"仪表盘","遥测","任务","分析"};
    for (int i = 0; i < 4; i++) {
        ImGui::PushStyleColor(ImGuiCol_Text, g_current_page==i ? T::Pri : T::TxtV);
        ImGui::PushStyleColor(ImGuiCol_Button, T::Card);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, T::Hover);
        if (ImGui::Button(tabs[i], {70,30})) g_current_page = i;
        ImGui::PopStyleColor(3);
        if (i < 3) ImGui::SameLine(0,8);
    }

    ImGui::SameLine(ww - 100);
    ImGui::TextColored(T::TxtV, "%.0f FPS", ImGui::GetIO().Framerate);

    ImGui::PopStyleVar();
    ImGui::End();
}

// ============================================================
// Trajectory Chart
// ============================================================

void DrawChart() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float wh = ImGui::GetIO().DisplaySize.y - 100;
    float ch = wh * 0.6f;

    ImGui::SetNextWindowPos({300,60});
    ImGui::SetNextWindowSize({ww, ch});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, T::Card);
    ImGui::Begin("##Chart", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {20,16});
    ImGui::Text("实时轨迹");

    // Legend
    ImGui::SameLine(ww - 300);
    struct { ImVec4 c; const char* l; } leg[] = {
        {T::Blue, "真实轨迹"}, {T::Green, "估计轨迹"}, {T::Red, "观测值"}
    };
    for (int i = 0; i < 3; i++) {
        ImGui::PushStyleColor(ImGuiCol_Text, leg[i].c);
        ImGui::Text(leg[i].l);
        ImGui::PopStyleColor();
        if (i < 2) ImGui::SameLine(0,12);
    }

    ImGui::PopStyleVar();

    ImVec2 cp = ImGui::GetCursorScreenPos();
    ImVec2 cs = ImGui::GetContentRegionAvail();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Background
    dl->AddRectFilled(cp, {cp.x+cs.x, cp.y+cs.y}, IM_COL32(0xF5,0xF5,0xF5,255));

    // Grid
    for (int i = 0; i <= 5; i++) {
        float y = cp.y + cs.y * i / 5.0f;
        dl->AddLine({cp.x,y}, {cp.x+cs.x,y}, IM_COL32(0xDD,0xDD,0xDD,100));
        float x = cp.x + cs.x * i / 5.0f;
        dl->AddLine({x,cp.y}, {x,cp.y+cs.y}, IM_COL32(0xDD,0xDD,0xDD,100));
    }

    if (g_sim.step > 1) {
        int n = g_sim.step;
        float m = 20, ph = cs.y - 2*m;
        float yMin = 50, yMax = 250;
        auto toY = [&](float v){ return cp.y + m + ph * (1.0f - (v-yMin)/(yMax-yMin)); };

        // Truth
        for (int i = 1; i < n; i++) {
            float x1 = cp.x + (i-1)*cs.x/199.0f, x2 = cp.x + i*cs.x/199.0f;
            dl->AddLine({x1, toY(g_sim.true_vals[i-1])}, {x2, toY(g_sim.true_vals[i])},
                IM_COL32(0x22,0x88,0xCC,255), 2.5f);
        }
        // Estimate
        for (int i = 1; i < n; i++) {
            float x1 = cp.x + (i-1)*cs.x/199.0f, x2 = cp.x + i*cs.x/199.0f;
            dl->AddLine({x1, toY(g_sim.est_vals[i-1])}, {x2, toY(g_sim.est_vals[i])},
                IM_COL32(0x2E,0xBC,0x68,255), 2.0f);
        }
        // Observations
        for (int i = 0; i < n; i += 5) {
            float x = cp.x + i*cs.x/199.0f;
            dl->AddCircleFilled({x, toY(g_sim.obs_vals[i])}, 4.0f,
                IM_COL32(0xEF,0x33,0x33,255));
        }
    }

    ImGui::End();
}

// ============================================================
// Bottom
// ============================================================

// ============================================================
// Telemetry Page (遥测)
// ============================================================

void DrawTelemetry() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float wh = ImGui::GetIO().DisplaySize.y - 100;

    // 外层窗口
    ImGui::SetNextWindowPos({300, 60});
    ImGui::SetNextWindowSize({ww, wh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.961f, 0.961f, 0.980f, 1.0f));
    ImGui::Begin("##TelOuter", NULL,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    // 内部可滚动区域
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::BeginChild("##TelScroll", {0, 0}, false,
        ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleVar();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 sp = ImGui::GetCursorScreenPos();
    float content_w = ww;
    float pad = 16;

    // ===== 左栏：数据卡片 =====
    float left_w = content_w * 0.38f;
    float cx = sp.x + pad;
    float cw = left_w - pad * 2;
    float cy = sp.y + pad;

    // 标题（加粗效果）
    dl->AddText({cx+1, cy+1}, IM_COL32(0x10, 0x1C, 0x2D, 60), "实时遥测数据 (Telemetry)");
    dl->AddText({cx, cy}, IM_COL32(0x10, 0x1C, 0x2D, 255), "实时遥测数据 (Telemetry)");
    char step_s[32]; snprintf(step_s, 32, "Step: %d/200", g_sim.step);
    ImVec2 ssz = ImGui::CalcTextSize(step_s);
    dl->AddRectFilled({cx + cw - ssz.x - 16, cy - 2}, {cx + cw, cy + 18},
        IM_COL32(0xF0, 0xF0, 0xF5, 255), 4.0f);
    dl->AddText({cx + cw - ssz.x - 10, cy}, IM_COL32(0x77, 0x77, 0x77, 255), step_s);
    dl->AddLine({cx, cy + 24}, {cx + cw, cy + 24}, IM_COL32(220, 220, 230, 200));

    float card_w = cw;
    float card_h = 60;
    float card_gap = 10;

    auto Card = [&](float y, const char* label, const char* sub, float val, ImVec4 col, const char* unit) {
        dl->AddRectFilled({cx, y}, {cx + card_w, y + card_h}, IM_COL32(255, 255, 255, 255), 8.0f);
        dl->AddRect({cx, y}, {cx + card_w, y + card_h}, IM_COL32(220, 220, 230, 200), 8.0f);
        // 左侧颜色条
        dl->AddRectFilled({cx, y + 4}, {cx + 3, y + card_h - 4},
            IM_COL32((int)(col.x*255), (int)(col.y*255), (int)(col.z*255), 255), 2.0f);
        // 标签（带颜色）
        dl->AddText({cx + 10, y + 8},
            IM_COL32((int)(col.x*255), (int)(col.y*255), (int)(col.z*255), 255), label);
        // 副标签（加粗效果：偏移重叠）
        dl->AddText({cx + 11, y + 29}, IM_COL32(0x88, 0x88, 0x88, 180), sub);
        dl->AddText({cx + 10, y + 28}, IM_COL32(0x88, 0x88, 0x88, 255), sub);
        // 大数值
        char vs[32]; snprintf(vs, 32, "%.2f", val);
        ImVec2 ts = ImGui::CalcTextSize(vs);
        dl->AddText({cx + card_w - ts.x - 28, y + 6},
            IM_COL32((int)(col.x*255), (int)(col.y*255), (int)(col.z*255), 255), vs);
        dl->AddText({cx + card_w - 22, y + 10}, IM_COL32(0x99, 0x99, 0x99, 255), unit);
    };

    if (g_sim.step > 0) {
        int idx = g_sim.step - 1;
        float tv = g_sim.true_vals[idx], ov = g_sim.obs_vals[idx];
        float ev = g_sim.est_vals[idx], inn = ov - ev;

        float y0 = cy + 32;
        Card(y0, "真实位置 (GT)", "仿真真值 (ground truth)", tv, T::Blue, "m");
        Card(y0 + (card_h+card_gap), "观测值 (OBS)", "含噪声与野值的观测", ov, T::Err, "m");
        Card(y0 + (card_h+card_gap)*2, "估计值 (EST)", "EKF 估计值", ev, T::Sec, "m");
        ImVec4 ic = fabsf(inn) > 0.5f ? T::Err : T::TxtV;
        Card(y0 + (card_h+card_gap)*3, "新息 (Innovation)", "THRESHOLD: < 0.5", inn, ic, "v");

        // 分割线 + 元数据
        float my = y0 + (card_h+card_gap)*4 + 8;
        dl->AddLine({cx, my}, {cx + card_w, my}, IM_COL32(220, 220, 230, 200));
        dl->AddText({cx, my + 10}, IM_COL32(0x99, 0x99, 0x99, 255), "算法版本");
        dl->AddText({cx + card_w - 120, my + 10}, IM_COL32(0x22, 0x22, 0x22, 255), AlgoName(g_algo));
        dl->AddText({cx, my + 30}, IM_COL32(0x99, 0x99, 0x99, 255), "参数比值");
        char qr[32]; snprintf(qr, 32, "%.3f", g_q/g_r);
        dl->AddText({cx + card_w - 60, my + 30}, IM_COL32(0x22, 0x22, 0x22, 255), qr);
    } else {
        dl->AddText({cx + 40, sp.y + wh * 0.4f}, IM_COL32(0x99, 0x99, 0x99, 255), "点击播放开始仿真");
    }

    // ===== 右栏：新息监控图表 =====
    float rx = sp.x + left_w + pad;
    float rw = content_w - left_w - pad * 2;

    // 标题（加粗效果）
    dl->AddText({rx+1, sp.y+pad+1}, IM_COL32(0x10, 0x1C, 0x2D, 60), "新息监控 (Innovation Monitor)");
    dl->AddText({rx, sp.y + pad}, IM_COL32(0x10, 0x1C, 0x2D, 255), "新息监控 (Innovation Monitor)");
    dl->AddText({rx + rw - 240, sp.y + pad}, IM_COL32(0x99, 0x99, 0x99, 255), "均值: ");
    dl->AddText({rx + rw - 190, sp.y + pad}, IM_COL32(0x00, 0x3D, 0x9B, 255), "—");
    dl->AddText({rx + rw - 140, sp.y + pad}, IM_COL32(0x99, 0x99, 0x99, 255), "方差: ");
    dl->AddText({rx + rw - 80, sp.y + pad}, IM_COL32(0x00, 0x3D, 0x9B, 255), "—");

    // 图表区域
    float gx = rx;
    float gy = sp.y + pad + 28;
    float gw = rw;
    float gh = wh - pad * 2 - 28;

    dl->AddRectFilled({gx, gy}, {gx + gw, gy + gh}, IM_COL32(255, 255, 255, 255), 8.0f);
    dl->AddRect({gx, gy}, {gx + gw, gy + gh}, IM_COL32(220, 220, 230, 200), 8.0f);

    // 网格
    float gm = 20;
    for (int i = 0; i <= 8; i++) {
        float y = gy + gm + (gh - gm*2) * i / 8.0f;
        dl->AddLine({gx + gm, y}, {gx + gw - gm, y}, IM_COL32(210, 210, 220, 60));
    }
    for (int i = 0; i <= 10; i++) {
        float x = gx + gm + (gw - gm*2) * i / 10.0f;
        dl->AddLine({x, gy + gm}, {x, gy + gh - gm}, IM_COL32(210, 210, 220, 60));
    }

    // ±2σ 阈值线
    float zy = gy + gh * 0.5f;
    float sg = 30.0f;
    for (int d = 0; d < (int)(gw - gm*2); d += 10) {
        dl->AddLine({gx + gm + (float)d, zy - sg}, {gx + gm + (float)d + 5, zy - sg},
            IM_COL32(0xBA, 0x1A, 0x1A, 120), 1.0f);
        dl->AddLine({gx + gm + (float)d, zy + sg}, {gx + gm + (float)d + 5, zy + sg},
            IM_COL32(0xBA, 0x1A, 0x1A, 120), 1.0f);
    }
    dl->AddText({gx + gm + 4, zy - sg - 16}, IM_COL32(0xBA, 0x1A, 0x1A, 200), "+2σ Limit");
    dl->AddText({gx + gm + 4, zy + sg + 4}, IM_COL32(0xBA, 0x1A, 0x1A, 200), "-2σ Limit");
    dl->AddLine({gx + gm, zy}, {gx + gw - gm, zy}, IM_COL32(180, 180, 190, 80));

    // 新息数据线
    if (g_sim.step > 1) {
        int n = g_sim.step;
        float pw = gw - gm * 2, xs = pw / 199.0f;
        float max_inn = 1.0f;
        for (int i = 0; i < n; i++) {
            float v = fabsf(g_sim.obs_vals[i] - g_sim.est_vals[i]);
            if (v > max_inn) max_inn = v;
        }
        float ys = (gh * 0.45f) / max_inn;

        for (int i = 1; i < n; i++) {
            float x1 = gx + gm + (i-1)*xs, x2 = gx + gm + i*xs;
            float y1 = zy - (g_sim.obs_vals[i-1] - g_sim.est_vals[i-1]) * ys;
            float y2 = zy - (g_sim.obs_vals[i] - g_sim.est_vals[i]) * ys;
            dl->AddLine({x1,y1}, {x2,y2}, IM_COL32(0x00, 0x3D, 0x9B, 255), 2.5f);
        }

        // 异常点
        float pulse = fabsf(sinf(ImGui::GetTime() * 3.0f));
        int out = 0;
        for (int i = 0; i < n; i++) {
            float inn = g_sim.obs_vals[i] - g_sim.est_vals[i];
            float iy = zy - inn * ys;
            if (iy > gy + gm && iy < gy + gh - gm && fabsf(inn) > max_inn * 0.3f) {
                out++;
                float x = gx + gm + i * xs;
                dl->AddCircleFilled({x, iy}, 4 + pulse*4, IM_COL32(0xBA,0x1A,0x1A,(int)(100+pulse*155)));
                dl->AddCircleFilled({x, iy}, 4, IM_COL32(0xBA,0x1A,0x1A,255));
            }
        }
        g_outlier_count = out;
    }

    // X轴标签
    const char* xl[] = {"T-60s","T-45s","T-30s","T-15s","Realtime"};
    for (int i = 0; i < 5; i++) {
        float x = gx + gm + (gw - gm*2) * i / 4.0f;
        dl->AddText({x - 15, gy + gh + 4}, IM_COL32(180,180,190,200), xl[i]);
    }

    // 图例
    dl->AddCircleFilled({gx + 10, gy + gh + 24}, 4, IM_COL32(0x00,0x3D,0x9B,255));
    dl->AddText({gx + 20, gy + gh + 18}, IM_COL32(0x77,0x77,0x77,255), "新息序列");
    dl->AddLine({gx+100, gy+gh+24}, {gx+120, gy+gh+24}, IM_COL32(0xBA,0x1A,0x1A,200));
    dl->AddText({gx+126, gy+gh+18}, IM_COL32(0x77,0x77,0x77,255), "置信阈值 (95%)");
    char os[32]; snprintf(os, 32, "! 异常点检测: %d", g_outlier_count);
    dl->AddText({gx + gw - 150, gy + gh + 18}, IM_COL32(0xBA,0x1A,0x1A,255), os);

    // ===== 底部状态卡片 =====
    float by = sp.y + wh + 12;
    float bcw = (content_w - pad*2 - 24) / 3.0f;
    float bch = 60;

    struct SC { const char* ic; const char* lb; const char* vl; ImVec4 bg; };
    SC sc[] = { {"S","SYSTEM STATUS","Nominal",T::Pri}, {"F","LOOP FREQUENCY","200 Hz",T::Sec}, {"G","运行模式","仿真",T::Ter} };

    for (int i = 0; i < 3; i++) {
        float x = sp.x + pad + i * (bcw + 12);
        dl->AddRectFilled({x, by}, {x+bcw, by+bch}, IM_COL32(245,245,250,255), 8.0f);
        dl->AddRect({x, by}, {x+bcw, by+bch}, IM_COL32(220,220,230,200), 8.0f);
        dl->AddCircleFilled({x+22, by+bch/2}, 16, IM_COL32((int)(sc[i].bg.x*255),(int)(sc[i].bg.y*255),(int)(sc[i].bg.z*255),255));
        ImVec2 isz = ImGui::CalcTextSize(sc[i].ic);
        dl->AddText({x+22-isz.x/2, by+bch/2-isz.y/2}, IM_COL32(255,255,255,255), sc[i].ic);
        dl->AddText({x+44, by+8}, IM_COL32(0x22,0x22,0x22,255), sc[i].lb);
        dl->AddText({x+44, by+30}, IM_COL32(0x22,0x22,0x22,255), sc[i].vl);
    }

    ImGui::EndChild(); // TelScroll
    ImGui::End();     // TelOuter
}

void DrawMissions() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float wh = ImGui::GetIO().DisplaySize.y - 100;
    float pad = 16;

    // 外层窗口
    ImGui::SetNextWindowPos({300, 60});
    ImGui::SetNextWindowSize({ww, wh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.969f, 0.976f, 0.984f, 1.0f));
    ImGui::Begin("##MisOuter", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleColor();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 wp = ImGui::GetWindowPos();
    float lx = wp.x + pad, lw = ww * 0.28f;
    float rx = lx + lw + pad, rw = ww - lw - pad * 2;
    float ty = wp.y + pad, bh = wh - pad * 2;

    // ===== 左栏 =====
    dl->AddRectFilled({lx, ty}, {lx+lw, ty+bh}, IM_COL32(255,255,255,255), 10);
    dl->AddRect({lx, ty}, {lx+lw, ty+bh}, IM_COL32(220,220,230,150), 10);

    dl->AddText({lx+12, ty+14}, IM_COL32(0x19,0x1C,0x1E,255), "飞行场景");
    dl->AddText({lx+80, ty+16}, IM_COL32(0x00,0x4A,0xC6,200), "4 个可用");

    const char* sn[] = {"正弦波","螺旋爬升","8字形","悬停"};
    const char* sd[] = {"测试EKF跟踪性能","评估高度耦合精度","复杂姿态估算","传感器漂移抑制"};
    float cy = ty + 40;
    float ch = (bh - 80) / 4.5f;
    if (ch > 48) ch = 48;

    for (int i = 0; i < 4; i++) {
        float y = cy + i * (ch + 6);
        bool act = (g_scenario == i);
        dl->AddRectFilled({lx+10, y}, {lx+lw-10, y+ch}, IM_COL32(255,255,255,255), 8);
        if (act) dl->AddRectFilled({lx+10, y+4}, {lx+13, y+ch-4}, IM_COL32(0x00,0x4A,0xC6,255), 2);
        else dl->AddRect({lx+10, y}, {lx+lw-10, y+ch}, IM_COL32(200,200,210,150), 8);

        float ix = lx + 18, iy = y + 8;
        dl->AddRectFilled({ix, iy}, {ix+24, iy+24}, act?IM_COL32(0x00,0x4A,0xC6,255):IM_COL32(230,232,235,255), 5);
        dl->AddText({lx+50, y+6}, IM_COL32(0x19,0x1C,0x1E,255), sn[i]);
        dl->AddText({lx+50, y+24}, IM_COL32(0x6B,0x6E,0x7B,255), sd[i]);

        ImGui::SetCursorPos({(float)(lx+10-wp.x), (float)(y-wp.y)});
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("##sc", {lw-20, ch})) { g_scenario=i; RecalcWithParams(); }
        ImGui::PopID();
    }

    // 预览区
    float pvy = cy + 4*(ch+6) + 8;
    float pvh = (ty+bh) - pvy - 8;
    if (pvh > 30) {
        dl->AddRectFilled({lx+10, pvy}, {lx+lw-10, pvy+pvh}, IM_COL32(0x2D,0x31,0x33,255), 10);
        // 轨迹线
        float pw = lw-20;
        for (int j=0;j<(int)pw-2;j+=2) {
            float x1=lx+10+j, x2=lx+10+j+2;
            float y1=pvy+pvh/2+sinf(j*0.05f)*(pvh*0.25f);
            float y2=pvy+pvh/2+sinf((j+2)*0.05f)*(pvh*0.25f);
            dl->AddLine({x1,y1},{x2,y2},IM_COL32(0x00,0x4A,0xC6,120),1.5f);
        }
        // 播放按钮
        dl->AddCircleFilled({lx+lw/2, pvy+pvh/2}, 20, IM_COL32(255,255,255,255));
        dl->AddText({lx+lw/2-4, pvy+pvh/2-6}, IM_COL32(0x00,0x4A,0xC6,255), ">");
        // 标签
        const char* pn[] = {"EXP-SINE-01","EXP-HELIX-01","EXP-FIGURE8-01","EXP-HOVER-01"};
        dl->AddRectFilled({lx+14, pvy+pvh-20}, {lx+120, pvy+pvh-4}, IM_COL32(0,0,0,180), 4);
        dl->AddText({lx+18, pvy+pvh-16}, IM_COL32(255,255,255,220), pn[g_scenario]);

        // 播放按钮可点击 - 用窗口内坐标
        float btn_x = lw/2 - 20;
        float btn_y = pvh - 60;
        ImGui::SetCursorPos({btn_x, btn_y});
        ImGui::PushID(999);
        if (ImGui::InvisibleButton("##pv", {40,40})) { g_paused=false; g_sim.running=true; }
        ImGui::PopID();
    }

    // ===== 右栏 =====
    // 播放控制卡片 - 计算固定高度
    float title_h = 30;  // 标题行
    float btn_h = 36;    // 按钮行
    float prog_h = 50;   // 进度区域（标题+数字+进度条+标签）
    float card_pad = 12;
    float ph = title_h + btn_h + prog_h + card_pad * 2; // 卡片总高度

    dl->AddRectFilled({rx, ty}, {rx+rw, ty+ph}, IM_COL32(255,255,255,255), 10);
    dl->AddRect({rx, ty}, {rx+rw, ty+ph}, IM_COL32(200,200,210,150), 10);

    // 标题行
    dl->AddText({rx+card_pad, ty+card_pad+6}, IM_COL32(0x19,0x1C,0x1E,255), "播放控制");
    dl->AddCircleFilled({rx+rw-90, ty+card_pad+10}, 4, IM_COL32(0x00,0x4A,0xC6,255));
    dl->AddText({rx+rw-82, ty+card_pad+4}, IM_COL32(0x00,0x4A,0xC6,255), "LIVE: READY");

    // 按钮行
    float by = ty + title_h + card_pad;
    float bw = (rw - card_pad*2 - 16) / 3.0f;

    auto PlayBtn = [&](float x, const char* label, ImVec4 bg, ImVec4 txt) {
        dl->AddRectFilled({x, by}, {x+bw, by+btn_h}, IM_COL32((int)(bg.x*255),(int)(bg.y*255),(int)(bg.z*255),255), 8);
        ImVec2 s = ImGui::CalcTextSize(label);
        dl->AddText({x+bw/2-s.x/2, by+9}, IM_COL32((int)(txt.x*255),(int)(txt.y*255),(int)(txt.z*255),255), label);
    };
    ImVec4 pw = T::Pri, pt = {1,1,1,1}, pb = T::Hover, pbt = T::TxtV;

    PlayBtn(rx+card_pad, "播放", pw, pt);
    ImGui::SetCursorPos({(float)(rx+card_pad-wp.x), (float)(by-wp.y)});
    ImGui::PushID(200);
    if (ImGui::InvisibleButton("##pl", {bw,btn_h})) { g_paused=false; g_sim.running=true; }
    ImGui::PopID();

    PlayBtn(rx+card_pad+bw+8, "暂停", pb, pbt);
    ImGui::SetCursorPos({(float)(rx+card_pad+bw+8-wp.x), (float)(by-wp.y)});
    ImGui::PushID(201);
    if (ImGui::InvisibleButton("##pa", {bw,btn_h})) { g_paused=true; }
    ImGui::PopID();

    PlayBtn(rx+card_pad+bw*2+16, "重置", pb, pbt);
    ImGui::SetCursorPos({(float)(rx+card_pad+bw*2+16-wp.x), (float)(by-wp.y)});
    ImGui::PushID(202);
    if (ImGui::InvisibleButton("##rs", {bw,btn_h})) { g_sim.step=0; g_sim.running=false; g_paused=false; }
    ImGui::PopID();

    // 进度区域（在卡片内）
    float prog_y = ty + title_h + btn_h + card_pad * 2;
    float pct = (g_sim.step > 0) ? (float)g_sim.step / 200.0f : 0.0f;
    float barw = rw - card_pad * 2;

    dl->AddText({rx+card_pad, prog_y}, IM_COL32(0x50,0x5F,0x76,255), "执行进度");
    char ps[16]; snprintf(ps,16,"%d%%",(int)(pct*100));
    ImVec2 pcts = ImGui::CalcTextSize(ps);
    dl->AddRectFilled({rx+rw-card_pad-pcts.x, prog_y}, {rx+rw-card_pad, prog_y+14}, IM_COL32(0xDB,0xE1,0xFF,255), 4);
    dl->AddText({rx+rw-card_pad-pcts.x+4, prog_y+1}, IM_COL32(0x00,0x4A,0xC6,255), ps);

    char ss[16]; snprintf(ss,16,"%d", g_sim.step);
    dl->AddText({rx+card_pad, prog_y+18}, IM_COL32(0x19,0x1C,0x1E,255), ss);
    dl->AddText({rx+card_pad+20, prog_y+22}, IM_COL32(0x73,0x76,0x86,255), "/ 200 步");
    float bary = prog_y + 34;
    dl->AddRectFilled({rx+card_pad, bary}, {rx+card_pad+barw, bary+4}, IM_COL32(230,232,235,255), 2);
    dl->AddRectFilled({rx+card_pad, bary}, {rx+card_pad+barw*pct, bary+4}, IM_COL32(0x00,0x4A,0xC6,255), 2);
    dl->AddText({rx+card_pad, bary+6}, IM_COL32(0xA0,0xA3,0xAB,255), "START_000");
    ImVec2 es = ImGui::CalcTextSize("END_200");
    dl->AddText({rx+card_pad+barw-es.x, bary+6}, IM_COL32(0xA0,0xA3,0xAB,255), "END_200");

    // 遥测三卡片 - 从播放控制卡片下方开始
    float by2 = ty + ph + 12;
    float bh3 = (ty+bh) - by2 - 40;
    if (bh3 < 60) bh3 = 60;
    float cw3 = (rw - 32) / 3.0f;

    // 根据场景和当前步数动态计算遥测值
    float conv_val = 0.001f + g_sim.step * 0.0001f * (g_scenario == 3 ? 0.5f : 1.0f);
    int gps_sats = 8 + (g_scenario == 1 ? 2 : g_scenario == 2 ? 3 : g_scenario == 3 ? 4 : 1);
    float lat_val = 5.0f + g_sim.step * 0.05f * (g_scenario == 2 ? 1.5f : 1.0f);

    char conv_s[16], gps_s[16], lat_s[16];
    snprintf(conv_s, 16, "%.4f", conv_val);
    snprintf(gps_s, 16, "%d", gps_sats);
    snprintf(lat_s, 16, "%.1f", lat_val);

    struct { const char* lb; const char* vl; const char* un; float r,g,b; }
    bi[] = { {"IMU CONV",conv_s,"RMS",0,0.29f,0.78f},
             {"GPS LOCK",gps_s,"SATS",0.31f,0.37f,0.46f},
             {"LATENCY",lat_s,"MS",0.73f,0.10f,0.10f} };

    for (int i = 0; i < 3; i++) {
        float bx = rx + 10 + i*(cw3+6);
        dl->AddRectFilled({bx, by2}, {bx+cw3, by2+bh3}, IM_COL32(255,255,255,255), 8);
        dl->AddRect({bx, by2}, {bx+cw3, by2+bh3}, IM_COL32(200,200,210,150), 8);
        dl->AddText({bx+8, by2+6}, IM_COL32((int)(bi[i].r*255),(int)(bi[i].g*255),(int)(bi[i].b*255),255), bi[i].lb);
        dl->AddText({bx+8, by2+24}, IM_COL32(0x19,0x1C,0x1E,255), bi[i].vl);
        ImVec2 us = ImGui::CalcTextSize(bi[i].un);
        dl->AddText({bx+8+ImGui::CalcTextSize(bi[i].vl).x+4, by2+28}, IM_COL32(0x73,0x76,0x86,255), bi[i].un);
        float fill = (i==0)?0.8f:(i==1)?0.75f:0.4f;
        float fr=(i==2)?0.73f:0.0f, fg=(i==2)?0.10f:0.29f, fb=(i==2)?0.10f:0.78f;
        dl->AddRectFilled({bx+8, by2+bh3-14}, {bx+8+cw3-16, by2+bh3-10}, IM_COL32(230,232,235,255), 2);
        dl->AddRectFilled({bx+8, by2+bh3-14}, {bx+8+(cw3-16)*fill, by2+bh3-10},
            IM_COL32((int)(fr*255),(int)(fg*255),(int)(fb*255),255), 2);
    }

    // 任务信息条
    const char* mission_names[] = {"EKF_SINE_TRACK", "EKF_HELIX_EVAL", "EKF_FIGURE8_TEST", "EKF_HOVER_STAB"};
    float iy = ty + bh - 36;
    dl->AddRectFilled({rx, iy}, {rx+rw, iy+32}, IM_COL32(218,226,253,255), 8);
    dl->AddText({rx+12, iy+8}, IM_COL32(0x19,0x1C,0x1E,255), mission_names[g_scenario]);
    dl->AddText({rx+rw-80, iy+8}, IM_COL32(0x00,0x4A,0xC6,255), "查看报告");

    ImGui::End();
}

void DrawBottom() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float wh = ImGui::GetIO().DisplaySize.y - 100;
    float bh = wh - wh*0.6f;
    float bw = ww/2 - 12;
    float cur_rmse = CalcRMSE();

    // Performance
    ImGui::SetNextWindowPos({300, 60 + wh*0.6f + 8});
    ImGui::SetNextWindowSize({bw, bh - 16});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, T::Card);
    ImGui::Begin("##Perf", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {20,16});
    ImGui::TextColored(T::Out, "性能对比");
    ImGui::Spacing();

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Bar helper
    auto bar = [&](const char* name, float rmse, float max_val, ImVec4 color, bool highlight) {
        if (highlight) ImGui::TextColored(T::Pri, "%s", name);
        else ImGui::Text("%s", name);
        ImGui::SameLine(bw - 80);
        ImGui::TextColored(color, "%.3f", rmse);
        ImVec2 bp = ImGui::GetCursorScreenPos();
        dl->AddRectFilled(bp, {bp.x+bw-40, bp.y+24}, IM_COL32(0xE8,0xE8,0xE8,255));
        float bar_w = (max_val > 0) ? ((bw-40) * rmse / max_val) : 0;
        if (bar_w < 4) bar_w = 4;
        dl->AddRectFilled(bp, {bp.x + bar_w, bp.y+24},
            IM_COL32((int)(color.x*255),(int)(color.y*255),(int)(color.z*255),255));
        ImGui::Dummy({0,28});
    };

    // 收集所有RMSE找出最大值
    float rmses[4] = {g_rmse_standard, g_rmse_student_t, g_rmse_adaptive, g_rmse_joseph};
    float worst = 0;
    for (float r : rmses) if (r > worst) worst = r;
    if (worst < 0.01f) worst = 1.0f;

    bar("标准EKF",  rmses[0], worst, T::Out,    g_algo == 0);
    bar("Student-t", rmses[1], worst, T::Out,    g_algo == 1);
    bar("自适应EKF", rmses[2], worst, T::Blue,   g_algo == 2);
    bar("Joseph",    rmses[3], worst, T::Out,    g_algo == 3);

    // Highlight current
    if (g_sim.step > 0) {
        ImGui::Separator();
        ImGui::Text("当前: %s", AlgoName(g_algo));
        ImGui::SameLine(bw-80);
        ImGui::TextColored(T::Green, "%.3f", cur_rmse);
    }

    ImGui::PopStyleVar();
    ImGui::End();

    // Algorithm Details
    ImGui::SetNextWindowPos({300 + bw + 12, 60 + wh*0.6f + 8});
    ImGui::SetNextWindowSize({bw, bh - 16});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, T::Card);
    ImGui::Begin("##Detail", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {20,16});
    ImGui::TextColored(T::Out, "算法详情");
    ImGui::Spacing(); ImGui::Spacing();

    struct { const char* t; const char* d; bool ok; } feats[] = {
        {"递归协方差缩放", "基于新息残差动态调整Q矩阵", true},
        {"模型失配检测", "统计假设检验，早期漂移识别", true},
        {"Student-t鲁棒更新", "抗野值干扰，适合模型失配", g_algo==1},
        {"自适应噪声估计", "动态窗口调整噪声协方差", g_algo==2},
        {"Joseph形式更新", "保证协方差矩阵正定性", g_algo==3},
    };
    for (auto& f : feats) {
        ImGui::PushStyleColor(ImGuiCol_Text, f.ok ? T::Sec : T::TxtV);
        ImGui::Text(f.ok ? "[OK]" : "[--]");
        ImGui::PopStyleColor();
        ImGui::SameLine(30);
        ImGui::Text("%s", f.t);
        ImGui::TextColored(T::Out, "    %s", f.d);
        ImGui::Spacing();
    }

    ImGui::PopStyleVar();
    ImGui::End();
}

// ============================================================
// Status Bar
// ============================================================

void DrawAnalysis() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float wh = ImGui::GetIO().DisplaySize.y - 100;
    float pad = 20;

    // 外层窗口
    ImGui::SetNextWindowPos({300, 60});
    ImGui::SetNextWindowSize({ww, wh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.969f, 0.976f, 0.984f, 1.0f));
    ImGui::Begin("##AnaOuter", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleColor();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 wp = ImGui::GetWindowPos();

    // 页面标题
    float title_y = wp.y + pad;
    dl->AddText({wp.x + pad, title_y}, IM_COL32(0x0F,0x17,0x2A,255), "滤波器性能分析报告");
    dl->AddText({wp.x + pad, title_y + 32}, IM_COL32(0x64,0x74,0x8B,255),
        "Drone EKF Algorithm Multi-variant Evaluation - Session 4029-X");

    // 状态指示器
    dl->AddCircleFilled({wp.x + ww - 180, title_y + 8}, 5, IM_COL32(0x22,0xC5,0x5E,255));
    dl->AddText({wp.x + ww - 170, title_y + 2}, IM_COL32(0x64,0x74,0x8B,255), "Real-time Tracking OK");

    // 分割线
    float div_y = title_y + 50;
    dl->AddLine({wp.x + pad, div_y}, {wp.x + ww - pad, div_y}, IM_COL32(0xE2,0xE8,0xF0,255));

    // ===== 左栏：算法对比 =====
    float left_w = ww * 0.55f;
    float lx = wp.x + pad;
    float ly = div_y + pad;

    dl->AddText({lx, ly}, IM_COL32(0x0F,0x17,0x2A,255), "算法对比分析");
    dl->AddText({lx + 130, ly + 2}, IM_COL32(0x0F,0x17,0x2A,200), "COMPARE_MODE: RMSE");
    ly += 36;

    // 4个算法卡片
    float card_w = left_w;
    float card_h = 70;
    float card_gap = 12;

    struct AlgoInfo { const char* name; const char* desc; float rmse; bool optimal; float diff; const char* diff_label; };
    float std_rmse = g_rmse_standard > 0 ? g_rmse_standard : 0.0f;
    float stu_rmse = g_rmse_student_t > 0 ? g_rmse_student_t : 0.0f;
    float ada_rmse = g_rmse_adaptive > 0 ? g_rmse_adaptive : 0.0f;
    float jos_rmse = g_rmse_joseph > 0 ? g_rmse_joseph : 0.0f;

    // 动态判断最优算法
    float rmse_arr[4] = {std_rmse, stu_rmse, ada_rmse, jos_rmse};
    int best_idx = 0;
    for (int i = 1; i < 4; i++) {
        if (rmse_arr[i] < rmse_arr[best_idx]) best_idx = i;
    }

    AlgoInfo ais[] = {
        {"标准EKF",  "传统扩展卡尔曼滤波实现",           std_rmse, best_idx==0, (1-std_rmse/std_rmse)*100, ""},
        {"Student-t","引入t分布处理非高斯噪声及离群点",   stu_rmse, best_idx==1, (1-stu_rmse/std_rmse)*100, ""},
        {"自适应EKF","动态调整过程噪声Q与测量噪声R",     ada_rmse, best_idx==2, (1-ada_rmse/std_rmse)*100, ""},
        {"Joseph",   "采用Joseph形式维持协方差矩阵正定性",jos_rmse, best_idx==3, (1-jos_rmse/std_rmse)*100, ""},
    };

    for (int i = 0; i < 4; i++) {
        float cy = ly + i * (card_h + card_gap);

        // 卡片背景
        if (ais[i].optimal) {
            // 最优：绿色边框
            dl->AddRectFilled({lx, cy}, {lx+card_w, cy+card_h}, IM_COL32(255,255,255,255), 4);
            dl->AddRect({lx, cy}, {lx+card_w, cy+card_h}, IM_COL32(0x22,0xC5,0x5E,255), 2);
            // "Optimal" 标签
            dl->AddRectFilled({lx+card_w-60, cy}, {lx+card_w, cy+20}, IM_COL32(0x22,0xC5,0x5E,255), 4);
            ImVec2 ols = ImGui::CalcTextSize("Optimal");
            dl->AddText({lx+card_w-58, cy+2}, IM_COL32(255,255,255,255), "Optimal");
        } else {
            dl->AddRectFilled({lx, cy}, {lx+card_w, cy+card_h}, IM_COL32(255,255,255,255), 4);
            dl->AddRect({lx, cy}, {lx+card_w, cy+card_h}, IM_COL32(226,232,240,255), 1);
        }

        // 算法名称
        ImVec4 nc = ais[i].optimal ? ImVec4(0.13f,0.77f,0.37f,1) : T::Txt;
        dl->AddText({lx+14, cy+10}, IM_COL32(0x19,0x1C,0x1E,255), ais[i].name);
        dl->AddText({lx+14, cy+32}, IM_COL32(0x6B,0x70,0x80,255), ais[i].desc);

        // RMSE 标签 + 数值（右对齐，固定位置）
        float rmse_x = lx + card_w - 130;
        dl->AddText({rmse_x, cy+6}, IM_COL32(0x76,0x77,0x7D,200), "RMSE");

        // RMSE 数值
        char rmse_s[16]; snprintf(rmse_s, 16, "%.2f", ais[i].rmse);
        dl->AddText({rmse_x+40, cy+6}, IM_COL32(0x19,0x1C,0x1E,255), rmse_s);
        dl->AddText({rmse_x+40+ImGui::CalcTextSize(rmse_s).x+2, cy+10}, IM_COL32(0x6B,0x70,0x80,200), "m");

        // 变化百分比或Baseline
        if (i == 0) {
            dl->AddText({rmse_x+40, cy+30}, IM_COL32(0x6B,0x70,0x80,200), "Baseline");
        } else if (ais[i].diff > 0) {
            char diff_s[16]; snprintf(diff_s, 16, "+%.1f%%", ais[i].diff);
            dl->AddText({rmse_x+40, cy+30}, IM_COL32(0x22,0xC5,0x5E,255), diff_s);
        }
    }

    // ===== 右栏：算法详情 =====
    float rx = wp.x + left_w + pad * 2;
    float rw = ww - left_w - pad * 3;
    float ry = ly;

    dl->AddText({rx, ry}, IM_COL32(0x0F,0x17,0x2A,255), "算法详情");
    ry += 36;

    // 算法详情卡片
    dl->AddRectFilled({rx, ry}, {rx+rw, ry + wh - pad*2 - 60}, IM_COL32(255,255,255,255), 4);
    dl->AddRect({rx, ry}, {rx+rw, ry + wh - pad*2 - 60}, IM_COL32(226,232,240,255), 1);

    float dy = ry + 16;
    float dw = rw - 32;

    // 详情内容
    struct DetailItem { const char* icon; const char* title; const char* desc; bool active; };
    DetailItem di[] = {
        {"[OK]", "递归协方差缩放", "基于新息残差动态调整Q矩阵", true},
        {"[OK]", "模型失配检测", "统计假设检验，早期漂移识别", true},
        {"[OK]", "Student-t鲁棒更新", "抗野值干扰，适合模型失配", true},
        {"[OK]", "自适应噪声估计", "动态窗口调整噪声协方差", true},
        {"[--]", "高计算负载", "NEON 向量加速 (示意)", false},
    };

    for (int i = 0; i < 5; i++) {
        ImVec4 ic = di[i].active ? ImVec4(0.22f,0.77f,0.37f,1) : ImVec4(0.76f,0.77f,0.77f,1);
        dl->AddText({rx+16, dy}, IM_COL32((int)(ic.x*255),(int)(ic.y*255),(int)(ic.z*255),255), di[i].icon);
        dl->AddText({rx+40, dy}, IM_COL32(0x19,0x1C,0x1E,255), di[i].title);
        dl->AddText({rx+40, dy+18}, IM_COL32(0x6B,0x70,0x80,255), di[i].desc);
        dy += 40;
        dl->AddLine({rx+16, dy-4}, {rx+dw+16, dy-4}, IM_COL32(226,232,240,100));
    }

    // FILTER CHANGES 区域
    dy += 12;
    dl->AddText({rx+16, dy}, IM_COL32(0x0F,0x17,0x2A,255), "FILTER CHANGES");
    dy += 20;

    // 模拟柱状图
    float bar_max_h = 80;
    float bar_w = 20;
    float bar_gap = 8;
    float bar_base = ry + wh - pad*2 - 80;
    for (int i = 0; i < 8; i++) {
        float bx = rx + 16 + i * (bar_w + bar_gap);
        float bh2 = bar_max_h * (0.3f + (i % 3) * 0.25f + (i % 2) * 0.15f);
        ImVec4 bc = (i == 4) ? ImVec4(0.22f,0.77f,0.37f,1) : ImVec4(0x19/255.0f,0x1C/255.0f,0x1E/255.0f,0.6f);
        dl->AddRectFilled({bx, bar_base-bh2}, {bx+bar_w, bar_base},
            IM_COL32((int)(bc.x*255),(int)(bc.y*255),(int)(bc.z*255),255), 2);
    }

    ImGui::End();
}

void DrawStatusBar() {
    float ww = ImGui::GetIO().DisplaySize.x - 240;
    float sh = 40;
    ImGui::SetNextWindowPos({300, ImGui::GetIO().DisplaySize.y - sh});
    ImGui::SetNextWindowSize({ww, sh});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, {0.957f,0.957f,0.957f,1});
    ImGui::Begin("##Status", NULL,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleColor();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {24,8});
    ImGui::PushStyleColor(ImGuiCol_Text, T::TxtV);

    ImGui::Text("算法: %s", AlgoName(g_algo));
    ImGui::SameLine(180);
    ImGui::Text("RMSE: "); ImGui::SameLine(); ImGui::TextColored(T::Sec, "%.3f", CalcRMSE());
    ImGui::SameLine(340);
    ImGui::Text("最大误差: "); ImGui::SameLine(); ImGui::TextColored(T::Ter, "%.2f", CalcMaxErr());
    ImGui::SameLine(520);
    ImGui::Text("步数: %d/200", g_sim.step);

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    ImGui::End();
}

// ============================================================
// WinMain
// ============================================================

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int) {
    WNDCLASSEXW wc = {sizeof(wc), CS_CLASSDC, WndProc, 0, 0,
                     GetModuleHandle(NULL), NULL, NULL, NULL, NULL,
                     L"EKF", NULL};
    RegisterClassExW(&wc);

    HWND hwnd = CreateWindowExW(0, wc.lpszClassName, L"EKF System",
        WS_OVERLAPPEDWINDOW, 100, 100, 1280, 720,
        NULL, NULL, hInstance, NULL);

    if (!CreateDeviceD3D(hwnd)) {
        CleanupDeviceD3D();
        UnregisterClassW(wc.lpszClassName, hInstance);
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = NULL; // 禁用ini保存，防止Debug窗口残留

    SetupTheme();

    io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\msyh.ttc", 16.0f,
        NULL, io.Fonts->GetGlyphRangesChineseFull());

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    RunAllAlgorithms();

    bool running = true;
    MSG msg = {};
    while (running) {
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) running = false;
        }
        if (!running) break;

        // Auto-play: advance one step per frame (~60fps, so ~3.3s for 200 steps)
        if (g_sim.running && !g_paused && g_sim.step < (int)g_sim.est_vals.size()) {
            g_sim.step++;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        if (g_current_page == -1) {
            // 首页：显示导航栏
            DrawNavBar();
            DrawHomePage();
        } else {
            // 功能页：显示参数调节栏 + 返回按钮
            DrawSidebar();
            DrawTopHeader();
            switch (g_current_page) {
            case 0: DrawChart(); DrawBottom(); break;
            case 1: DrawTelemetry(); break;
            case 2: DrawMissions(); break;
            case 3: DrawAnalysis(); break;
            }
            DrawStatusBar();
        }

        ImGui::Render();
        const float cc[4] = {1,1,1,1};
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, cc);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_pSwapChain->Present(1, 0);
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, hInstance);
    return 0;
}

// ============================================================
// DirectX Implementation (unchanged)
// ============================================================

bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate = {60,1};
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc = {1,0};
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    D3D_FEATURE_LEVEL fl;
    const D3D_FEATURE_LEVEL flArr[2] = {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0};
    if (D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0,
        flArr, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &fl,
        &g_pd3dDeviceContext) != S_OK) return false;
    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = NULL; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = NULL; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = NULL; }
}

void CreateRenderTarget() {
    ID3D11Texture2D* bb;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&bb));
    g_pd3dDevice->CreateRenderTargetView(bb, NULL, &g_mainRenderTargetView);
    bb->Release();
}

void CleanupRenderTarget() {
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = NULL; }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;
    switch (msg) {
    case WM_SIZE:
        if (g_pd3dDevice && wParam != SIZE_MINIMIZED) {
            CleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, LOWORD(lParam), HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
            CreateRenderTarget();
        }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) return 0;
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}
