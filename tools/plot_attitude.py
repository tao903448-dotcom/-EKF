#!/usr/bin/env python3
"""
plot_attitude.py —— 把 attitude_demo 导出的轨迹 CSV 画成对比图。

用法：
    ./build/attitude_demo traj.csv          # 先生成 CSV
    python3 tools/plot_attitude.py traj.csv # 再出图(默认存 attitude_plot.png)

CSV 列：tag,step,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,att_err_deg
其中 tag ∈ {MANEUVER_standard, MANEUVER_adaptive}，对比标准 EKF 与自适应 EKF。

依赖：matplotlib（pip install matplotlib）。
"""
import csv
import sys
from collections import defaultdict


def load(path):
    data = defaultdict(lambda: defaultdict(list))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            tag = row["tag"]
            for k, v in row.items():
                if k != "tag":
                    data[tag][k].append(float(v))
    return data


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "traj.csv"
    out = sys.argv[2] if len(sys.argv) > 2 else "attitude_plot.png"
    data = load(path)
    tags = list(data.keys())
    if not tags:
        print("CSV 为空或格式不符")
        return

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("需要 matplotlib：pip install matplotlib")
        return

    dt = 0.005
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    # 用英文标签，避免不同机器缺少 CJK 字体时出现方框
    fig.suptitle("Quadrotor Attitude EKF: Standard vs Adaptive (MANEUVER mismatch)",
                 fontsize=13)

    ref = next((t for t in tags if "standard" in t), tags[0])
    t = [s * dt for s in data[ref]["step"]]

    for ax, comp, name in zip(
        [axes[0, 0], axes[0, 1], axes[1, 0]],
        ["roll", "pitch", "yaw"],
        ["Roll", "Pitch", "Yaw"],
    ):
        ax.plot(t, data[ref][comp + "_true"], "k-", lw=1.6, label="truth")
        for tag in tags:
            style = "--" if "standard" in tag else "-"
            label = "Standard EKF" if "standard" in tag else "Adaptive EKF"
            ax.plot([s * dt for s in data[tag]["step"]],
                    data[tag][comp + "_est"], style, lw=1.0, alpha=0.85, label=label)
        ax.set_title(name); ax.set_xlabel("time (s)"); ax.set_ylabel("angle (deg)")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = axes[1, 1]
    for tag in tags:
        label = "Standard EKF" if "standard" in tag else "Adaptive EKF"
        ax.plot([s * dt for s in data[tag]["step"]], data[tag]["att_err_deg"],
                lw=1.0, label=label)
    ax.set_title("Attitude error"); ax.set_xlabel("time (s)"); ax.set_ylabel("error (deg)")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=130)
    print(f"已保存图像：{out}")


if __name__ == "__main__":
    main()
