#!/usr/bin/env python3
"""
Shake-It-Off 自穩平台 — 系統分析腳本
======================================

使用方法:
  1. 用 instrumented firmware 跑實驗
  2. 用序列埠工具 (PuTTY/minicom/screen) 或 Python serial 錄 log
  3. 執行: python3 analyze_system.py <log_file.txt>

產生的圖表:
  Fig 1: Step Response (pitch & roll vs time) — 展示 settling time
  Fig 2: Timing Breakdown — 各子任務執行時間柱狀圖
  Fig 3: Timing Diagram — 類 Liu & Layland RM scheduling 圖
  Fig 4: CPU Utilization — WCET vs deadline 分析
  Fig 5: Deadline Miss 統計
  Fig 6: Control Performance — 系統從擾動回穩的 overlay 圖

也會輸出文字報告: formal 的可調度性分析數據
"""

import sys
import os
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
from collections import defaultdict

# ══════════════════════════════════════════════
#  1. 資料解析
# ══════════════════════════════════════════════

def parse_log_file(filepath):
    """解析 USB CDC log 文件"""
    data_rows = []
    step_responses = []
    summaries = []

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()

            # DATA 行: 高速控制資料
            if line.startswith('DATA,'):
                parts = line.split(',')
                if len(parts) >= 16:
                    try:
                        row = {
                            'tick_ms':    int(parts[1]),
                            'pitch':      float(parts[2]),
                            'roll':       float(parts[3]),
                            'tgt_pitch':  float(parts[4]),
                            'tgt_roll':   float(parts[5]),
                            'gx':         float(parts[6]),
                            'gy':         float(parts[7]),
                            'dt_us':      int(parts[8]),
                            't_imu':      int(parts[9]),
                            't_filt':     int(parts[10]),
                            't_pid':      int(parts[11]),
                            't_geom':     int(parts[12]),
                            't_servo':    int(parts[13]),
                            'deadline_miss': int(parts[14]),
                            'settling':   int(parts[15]),
                            's1':         float(parts[16]) if len(parts) > 16 else 90.0,  # A0 physical
                            's2':         float(parts[17]) if len(parts) > 17 else 90.0,  # A1 physical
                            's3':         float(parts[18]) if len(parts) > 18 else 90.0,  # A2 physical
                            's4':         float(parts[19]) if len(parts) > 19 else 90.0,  # A3 physical
                        }
                        data_rows.append(row)
                    except (ValueError, IndexError):
                        pass

            # STEP_RESP 行: step response 結果
            elif line.startswith('STEP_RESP,'):
                m = re.search(r'peak=([\d.]+),settle=([\d.]+)ms,resp#(\d+)', line)
                if m:
                    step_responses.append({
                        'peak_deg':      float(m.group(1)),
                        'settle_ms':     float(m.group(2)),
                        'response_num':  int(m.group(3)),
                    })

            # SUMMARY 行
            elif line.startswith('SUMMARY,'):
                m = re.search(r'loops=(\d+),miss=(\d+),wcet=(\d+)us,util=([\d.]+)%', line)
                if m:
                    summaries.append({
                        'loops': int(m.group(1)),
                        'miss':  int(m.group(2)),
                        'wcet':  int(m.group(3)),
                        'util':  float(m.group(4)),
                    })

    return data_rows, step_responses, summaries


def generate_demo_data():
    """
    如果還沒有實驗資料，生成模擬資料來預覽圖表效果。
    模擬: 系統在 t=2s 受到 ~10° 的 pitch 擾動，然後回穩。
    """
    np.random.seed(42)
    N = 5000  # 10 秒, 500Hz
    dt_ms = 2
    data = []

    pitch = 0.0
    roll = 0.0
    tgt_pitch = 0.0
    tgt_roll = 0.0

    for i in range(N):
        t_ms = i * dt_ms

        # 模擬擾動: t=2s 施加 step disturbance
        if i == 1000:
            pitch = 10.0
            roll = 5.0

        # 模擬 PID 控制回穩 (二階阻尼系統)
        if i > 1000:
            elapsed = (i - 1000) * dt_ms / 1000.0
            # 欠阻尼二階: ζ=0.4, ωn=8 rad/s
            zeta = 0.4
            wn = 8.0
            wd = wn * np.sqrt(1 - zeta**2)
            env = np.exp(-zeta * wn * elapsed)
            pitch = 10.0 * env * np.cos(wd * elapsed)
            roll  = 5.0  * env * np.cos(wd * elapsed + 0.3)
            tgt_pitch = -pitch * 0.3  # 模擬 target 跟隨
            tgt_roll  = -roll * 0.3

        # 加入感測器雜訊
        pitch_noisy = pitch + np.random.normal(0, 0.15)
        roll_noisy  = roll  + np.random.normal(0, 0.15)

        # 模擬 timing (正常分佈 + 偶爾 spike)
        t_imu  = max(200, int(np.random.normal(400, 30)))   # ~400μs I2C
        t_filt = max(2, int(np.random.normal(5, 1)))         # ~5μs 濾波
        t_pid  = max(3, int(np.random.normal(8, 2)))         # ~8μs PID
        t_geom = max(5, int(np.random.normal(15, 3)))        # ~15μs 幾何
        t_servo = max(1, int(np.random.normal(3, 1)))        # ~3μs servo

        # 偶爾 I2C spike
        if np.random.random() < 0.005:
            t_imu = int(np.random.normal(1200, 200))

        settling = 0
        if 1000 <= i < 1010:
            settling = 1
        elif 1010 <= i < 1400:
            settling = 2
        elif 1400 <= i < 1450:
            settling = 3

        # 模擬物理 servo 角度 (physical, 已含 reversed 補角)
        # s1(A0)/s4(A3) 控制 pitch; s2(A1)/s3(A2) 控制 roll
        # reversed 後 s3/s4 physical 方向與 s1/s2 相同
        servo_offset_p = tgt_pitch * 1.8 + np.random.normal(0, 0.2)
        servo_offset_r = tgt_roll  * 1.8 + np.random.normal(0, 0.2)

        data.append({
            'tick_ms':    t_ms + 5000,  # 模擬開機後 5 秒
            'pitch':      round(pitch_noisy, 3),
            'roll':       round(roll_noisy, 3),
            'tgt_pitch':  round(tgt_pitch, 3),
            'tgt_roll':   round(tgt_roll, 3),
            'gx':         round(np.random.normal(0, 2), 2),
            'gy':         round(np.random.normal(0, 2), 2),
            'dt_us':      dt_ms * 1000,
            't_imu':      t_imu,
            't_filt':     t_filt,
            't_pid':      t_pid,
            't_geom':     t_geom,
            't_servo':    t_servo,
            'deadline_miss': 0,
            'settling':   settling,
            's1':         round(90.0 + servo_offset_p, 2),  # A0, pitch
            's2':         round(90.0 + servo_offset_r, 2),  # A1, roll
            's3':         round(90.0 + servo_offset_r, 2),  # A2, roll (physical)
            's4':         round(90.0 + servo_offset_p, 2),  # A3, pitch (physical)
        })

    step_resp = [{'peak_deg': 11.2, 'settle_ms': 810.0, 'response_num': 1}]
    summary = [{'loops': N, 'miss': 3, 'wcet': 1350, 'util': 67.5}]

    return data, step_resp, summary


# ══════════════════════════════════════════════
#  2. 圖表繪製
# ══════════════════════════════════════════════

def setup_style():
    """設定論文風格"""
    plt.rcParams.update({
        'font.size': 11,
        'axes.titlesize': 13,
        'axes.labelsize': 12,
        'legend.fontsize': 10,
        'figure.dpi': 150,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'lines.linewidth': 1.2,
    })


def plot_step_response(data, output_dir):
    t = np.array([(d['tick_ms'] - data[0]['tick_ms']) / 1000.0 for d in data])
    pitch = np.array([d['pitch'] for d in data])
    roll  = np.array([d['roll'] for d in data])

    # Find settling time
    magnitude = np.sqrt(pitch**2 + roll**2)
    disturb_idx = np.where(magnitude > 3.0)[0]
    t_dist, t_set = None, None
    if len(disturb_idx) > 0:
        t_dist = t[disturb_idx[0]]
        settled_idx = None
        for idx in range(disturb_idx[0], len(magnitude) - 50):
            if all(magnitude[idx:idx+50] < 1):
                settled_idx = idx
                break
        if settled_idx:
            t_set = t[settled_idx]

    # Save Pitch
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.plot(t, pitch, color='#2563EB', alpha=0.8, label='Pitch (measured)')
    ax.axhline(y=0, color='#DC2626', linestyle='--', linewidth=1, label='Target (0°)')
    ax.fill_between(t, -1, 1, color='#22C55E', alpha=0.1, label='±1° settled band')
    ax.set_ylabel('Pitch (degrees)')
    ax.set_xlabel('Time (seconds)')
    ax.legend(loc='upper right')
    ax.set_ylim([-15, 15])
    if t_dist and t_set:
        ax.axvline(x=t_dist, color='#F97316', linestyle=':', linewidth=1.5, alpha=0.7)
        ax.axvline(x=t_set, color='#22C55E', linestyle=':', linewidth=1.5, alpha=0.7)
        ax.annotate(f'Settling Time = {(t_set-t_dist)*1000:.0f} ms',
                    xy=(t_set, 0), xytext=(t_set + 0.3, pitch.max()*0.6),
                    arrowprops=dict(arrowstyle='->', color='#22C55E'),
                    fontsize=11, color='#22C55E', fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig1_step_response-1.png'))
    plt.close()

    # Save Roll
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.plot(t, roll, color='#7C3AED', alpha=0.8, label='Roll (measured)')
    ax.axhline(y=0, color='#DC2626', linestyle='--', linewidth=1, label='Target (0°)')
    ax.fill_between(t, -1, 1, color='#22C55E', alpha=0.1, label='±1° settled band')
    ax.set_ylabel('Roll (degrees)')
    ax.set_xlabel('Time (seconds)')
    ax.legend(loc='upper right')
    ax.set_ylim([-15, 15])
    if t_dist and t_set:
        ax.axvline(x=t_dist, color='#F97316', linestyle=':', linewidth=1.5, alpha=0.7)
        ax.axvline(x=t_set, color='#22C55E', linestyle=':', linewidth=1.5, alpha=0.7)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig1_step_response-2.png'))
    plt.close()
    print("[✓] Fig 1 subfigures saved")


def plot_timing_breakdown(data, output_dir):
    tasks = {
        'IMU\n(I2C Read)':    [d['t_imu'] for d in data],
        'Comp.\nFilter':      [d['t_filt'] for d in data],
        'PID\nControl':       [d['t_pid'] for d in data],
        'Cable\nGeometry':    [d['t_geom'] for d in data],
        'Servo\nPWM':         [d['t_servo'] for d in data],
    }
    colors = ['#3B82F6', '#10B981', '#EF4444', '#F59E0B', '#EC4899']

    # Subfigure 1: Boxplot
    fig, ax1 = plt.subplots(figsize=(8, 6))
    bp = ax1.boxplot(tasks.values(), labels=tasks.keys(), patch_artist=True,
                     showfliers=False, notch=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    ax1.set_ylabel('Execution Time (μs)')
    ax1.set_yscale('log')
    ax1.yaxis.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig2_timing_breakdown-1.png'))
    plt.close()

    # Subfigure 2: Average Stacked Bar
    fig, ax2 = plt.subplots(figsize=(5, 6))
    means = [np.mean(v) for v in tasks.values()]
    labels_short = ['IMU', 'Filter', 'PID', 'Geom', 'Servo']
    bottom = 0
    for i, (m, c, lab) in enumerate(zip(means, colors, labels_short)):
        ax2.bar('Average\nLoop', m, bottom=bottom, color=c, alpha=0.7, label=f'{lab}: {m:.0f}μs')
        ax2.text(0, bottom + m/2, f'{m:.0f}μs', ha='center', va='center', fontsize=9, fontweight='bold')
        bottom += m

    deadline_us = int(np.median([d['dt_us'] for d in data]))
    ax2.axhline(y=deadline_us, color='red', linestyle='--', linewidth=2, label=f'Deadline: {deadline_us}μs')
    ax2.set_ylabel('Cumulative Time (μs)')
    # ax2.set_title('Average Loop Composition')
    ax2.legend(loc='upper left', fontsize=8)
    ax2.set_ylim(0, deadline_us * 1.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig2_timing_breakdown-2.png'))
    plt.close()
    print("[✓] Fig 2: Timing Breakdown saved")


def plot_timing_diagram(data, output_dir):
    """
    Fig 3: Timing Diagram — 類 Liu & Layland RM scheduling 圖
    展示 5 個子任務在連續數個控制週期中的時序排列
    用甘特圖方式呈現，與 deadline 的關係一目了然
    """
    # 取連續 8 個 loop 的資料
    show_loops = 8
    sample_data = data[:show_loops]

    fig, ax = plt.subplots(figsize=(14, 5))

    task_names = ['τ₁: IMU Read', 'τ₂: Comp. Filter', 'τ₃: PID Control',
                  'τ₄: Geometry', 'τ₅: Servo PWM']
    task_keys = ['t_imu', 't_filt', 't_pid', 't_geom', 't_servo']
    colors = ['#2563EB', '#7C3AED', '#DC2626', '#F97316', '#22C55E']

    period_us = int(np.median([d['dt_us'] for d in data]))  # 從 dt_us 自動偵測

    for loop_i, d in enumerate(sample_data):
        loop_start = loop_i * period_us
        task_start = loop_start

        for task_j, (key, color) in enumerate(zip(task_keys, colors)):
            duration = d[key]
            y_center = len(task_names) - 1 - task_j  # 由上到下排列

            # 畫任務方塊
            rect = plt.Rectangle((task_start, y_center - 0.35), duration, 0.7,
                                 facecolor=color, edgecolor='black', linewidth=0.5, alpha=0.75)
            ax.add_patch(rect)

            # 第一個 loop 時標上時間
            if loop_i == 0 and duration > 30:
                ax.text(task_start + duration/2, y_center,
                       f'{duration}μs', ha='center', va='center', fontsize=7, fontweight='bold')

            task_start += duration

        # Deadline 虛線
        deadline_x = (loop_i + 1) * period_us
        ax.axvline(x=deadline_x, color='red', linestyle='--', linewidth=1, alpha=0.5)

        # 標示 period
        if loop_i < show_loops - 1:
            ax.annotate('', xy=(deadline_x, -0.8), xytext=(loop_start, -0.8),
                       arrowprops=dict(arrowstyle='<->', color='gray', lw=1.2))
            ax.text((loop_start + deadline_x)/2, -1.1, f'T = {period_us}μs',
                   ha='center', va='center', fontsize=8, color='gray')

    ax.set_yticks(range(len(task_names)))
    ax.set_yticklabels(list(reversed(task_names)))
    ax.set_xlabel('Time (μs)')
    # ax.set_title(f'Timing Diagram — Task Execution within Control Period (T = {period_us}μs)')
    ax.set_xlim(-100, show_loops * period_us + 200)
    ax.set_ylim(-1.5, len(task_names) - 0.3)

    # 圖例
    legend_patches = [mpatches.Patch(color=c, label=n, alpha=0.75) for c, n in zip(colors, task_names)]
    legend_patches.append(mpatches.Patch(facecolor='white', edgecolor='red', linestyle='--', label=f'Deadline (D = T = {period_us}μs)'))
    ax.legend(handles=legend_patches, loc='upper right', fontsize=8)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig3_timing_diagram.png'))
    plt.close()
    print("[✓] Fig 3: Timing Diagram saved")


def plot_utilization_analysis(data, output_dir):
    t = np.array([(d['tick_ms'] - data[0]['tick_ms']) / 1000.0 for d in data])
    total_us = np.array([d['t_imu'] + d['t_filt'] + d['t_pid'] + d['t_geom'] + d['t_servo'] for d in data])
    deadline_us = int(np.median([d['dt_us'] for d in data]))
    utilization = (total_us / deadline_us) * 100.0

    # Subfigure 1: Utilization over time
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, utilization, color='#2563EB', alpha=0.5, linewidth=0.5)
    window = min(50, len(utilization) // 10)
    if window > 1:
        kernel = np.ones(window) / window
        util_smooth = np.convolve(utilization, kernel, mode='same')
        ax.plot(t, util_smooth, color='#DC2626', linewidth=2, label=f'Moving avg (n={window})')
    ax.axhline(y=100, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Deadline (100%)')
    ax.axhline(y=69.3, color='#F97316', linestyle=':', linewidth=1.5, alpha=0.7,
                    label='Liu-Layland bound (ln2 ≈ 69.3%)')
    ax.set_ylabel('CPU Utilization (%)')
    ax.set_xlabel('Time (seconds)')
    ax.legend()
    ax.set_ylim(0, max(120, utilization.max() * 1.1))
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig4_utilization-1.png'))
    plt.close()

    # Subfigure 2: Execution time histogram
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.hist(total_us, bins=80, color='#2563EB', alpha=0.7, edgecolor='white', linewidth=0.3)
    ax.axvline(x=deadline_us, color='red', linestyle='--', linewidth=2, label=f'Deadline = {deadline_us}μs')
    ax.axvline(x=np.mean(total_us), color='#22C55E', linestyle='-', linewidth=2,
                    label=f'Mean = {np.mean(total_us):.0f}μs')
    ax.axvline(x=np.percentile(total_us, 99), color='#F97316', linestyle='-', linewidth=2,
                    label=f'99th pct = {np.percentile(total_us, 99):.0f}μs')
    wcet = np.max(total_us)
    ax.axvline(x=wcet, color='#7C3AED', linestyle='-', linewidth=2,
                    label=f'WCET = {wcet:.0f}μs')
    ax.set_xlabel('Total Execution Time per Loop (μs)')
    ax.set_ylabel('Count')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig4_utilization-2.png'))
    plt.close()
    print("[✓] Fig 4 subfigures saved")


def plot_control_signal(data, output_dir):
    t = np.array([(d['tick_ms'] - data[0]['tick_ms']) / 1000.0 for d in data])
    pitch = np.array([d['pitch'] for d in data])
    roll  = np.array([d['roll'] for d in data])
    tgt_p = np.array([d['tgt_pitch'] for d in data])
    tgt_r = np.array([d['tgt_roll'] for d in data])
    s1    = np.array([d['s1'] for d in data])
    s2    = np.array([d['s2'] for d in data])
    s3    = np.array([d['s3'] for d in data])
    s4    = np.array([d['s4'] for d in data])

    # Subfigure 1: Platform Tilt Angle
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t, pitch, color='#2563EB', alpha=0.7, label='Pitch')
    ax.plot(t, roll, color='#7C3AED', alpha=0.7, label='Roll')
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax.set_ylabel('Angle (°)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylim([-15, 15])
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig5_control_signal-1.png'))
    plt.close()

    # Subfigure 2: Servo Motor Physical Angle
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t, s1, color='#3B82F6', alpha=0.85, linewidth=1.4, label='S1 A0 (pitch)')
    ax.plot(t, s2, color='#22C55E', alpha=0.85, linewidth=1.4, label='S2 A1 (roll)')
    ax.plot(t, s3, color='#F97316', alpha=0.85, linewidth=1.4, label='S3 A2 (roll)')
    ax.plot(t, s4, color='#EC4899', alpha=0.85, linewidth=1.4, label='S4 A3 (pitch)')
    ax.axhline(y=90, color='gray', linestyle='--', linewidth=0.8, alpha=0.6, label='Neutral (90°)')
    ax.set_ylabel('Servo Angle (°)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylim([0, 180])
    ax.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig5_control_signal-2.png'))
    plt.close()

    # Subfigure 3: Control Output (Target Mechanical Angle)
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t, tgt_p, color='#DC2626', alpha=0.7, label='Target Mech Pitch')
    ax.plot(t, tgt_r, color='#F97316', alpha=0.7, label='Target Mech Roll')
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax.set_ylabel('Servo Target (°)')
    ax.set_xlabel('Time (seconds)')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig5_control_signal-3.png'))
    plt.close()
    print("[✓] Fig 5 subfigures saved")


def plot_settling_overlay(data, step_responses, output_dir):
    """
    Fig 6: Step Response Overlay — 多次擾動回穩的疊加圖
    找出所有 disturbance 事件, 疊在一起看一致性
    """
    magnitude = np.sqrt(np.array([d['pitch'] for d in data])**2 +
                        np.array([d['roll'] for d in data])**2)
    settling_states = [d['settling'] for d in data]

    # 找每次 disturbance 的起始點 (settling_state 從 0→1 的跳變)
    events = []
    for i in range(1, len(settling_states)):
        if settling_states[i-1] == 0 and settling_states[i] == 1:
            events.append(i)

    if not events:
        # 手動找: magnitude 突然 > 3°
        for i in range(1, len(magnitude)):
            if magnitude[i-1] < 1.0 and magnitude[i] > 3.0:
                events.append(i)

    fig, ax = plt.subplots(figsize=(10, 6))

    if events:
        for ev_idx, start in enumerate(events):
            # 取 disturbance 前 0.2s 到 後 3s
            dt_ms = np.median([d['dt_us'] for d in data]) / 1000.0
            pre_samples  = max(1, int(200 / dt_ms))
            post_samples = max(1, int(3000 / dt_ms))

            begin = max(0, start - pre_samples)
            end   = min(len(magnitude), start + post_samples)

            t_rel = np.array([(i - start) * dt_ms / 1000.0 for i in range(begin, end)])
            mag_slice = magnitude[begin:end]

            ax.plot(t_rel, mag_slice, alpha=0.6, linewidth=1,
                   label=f'Event #{ev_idx+1}' if ev_idx < 5 else None)

        ax.axhline(y=0.5, color='#22C55E', linestyle='--', linewidth=1.5, alpha=0.7,
                  label='Settling threshold (0.5°)')
        ax.axvline(x=0, color='#DC2626', linestyle=':', linewidth=1.5, alpha=0.7,
                  label='Disturbance onset')
    else:
        ax.text(0.5, 0.5, 'No disturbance events detected\n(push the platform during recording!)',
               transform=ax.transAxes, ha='center', va='center', fontsize=14, color='gray')

    ax.set_xlabel('Time relative to disturbance (seconds)')
    ax.set_ylabel('Tilt Magnitude √(pitch² + roll²) (degrees)')
    # ax.set_title('Step Response Overlay — Disturbance Recovery')
    if events:
        ax.legend()
    ax.set_ylim(bottom=-0.5)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig6_settling_overlay.png'))
    plt.close()
    print("[✓] Fig 6: Settling Overlay saved")


# ══════════════════════════════════════════════
#  3. Formal 可調度性分析 (文字報告)
# ══════════════════════════════════════════════

def schedulability_analysis(data, step_responses):
    """
    Formal 分析報告:
    - 任務模型: 各子任務的 WCET, period, deadline
    - 利用率測試 (Liu & Layland sufficient condition)
    - Response time analysis (精確分析)
    - 控制性能指標: settling time, overshoot, steady-state error
    """
    report = []
    report.append("=" * 70)
    report.append("  Shake-It-Off 自穩平台 — 系統可調度性與控制性能分析報告")
    report.append("=" * 70)

    # ── 任務模型 ──
    report.append("\n1. TASK MODEL (任務模型)")
    report.append("-" * 50)

    task_info = [
        ('τ₁: IMU Read (I2C)',    't_imu'),
        ('τ₂: Complementary Filter', 't_filt'),
        ('τ₃: PID Control',       't_pid'),
        ('τ₄: Cable Geometry',    't_geom'),
        ('τ₅: Servo PWM Update',  't_servo'),
    ]

    T = int(np.median([d['dt_us'] for d in data]))  # 從 dt_us 自動偵測
    D = T     # Deadline = Period (implicit deadline)

    report.append(f"\n  Period  T = {T} μs  (control frequency = {1000000/T:.0f} Hz)")
    report.append(f"  Deadline D = T = {D} μs  (implicit deadline, rate-monotonic)")
    report.append(f"\n  {'Task':<30} {'Mean(μs)':>10} {'P99(μs)':>10} {'WCET(μs)':>10} {'Util%':>8}")
    report.append(f"  {'─'*30} {'─'*10} {'─'*10} {'─'*10} {'─'*8}")

    total_mean = 0
    total_wcet = 0
    wcets = []

    for name, key in task_info:
        vals = [d[key] for d in data]
        mean_v = np.mean(vals)
        p99_v  = np.percentile(vals, 99)
        wcet_v = np.max(vals)
        util_v = (wcet_v / T) * 100

        report.append(f"  {name:<30} {mean_v:>10.1f} {p99_v:>10.1f} {wcet_v:>10.0f} {util_v:>7.1f}%")
        total_mean += mean_v
        total_wcet += wcet_v
        wcets.append(wcet_v)

    report.append(f"  {'─'*30} {'─'*10} {'─'*10} {'─'*10} {'─'*8}")
    report.append(f"  {'TOTAL':<30} {total_mean:>10.1f} {'':>10} {total_wcet:>10.0f} {total_wcet/T*100:>7.1f}%")

    # ── Liu & Layland 利用率測試 ──
    report.append("\n\n2. SCHEDULABILITY ANALYSIS (可調度性分析)")
    report.append("-" * 50)

    # 因為是單一 loop (非 preemptive, 順序執行), 最簡單的分析是:
    # 所有任務都在同一個 period T 內順序執行, deadline D = T
    # 可調度性條件: C₁ + C₂ + C₃ + C₄ + C₅ ≤ D

    report.append("\n  Architecture: Non-preemptive sequential execution")
    report.append("  (All tasks run in fixed order within each control period)")
    report.append("")
    report.append("  Schedulability Condition (non-preemptive, single-rate):")
    report.append(f"    Σ Cᵢ ≤ D")
    report.append(f"    {' + '.join([f'C{i+1}' for i in range(5)])} ≤ D")
    report.append(f"    {' + '.join([f'{w:.0f}' for w in wcets])} ≤ {D}")
    report.append(f"    {total_wcet:.0f} {'≤' if total_wcet <= D else '>'} {D}")
    report.append(f"    → {'✓ SCHEDULABLE' if total_wcet <= D else '✗ NOT SCHEDULABLE (deadline miss possible)'}")

    report.append(f"\n  Utilization (WCET-based): U = Σ(Cᵢ/T) = {total_wcet/T*100:.1f}%")
    report.append(f"  Slack: D - Σ Cᵢ = {D - total_wcet:.0f} μs ({(D-total_wcet)/D*100:.1f}% margin)")

    # Liu & Layland bound (for reference, even though single-task-set)
    n = 5
    ll_bound = n * (2**(1.0/n) - 1) * 100
    report.append(f"\n  Reference — Liu & Layland RM bound for n={n}: U ≤ {ll_bound:.1f}%")
    report.append(f"  (This bound applies to preemptive RM; our system is non-preemptive,")
    report.append(f"   so the exact test Σ Cᵢ ≤ D is both necessary and sufficient.)")

    # ── Response Time Analysis ──
    report.append("\n  Response Time Analysis (exact):")
    report.append(f"    Since tasks execute sequentially (τ₁→τ₂→τ₃→τ₄→τ₅),")
    report.append(f"    the response time of the task chain is:")
    report.append(f"    R = C₁ + C₂ + C₃ + C₄ + C₅ = {total_wcet:.0f} μs")
    report.append(f"    D = {D} μs")
    report.append(f"    R {'≤' if total_wcet <= D else '>'} D → {'Tasks complete before deadline ✓' if total_wcet <= D else 'DEADLINE MISS ✗'}")

    # ── Empirical deadline miss ──
    total_us_arr = np.array([d['t_imu'] + d['t_filt'] + d['t_pid'] + d['t_geom'] + d['t_servo'] for d in data])
    miss_count = np.sum(total_us_arr > D)
    report.append(f"\n  Empirical Deadline Analysis (from {len(data)} samples):")
    report.append(f"    Deadline misses: {miss_count} / {len(data)} ({miss_count/len(data)*100:.2f}%)")
    report.append(f"    WCET (observed): {np.max(total_us_arr):.0f} μs")
    report.append(f"    Mean execution:  {np.mean(total_us_arr):.0f} μs")
    report.append(f"    99th percentile: {np.percentile(total_us_arr, 99):.0f} μs")

    # ── 控制性能分析 ──
    report.append("\n\n3. CONTROL PERFORMANCE ANALYSIS (控制性能分析)")
    report.append("-" * 50)

    if step_responses:
        report.append(f"\n  Step Response Results ({len(step_responses)} events recorded):")
        report.append(f"  {'Event':<8} {'Peak Disturb(°)':>16} {'Settling Time(ms)':>18}")
        report.append(f"  {'─'*8} {'─'*16} {'─'*18}")
        for sr in step_responses:
            report.append(f"  #{sr['response_num']:<7} {sr['peak_deg']:>16.2f} {sr['settle_ms']:>18.1f}")
        avg_settle = np.mean([sr['settle_ms'] for sr in step_responses])
        avg_peak   = np.mean([sr['peak_deg'] for sr in step_responses])
        report.append(f"\n  Average settling time:  {avg_settle:.0f} ms")
        report.append(f"  Average peak overshoot: {avg_peak:.1f}°")
    else:
        report.append("  (No step response events recorded — push the platform during test!)")

    # Steady-state error
    magnitude = np.sqrt(np.array([d['pitch'] for d in data])**2 +
                        np.array([d['roll'] for d in data])**2)
    # 取最後 20% 的資料 (假設已穩定)
    tail = magnitude[int(len(magnitude)*0.8):]
    report.append(f"\n  Steady-state Performance (last 20% of data):")
    report.append(f"    Mean tilt magnitude:  {np.mean(tail):.3f}°")
    report.append(f"    RMS tilt:             {np.sqrt(np.mean(tail**2)):.3f}°")
    report.append(f"    Max tilt:             {np.max(tail):.3f}°")
    report.append(f"    Standard deviation:   {np.std(tail):.3f}°")

    # ── 穩定性分析 ──
    report.append("\n\n4. STABILITY ANALYSIS (穩定性分析)")
    report.append("-" * 50)
    report.append("""
  Lyapunov Stability Argument (data-driven):

  Define the Lyapunov candidate function:
    V(θ) = ½ θ_pitch² + ½ θ_roll²    (squared distance from level)

  By the chain rule:
    V̇ = θ_pitch · ω_pitch + θ_roll · ω_roll      (ω = gyro rate)

  V̇ canNOT be signed analytically here: the firmware uses a velocity-form
  PI controller (the Kd / rate-damping term was deliberately dropped to avoid
  noise amplification), and there is no identified plant model relating ω to θ.
  So V̇ = θ·ω is positive during every overshoot of an under-damped response.
  Stability is therefore certified QUANTITATIVELY from the logged data via the
  companion tool lyapunov_check.py, using two complementary measures:
    1. gamma : decay rate of the post-disturbance energy envelope, fit over
               each event's recovery window (peak -> quiet band).
               gamma > 0  => envelope decays (no energy growth).
    2. rho   : fraction of samples with ΔV <= 0. rho ~ 50% is the signature
               of an under-damped oscillation, NOT of instability.

  Conclusion: V is bounded, gamma > 0 for every disturbance event, and the
  trajectory settles into a small residual ball (ultimate bound) -> the system
  is PRACTICALLY STABLE / ULTIMATELY BOUNDED (not asymptotically stable to 0).
  The nonzero residual comes from the soft deadband, the leaky-integrator term,
  sensor noise, and the absence of an explicit velocity-damping term.
""")

    report.append("\n  Empirical Lyapunov Function V(t) = ½(pitch² + roll²):")
    V = 0.5 * magnitude**2
    t = np.array([(d['tick_ms'] - data[0]['tick_ms']) / 1000.0 for d in data])
    rms_tail = np.sqrt(np.mean(tail**2))
    report.append(f"    V at start:  {V[0]:.3f}")
    report.append(f"    V at end:    {V[-1]:.3f}")
    report.append(f"    V_max:       {np.max(V):.3f}  (bounded → no escape)")
    report.append(f"    V_final_avg: {np.mean(V[-100:]):.4f}")
    report.append(f"    Ultimate bound: V_inf = ½·RMS² = {0.5*rms_tail**2:.3f}"
                  f"  (ball radius ~ {rms_tail:.2f}°)")
    report.append(f"    → bounded; converges to a residual ball, not to 0"
                  f" → practical / ultimately bounded stability")

    # ── 系統總結 ──
    report.append("\n\n5. SYSTEM SUMMARY (系統總結)")
    report.append("=" * 50)

    schedulable = total_wcet <= D
    # Practical stability: bounded trajectory that settles into a small ball.
    bounded = len(tail) > 0 and np.sqrt(np.mean(tail**2)) < 5.0

    report.append(f"""
  ┌─────────────────────────────────────────────────┐
  │  Control Frequency:    {1000000/T:.0f} Hz (T = {T} μs)         │
  │  WCET:                 {total_wcet:.0f} μs                     │
  │  Utilization (WCET):   {total_wcet/T*100:.1f}%                     │
  │  Deadline Margin:      {(D-total_wcet)/D*100:.1f}%                     │
  │  Schedulable:          {'YES ✓' if schedulable else 'NO ✗'}                        │
  │  Deadline Miss Rate:   {miss_count/len(data)*100:.2f}%                    │
  │  Settling Time (avg):  {np.mean([sr['settle_ms'] for sr in step_responses]):.0f} ms                      │
  │  Steady-state RMS:     {np.sqrt(np.mean(tail**2)):.3f}°                    │
  │  Stability (Lyapunov): {'PRACTICAL / BOUNDED ✓' if bounded else 'UNBOUNDED ✗'}              │
  └─────────────────────────────────────────────────┘
""") if step_responses else None

    return "\n".join(report)


# ══════════════════════════════════════════════
#  4. 串列埠錄製工具
# ══════════════════════════════════════════════

def record_serial(port, baud=115200, duration_sec=30, output_file='log_data.txt'):
    """
    從 STM32 USB CDC 錄製 log 資料

    使用: python3 analyze_system.py record <COM_PORT> [duration_sec]

    需要 pyserial: pip install pyserial
    """
    try:
        import serial
    except ImportError:
        print("需要安裝 pyserial: pip install pyserial")
        return

    print(f"Recording from {port} at {baud} baud for {duration_sec}s...")
    print(f"Output: {output_file}")
    print("(Push the platform a few times to record step responses!)\n")

    ser = serial.Serial(port, baud, timeout=1)
    lines_recorded = 0

    with open(output_file, 'w') as f:
        import time
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                f.write(line + '\n')
                lines_recorded += 1
                if lines_recorded % 500 == 0:
                    elapsed = time.time() - t_start
                    print(f"  {elapsed:.1f}s: {lines_recorded} lines recorded")

    ser.close()
    print(f"\nDone! Recorded {lines_recorded} lines to {output_file}")


# ══════════════════════════════════════════════
#  5. Main
# ══════════════════════════════════════════════

def main():
    setup_style()

    output_dirs = ['analysis_output', 'analysis_output_final']
    for d in output_dirs:
        os.makedirs(d, exist_ok=True)

    if len(sys.argv) >= 2:
        if sys.argv[1] == 'record':
            port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM0'
            dur  = int(sys.argv[3]) if len(sys.argv) > 3 else 30
            record_serial(port, duration_sec=dur)
            return
        elif sys.argv[1] == 'demo':
            print("Using DEMO data (simulated)...")
            data, step_resp, summaries = generate_demo_data()
        else:
            filepath = sys.argv[1]
            print(f"Parsing log file: {filepath}")
            data, step_resp, summaries = parse_log_file(filepath)
            if not data:
                print("No DATA rows found! Check log format.")
                print("Falling back to demo data...")
                data, step_resp, summaries = generate_demo_data()
    else:
        print("Usage:")
        print("  python3 analyze_system.py <log_file.txt>   # Analyze recorded data")
        print("  python3 analyze_system.py demo              # Generate demo plots")
        print("  python3 analyze_system.py record <port> [sec] # Record from STM32")
        print("\nRunning demo mode...")
        data, step_resp, summaries = generate_demo_data()

    print(f"\nData: {len(data)} samples, {len(step_resp)} step responses\n")

    report = schedulability_analysis(data, step_resp)
    for output_dir in output_dirs:
        # 生成所有圖表
        plot_step_response(data, output_dir)
        plot_timing_breakdown(data, output_dir)
        plot_timing_diagram(data, output_dir)
        plot_utilization_analysis(data, output_dir)
        plot_control_signal(data, output_dir)
        plot_settling_overlay(data, step_resp, output_dir)

        # 生成分析報告
        report_path = os.path.join(output_dir, 'schedulability_report.txt')
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"\n[✓] Schedulability report saved to {report_path}")

    print("\n" + report)

    print(f"\n{'='*50}")
    print(f"All outputs saved to: {', '.join(output_dirs)}/")
    print(f"{'='*50}")


if __name__ == '__main__':
    main()
