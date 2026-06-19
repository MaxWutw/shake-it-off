#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lyapunov_check.py  —  Shake-It-Off 自穩平台「量化判穩」工具

讀取 main.c 透過 USB-CDC 印出的 DATA log，計算 Lyapunov 函數
    V[k] = 1/2 (pitch[k]^2 + roll[k]^2)              (純角度版)

然後對每一次擾動事件量測：
  1. 擾動後「回復段 (peak -> settled)」V 的包絡是否指數衰減 -> 衰減率 gamma
     (>0 才代表回復段能量在收斂；<0 代表發散)
  2. ΔV = V[k]-V[k-1] <= 0 的比例 ρ (~50% 是欠阻尼振盪的特徵，非不穩)
  3. 穩態 ball：用最後段資料的 RMS 角度算 V_inf，得到 ultimate bound 半徑

⚠ 事件偵測說明（重要）：
  早期版本以韌體的 settling_state 0->1 旗標當作擾動起點，並且把「這次起點到
  下一次起點」整段拿去擬合 gamma。當擾動小、回復快時，這整段幾乎都是穩態噪聲，
  擬合出來的是噪聲斜率（會出現假性的 gamma<0）。本版改為：
    (a) 直接用傾角量值 |theta| 以遲滯 (hysteresis) 偵測擾動事件，
        不依賴可能漏觸發的 settling_state；
    (b) gamma 只在「回復段」(從事件峰值到回到 quiet band) 擬合，
        避免被穩態噪聲污染。

用法:
    python3 lyapunov_check.py  your_log.txt
    python3 lyapunov_check.py  your_log.txt --plot out.png
DATA 欄位順序 (對應 main.c 的 DATA_HEADER)：
    DATA,tick_ms,pitch,roll,tgt_pitch,tgt_roll,gx,gy,dt_us,
         t_imu,t_filt,t_pid,t_geom,t_servo,deadline_miss,settling_state,s1,s2,s3,s4
"""
import sys
import numpy as np

# ── 擾動偵測門檻 (deg) ──────────────────────────────────────────────
#   DISTURB_DEG : |傾角| 升破此值 (且先前處於 quiet) 視為一次擾動事件開始
#   QUIET_DEG   : |傾角| 連續低於此值並維持 HOLD_S 秒，視為回復完成
DISTURB_DEG = 2.0
QUIET_DEG   = 0.8
HOLD_S      = 0.4


def parse_log(path):
    t_ms, pitch, roll, gx, gy, state = [], [], [], [], [], []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("DATA,"):
                continue
            p = line.split(",")
            if len(p) < 16:
                continue
            try:
                t_ms.append(float(p[1]))
                pitch.append(float(p[2]))
                roll.append(float(p[3]))
                gx.append(float(p[6]))
                gy.append(float(p[7]))
                state.append(int(float(p[15])))
            except (ValueError, IndexError):
                continue
    return (np.array(t_ms), np.array(pitch), np.array(roll),
            np.array(gx), np.array(gy), np.array(state))


def detect_episodes(t, mag, dt,
                    disturb=DISTURB_DEG, quiet=QUIET_DEG, hold_s=HOLD_S):
    """以傾角量值做遲滯偵測。回傳 [(onset_idx, peak_idx, end_idx), ...]。

    onset : |theta| 由 quiet 升破 disturb 的時刻
    peak  : 此事件內 |theta| 的最大值位置
    end   : |theta| 回到 quiet band 並維持 hold_s 秒（即回復完成）的時刻
    """
    hold = max(1, int(round(hold_s / dt)))
    episodes = []
    i, N = 0, len(t)
    armed = True
    while i < N:
        if armed and mag[i] > disturb:
            onset = i
            peak = i
            below = 0
            j = i
            while j < N:
                if mag[j] > mag[peak]:
                    peak = j
                if mag[j] < quiet:
                    below += 1
                    if below >= hold:
                        break
                else:
                    below = 0
                j += 1
            episodes.append((onset, peak, min(j, N - 1)))
            i = j + 1
            armed = True
        else:
            i += 1
    return episodes


def fit_decay_rate(t_s, V_seg):
    """對回復段 V 取移動最大值包絡，擬合 ln(env)=ln(V0)-gamma*t -> 回傳 gamma (1/s)。"""
    if len(V_seg) < 8:
        return None
    w = max(3, len(V_seg) // 20)
    env = np.array([V_seg[i:i + w].max() for i in range(len(V_seg) - w)])
    te = t_s[:len(env)]
    env = np.maximum(env, 1e-6)
    A = np.vstack([te, np.ones_like(te)]).T
    slope, _ = np.linalg.lstsq(A, np.log(env), rcond=None)[0]
    return -slope  # gamma>0 => 衰減


def analyze(path, plot_path=None):
    t_ms, pitch, roll, gx, gy, state = parse_log(path)
    if len(t_ms) == 0:
        print("找不到 DATA 行，請確認 log 檔內容。")
        return
    t = (t_ms - t_ms[0]) / 1000.0          # 秒
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.002
    mag = np.sqrt(pitch ** 2 + roll ** 2)  # 傾斜量值 (deg)
    V = 0.5 * mag ** 2                     # 純角度 Lyapunov

    print("=" * 64)
    print("  量化 Lyapunov / 實務穩定性分析")
    print("=" * 64)
    print(f"  樣本數 N = {len(t)}，總時長 = {t[-1]:.1f} s")
    print(f"  V(0)={V[0]:.3f}  V_max={V.max():.3f}  V_end_avg={V[-200:].mean():.4f}")

    # --- 以傾角量值偵測擾動事件，gamma 只在回復段擬合 ---
    episodes = detect_episodes(t, mag, dt)
    print(f"\n  偵測到擾動事件 (|θ|>{DISTURB_DEG}°)：{len(episodes)} 次"
          f"  (gamma 僅在回復段 peak→quiet 擬合)")
    gammas, dvfrac = [], []
    for j, (i0, ipk, i1) in enumerate(episodes):
        seg_t = t[ipk:i1 + 1] - t[ipk]
        seg_V = V[ipk:i1 + 1]
        if len(seg_V) < 8:
            continue
        g = fit_decay_rate(seg_t, seg_V)
        dV = np.diff(V[i0:i1 + 1])
        frac = np.mean(dV <= 0) * 100.0
        gammas.append(g)
        dvfrac.append(frac)
        if g is not None:
            tau = (1.0 / g) if g > 1e-6 else float("inf")
            print(f"    事件#{j+1:>2}: t={t[i0]:5.1f}s  peak={mag[ipk]:.2f}°  peakV={seg_V.max():5.2f}  "
                  f"gamma={g:+.3f}/s  (τ~{tau:5.1f}s)  回復={t[i1]-t[ipk]:4.1f}s  ΔV<=0={frac:4.1f}%")

    gv = np.array([g for g in gammas if g is not None]) if gammas else np.array([])
    if gv.size:
        print(f"\n  衰減率 gamma：全部 > 0 ? {bool(np.all(gv > 0))}")
        print(f"    median gamma = {np.median(gv):+.3f} /s   "
              f"(range {gv.min():+.3f} ~ {gv.max():+.3f} /s)")
        print(f"  平均 ΔV<=0 比例 = {np.mean(dvfrac):.1f}%")

    # --- 穩態 ultimate bound ---
    tail = mag[int(len(mag) * 0.8):]
    rms = np.sqrt(np.mean(tail ** 2))
    V_inf = 0.5 * rms ** 2
    print(f"\n  穩態 (末 20%)：RMS 傾角 = {rms:.3f} deg，max = {tail.max():.3f} deg")
    print(f"  Ultimate bound: 軌跡最終收斂到 ball  V <= V_inf = {V_inf:.3f}")
    print(f"                  => 半徑 ~ {rms:.2f} deg 的傾斜球內")

    # --- 結論判斷 ---
    print("\n  ─ 判讀 ─")
    if V.max() < 1e4 and np.isfinite(V).all():
        print("  • V 全程有界、未發散  -> Lyapunov 意義下『有界穩定 (bounded)』成立。")
    if gv.size and np.all(gv > 0):
        print(f"  • 每次擾動的回復段 V 包絡皆衰減 (gamma>0, median={np.median(gv):.3f}/s)")
        print("    -> 擾動後能量不增、會收斂回穩態球，無發散趨勢。")
    elif gv.size:
        print("  • 部分事件回復段 gamma<=0 -> 該段近臨界/振盪，建議加阻尼。")
    print(f"  • 系統最終落在 ~{rms:.2f} deg 的球內，屬『practical stability / 終值有界』，")
    print("    非收斂到 0；殘差來自死區、洩放項、感測噪聲與缺少速度阻尼項。")
    if dvfrac:
        print(f"  • ΔV<=0 比例 ~{np.mean(dvfrac):.0f}% 為欠阻尼振盪特徵 (非不穩)：")
        print("    能量在過衝時上升、回正時下降，各約一半；收斂與否由包絡 (gamma) 判定。")

    if plot_path:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            # Size for the IEEE single column (≈3.45 in) so text is not shrunk
            # when included at \linewidth. Font sizes are set for that size.
            COL_W = 3.45
            plt.rcParams.update({
                "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 6.5,
                "xtick.labelsize": 7, "ytick.labelsize": 7,
                "savefig.dpi": 300, "savefig.bbox": "tight",
            })

            base_path = plot_path.replace(".png", "")

            # Subfigure 1: Tilt Magnitude
            fig, ax = plt.subplots(figsize=(COL_W, 1.9))
            ax.plot(t, mag, lw=0.6, label="tilt magnitude")
            ax.axhline(rms, color="r", ls="--", label=f"steady RMS={rms:.2f}°")
            for k, (i0, ipk, i1) in enumerate(episodes):
                ax.axvline(t[i0], color="orange", ls=":", alpha=0.6,
                              label="disturbance onset" if k == 0 else None)
                ax.plot(t[ipk], mag[ipk], "v", color="purple", ms=5,
                           label="event peak" if k == 0 else None)
            ax.set_ylabel("|tilt| (deg)")
            ax.set_xlabel("time (s)")
            ax.legend(loc="upper right", ncol=2, columnspacing=0.8, handlelength=1.2)
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(f"{base_path}-1.png")
            plt.close()

            # Subfigure 2: Lyapunov V(t)
            fig, ax = plt.subplots(figsize=(COL_W, 1.9))
            ax.plot(t, V, lw=0.6, label=r"V = $\frac{1}{2}\,|\theta|^2$")
            ax.axhline(V_inf, color="r", ls="--", label=f"V_inf={V_inf:.2f}")
            for (i0, ipk, i1) in episodes:
                ax.axvline(t[i0], color="orange", ls=":", alpha=0.6)
            ax.set_ylabel("V")
            ax.set_xlabel("time (s)")
            ax.legend(loc="upper right")
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(f"{base_path}-2.png")
            plt.close()
            
            print(f"\n  已輸出圖檔: {base_path}-1.png 及 {base_path}-2.png")
        except Exception as e:
            print(f"  (繪圖略過: {e})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    pp = None
    if "--plot" in sys.argv:
        pp = sys.argv[sys.argv.index("--plot") + 1]
    analyze(sys.argv[1], pp)
