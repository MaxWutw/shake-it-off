#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lyapunov_check.py  —  Shake-It-Off 自穩平台「量化判穩」工具

讀取 main.c 透過 USB-CDC 印出的 DATA log，計算 Lyapunov 函數
    V[k] = 1/2 (pitch[k]^2 + roll[k]^2)              (純角度版)
    V_E[k] = 1/2 (pitch^2 + roll^2) + 1/2 * c * (gx^2 + gy^2)  (含角速度的能量版)

然後對每一次擾動事件 (settling_state 0->1) 量測：
  1. 擾動後 V 的「包絡線」是否指數衰減 -> 衰減率 gamma (>0 才代表會收斂)
  2. ΔV = V[k]-V[k-1] <= 0 的比例 (越接近 / 超過 50% 越好)
  3. 穩態 ball：用最後段資料的 RMS 角度算 V_inf，得到 ultimate bound 半徑

用法:
    python3 lyapunov_check.py  your_log.csv
    python3 lyapunov_check.py  your_log.csv --plot out.png
DATA 欄位順序 (對應 main.c 的 DATA_HEADER)：
    DATA,tick_ms,pitch,roll,tgt_pitch,tgt_roll,gx,gy,dt_us,
         t_imu,t_filt,t_pid,t_geom,t_servo,deadline_miss,settling_state,s1,s2,s3,s4
"""
import sys
import numpy as np

# gyro 角速度在能量版裡的權重 c。物理意義 ~ 1/omega_n^2，這裡給一個保守的小值，
# 只是為了讓「正在往回盪」的能量也被算進去；改大會更看重角速度。
GYRO_WEIGHT = 1.0 / (50.0 ** 2)   # 假設特徵頻率 ~50 deg/s 等級，可自行調整


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


def fit_decay_rate(t_s, V_seg):
    """對一段 V 取局部峰值包絡，擬合 ln(V) = ln(V0) - gamma*t -> 回傳 gamma (1/s)。"""
    if len(V_seg) < 8:
        return None
    # 取移動視窗最大值當包絡，避免被噪聲谷底拉低
    w = max(3, len(V_seg) // 20)
    env = np.array([V_seg[i:i + w].max() for i in range(len(V_seg) - w)])
    te = t_s[:len(env)]
    env = np.maximum(env, 1e-6)
    # 線性回歸 ln(env) vs t
    A = np.vstack([te, np.ones_like(te)]).T
    slope, _ = np.linalg.lstsq(A, np.log(env), rcond=None)[0]
    return -slope  # gamma>0 => 衰減


def analyze(path, plot_path=None):
    t_ms, pitch, roll, gx, gy, state = parse_log(path)
    if len(t_ms) == 0:
        print("找不到 DATA 行，請確認 log 檔內容。")
        return
    t = (t_ms - t_ms[0]) / 1000.0          # 秒
    mag = np.sqrt(pitch ** 2 + roll ** 2)  # 傾斜量值 (deg)
    V = 0.5 * mag ** 2                     # 純角度 Lyapunov
    V_E = 0.5 * (pitch ** 2 + roll ** 2) + 0.5 * GYRO_WEIGHT * (gx ** 2 + gy ** 2)

    print("=" * 64)
    print("  量化 Lyapunov / 實務穩定性分析")
    print("=" * 64)
    print(f"  樣本數 N = {len(t)}，總時長 = {t[-1]:.1f} s")
    print(f"  V(0)={V[0]:.3f}  V_max={V.max():.3f}  V_end_avg={V[-200:].mean():.4f}")

    # --- 擾動事件 (settling_state 0 -> 1) ---
    onsets = [i for i in range(1, len(state)) if state[i - 1] == 0 and state[i] == 1]
    print(f"\n  偵測到擾動事件 (0->1)：{len(onsets)} 次")
    gammas, dvfrac = [], []
    for j, i0 in enumerate(onsets):
        i1 = onsets[j + 1] if j + 1 < len(onsets) else len(t)
        seg_t = t[i0:i1] - t[i0]
        seg_V = V[i0:i1]
        if len(seg_V) < 8:
            continue
        g = fit_decay_rate(seg_t, seg_V)
        dV = np.diff(seg_V)
        frac = np.mean(dV <= 0) * 100.0
        gammas.append(g)
        dvfrac.append(frac)
        if g is not None:
            tau = (1.0 / g) if g > 1e-6 else float("inf")
            print(f"    事件#{j+1:>2}: peakV={seg_V.max():6.2f}  "
                  f"gamma={g:+.3f}/s  (時間常數~{tau:5.1f}s)  ΔV<=0 比例={frac:4.1f}%")

    if gammas:
        gv = np.array([g for g in gammas if g is not None])
        print(f"\n  平均衰減率 gamma = {gv.mean():+.3f} /s")
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
    avg_g = np.mean([g for g in gammas if g is not None]) if gammas else 0.0
    if V.max() < 1e4 and np.isfinite(V).all():
        print("  • V 全程有界、未發散  -> Lyapunov 意義下『有界穩定 (bounded)』成立。")
    if avg_g > 0.05:
        print(f"  • 擾動後 V 包絡平均以 gamma={avg_g:.3f}/s 衰減 -> 具收斂性 (實務漸近穩定)。")
    elif avg_g > 0:
        print(f"  • 衰減率偏小 (gamma={avg_g:.3f}/s) -> 收斂很慢，阻尼不足。")
    else:
        print("  • 包絡未明顯衰減 -> 接近臨界/振盪，需加阻尼 (建議導入陀螺儀速度回授)。")
    print(f"  • 系統最終落在 ~{rms:.2f} deg 的球內，屬『practical stability / 終值有界』，")
    print("    非收斂到 0；殘差來自死區、洩放項、感測噪聲與缺少速度阻尼項。")

    if plot_path:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            ax[0].plot(t, mag, lw=0.8, label="tilt magnitude")
            ax[0].axhline(rms, color="r", ls="--", label=f"steady RMS={rms:.2f}deg")
            for i0 in onsets:
                ax[0].axvline(t[i0], color="orange", ls=":", alpha=0.5)
            ax[0].set_ylabel("|tilt| (deg)"); ax[0].legend(); ax[0].grid(alpha=0.3)
            ax[1].plot(t, V, lw=0.8, label="V=1/2|θ|²")
            ax[1].axhline(V_inf, color="r", ls="--", label=f"V_inf={V_inf:.2f}")
            ax[1].set_ylabel("V"); ax[1].set_xlabel("time (s)")
            ax[1].legend(); ax[1].grid(alpha=0.3)
            ax[0].set_title("Empirical Lyapunov V(t) — disturbance onsets marked")
            fig.tight_layout(); fig.savefig(plot_path, dpi=130)
            print(f"\n  已輸出圖檔: {plot_path}")
        except Exception as e:
            print(f"  (繪圖略過: {e})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(0)
    pp = None
    if "--plot" in sys.argv:
        pp = sys.argv[sys.argv.index("--plot") + 1]
    analyze(sys.argv[1], pp)
