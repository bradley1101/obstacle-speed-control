"""Simulation of the obstacle-based speed control.

Re-implements the firmware algorithm (constants read from firmware/config.h)
on a simple robot model and plots:
  1. the static speed law  duty = f(distance)
  2. a run towards a wall, with sensor noise and an outlier reading

Usage:  python simulate.py            (writes ../docs/simulation.png)
"""
import re
from pathlib import Path
import random
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
CFG = {k: int(v) for k, v in re.findall(r"#define\s+(\w+)\s+(\d+)\b", (HERE.parent / "firmware/config.h").read_text())}

V_MAX = 0.60      # m/s at duty 255 (TT gear motors, 6 V)
DEADBAND = 60     # duty below which the robot does not move
TAU = 0.25        # s, mechanical time constant
DT = CFG["PERIOD_MS"] / 1000


def target_duty(d):
    if d < CFG["D_STOP"]:
        return 0
    if d < CFG["D_SLOW"]:
        return CFG["DUTY_CREEP"]
    if d >= CFG["D_FULL"]:
        return CFG["DUTY_MAX"]
    return CFG["DUTY_CREEP"] + (d - CFG["D_SLOW"]) * (CFG["DUTY_MAX"] - CFG["DUTY_CREEP"]) // (CFG["D_FULL"] - CFG["D_SLOW"])


def ramp(cur, tgt):
    if tgt > cur:
        return min(tgt, cur + CFG["RAMP_UP"])
    if tgt == 0:
        return 0
    return max(tgt, cur - CFG["RAMP_DOWN"])


def speed_of(duty):
    return 0 if duty < DEADBAND else V_MAX * (duty - DEADBAND) / (255 - DEADBAND)


def run(start_cm=250, seconds=9, seed=1):
    random.seed(seed)
    pos = start_cm / 100      # distance to the wall (m)
    v = 0.0
    duty = 0
    hist = []
    h = [start_cm, start_cm]
    t = 0.0
    while t < seconds:
        raw = pos * 100 + random.gauss(0, 0.7)
        if abs(t - 2.0) < DT / 2:
            raw = 400             # one wrong reading (lost echo / reflection)
        d = sorted([raw, h[0], h[1]])[1]
        h = [raw, h[0]]
        duty = ramp(duty, target_duty(int(d)))
        # motor first-order response, integrated in small steps
        for _ in range(10):
            v += (speed_of(duty) - v) * (DT / 10) / TAU
            pos -= v * DT / 10
        hist.append((t, pos * 100, duty, raw))
        t += DT
    return hist


def main():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.4))

    ds = list(range(0, 121))
    a1.plot(ds, [target_duty(d) for d in ds], color="#3b7dd8", lw=2)
    for x in (CFG["D_STOP"], CFG["D_SLOW"], CFG["D_FULL"]):
        a1.axvline(x, color="#999", ls="--", lw=0.8)
    for x0, x1, lab in ((0, CFG["D_STOP"], "STOP"), (CFG["D_STOP"], CFG["D_SLOW"], "CREEP"),
                        (CFG["D_SLOW"], CFG["D_FULL"], "LINEAR"), (CFG["D_FULL"], 120, "FULL")):
        a1.text((x0 + x1) / 2, 243, lab, fontsize=8, color="#555", ha="center")
    a1.axvspan(0, CFG["D_STOP"], color="#d64545", alpha=0.12)
    a1.axvspan(CFG["D_STOP"], CFG["D_SLOW"], color="#e0a800", alpha=0.12)
    a1.axvspan(CFG["D_SLOW"], 120, color="#2e9e44", alpha=0.08)
    a1.set(xlabel="distance to obstacle (cm)", ylabel="PWM duty (0-255)", title="Speed law", ylim=(0, 255), xlim=(0, 120))

    h = run()
    t = [x[0] for x in h]
    a2.plot(t, [x[1] for x in h], color="#222", lw=2, label="distance (cm)")
    a2.axhline(CFG["D_STOP"], color="#d64545", ls="--", lw=0.8, label=f"D_STOP = {CFG['D_STOP']} cm")
    a2.set(xlabel="time (s)", ylabel="distance (cm)", title="Approach towards a wall (start 250 cm)")
    ax = a2.twinx()
    ax.step(t, [x[2] for x in h], where="post", color="#3b7dd8", lw=1.3, label="PWM duty")
    ax.set_ylabel("PWM duty", color="#3b7dd8")
    ax.set_ylim(0, 260)
    a2.annotate("outlier reading (400 cm)\nrejected by the median filter", xy=(2.0, [x[1] for x in h][int(2.0 / DT)]),
                xytext=(2.3, 45), fontsize=8, arrowprops=dict(arrowstyle="->", color="#777"))
    a2.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    out = HERE.parent / "docs/simulation.png"
    fig.savefig(out, dpi=130)
    final = h[-1][1]
    print(f"saved {out}\nfinal distance: {final:.1f} cm  (minimum {min(x[1] for x in h):.1f} cm)")


if __name__ == "__main__":
    main()
