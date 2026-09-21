#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot.py — 把仿真 CSV 画成波形图（需要 matplotlib）

用法：python tools/plot.py out/discharge.csv out/charge.csv out/sleep.csv
      输出 out/waveforms.png
"""
import sys
import csv
import os
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("需要 matplotlib：pip install matplotlib", file=sys.stderr)
    sys.exit(1)

NUM_COLS = {"t_ms", "soc", "v_term", "i_load", "temp_c", "contract_v",
            "static_ma", "wake_events"}


def load(path):
    cols = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                k = k.strip()
                if k in NUM_COLS:
                    try:
                        cols[k].append(float(v))
                    except ValueError:
                        cols[k].append(float("nan"))
                else:
                    cols[k].append(v)
    return cols


def spans(d, key):
    out, cur, t0 = [], None, 0.0
    for t, s in zip(d["t_ms"], d[key]):
        if s != cur:
            if cur is not None:
                out.append((t0, t, cur))
            cur, t0 = s, t
    if cur is not None:
        out.append((t0, d["t_ms"][-1], cur))
    return out


def main(paths):
    if not paths:
        paths = ["out/discharge.csv", "out/charge.csv", "out/sleep.csv"]
    paths = [p for p in paths if os.path.exists(p)]
    if not paths:
        print("没有可用的 CSV，先跑 make sim", file=sys.stderr)
        return 1

    fig, axes = plt.subplots(len(paths), 3, figsize=(15, 3.4 * len(paths)),
                             squeeze=False)
    for r, path in enumerate(paths):
        d = load(path)
        t = d["t_ms"]
        title = os.path.basename(path)

        ax = axes[r][0]
        ax.plot(t, d["v_term"], lw=1.2, label="V_term")
        ax.set_ylabel("V")
        ax.set_title(f"{title} — 端电压")
        ax.grid(alpha=0.3)
        for a, b, s in spans(d, "fault"):
            if s != "NONE":
                ax.axvspan(a, b, color="red", alpha=0.18)
        ax.legend(fontsize=8)

        ax = axes[r][1]
        ax.plot(t, d["soc"], lw=1.2, color="tab:green", label="SOC")
        ax.set_ylim(-0.02, 1.02)
        ax.set_title(f"{title} — SOC")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

        ax = axes[r][2]
        for a, b, s in spans(d, "state"):
            ax.axvspan(a, b, color="grey", alpha=0.10)
            ax.text((a + b) / 2, 0.5, s, ha="center", va="center",
                    fontsize=6, rotation=90)
        ax.set_title(f"{title} — 整机状态")
        ax.set_yticks([])
        ax.set_xlabel("t (ms)")

    fig.tight_layout()
    out = "out/waveforms.png"
    fig.savefig(out, dpi=130)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
