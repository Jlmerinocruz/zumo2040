# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib"]
# ///
"""Mapa de waypoints, ruta deseada y trayectoria medida (para el reporte).

Uso (desde SecondDelivery/):
    uv run analysis/plot_route.py                       # solo mapa + ruta deseada
    uv run analysis/plot_route.py sim/vuelta.csv        # + trayectoria simulada
    uv run analysis/plot_route.py log_zumo.txt          # + trayectoria real del Zumo

La ruta deseada se toma de `sim/sim_follower --ruta`, que usa los mismos tramos que
arma el Zumo. El log del Zumo es la salida serial completa; se usan las líneas "L,...".
"""
import csv
import io
import re
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "sim" / "sim_follower"
WAYPOINTS_C = ROOT / "ZumoWaypointFollower" / "waypoints.c"


def load_waypoints():
    pat = re.compile(r"/\*\s*P(\d+)\s*\*/\s*\{\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f")
    return [(int(m[1]), float(m[2]), float(m[3]), float(m[4]))
            for m in pat.finditer(WAYPOINTS_C.read_text())]


def load_route():
    if not SIM.exists():
        subprocess.run(["make", "-s", "-C", str(SIM.parent)], check=True)
    out = subprocess.run([str(SIM), "--ruta"], capture_output=True, text=True, check=True).stdout
    rows = list(csv.DictReader(io.StringIO(out)))
    return [float(r["x"]) for r in rows], [float(r["y"]) for r in rows]


def load_trajectory(path):
    text = Path(path).read_text(errors="replace")
    if text.startswith("t,x,y"):                      # CSV del simulador
        rows = list(csv.DictReader(io.StringIO(text)))
        return [float(r["x"]) for r in rows], [float(r["y"]) for r in rows], "simulada"
    xs, ys = [], []                                   # log serial del Zumo
    for line in text.splitlines():
        parts = line.strip().split(",")
        if parts[0] == "L" and len(parts) >= 4 and parts[1] != "t_ms":
            xs.append(float(parts[2]))
            ys.append(float(parts[3]))
    return xs, ys, "medida (cámara)"


def main():
    wps = load_waypoints()
    rx, ry = load_route()
    fig, ax = plt.subplots(figsize=(6, 8))
    ax.plot(rx, ry, color="0.75", lw=10, solid_capstyle="round", label="pista (ruta deseada)", zorder=1)
    ax.plot(rx, ry, color="#2a6fdb", lw=1.4, label="ruta deseada", zorder=2)

    if len(sys.argv) > 1:
        tx, ty, kind = load_trajectory(sys.argv[1])
        ax.plot(tx, ty, color="#d9480f", lw=1.2, label=f"trayectoria {kind}", zorder=3)

    for n, x, y, th in wps:
        ax.scatter([x], [y], s=28, color="#1b5e20", zorder=4)
        dx = -0.045 if x < 0 else 0.012
        dy = 0.025 if y > 0.7 else -0.035
        ax.annotate(f"P{n}", (x, y), xytext=(x + dx, y + dy), fontsize=9, zorder=5)
    start = next(w for w in wps if w[0] == 14)
    ax.annotate("inicio / fin", (start[1], start[2]), xytext=(start[1] - 0.05, start[2] - 0.12),
                arrowprops=dict(arrowstyle="->", lw=0.8), fontsize=9)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Waypoints y ruta — sistema de posicionamiento local")
    ax.grid(True, lw=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8, frameon=False)
    out = ROOT / "analysis" / ("ruta_vs_" + Path(sys.argv[1]).stem + ".png" if len(sys.argv) > 1 else "mapa_waypoints.png")
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print(f"guardado: {out}")


if __name__ == "__main__":
    main()
