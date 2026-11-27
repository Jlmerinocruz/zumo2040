# /// script
# requires-python = ">=3.10"
# ///
"""Dónde se pierde tiempo y de dónde sale el error en una vuelta real.

Uso (desde SecondDelivery/):
    uv run analysis/analizar_vueltas.py logs/vuelta4.txt [logs/vuelta2.txt ...]
    uv run analysis/analizar_vueltas.py --wb 0.085 logs/vuelta4.txt

Por cada log serial del Zumo (líneas "L,...") imprime:
  - tiempo de vuelta, poses recibidas y huecos sin pose (> 250 ms);
  - por tramo: tiempo, |e_y| máximo, RMS y media de e_y (+ = a la izquierda). En los
    arcos también dice si en promedio el Zumo fue por afuera o por adentro;
  - por tramo: velocidad y giro reales (cámara) contra lo mandado a las orugas
    (vl, vr). En el firmware con ajuste de ganancias, esos cocientes son g_v y g_ω.
`--wb` es la separación de orugas con la que se convierten vl, vr a ω (WHEEL_BASE).
"""
import math
import sys

GAP_MS = 250


def load(path):
    rows, names = [], {}
    for line in open(path, errors="ignore"):
        if line.startswith("L,") and not line.startswith("L,t_ms"):
            try:
                t, x, y, th, seg, _, ey, vl, vr = (float(v) for v in line.split(",")[1:10])
            except ValueError:
                continue
            rows.append((t / 1000, x, y, math.radians(th), int(seg), ey, vl, vr))
        elif line.startswith("# tramo"):
            k, name = line[len("# tramo"):].split(":", 1)
            names[int(k)] = name.strip()
    return rows, names


def report(path, wb):
    rows, names = load(path)
    if len(rows) < 2:
        print(f"{path}: sin líneas L,...")
        return
    dts = [(b[0] - a[0]) * 1000 for a, b in zip(rows, rows[1:])]
    gaps = [d for d in dts if d > GAP_MS]
    eys = [r[5] for r in rows]
    print(f"\n=== {path}: {rows[-1][0]:.1f} s, {len(rows)} poses")
    print(f"  huecos > {GAP_MS} ms: {len(gaps)}, suman {sum(gaps) / 1000:.1f} s "
          f"({100 * sum(gaps) / 1000 / rows[-1][0]:.0f} % de la vuelta), el mayor {max(dts):.0f} ms")
    print(f"  |e_y| máx {max(map(abs, eys)):.0f} mm, RMS {math.sqrt(sum(e * e for e in eys) / len(eys)):.1f} mm")
    print("  tramo                     t[s]  |ey|máx  RMS  media         | v real/mandada  ω real/mandada")
    for seg in sorted({r[4] for r in rows}):
        w = [r for r in rows if r[4] == seg]
        if len(w) < 3:
            continue
        dt = [b[0] - a[0] for a, b in zip(w, w[1:])]
        T = sum(dt)
        e = [r[5] for r in w]
        rms = math.sqrt(sum(x * x for x in e) / len(e))
        # Lo mandado se pondera por el tiempo que estuvo aplicado.
        v_cmd = sum((r[6] + r[7]) / 2 * d for r, d in zip(w, dt)) / T
        w_cmd = sum((r[7] - r[6]) / wb * d for r, d in zip(w, dt)) / T
        # Avance real con una pose de cada 5 (para no sumar el ruido de la cámara).
        pts = w[::5] + [w[-1]]
        dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))
        dth = sum((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi for a, b in zip(w, w[1:]))
        mean = sum(e) / len(e)
        arc = abs(dth) > 1.0                # el tramo giró más de ~60°
        side = ("afuera" if mean * dth < 0 else "adentro") if arc else ""
        ratio_w = f"{dth / T / w_cmd:.2f}" if arc else "-"
        print(f"  {seg:2d} {names.get(seg, ''):22s}{T:5.1f}  {max(map(abs, e)):6.0f}  {rms:4.0f}  {mean:+5.0f} {side:7s}"
              f"| {dist / T / v_cmd:13.2f}  {ratio_w:>14s}")


def main(args):
    wb = 0.085
    if args[:1] == ["--wb"]:
        wb, args = float(args[1]), args[2:]
    if not args:
        sys.exit(__doc__)
    for path in args:
        report(path, wb)


if __name__ == "__main__":
    main(sys.argv[1:])
