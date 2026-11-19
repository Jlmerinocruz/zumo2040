"""Figuras del reporte Second Delivery con estilo de documento técnico (Office clásico)."""
import csv, io, re, subprocess, sys, math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Circle, Rectangle, Arc
from matplotlib import font_manager

OUT = Path(sys.argv[1])
SD = Path("/home/albertg/SistemasEmbebidosAvanzados/Reto/SecondDelivery")
for f in font_manager.findSystemFonts():
    if "Carlito" in f:
        font_manager.fontManager.addfont(f)
plt.rcParams.update({"font.family": "Carlito", "font.size": 11, "text.color": "#404040",
                     "axes.labelcolor": "#404040", "xtick.color": "#404040", "ytick.color": "#404040",
                     "axes.edgecolor": "#7F7F7F", "savefig.facecolor": "white"})

BLUE, YEL, GRN, CYAN, PALE = "#4F81BD", "#FFC000", "#92D050", "#9FE4F0", "#FFF2A8"
HDR, PAY, TRL, RED, GRAY = "#DCE6F2", "#D7E4BD", "#FCD5B5", "#C00000", "#A6A6A6"
DBLUE, DYEL, DGRN, EDGE, TXT = "#2F5597", "#9C6500", "#4E7A1E", "#7F7F7F", "#404040"


def canvas(w, h, title, sub, foot="Fuente: elaboración propia."):
    fig = plt.figure(figsize=(w, h))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w); ax.set_ylim(0, h); ax.axis("off")
    ax.text(0.25, h - 0.3, title, fontsize=14, weight="bold", color="black", va="top")
    ax.text(0.25, h - 0.62, sub, fontsize=10.5, color="#7F7F7F", va="top")
    ax.text(0.25, 0.12, foot, fontsize=8.5, color="#7F7F7F", va="bottom")
    return fig, ax


def box(ax, x, y, w, h, title, sub=None, fc="white", ec=EDGE, tc="black", r=0.06, fs=10.5, sfs=9, lw=1.4, ls="-"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                fc=fc, ec=ec, lw=lw, ls=ls, zorder=2))
    if sub:
        ax.text(x + w / 2, y + h / 2 + 0.06, title, ha="center", va="bottom", fontsize=fs, weight="bold", color=tc, zorder=3)
        ax.text(x + w / 2, y + h / 2 + 0.0, sub, ha="center", va="top", fontsize=sfs, color=TXT, zorder=3, linespacing=1.25)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=fs, color=tc, zorder=3, linespacing=1.25)


def arrow(ax, pts, label=None, lpos=None, color="black", lw=1.2, ls="-", ha="center", lc=TXT, fs=8.8, head=True):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] + (xs[-1],), ys, color=color, lw=lw, ls=ls, zorder=1, solid_capstyle="butt")
    if head:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>,head_length=0.5,head_width=0.22", color=color, lw=lw, shrinkA=0, shrinkB=0), zorder=1)
    if label:
        lx, ly = lpos if lpos else ((xs[0] + xs[-1]) / 2, (ys[0] + ys[-1]) / 2)
        ax.text(lx, ly, label, fontsize=fs, color=lc, ha=ha, va="center", zorder=4, linespacing=1.2,
                bbox=dict(fc="white", ec="none", pad=1.0))


def diamond(ax, cx, cy, w, h, text, fs=9.2):
    ax.add_patch(Polygon([(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)],
                         closed=True, fc=PALE, ec=EDGE, lw=1.4, zorder=2))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, zorder=3, linespacing=1.15)


# ---------------------------------------------------------------- contexto
def contexto():
    # Estilo simple en blanco y negro: rectángulos de esquinas rectas, sin relleno de color.
    W, H = 12, 7.2
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    ax.text(0.25, H - 0.3, "Diagrama de contexto del Zumo", fontsize=14, weight="bold", color="black", va="top")
    ax.text(0.25, H - 0.62, "El sistema (borde grueso) y las entidades externas con las que intercambia información o energía",
            fontsize=10.5, color="black", va="top")
    ax.text(0.25, 0.12, "Fuente: elaboración propia.", fontsize=8.5, color="black", va="bottom")

    def sq(x, y, w, h, title, sub, lw=1.0, fs=10.5, sfs=9):
        ax.add_patch(Rectangle((x, y), w, h, fc="white", ec="black", lw=lw, zorder=2))
        ax.text(x + w / 2, y + h / 2 + 0.06, title, ha="center", va="bottom", fontsize=fs, weight="bold", color="black", zorder=3)
        ax.text(x + w / 2, y + h / 2, sub, ha="center", va="top", fontsize=sfs, color="black", zorder=3, linespacing=1.25)

    def ar(pts, label, lpos):
        arrow(ax, pts, label, lpos, color="black", lw=1.0, lc="black")

    sx, sy, sw, sh = 4.0, 2.75, 4.0, 1.7
    sq(sx, sy, sw, sh, "Zumo (tractor autónomo)", "Zumo 2040 + PCB\n(Pico 2 W, 2× MCP2515 + TJA1050)", lw=2.2, fs=13, sfs=10)
    sq(0.3, 5.2, 2.9, 1.0, "Cámara del techo", "PC de posicionamiento local\n(192.168.12.231)")
    sq(4.55, 5.35, 2.9, 0.85, "Router WiFi", "red “embedded”")
    sq(8.8, 5.2, 2.9, 1.0, "Pista en serpentina", "14 waypoints, inicio/fin en P14")
    sq(0.3, 3.1, 2.9, 1.0, "Operador / equipo", "botón A: vuelta\nB (3 s): calibrar · B: paro")
    sq(8.8, 3.1, 2.9, 1.0, "Laptop del equipo", "monitor serial USB (log L,…)")
    sq(0.3, 0.75, 2.9, 1.0, "Power bank 5 V", "alimenta Pico 2 W y TJA1050")
    sq(4.55, 0.75, 2.9, 1.0, "4 pilas AA", "motores, Zumo y MCP2515")
    sq(8.8, 0.75, 2.9, 1.0, "Terreno con gradiente", "nivel de gris radial (parte E)")
    ar([(3.2, 5.7), (4.55, 5.75)], "JSON UDP\n~9.5 Hz", (3.88, 6.15))
    ar([(6.0, 5.35), (6.0, 4.45)], "pose (x, y, θ) del marcador 4\nbroadcast UDP :5000", (6.0, 4.9))
    ar([(5.0, 4.45), (2.2, 5.2)], "marcador ArUco visible", (3.3, 4.62))
    ar([(8.0, 4.3), (9.3, 5.2)], "se desplaza sobre", (8.95, 4.6))
    ar([(3.2, 3.6), (4.0, 3.6)], "botones", (3.6, 3.85))
    ar([(8.0, 3.6), (8.8, 3.6)], "log", (8.4, 3.85))
    ar([(2.4, 1.75), (4.6, 2.75)], "5 V (USB)", (3.3, 2.38))
    ar([(6.0, 1.75), (6.0, 2.75)], "≈6.3 V", (6.0, 2.25))
    ar([(9.6, 1.75), (7.5, 2.75)], "reflectancia\n(5 sensores)", (8.75, 2.28))
    fig.savefig(OUT / "fig_contexto.png", dpi=200); plt.close(fig)


# ---------------------------------------------------------------- despliegue
def despliegue():
    fig, ax = canvas(13, 8.9, "Diagrama de despliegue",
                     "Nodos físicos, artefactos de software y enlaces entre ellos (versión del entregable, 17/11)")
    def node(x, y, w, h, name, stereo="«device»"):
        ax.add_patch(Rectangle((x, y), w, h, fc="white", ec=EDGE, lw=1.4, zorder=0))
        ax.text(x + 0.12, y + h - 0.12, f"{stereo}\n{name}", fontsize=10, weight="bold", va="top", color="black", linespacing=1.2)
    def art(x, y, w, h, title, sub, fc=PAY):
        box(ax, x, y, w, h, title, sub, fc=fc, ec=EDGE, r=0.03, fs=10, sfs=8.6)

    node(0.3, 4.6, 3.6, 2.6, "PC del laboratorio (192.168.12.231)")
    art(0.55, 4.85, 3.1, 1.25, "«artifact» Software LP", "detección ArUco\nbroadcast UDP :5000")
    node(0.3, 1.0, 3.6, 2.6, "Router TP-Link (192.168.12.1)")
    art(0.55, 1.25, 3.1, 1.2, "Hotspot WiFi", "SSID “embedded”, 2.4 GHz", fc=HDR)

    node(4.6, 0.75, 4.0, 6.45, "PCB del tractor", stereo="«board»")
    node(4.85, 4.35, 3.5, 2.2, "Raspberry Pi Pico 2 W")
    art(5.05, 4.55, 3.1, 1.25, "«artifact» PicoLPBridge.uf2", "cyw43 + lwIP: UDP :5000\nfiltra id 4 → LP_POSE 0x100")
    art(4.85, 3.1, 3.5, 0.7, "MCP2515 + TJA1050 (RASP)", "nodo CAN de la Pico", fc=HDR)
    art(4.85, 1.0, 3.5, 0.7, "MCP2515 + TJA1050 (ZUMO)", "nodo CAN del Zumo", fc=HDR)

    node(9.3, 1.0, 3.45, 6.2, "Pololu Zumo 2040 (RP2040)")
    art(9.5, 3.0, 3.05, 3.2, "«artifact»\nZumoWaypointFollower.uf2",
        "main.c · máquina de estados,\nbotones, pausa/ráfagas\nlp_can.c · decodifica 0x100\nfollower.c · seguidor por tramos\nwaypoints.c · tabla de 14 WP\nmcp2515.c · driver SPI")
    art(9.5, 1.25, 3.05, 1.2, "Motores y orugas", "librería Pololu", fc=HDR)

    arrow(ax, [(2.1, 3.6), (2.1, 4.6)], "WiFi / Ethernet", (2.1, 4.1))
    arrow(ax, [(3.9, 2.0), (4.4, 2.0), (4.4, 5.2), (5.05, 5.2)], None, color=BLUE, lw=1.5)
    ax.text(4.4, 3.6, "WiFi UDP broadcast · JSON ~9.5 Hz", fontsize=8.6, color=DBLUE, ha="center", va="center", rotation=90, bbox=dict(fc="white", ec="none", pad=1))
    arrow(ax, [(6.6, 4.35), (6.6, 3.8)], "SPI GP16–GP20", (7.5, 4.08))
    # bus CAN: dos hilos (CAN_H, CAN_L) y la resistencia de 120 Ω (jumper J1) en cada extremo
    xh, xl = 6.35, 6.95
    for xx in (xh, xl):
        ax.plot([xx, xx], [3.1, 1.7], color=BLUE, lw=1.8, zorder=1)
    ax.text(xh - 0.07, 2.4, "CAN_H", fontsize=8.4, color=DBLUE, ha="right", va="center")
    ax.text(xl + 0.07, 2.4, "CAN_L", fontsize=8.4, color=DBLUE, ha="left", va="center")
    for yy in (2.85, 1.95):
        ax.add_patch(Rectangle(((xh + xl) / 2 - 0.16, yy - 0.055), 0.32, 0.11, fc="white", ec="black", lw=1.0, zorder=3))
        ax.plot([xh, (xh + xl) / 2 - 0.16], [yy, yy], color="black", lw=1.0, zorder=2)
        ax.plot([(xh + xl) / 2 + 0.16, xl], [yy, yy], color="black", lw=1.0, zorder=2)
        ax.text(xl + 0.07, yy, "120 Ω (J1)", fontsize=8.0, color=TXT, ha="left", va="center")
    ax.text(5.45, 2.4, "CAN 500 kbps\nLP_POSE 0x100\nDLC 6", fontsize=8.4, color=DBLUE, ha="center", va="center", linespacing=1.2)
    arrow(ax, [(8.35, 1.35), (9.0, 1.35), (9.0, 4.6), (9.5, 4.6)], None)
    ax.text(9.0, 3.0, "SPI GP0–GP3 · INT GP28", fontsize=8.6, color=TXT, ha="center", va="center", rotation=90, bbox=dict(fc="white", ec="none", pad=1))
    arrow(ax, [(11.0, 3.0), (11.0, 2.45)], "PWM", (11.35, 2.72))

    # alimentación
    box(ax, 4.85, 7.45, 2.0, 0.6, "Power bank 5 V", fc=CYAN, fs=9.5)
    arrow(ax, [(5.85, 7.45), (5.85, 6.55)])
    ax.text(5.98, 6.85, "micro-USB → VBUS/VSYS", fontsize=8.4, color=TXT, va="center")
    arrow(ax, [(9.3, 0.5), (8.2, 0.5), (8.2, 0.75)], None, color=RED, ls=(0, (2, 2)))
    ax.text(10.95, 0.5, "VSW → D1, D2 → VCC_CAN: no se usa (falla CAN, problema 2)",
            fontsize=8.6, color=RED, ha="center", va="center", bbox=dict(fc="white", ec="none", pad=1))
    box(ax, 9.75, 7.45, 2.6, 0.6, "Laptop: monitor serial", fc=CYAN, fs=9.5)
    arrow(ax, [(11.05, 7.2), (11.05, 7.45)], None)
    ax.text(11.2, 7.32, "USB", fontsize=8.4, va="center")
    fig.savefig(OUT / "fig_despliegue.png", dpi=200); plt.close(fig)


# ---------------------------------------------------------------- actividades
def actividad():
    W, H = 11.5, 15.2
    fig, ax = canvas(W, H, "Diagrama de actividades: seguimiento de la vuelta",
                     "Del arranque en P14 al regreso a P14 (firmware ZumoWaypointFollower, versión del entregable)")
    cx, bw = 5.2, 4.6
    def act(y, text, h=0.62, x=cx, w=bw, fc="white"):
        box(ax, x - w / 2, y - h / 2, w, h, text, fc=fc, r=0.15, fs=9.4)
    def down(y1, y2, x=cx, label=None, lp=None):
        arrow(ax, [(x, y1), (x, y2)], label, lp)

    y = 13.95
    ax.add_patch(Circle((cx, y), 0.16, fc="black", zorder=3))
    down(y - 0.16, 13.45)
    act(13.15, "Armar la ruta con los 14 waypoints: 7 rectas, 5 arcos de 180° y 2 de 90°\n(centro, radio y sentido de cada arco con los dos waypoints medidos)", h=0.62)
    down(12.84, 12.45)
    act(12.15, "Leer el CAN e imprimir la pose y la distancia a P14", h=0.5)
    down(11.9, 11.55)
    diamond(ax, cx, 11.05, 2.6, 1.0, "¿Botón?")
    # B 3 s -> calibrar
    act(11.05, "Calibrar con la cámara\n(MOTOR_K, MOTOR_DEAD, WHEEL_BASE)", x=1.45, w=2.5, h=0.75, fc=HDR)
    arrow(ax, [(cx - 1.3, 11.05), (2.7, 11.05)], "B 3 s", (3.3, 11.25))
    arrow(ax, [(1.45, 11.43), (1.45, 12.15), (cx - bw / 2, 12.15)])
    arrow(ax, [(cx + 1.3, 11.05), (8.3, 11.05), (8.3, 12.15), (cx + bw / 2, 12.15)], "ninguno", (8.3, 11.6))
    down(10.55, 10.2, label="A", lp=(cx + 0.2, 10.38))
    act(9.95, "Esperar una pose nueva (≤ 500 ms)", h=0.5)
    down(9.7, 9.4)
    diamond(ax, cx, 8.9, 3.4, 1.0, "¿Pose reciente y a\n≤ 10 cm de P14?")
    arrow(ax, [(cx + 1.7, 8.9), (9.6, 8.9), (9.6, 12.15), (8.3, 12.15)], "No", (9.0, 9.1))
    down(8.4, 8.05, label="Sí", lp=(cx + 0.25, 8.23))
    act(7.8, "Iniciar la vuelta (t₀), primer tramo P14 → P1", h=0.5)
    down(7.55, 7.2)
    diamond(ax, cx, 6.7, 2.6, 1.0, "¿Pose nueva?")
    # rama sin pose
    diamond(ax, 9.3, 6.7, 2.6, 1.0, "¿> 800 ms\nsin pose?", fs=8.8)
    arrow(ax, [(cx + 1.3, 6.7), (8.0, 6.7)], "No", (7.0, 6.9))
    act(7.85, "Seguir con el último comando", x=9.3, w=2.5, h=0.5)
    arrow(ax, [(9.3, 7.2), (9.3, 7.6)], "No", (9.5, 7.4))
    arrow(ax, [(9.3 - 1.25, 7.85), (7.95, 7.85), (7.95, 7.38)], head=False)
    act(5.3, "Motores en 0: PAUSA\n(> 3 s → ALTO · B → paro)", x=9.3, w=2.5, h=0.7, fc=TRL)
    arrow(ax, [(9.3, 6.2), (9.3, 5.65)], "Sí", (9.5, 5.92))
    arrow(ax, [(10.55, 5.3), (11.1, 5.3), (11.1, 8.4), (7.95, 8.4), (7.95, 7.38)], head=False)
    down(6.2, 5.85, label="Sí", lp=(cx + 0.25, 6.03))
    act(5.55, "Si hubo hueco > 250 ms: juntar la ráfaga 20 ms\ny usar solo la pose más nueva", h=0.6)
    down(5.25, 4.95)
    act(4.7, "Compensar el retardo de la cámara con el último comando", h=0.5)
    down(4.45, 4.15)
    act(3.9, "Proyectar la pose sobre el tramo: avance t y error lateral e_y", h=0.5)
    down(3.65, 3.35)
    diamond(ax, cx, 2.9, 2.2, 0.9, "¿t ≥ 1?")
    diamond(ax, 1.45, 2.9, 2.5, 0.9, "¿Último tramo?", fs=8.8)
    arrow(ax, [(cx - 1.1, 2.9), (2.7, 2.9)], "Sí", (3.4, 3.08))
    act(3.9, "Pasar al siguiente\ntramo, en orden", x=1.45, w=2.3, h=0.6)
    arrow(ax, [(1.45, 3.35), (1.45, 3.6)], "No", (1.7, 3.47))
    arrow(ax, [(2.6, 3.9), (cx - bw / 2, 3.9)])
    box(ax, 0.3, 1.0, 2.3, 0.7, "Detener motores e\nimprimir el tiempo", fc=TRL, r=0.15, fs=9)
    arrow(ax, [(1.45, 2.45), (1.45, 1.7)], "Sí", (1.7, 2.1))
    ax.add_patch(Circle((1.45, 0.62), 0.17, fc="white", ec="black", lw=1.4, zorder=3))
    ax.add_patch(Circle((1.45, 0.62), 0.1, fc="black", zorder=4))
    arrow(ax, [(1.45, 1.0), (1.45, 0.79)])
    down(2.45, 2.21, label="No", lp=(cx - 0.3, 2.33))
    act(1.9, "v: 0.08 m/s en rectas, 0.06 m/s en arcos (frena antes del arco)\nω = v/R + k_θ·(θ_ref − θ),  θ_ref = tangente − atan(k_e·e_y)", h=0.62, w=5.4)
    down(1.59, 1.3)
    act(1.05, "Orugas: v − ω·WB/2 y v + ω·WB/2 → motores (zona muerta) · log L,…", h=0.5, w=5.0)
    arrow(ax, [(cx + 2.5, 1.05), (7.95, 1.05), (7.95, 7.38), (cx, 7.38)])
    fig.savefig(OUT / "fig_actividad.png", dpi=200); plt.close(fig)


# ---------------------------------------------------------------- alfa
def alfa():
    fig, ax = canvas(11, 6.2, "Geometría para medir el ángulo α",
                     "Fila de 5 sensores girada α respecto a la tangente del arco que pasa por el sensor central")
    ox, oy, R0, phi = 0.8, 0.7, 4.6, math.radians(35)
    cs = (ox + R0 * math.cos(phi), oy + R0 * math.sin(phi))
    for r, c, lw in [(R0 - 0.7, "#D9D9D9", 1.1), (R0, GRAY, 1.6), (R0 + 0.7, "#D9D9D9", 1.1)]:
        ax.add_patch(Arc((ox, oy), 2 * r, 2 * r, theta1=8, theta2=56, color=c, lw=lw, zorder=1))
    ax.plot([ox], [oy], "o", color=TXT, ms=4)
    ax.text(ox + 0.1, oy - 0.3, "centro del gradiente (r = 0)", fontsize=9)
    ax.plot([ox, cs[0]], [oy, cs[1]], color=GRAY, lw=1, ls=(0, (3, 3)))
    ax.text((ox + cs[0]) / 2 + 0.05, (oy + cs[1]) / 2 - 0.3, "r₀", fontsize=12)
    t = (-math.sin(phi), math.cos(phi))
    L = 2.0
    ax.plot([cs[0] - L * t[0], cs[0] + L * t[0]], [cs[1] - L * t[1], cs[1] + L * t[1]], color=BLUE, lw=1.4, ls=(0, (5, 3)))
    ax.text(cs[0] + L * t[0] - 0.15, cs[1] + L * t[1] + 0.1, "tangente", color=DBLUE, fontsize=10, ha="center")
    al = math.radians(28)
    a = phi + math.pi / 2 + al
    u = (math.cos(a), math.sin(a)); n = (math.cos(a - math.pi / 2), math.sin(a - math.pi / 2))
    k = 0.040
    ax.plot([cs[0] - 50 * k * u[0], cs[0] + 50 * k * u[0]], [cs[1] - 50 * k * u[1], cs[1] + 50 * k * u[1]], color=TXT, lw=1.2)
    for i, sv in enumerate([-45, -10, 0, 10, 45]):
        p = (cs[0] + sv * k * u[0], cs[1] + sv * k * u[1])
        ax.add_patch(Rectangle((p[0] - 0.08, p[1] - 0.08), 0.16, 0.16, fc=YEL, ec=DYEL, lw=1, zorder=3))
        sg = 1 if i in (0, 2, 4) else -1
        d = 0.32 if i in (0, 2, 4) else 0.3
        ax.text(p[0] + sg * d * n[0], p[1] + sg * d * n[1], f"s{5-i}", fontsize=9.5, color=DYEL, ha="center", va="center", weight="bold")
    ax.add_patch(Arc(cs, 2.4, 2.4, theta1=math.degrees(phi) + 90, theta2=math.degrees(a), color=RED, lw=1.5))
    mid = (math.radians(math.degrees(phi) + 90) + a) / 2
    ax.text(cs[0] + 1.45 * math.cos(mid), cs[1] + 1.45 * math.sin(mid), "α", fontsize=15, color=RED, ha="center", va="center")
    x0 = 7.0
    ax.text(x0, 5.1, "Posición de cada sensor sobre la fila", fontsize=10.5, weight="bold", color="black")
    ax.text(x0, 4.75, "s = −45, −10, 0, +10, +45 mm", fontsize=10, color=DYEL)
    ax.text(x0, 4.25, "Relación usada", fontsize=10.5, weight="bold", color="black")
    ax.text(x0, 3.9, "rᵢ = r₀ + sᵢ · sen α", fontsize=12)
    ax.text(x0, 3.4, "Pasos", fontsize=10.5, weight="bold", color="black")
    ax.text(x0, 3.1, "1. tᵢ (µs) → rᵢ con la tabla del sensor i\n2. Recta rᵢ contra sᵢ (mínimos cuadrados)\n3. pendiente = sen α\n4. α = asen(pendiente)",
            fontsize=10, va="top", linespacing=1.55)
    fig.savefig(OUT / "fig_alfa.png", dpi=200); plt.close(fig)


# ---------------------------------------------------------------- mapa y vuelta
def load_wps():
    pat = re.compile(r"/\*\s*P(\d+)\s*\*/\s*\{\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f")
    return [(int(m[1]), float(m[2]), float(m[3]), float(m[4])) for m in pat.finditer((SD / "ZumoWaypointFollower/waypoints.c").read_text())]


def load_route():
    out = subprocess.run([str(SD / "sim/sim_follower"), "--ruta"], capture_output=True, text=True, check=True).stdout
    rows = list(csv.DictReader(io.StringIO(out)))
    return [float(r["x"]) for r in rows], [float(r["y"]) for r in rows]


def load_log(p):
    xs, ys = [], []
    for line in Path(p).read_text(errors="replace").splitlines():
        q = line.strip().split(",")
        if q[0] == "L" and len(q) >= 4 and q[1] != "t_ms":
            xs.append(float(q[2])); ys.append(float(q[3]))
    return xs, ys


def ruta(log, name, title, sub, foot):
    wps = load_wps(); rx, ry = load_route()
    fig = plt.figure(figsize=(6.6, 9.4))
    ax = fig.add_axes([0.13, 0.15, 0.82, 0.74])
    fig.text(0.04, 0.975, title, fontsize=14, weight="bold", color="black", va="top")
    fig.text(0.04, 0.945, sub, fontsize=10.5, color="#7F7F7F", va="top")
    fig.text(0.04, 0.008, foot, fontsize=8.5, color="#7F7F7F")
    ax.plot(rx, ry, color="#E3E3E3", lw=13, solid_capstyle="round", zorder=1, label="pista (ancho aprox.)")
    ax.plot(rx, ry, color=BLUE, lw=1.6, zorder=2, label="ruta deseada")
    if log:
        tx, ty = load_log(log)
        ax.plot(tx, ty, color=YEL, lw=1.9, zorder=3, label="trayectoria medida (cámara)")
        ax.plot(tx[0], ty[0], "o", color=DYEL, ms=5, zorder=5)
    for n, x, y, th in wps:
        is90 = n in (11, 12, 13, 14)
        ax.scatter([x], [y], s=46, marker="s" if is90 else "o", color=GRN, edgecolor=DGRN, lw=1, zorder=4)
        dx = -0.058 if x < 0 else 0.014
        dy = 0.022 if y > 0.7 else -0.04
        ax.text(x + dx, y + dy, f"P{n}", fontsize=10, color=DGRN, weight="bold", zorder=5)
    s = next(w for w in wps if w[0] == 14)
    ax.annotate("inicio / fin", (s[1], s[2]), xytext=(s[1] + 0.07, s[2] - 0.02), fontsize=9.5,
                arrowprops=dict(arrowstyle="-|>", color="black", lw=0.9))
    ax.scatter([], [], s=46, marker="o", color=GRN, edgecolor=DGRN, label="waypoint de vuelta de 180°")
    ax.scatter([], [], s=46, marker="s", color=GRN, edgecolor=DGRN, label="waypoint de vuelta de 90°")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    for sp in ("top", "right"): ax.spines[sp].set_visible(False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.065), ncol=2, fontsize=9, frameon=False)
    fig.savefig(OUT / name, dpi=200); plt.close(fig)


contexto(); despliegue(); actividad(); alfa()
ruta(None, "fig_mapa.png", "Mapa calibrado con los waypoints numerados",
     "Coordenadas del sistema de posicionamiento local (m), medidas el 17/11",
     "Fuente: elaboración propia con waypoints.c y la ruta que arma el firmware.")
ruta(SD / "logs/vuelta4.txt", "fig_vuelta3.png", "Ruta deseada vs trayectoria medida",
     "Vuelta 3 (17/11): 89.04 s, error lateral RMS 21 mm",
     "Fuente: log serial del Zumo, poses de la cámara.")
print("ok")
