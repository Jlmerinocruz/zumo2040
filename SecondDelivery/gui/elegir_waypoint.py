"""GUI para elegir a cuál de los 14 waypoints debe ir el Zumo (función innovadora).

Uso (desde SecondDelivery/, conectado al WiFi del laboratorio):
    python gui/elegir_waypoint.py --pico 192.168.12.57
    python gui/elegir_waypoint.py            # sin --pico: broadcast

Clic en un waypoint = el Zumo va hacia allá siguiendo la pista. "Alto" lo detiene.

Comandos (texto por UDP al puerto 5002 de la Pico 2 W):
    GOTO n   (n = 1..14)   ->  Pico: CAN 0x110 WP_GOTO con el byte n
    STOP                   ->  Pico: CAN 0x110 WP_GOTO con el byte 0
Respuestas de la Pico a la dirección que mandó el último comando:
    OK GOTO n / OK STOP    el Zumo confirmó el frame por CAN (ACK)
    ERR CAN                nadie dio ACK (revisar bus o Zumo)
    ESTADO e n             reenvío de CAN 0x111 WP_STATUS (e = código, n = destino)

Además se suscribe a la cámara (como la Pico) para dibujar al Zumo en vivo con su rastro,
y muestra la velocidad medida, el tiempo del recorrido y la distancia que falta.
Solo usa la biblioteca estándar de Python (tkinter incluido).
"""
import argparse
import json
import math
import queue
import re
import socket
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path

WAYPOINTS_C = Path(__file__).resolve().parent.parent / "ZumoWaypointFollower" / "waypoints.c"
PICO_PORT = 5002
CAMERA_PORT = 5000
SUBSCRIBE_PORT = 5001
MARKER_ID = 4
POSE_TIMEOUT_S = 1.0

# Códigos del byte 0 de WP_STATUS (ver goto en ZumoWaypointFollower/main.c).
STATUS = {
    0: "en espera",
    1: "en camino a P{n}",
    2: "llegó a P{n}",
    3: "no arrancó hacia P{n}: sin pose o fuera de la pista",
    4: "detenido (alto o se perdió la pose)",
    5: "detenido por un obstáculo al frente (va a P{n})",
}

# Área de la pista en metros y tamaño del mapa en pixeles.
X_MIN, X_MAX, Y_MIN, Y_MAX = -0.50, 0.50, -0.02, 1.18
SCALE = 520
MARGIN = 30

TRAIL_MAX = 1500        # puntos del rastro (~3 min de cámara a 8 Hz)
SPEED_WINDOW_S = 1.0    # ventana para la velocidad medida con la cámara
BLINK_MS = 400          # periodo del parpadeo del destino

# Paleta y tipografía.
GREEN, GREEN_DARK, YELLOW = "#2f7d32", "#1b4d1e", "#f9d71c"
BG, PANEL, TEXT, MUTED = "#f4f6f1", "#ffffff", "#1f2a1f", "#6b766b"
WP_IDLE, WP_TARGET, WP_DONE, ALERT = "#2e86c1", "#e67e22", "#27ae60", "#c0392b"
FONT = "Segoe UI"


def load_waypoints():
    pat = re.compile(r"/\*\s*P(\d+)\s*\*/\s*\{\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f")
    return [(int(m[1]), float(m[2]), float(m[3]), float(m[4]))
            for m in pat.finditer(WAYPOINTS_C.read_text())]


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def arc_points(ax, ay, ta, bx, by, tb, n=24):
    """Arco entre dos waypoints, calculado igual que build_arc() en follower.c."""
    dx, dy = bx - ax, by - ay
    d = 1 if math.cos(ta) * dy - math.sin(ta) * dx >= 0 else -1
    nax, nay = -math.sin(ta) * d, math.cos(ta) * d
    nbx, nby = -math.sin(tb) * d, math.cos(tb) * d
    mx, my = nax - nbx, nay - nby
    den = mx * mx + my * my
    r = -((ax - bx) * mx + (ay - by) * my) / den if den > 1e-6 else 0.5 * math.hypot(dx, dy)
    if r < 0.02:
        r = 0.5 * math.hypot(dx, dy)
    cx = 0.5 * ((ax + r * nax) + (bx + r * nbx))
    cy = 0.5 * ((ay + r * nay) + (by + r * nby))
    phi0 = math.atan2(ay - cy, ax - cx)
    span = ((math.atan2(by - cy, bx - cx) - phi0) * d) % (2 * math.pi)
    return [(cx + r * math.cos(phi0 + d * span * k / n), cy + r * math.sin(phi0 + d * span * k / n))
            for k in range(n + 1)]


def route_points(waypoints):
    """Ruta que sigue el Zumo: rectas entre waypoints con el mismo rumbo y arcos en las vueltas.

    Devuelve los puntos y, por cada waypoint, el índice del punto donde empieza su tramo."""
    pts, starts = [], {}
    for i, (n, ax, ay, ta) in enumerate(waypoints):
        _, bx, by, tb = waypoints[(i + 1) % len(waypoints)]
        ta, tb = math.radians(ta), math.radians(tb)
        starts[n] = len(pts)
        if abs(wrap_pi(tb - ta)) < math.radians(30):
            pts += [(ax, ay), (bx, by)]
        else:
            pts += arc_points(ax, ay, ta, bx, by, tb)
    return pts, starts


class Route:
    """Ruta como polilínea cerrada con distancia acumulada, para medir lo que falta."""

    def __init__(self, waypoints):
        self.pts, starts = route_points(waypoints)
        self.s = [0.0]
        for (ax, ay), (bx, by) in zip(self.pts, self.pts[1:]):
            self.s.append(self.s[-1] + math.hypot(bx - ax, by - ay))
        self.length = self.s[-1]
        self.wp_s = {n: self.s[i] for n, i in starts.items()}

    def locate(self, x, y):
        """Distancia sobre la ruta del punto de la ruta más cercano a (x, y)."""
        best_d, best_s = float("inf"), 0.0
        for i, ((ax, ay), (bx, by)) in enumerate(zip(self.pts, self.pts[1:])):
            vx, vy = bx - ax, by - ay
            seg = vx * vx + vy * vy
            t = 0.0 if seg < 1e-12 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / seg))
            d = math.hypot(ax + t * vx - x, ay + t * vy - y)
            if d < best_d:
                best_d, best_s = d, self.s[i] + t * (self.s[i + 1] - self.s[i])
        return best_s

    def remaining(self, x, y, wp):
        """Metros que faltan para llegar a wp siguiendo la pista en el sentido de la vuelta."""
        return (self.wp_s[wp] - self.locate(x, y)) % self.length


def to_px(x, y):
    return MARGIN + (x - X_MIN) * SCALE, MARGIN + (Y_MAX - y) * SCALE


def parse_pose(data):
    """Pose (x, y, ángulo) del marcador del Zumo en el JSON de la cámara, o None."""
    try:
        msg = json.loads(data)
    except ValueError:
        return None
    for m in msg.get("markers", []):
        if m.get("id") == MARKER_ID:
            return m["x"], m["y"], m["angle"]
    return None


class App:
    def __init__(self, root, pico_ip, server_ip):
        self.root = root
        self.events = queue.Queue()   # (tipo, dato) desde los hilos de red
        self.target = None
        self.moving = False           # el Zumo reportó "en camino" u "obstáculo"
        self.blink_on = False
        self.pose = None
        self.pose_time = 0.0
        self.trail = deque(maxlen=TRAIL_MAX)
        self.recent = deque()         # (t, x, y) del último segundo, para la velocidad
        self.trip_start = None
        self.trip_time = None

        root.title("AgroZumo: elegir waypoint")
        root.configure(bg=BG)
        self.build_header()
        w = MARGIN * 2 + (X_MAX - X_MIN) * SCALE
        h = MARGIN * 2 + (Y_MAX - Y_MIN) * SCALE
        self.canvas = tk.Canvas(root, width=w, height=h, bg=PANEL, highlightthickness=0)
        self.canvas.grid(row=1, column=0, padx=(14, 8), pady=14, sticky="n")
        self.build_panel(pico_ip)

        self.waypoints = load_waypoints()
        self.route = Route(self.waypoints)
        self.draw_map()

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.cmd_sock.bind(("", 0))
        threading.Thread(target=self.rx_pico, daemon=True).start()
        threading.Thread(target=self.rx_camera, args=(server_ip,), daemon=True).start()
        self.root.after(50, self.process_events)
        self.root.after(BLINK_MS, self.blink)

    def build_header(self):
        bar = tk.Frame(self.root, bg=GREEN, height=56)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        tk.Frame(bar, bg=YELLOW, width=8).pack(side="left", fill="y")
        tk.Label(bar, text="AgroZumo", bg=GREEN, fg="white",
                 font=(FONT, 20, "bold")).pack(side="left", padx=(12, 10), pady=8)
        tk.Label(bar, text="Clic en un punto: el tractor va hacia allá siguiendo la pista",
                 bg=GREEN, fg="#dfeedd", font=(FONT, 11)).pack(side="left", pady=8)

    def build_panel(self, pico_ip):
        panel = tk.Frame(self.root, bg=BG)
        panel.grid(row=1, column=1, sticky="n", padx=(0, 14), pady=14)

        tk.Label(panel, text="IP de la Pico 2 W (vacío = broadcast)", bg=BG, fg=MUTED,
                 font=(FONT, 9)).pack(anchor="w")
        self.pico_entry = tk.Entry(panel, width=22, font=(FONT, 10), relief="flat")
        self.pico_entry.insert(0, pico_ip)
        self.pico_entry.pack(anchor="w", pady=(2, 10), ipady=3)
        tk.Button(panel, text="ALTO", bg=ALERT, fg="white", activebackground="#962d22",
                  activeforeground="white", relief="flat", font=(FONT, 12, "bold"),
                  command=lambda: self.send("STOP")).pack(fill="x", ipady=6)

        card = tk.Frame(panel, bg=PANEL, padx=12, pady=10)
        card.pack(fill="x", pady=12)
        tk.Label(card, text="ESTADO", bg=PANEL, fg=MUTED, font=(FONT, 9, "bold")).pack(anchor="w")
        self.status = tk.Label(card, text="Elige un waypoint en el mapa", bg=PANEL, fg=TEXT,
                               wraplength=240, justify="left", font=(FONT, 12, "bold"))
        self.status.pack(anchor="w")
        self.alert = tk.Label(card, text="⚠ OBSTÁCULO AL FRENTE: detenido", bg=ALERT, fg="white",
                              font=(FONT, 11, "bold"), pady=6)

        metrics = tk.Frame(panel, bg=BG)
        metrics.pack(fill="x")
        self.speed_lbl = self.metric(metrics, 0, "VELOCIDAD")
        self.time_lbl = self.metric(metrics, 1, "TIEMPO")
        self.dist_lbl = self.metric(metrics, 2, "FALTA")

        tk.Label(panel, text="Registro", bg=BG, fg=MUTED, font=(FONT, 9, "bold")).pack(
            anchor="w", pady=(12, 2))
        self.log = tk.Text(panel, width=36, height=14, state="disabled", relief="flat",
                           bg=PANEL, fg=TEXT, font=("Consolas", 9))
        self.log.pack(fill="x")

    def metric(self, parent, col, title):
        box = tk.Frame(parent, bg=PANEL, padx=8, pady=6)
        box.grid(row=0, column=col, padx=(0 if col == 0 else 6, 0), sticky="nsew")
        parent.columnconfigure(col, weight=1, uniform="metric")
        tk.Label(box, text=title, bg=PANEL, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w")
        value = tk.Label(box, text="—", bg=PANEL, fg=GREEN_DARK, font=(FONT, 14, "bold"), width=8,
                         anchor="w")
        value.pack(anchor="w")
        return value

    def draw_map(self):
        c = self.canvas
        pts = [to_px(x, y) for x, y in self.route.pts]
        c.create_line(*[v for p in pts for v in p], fill="#d5dbd2", width=9, capstyle="round",
                      joinstyle="round")
        c.create_line(*[v for p in pts for v in p], fill="#9aa596", width=2, dash=(6, 4))
        for n, x, y, _ in self.waypoints:
            px, py = to_px(x, y)
            tag = f"wp{n}"
            c.create_oval(px - 12, py - 12, px + 12, py + 12, fill=WP_IDLE, outline="white",
                          width=2, tags=(tag, "wp"))
            c.create_text(px, py, text=str(n), fill="white", font=(FONT, 9, "bold"), tags=(tag,))
            c.tag_bind(tag, "<Button-1>", lambda _e, n=n: self.goto(n))
            c.tag_bind(tag, "<Enter>", lambda _e: c.configure(cursor="hand2"))
            c.tag_bind(tag, "<Leave>", lambda _e: c.configure(cursor=""))

    def paint_wp(self, n, color):
        self.canvas.itemconfigure(f"wp{n}&&wp", fill=color)

    def goto(self, n):
        if self.target is not None:
            self.paint_wp(self.target, WP_IDLE)
        self.target = n
        self.paint_wp(n, WP_TARGET)
        self.trail.clear()
        self.send(f"GOTO {n}")

    def send(self, text):
        ip = self.pico_entry.get().strip() or "255.255.255.255"
        try:
            self.cmd_sock.sendto(text.encode(), (ip, PICO_PORT))
            self.add_log(f"-> {ip}: {text}")
        except OSError as e:
            self.add_log(f"error al enviar: {e}")

    def rx_pico(self):
        while True:
            data, addr = self.cmd_sock.recvfrom(256)
            self.events.put(("pico", (data.decode(errors="replace").strip(), addr[0])))

    def rx_camera(self, server_ip):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", CAMERA_PORT))
        except OSError as e:
            self.events.put(("info", f"sin posición en vivo (puerto {CAMERA_PORT}: {e})"))
            return
        sock.settimeout(0.5)
        next_sub = 0.0
        while True:
            # Igual que la Pico: el servidor registra nuestra IP si le mandamos algo
            # al 5001 desde el puerto 5000, por si su broadcast no llega.
            if time.monotonic() >= next_sub:
                next_sub = time.monotonic() + 2.0
                try:
                    sock.sendto(b"subscribe", (server_ip, SUBSCRIBE_PORT))
                except OSError:
                    pass
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            pose = parse_pose(data)
            if pose:
                self.events.put(("pose", pose))

    def process_events(self):
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == "pose":
                self.on_pose(value)
            elif kind == "info":
                self.add_log(value)
            else:
                self.on_pico(*value)
        self.draw_zumo()
        self.update_metrics()
        self.root.after(50, self.process_events)

    def on_pose(self, pose):
        now = time.monotonic()
        self.pose, self.pose_time = pose, now
        x, y, _ = pose
        if self.moving:
            self.trail.append(to_px(x, y))
        self.recent.append((now, x, y))
        while now - self.recent[0][0] > SPEED_WINDOW_S:
            self.recent.popleft()

    def on_pico(self, text, ip):
        self.add_log(f"<- {ip}: {text}")
        parts = text.split()
        if parts[:1] == ["ESTADO"] and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
            self.on_status(int(parts[1]), int(parts[2]))
        elif text == "ERR CAN":
            self.status.configure(text="La Pico no recibió ACK del Zumo por CAN", fg=ALERT)

    def on_status(self, code, n):
        self.status.configure(text="Zumo " + STATUS.get(code, f"estado {code}").format(n=n),
                              fg=ALERT if code in (3, 5) else TEXT)
        if code == 5:
            self.alert.pack(fill="x", pady=(8, 0))
        else:
            self.alert.pack_forget()
        was_moving, self.moving = self.moving, code in (1, 5)
        if self.moving and not was_moving:
            self.trip_start, self.trip_time = time.monotonic(), None
        elif not self.moving and self.trip_start is not None:
            self.trip_time, self.trip_start = time.monotonic() - self.trip_start, None
        if code == 2:
            self.paint_wp(n, WP_DONE)
        elif not self.moving and self.target is not None:
            self.paint_wp(self.target, WP_TARGET)

    def blink(self):
        if self.moving and self.target is not None:
            self.blink_on = not self.blink_on
            self.paint_wp(self.target, YELLOW if self.blink_on else WP_TARGET)
        self.root.after(BLINK_MS, self.blink)

    def update_metrics(self):
        fresh = self.pose and time.monotonic() - self.pose_time <= POSE_TIMEOUT_S
        if fresh and len(self.recent) > 1:
            pts = list(self.recent)
            path = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))
            dt = self.recent[-1][0] - self.recent[0][0]
            self.speed_lbl.configure(text=f"{100 * path / dt:.1f} cm/s" if dt > 0 else "—")
        else:
            self.speed_lbl.configure(text="—")
        if self.trip_start is not None:
            self.time_lbl.configure(text=f"{time.monotonic() - self.trip_start:.1f} s")
        elif self.trip_time is not None:
            self.time_lbl.configure(text=f"{self.trip_time:.1f} s")
        if fresh and self.moving and self.target is not None:
            x, y, _ = self.pose
            self.dist_lbl.configure(text=f"{100 * self.route.remaining(x, y, self.target):.0f} cm")
        elif not self.moving:
            self.dist_lbl.configure(text="—")

    def draw_zumo(self):
        c = self.canvas
        c.delete("zumo")
        if len(self.trail) > 1:
            c.create_line(*[v for p in self.trail for v in p], fill=GREEN, width=3,
                          capstyle="round", joinstyle="round", tags=("zumo", "trail"))
            c.tag_lower("trail", "wp")   # el rastro va debajo de los waypoints
        if not self.pose or time.monotonic() - self.pose_time > POSE_TIMEOUT_S:
            return
        x, y, ang = self.pose
        px, py = to_px(x, y)
        rad = math.radians(ang)
        c.create_oval(px - 11, py - 11, px + 11, py + 11, fill=YELLOW, outline=GREEN_DARK,
                      width=2, tags="zumo")
        c.create_line(px, py, px + 24 * math.cos(rad), py - 24 * math.sin(rad), arrow="last",
                      arrowshape=(10, 12, 4), fill=GREEN_DARK, width=3, tags="zumo")

    def add_log(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S ") + line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pico", default="", help="IP de la Pico 2 W (la imprime por serial)")
    ap.add_argument("--servidor", default="192.168.12.1", help="IP del servidor de la cámara")
    args = ap.parse_args()
    root = tk.Tk()
    App(root, args.pico, args.servidor)
    root.mainloop()


if __name__ == "__main__":
    main()
