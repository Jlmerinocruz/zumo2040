"""GUI para elegir a cuál de los 14 waypoints debe ir el Zumo (función innovadora).

Uso (desde SecondDelivery/, conectado al WiFi del laboratorio):
    python gui/elegir_waypoint.py --pico 192.168.12.57
    python gui/elegir_waypoint.py            # sin --pico: broadcast

Clic en un waypoint = el Zumo va hacia allá siguiendo la pista. "Alto" lo detiene.
Mapa de calor del suelo: durante cualquier recorrido (vuelta con el botón A o GOTO) el
Zumo manda sus 5 sensores de reflectancia y la GUI los pinta sobre la pista, de claro
(amarillo) a oscuro (morado). "Borrar mapa" empieza uno nuevo.

Comandos (texto por UDP al puerto 5002 de la Pico 2 W):
    GOTO n   (n = 1..14)   ->  Pico: CAN 0x110 WP_GOTO con el byte n
    STOP                   ->  Pico: CAN 0x110 WP_GOTO con el byte 0
    HOLA (cada 2 s)        ->  solo registra esta dirección para recibir las respuestas
Respuestas de la Pico a la dirección que mandó el último comando:
    OK GOTO n / OK STOP    el Zumo confirmó el frame por CAN (ACK)
    ERR CAN                nadie dio ACK (revisar bus o Zumo)
    ESTADO e n             reenvío de CAN 0x111 WP_STATUS (e = código, n = destino)
    PISO b1 b2 b3 b4 b5    reenvío de CAN 0x112 FLOOR (tiempo de descarga de S1..S5
                           en unidades de 4 µs; S1 = izquierda del Zumo)

Además se suscribe a la cámara (como la Pico) para dibujar al Zumo en vivo.
Solo usa la biblioteca estándar de Python (tkinter incluido).
"""
import argparse
import bisect
import collections
import json
import math
import queue
import re
import socket
import threading
import time
import tkinter as tk
from pathlib import Path

WAYPOINTS_C = Path(__file__).resolve().parent.parent / "ZumoWaypointFollower" / "waypoints.c"
PICO_PORT = 5002
CAMERA_PORT = 5000
SUBSCRIBE_PORT = 5001
MARKER_ID = 4
POSE_TIMEOUT_S = 1.0
HELLO_PERIOD_MS = 2000

# Mapa de calor del suelo. La lectura de los sensores llega casi al instante y la pose
# de la cámara ~0.12 s después de tomada la foto, así que cada lectura se pinta con la
# primera pose que llegue FLOOR_DELAY_S después de ella.
FLOOR_DELAY_S = 0.12
FLOOR_MAX_WAIT_S = 1.0          # sin pose en ese tiempo, la lectura se descarta
# Posición de los sensores respecto al marcador: SENSOR_AHEAD_M hacia el frente (aprox.,
# medir con regla en el Zumo) y hacia la izquierda S1..S5 (medido, ver CLAUDE.md).
SENSOR_AHEAD_M = 0.04
SENSOR_LEFT_M = (0.045, 0.010, 0.0, -0.010, -0.045)
DOT_PX = 3
# Escala de cada sensor: del percentil 5 al 95 de lo visto, no del mínimo al máximo.
# En la prueba del 26/11 S1 y S5 cruzaban la línea negra en las vueltas y esas pocas
# lecturas estiraban la escala: todas las parcelas salían del mismo amarillo.
# FLOOR_MIN_SPAN (unidades de 4 µs) evita pintar como contraste el ruido de un piso parejo.
FLOOR_LOW_PCT, FLOOR_HIGH_PCT = 0.05, 0.95
FLOOR_MIN_SPAN = 8
FLOOR_RESCALE_MS = 1000
# De claro a oscuro (paleta tipo viridis invertida).
HEAT_COLORS = ((253, 231, 37), (94, 201, 98), (33, 145, 140), (59, 82, 139), (68, 1, 84))

# Códigos del byte 0 de WP_STATUS (ver goto en ZumoWaypointFollower/main.c).
STATUS = {
    0: "en espera",
    1: "en camino a P{n}",
    2: "llegó a P{n}",
    3: "no arrancó hacia P{n}: sin pose o fuera de la pista",
    4: "detenido (alto o se perdió la pose)",
}

# Área de la pista en metros y tamaño del mapa en pixeles.
X_MIN, X_MAX, Y_MIN, Y_MAX = -0.50, 0.50, -0.02, 1.18
SCALE = 520
MARGIN = 30


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
    """Ruta que sigue el Zumo: rectas entre waypoints con el mismo rumbo y arcos en las vueltas."""
    pts = []
    for i, (_, ax, ay, ta) in enumerate(waypoints):
        _, bx, by, tb = waypoints[(i + 1) % len(waypoints)]
        ta, tb = math.radians(ta), math.radians(tb)
        if abs(wrap_pi(tb - ta)) < math.radians(30):
            pts += [(ax, ay), (bx, by)]
        else:
            pts += arc_points(ax, ay, ta, bx, by, tb)
    return pts


def heat_color(v):
    """Color de la paleta para v en [0, 1] (0 = claro, 1 = oscuro)."""
    v = min(max(v, 0.0), 1.0) * (len(HEAT_COLORS) - 1)
    k = min(int(v), len(HEAT_COLORS) - 2)
    a, b = HEAT_COLORS[k], HEAT_COLORS[k + 1]
    f = v - k
    return "#%02x%02x%02x" % tuple(round(a[j] + (b[j] - a[j]) * f) for j in range(3))


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
        self.pose = None
        self.pose_time = 0.0
        self.floor_pending = collections.deque()   # (hora, 5 lecturas) sin pintar
        self.floor_dots = []                       # (item del canvas, sensor, lectura)
        self.floor_values = [[] for _ in range(5)]  # lecturas ordenadas de cada sensor
        self.floor_scale = [(0, FLOOR_MIN_SPAN)] * 5  # (claro, oscuro) de cada sensor

        root.title("AgroZumo: elegir waypoint")
        w = MARGIN * 2 + (X_MAX - X_MIN) * SCALE
        h = MARGIN * 2 + (Y_MAX - Y_MIN) * SCALE
        self.canvas = tk.Canvas(root, width=w, height=h, bg="white")
        self.canvas.grid(row=0, column=0, rowspan=6, padx=8, pady=8)

        tk.Label(root, text="IP de la Pico 2 W (vacío = broadcast)").grid(row=0, column=1, sticky="sw")
        self.pico_entry = tk.Entry(root, width=18)
        self.pico_entry.insert(0, pico_ip)
        self.pico_entry.grid(row=1, column=1, sticky="nw")
        tk.Button(root, text="Alto", bg="#c0392b", fg="white", width=14,
                  command=lambda: self.send("STOP")).grid(row=2, column=1, sticky="nw")
        self.status = tk.Label(root, text="Elige un waypoint en el mapa", wraplength=230,
                               justify="left", font=("TkDefaultFont", 11, "bold"))
        self.status.grid(row=3, column=1, sticky="nw", pady=8)
        self.log = tk.Text(root, width=34, height=16, state="disabled")
        self.log.grid(row=4, column=1, sticky="nw", padx=(0, 8))
        self.make_legend(root).grid(row=5, column=1, sticky="nw", pady=8)

        self.waypoints = load_waypoints()
        self.draw_map()

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.cmd_sock.bind(("", 0))
        threading.Thread(target=self.rx_pico, daemon=True).start()
        threading.Thread(target=self.rx_camera, args=(server_ip,), daemon=True).start()
        self.root.after(50, self.process_events)
        self.say_hello()
        self.rescale_floor()

    def make_legend(self, root):
        frame = tk.Frame(root)
        tk.Label(frame, text="Mapa de calor del suelo (reflectancia)").pack(anchor="w")
        bar = tk.Canvas(frame, width=230, height=16, highlightthickness=0)
        for k in range(230):
            bar.create_line(k, 0, k, 16, fill=heat_color(k / 229))
        bar.pack(anchor="w")
        tk.Label(frame, text="claro" + " " * 44 + "oscuro").pack(anchor="w")
        tk.Label(frame, text="Escala automática: percentiles 5 y 95 de cada sensor",
                 fg="#666", wraplength=230, justify="left").pack(anchor="w")
        tk.Button(frame, text="Borrar mapa", command=self.clear_floor).pack(anchor="w", pady=4)
        return frame

    def say_hello(self):
        """Registra la GUI en la Pico para recibir PISO aunque no se haya mandado un comando."""
        ip = self.pico_entry.get().strip() or "255.255.255.255"
        try:
            self.cmd_sock.sendto(b"HOLA", (ip, PICO_PORT))
        except OSError:
            pass
        self.root.after(HELLO_PERIOD_MS, self.say_hello)

    def draw_map(self):
        c = self.canvas
        pts = [to_px(x, y) for x, y in route_points(self.waypoints)]
        c.create_line(*[v for p in pts for v in p], fill="#bbb", width=3)
        for n, x, y, _ in self.waypoints:
            px, py = to_px(x, y)
            tag = f"wp{n}"
            c.create_oval(px - 11, py - 11, px + 11, py + 11, fill="#2e86c1", outline="",
                          tags=(tag, "wp"))
            c.create_text(px, py, text=str(n), fill="white", font=("TkDefaultFont", 9, "bold"),
                          tags=(tag,))
            c.tag_bind(tag, "<Button-1>", lambda _e, n=n: self.goto(n))
        c.create_text(MARGIN, MARGIN // 2, anchor="w", fill="#666",
                      text="Clic en un punto: el Zumo va hacia allá siguiendo la pista")

    def goto(self, n):
        self.target = n
        for wn, *_ in self.waypoints:
            self.canvas.itemconfigure(f"wp{wn}&&wp", fill="#e67e22" if wn == n else "#2e86c1")
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
            self.events.put(("pico", (data.decode(errors="replace").strip(), addr[0],
                                      time.monotonic())))

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
                self.events.put(("pose", (pose, time.monotonic())))

    def process_events(self):
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == "pose":
                self.pose, self.pose_time = value
                self.paint_floor()
            elif kind == "info":
                self.add_log(value)
            else:
                self.on_pico(*value)
        self.draw_zumo()
        self.root.after(50, self.process_events)

    def on_pico(self, text, ip, t):
        parts = text.split()
        if parts[:1] == ["PISO"] and len(parts) == 6 and all(p.isdigit() for p in parts[1:]):
            self.floor_pending.append((t, [int(p) for p in parts[1:]]))
            return
        self.add_log(f"<- {ip}: {text}")
        if parts[:1] == ["ESTADO"] and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
            code, n = int(parts[1]), int(parts[2])
            self.status.configure(text="Zumo " + STATUS.get(code, f"estado {code}").format(n=n))
            if code == 2:
                self.canvas.itemconfigure(f"wp{n}&&wp", fill="#27ae60")
        elif text == "ERR CAN":
            self.status.configure(text="La Pico no recibió ACK del Zumo por CAN")

    def paint_floor(self):
        """Pinta con la pose actual las lecturas tomadas cuando se tomó su foto."""
        x, y, ang = self.pose
        th = math.radians(ang)
        while self.floor_pending and self.pose_time - self.floor_pending[0][0] >= FLOOR_DELAY_S:
            t, values = self.floor_pending.popleft()
            if self.pose_time - t > FLOOR_MAX_WAIT_S:
                continue
            for i, b in enumerate(values):
                bisect.insort(self.floor_values[i], b)
                left = SENSOR_LEFT_M[i]
                px, py = to_px(x + SENSOR_AHEAD_M * math.cos(th) - left * math.sin(th),
                               y + SENSOR_AHEAD_M * math.sin(th) + left * math.cos(th))
                item = self.canvas.create_rectangle(px - DOT_PX, py - DOT_PX, px + DOT_PX,
                                                    py + DOT_PX, outline="", tags="piso",
                                                    fill=self.floor_color(i, b))
                self.canvas.tag_lower(item, "wp")
                self.floor_dots.append((item, i, b))

    def rescale_floor(self):
        """Recalcula la escala de cada sensor (1 vez por segundo) y repinta si cambió."""
        scale = []
        for vals in self.floor_values:
            if not vals:
                scale.append((0, FLOOR_MIN_SPAN))
                continue
            lo = vals[int(FLOOR_LOW_PCT * (len(vals) - 1))]
            hi = vals[int(FLOOR_HIGH_PCT * (len(vals) - 1))]
            scale.append((lo, max(hi, lo + FLOOR_MIN_SPAN)))
        if scale != self.floor_scale:
            self.floor_scale = scale
            for item, i, b in self.floor_dots:
                self.canvas.itemconfigure(item, fill=self.floor_color(i, b))
        self.root.after(FLOOR_RESCALE_MS, self.rescale_floor)

    def floor_color(self, i, b):
        lo, hi = self.floor_scale[i]
        return heat_color((b - lo) / (hi - lo))

    def clear_floor(self):
        self.canvas.delete("piso")
        self.floor_dots.clear()
        self.floor_pending.clear()
        self.floor_values = [[] for _ in range(5)]

    def draw_zumo(self):
        self.canvas.delete("zumo")
        if not self.pose or time.monotonic() - self.pose_time > POSE_TIMEOUT_S:
            return
        x, y, ang = self.pose
        px, py = to_px(x, y)
        c = self.canvas
        c.create_oval(px - 9, py - 9, px + 9, py + 9, outline="#111", width=2, tags="zumo")
        rad = math.radians(ang)
        c.create_line(px, py, px + 20 * math.cos(rad), py - 20 * math.sin(rad), arrow="last",
                      width=2, tags="zumo")

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
