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

Además se suscribe a la cámara (como la Pico) para dibujar al Zumo en vivo.
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
}

# Área de la pista en metros y tamaño del mapa en pixeles.
X_MIN, X_MAX, Y_MIN, Y_MAX = -0.50, 0.50, -0.02, 1.18
SCALE = 520
MARGIN = 30


def load_waypoints():
    pat = re.compile(r"/\*\s*P(\d+)\s*\*/\s*\{\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f")
    return [(int(m[1]), float(m[2]), float(m[3])) for m in pat.finditer(WAYPOINTS_C.read_text())]


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

        self.waypoints = load_waypoints()
        self.draw_map()

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.cmd_sock.bind(("", 0))
        threading.Thread(target=self.rx_pico, daemon=True).start()
        threading.Thread(target=self.rx_camera, args=(server_ip,), daemon=True).start()
        self.root.after(50, self.process_events)

    def draw_map(self):
        c = self.canvas
        pts = [to_px(x, y) for _, x, y in self.waypoints]
        c.create_line(*[v for p in pts + pts[:1] for v in p], fill="#bbb", width=3)
        for n, x, y in self.waypoints:
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
        for wn, _, _ in self.waypoints:
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
                self.pose, self.pose_time = value, time.monotonic()
            elif kind == "info":
                self.add_log(value)
            else:
                self.on_pico(*value)
        self.draw_zumo()
        self.root.after(50, self.process_events)

    def on_pico(self, text, ip):
        self.add_log(f"<- {ip}: {text}")
        parts = text.split()
        if parts[:1] == ["ESTADO"] and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
            code, n = int(parts[1]), int(parts[2])
            self.status.configure(text="Zumo " + STATUS.get(code, f"estado {code}").format(n=n))
            if code == 2:
                self.canvas.itemconfigure(f"wp{n}&&wp", fill="#27ae60")
        elif text == "ERR CAN":
            self.status.configure(text="La Pico no recibió ACK del Zumo por CAN")

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
