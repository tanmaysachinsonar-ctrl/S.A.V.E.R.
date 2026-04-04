"""
S.A.V.E.R – drone.py
PicoSender (Drohnen-Kommunikation via TCP) + autonomer Navigations-Thread.
"""

import threading
import time
import socket
import numpy as np
from collections import deque

from config import (
    g, _lock,
    PICO_IP, PICO_PORT,
    SOCKET_CONNECT_TIMEOUT, SOCKET_RECV_TIMEOUT,
    MAX_TRIM_COUNT, COMMAND_INTERVAL,
    TARGET_REACHED_DIST, SLOWDOWN_RADIUS, SLOWDOWN_TRIMS,
    ORIENTATION_UPDATE_RADIUS, ORIENT_EMA_ALPHA,
    CORRIDOR_HALF_WIDTH, DEADZONE_PIXELS, PIXELS_MIN_MARKER,
    INVERT_LATERAL, PRIORITIZE_BY_ANGLE, PRIORITIZE_LATERAL,
    PRIORITIZE_LATERAL_THRESHOLD, ANGLE_PRIORITY_THRESHOLD_DEG,
    angle_wrap, angle_diff, image_to_drone_coords, point_in_corridor,
)

# Globale PicoSender-Instanz – wird von main.py gesetzt
_pico_sender = None


# ═══════════════════════════════════════════════════════════════════════════════
# ─── PICO SENDER ──────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class PicoSender:
    def __init__(self, ip, port):
        self.ip   = ip
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.command_queue = deque(maxlen=3)
        self.running = True
        self.last_status = "Nicht verbunden"
        self.seq  = 1
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(SOCKET_CONNECT_TIMEOUT)
            s.connect((self.ip, self.port))
            s.settimeout(SOCKET_RECV_TIMEOUT)
            with self.lock:
                self.sock = s
                self.last_status = "Verbunden"
            print(f"[SENDER] Verbunden mit {self.ip}:{self.port}")
            return True
        except Exception as e:
            with self.lock:
                self.sock = None
                self.last_status = f"Fehler: {e}"
            return False

    def _send(self, cmd):
        with self.lock:
            if self.sock is None:
                return False
            try:
                self.sock.sendall(f"{cmd}\n".encode())
                try:
                    reply = self.sock.recv(128).decode().strip()
                except socket.timeout:
                    reply = "(timeout)"
                print(f"[SEND] {cmd} | Pico: {reply}")
                return True
            except Exception as e:
                print(f"[SENDER] Send error: {e}")
                try: self.sock.close()
                except Exception: pass
                self.sock = None
                self.last_status = "Verbindung verloren"
                return False

    def _run(self):
        last_connect_try = 0
        while self.running:
            with self.lock:
                connected = self.sock is not None
            if not connected:
                if time.time() - last_connect_try > 2.0:
                    self._connect()
                    last_connect_try = time.time()
                else:
                    time.sleep(0.1)
                continue
            try:
                if len(self.command_queue) > 0:
                    cmd = self.command_queue.popleft()
                    self._send(cmd)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"[SENDER] Queue error: {e}")
                time.sleep(0.1)

    def send(self, cmd):
        self.command_queue.append(cmd)
        with self.lock:
            g["drone_pico_status"] = self.last_status

    def clear_queue(self):
        self.command_queue.clear()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        with self.lock:
            if self.sock:
                try: self.sock.close()
                except Exception: pass

    def get_status(self):
        with self.lock:
            return self.last_status

    def queue_size(self):
        return len(self.command_queue)


# ═══════════════════════════════════════════════════════════════════════════════
# ─── NAVIGATIONS-THREAD ───────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def drone_nav_loop():
    """
    Läuft als Daemon-Thread.
    Liest g[] (ArUco-Position, Ziel, Orientierung) und sendet
    Steuerkommandos an die Drohne via _pico_sender.
    """
    global _pico_sender

    last_command_time        = 0.0
    trim_counters            = {"VOR": 0, "ZURUECK": 0, "LINKS": 0, "RECHTS": 0}
    corridor_defined         = False
    corridor_start           = None
    corridor_end             = None
    target_zone_enabled      = True
    slowdown_state           = False
    slowdown_trims_remaining = 0
    orientation_update_in_progress = False

    def try_send(cmd):
        sender = _pico_sender
        if sender is None:
            return False
        if cmd in trim_counters:
            if trim_counters[cmd] >= MAX_TRIM_COUNT:
                print(f"[BLOCK] Max trims für {cmd} erreicht ({trim_counters[cmd]})")
                return False
            sender.send(cmd)
            trim_counters[cmd] += 1
            g["drone_trim_counters"] = dict(trim_counters)
            return True
        else:
            sender.send(cmd)
            if cmd == "STOP":
                for k in trim_counters:
                    trim_counters[k] = 0
                g["drone_trim_counters"] = dict(trim_counters)
            return True

    def emergency_stop():
        sender = _pico_sender
        if sender:
            sender.clear_queue()
            sender.send("STOP")
        for k in trim_counters:
            trim_counters[k] = 0
        g["drone_trim_counters"] = dict(trim_counters)

    print("[DRONE-NAV] Navigations-Thread gestartet.")

    while g.get("running", False):
        time.sleep(0.02)  # ~50 Hz

        autonomous              = g.get("drone_autonomous", False)
        drone_orientation_angle = g.get("drone_orientation_angle")
        found_a                 = g.get("drone_aruco_pos")
        found_b                 = g.get("drone_target_pos")
        now                     = time.time()

        # ── Korridor zurücksetzen wenn kein Ziel mehr (Missions-Ende) ─────────
        if found_b is None:
            corridor_defined = False
            corridor_start   = None
            corridor_end     = None

        # ── Korridor beim ersten gemeinsamen Sichten definieren (wie AUTONOME) ─
        if not corridor_defined and found_a is not None and found_b is not None:
            corridor_start   = found_a
            corridor_end     = found_b
            corridor_defined = True
            print(f"[CORRIDOR] Definiert: {corridor_start} → {corridor_end}")

        # ── Stoppen wenn kein Autonom-Modus ───────────────────────────────────
        if not autonomous:
            target_zone_enabled      = True
            slowdown_state           = False
            slowdown_trims_remaining = 0
            continue

        # ── Navigations-Voraussetzungen prüfen ────────────────────────────────
        if not (found_a and found_b and drone_orientation_angle is not None):
            continue

        # ── Zeitsteuerung ─────────────────────────────────────────────────────
        if now - last_command_time < COMMAND_INTERVAL:
            continue

        dx_img = found_b[0] - found_a[0]
        dy_img = found_b[1] - found_a[1]
        dist   = np.hypot(dx_img, dy_img)

        command_text = ""

        # ── FALL A: Ziel erreicht ──────────────────────────────────────────────
        if dist < TARGET_REACHED_DIST:
            command_text = "ZIEL ERREICHT -> STOP"
            emergency_stop()
            last_command_time = now
            g["drone_reached"] = True
            with _lock:
                g["drone_autonomous"] = False
            g["drone_nav_cmd"] = command_text
            print(f"[DRONE-NAV] {command_text}")
            continue

        # ── FALL B: Slowdown-Zone ─────────────────────────────────────────────
        if dist < SLOWDOWN_RADIUS:
            if not slowdown_state:
                slowdown_state           = True
                slowdown_trims_remaining = SLOWDOWN_TRIMS
                print(f"[SLOWDOWN] Eingetreten – sende {slowdown_trims_remaining} Trims")
            if slowdown_state and slowdown_trims_remaining <= 0:
                command_text = "SLOWDOWN -> STOP"
                emergency_stop()
                last_command_time = now
                slowdown_state = False
                g["drone_nav_cmd"] = command_text
                continue
        else:
            if slowdown_state:
                slowdown_state           = False
                slowdown_trims_remaining = 0

        # ── FALL C: Korridor / Orientierungs-Zone ─────────────────────────────
        # Korridorbreite wird dynamisch skaliert:
        # Bei diagonalem Ziel (z.B. 5-Uhr) fliegt die Drohne wegen der
        # Priorisierung erst seitlich, DANN rückwärts – das ergibt eine
        # L-förmige Bahn statt Diagonale. Die Breite wird daher auf die
        # maximale seitliche Abweichung dieser L-Bahn vergrößert: das ist
        # einfach die laterale Komponente des Gesamtweges (|dy_img|), damit
        # die Drohne den Korridor während der sequenziellen Bewegung nicht
        # verlässt und einen Fehlstopp auslöst.
        if corridor_defined:
            c_dx = corridor_end[0] - corridor_start[0]
            c_dy = corridor_end[1] - corridor_start[1]
            # Lateralanteil des Korridors in Drohnen-Koordinaten
            _, c_lat = image_to_drone_coords(c_dx, c_dy, drone_orientation_angle) \
                if drone_orientation_angle is not None else (0, 0)
            dynamic_half_width = max(CORRIDOR_HALF_WIDTH, int(abs(c_lat) * 0.55))
            in_corridor   = point_in_corridor(found_a, corridor_start, corridor_end, dynamic_half_width)
            in_outer_zone = not in_corridor
        else:
            in_outer_zone = (dist < ORIENTATION_UPDATE_RADIUS)

        if in_outer_zone and target_zone_enabled:
            command_text = "ZONE ERREICHT -> STOP & RESET ORIENT"
            emergency_stop()
            last_command_time = now
            g["drone_orientation_angle"] = None
            target_zone_enabled = False
            orientation_update_in_progress = False
            g["drone_nav_cmd"] = command_text
            continue

        # ── Transformiere in Drohnen-Koordinaten ──────────────────────────────
        x_dr, y_dr = image_to_drone_coords(dx_img, dy_img, drone_orientation_angle)

        # ── Adaptive Deadzone (Marker-Größe) – wie AUTONOME ───────────────────
        deadzone_pixels = DEADZONE_PIXELS
        try:
            mps = g.get("drone_marker_pixel_size")
            if mps is not None and mps > 0 and mps < PIXELS_MIN_MARKER:
                deadzone_pixels += int((PIXELS_MIN_MARKER - mps) / 4)
        except Exception:
            pass
        y_check = -y_dr if INVERT_LATERAL else y_dr

        lat_cmd  = None
        long_cmd = None

        if y_check > deadzone_pixels:
            lat_cmd = "RECHTS"
        elif y_check < -deadzone_pixels:
            lat_cmd = "LINKS"

        if x_dr > deadzone_pixels:
            long_cmd = "VOR"
        elif x_dr < -deadzone_pixels:
            long_cmd = "ZURUECK"

        # ── Priorisierung ─────────────────────────────────────────────────────
        if lat_cmd and long_cmd:
            if PRIORITIZE_BY_ANGLE:
                try:
                    ang_dr = np.arctan2(y_dr, x_dr)
                    if abs(angle_wrap(ang_dr)) > np.deg2rad(ANGLE_PRIORITY_THRESHOLD_DEG):
                        long_cmd = None
                except Exception:
                    pass
            elif PRIORITIZE_LATERAL:
                try:
                    if abs(y_check) > abs(x_dr) * PRIORITIZE_LATERAL_THRESHOLD:
                        long_cmd = None
                except Exception:
                    pass

        sent = False

        if lat_cmd:
            print(f"[NAV] lat={lat_cmd}  x_dr={x_dr:.1f}  y_dr={y_dr:.1f}")
            if slowdown_state:
                if slowdown_trims_remaining > 0:
                    if try_send(lat_cmd):
                        command_text = lat_cmd
                        slowdown_trims_remaining -= 1
                        sent = True
                else:
                    emergency_stop()
                    command_text = "SLOWDOWN STOP"
                    sent = True
            else:
                if try_send(lat_cmd):
                    command_text = lat_cmd
                    sent = True

        if long_cmd:
            print(f"[NAV] long={long_cmd}  x_dr={x_dr:.1f}  y_dr={y_dr:.1f}")
            if slowdown_state:
                if slowdown_trims_remaining > 0:
                    if try_send(long_cmd):
                        command_text = (command_text + " " + long_cmd).strip()
                        slowdown_trims_remaining -= 1
                        sent = True
                else:
                    emergency_stop()
                    command_text = "SLOWDOWN STOP"
                    sent = True
            else:
                if try_send(long_cmd):
                    command_text = (command_text + " " + long_cmd).strip()
                    sent = True

        if not lat_cmd and not long_cmd:
            emergency_stop()
            command_text = "STOP"
            sent = True

        if sent:
            last_command_time = now
            g["drone_nav_cmd"] = command_text

    print("[DRONE-NAV] Navigations-Thread beendet.")
