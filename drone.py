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
    ARUCO_NAV_LOST_TOLERANCE, ARUCO_BRAKE_TRIMS,
    CORRIDOR_CENTERING_GAIN,
    angle_wrap, angle_diff, image_to_drone_coords, point_in_corridor,
    corridor_correction_vector,
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

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │  ALT: drone_nav_loop (auskommentiert – ersetzt durch Ideal-Logik)          │
# │                                                                             │
# │  Unterschiede zum Ideal (AUTONOME DRONE NO ARUCO 1.py):                    │
# │   - Kein Frame-Gating (lief bei ~50 Hz unabhängig von Kamera-Frames)       │
# │   - Dynamische Korridorbreite statt fester CORRIDOR_HALF_WIDTH             │
# │   - Komplexer ArUco-Verlust-Timer mit periodischem emergency_stop          │
# │   - Kein Snapshot → Race Conditions zwischen Detection- & Nav-Thread       │
# └─────────────────────────────────────────────────────────────────────────────┘
# def drone_nav_loop_ALT():
#     global _pico_sender
#     last_command_time        = 0.0
#     trim_counters            = {"VOR": 0, "ZURUECK": 0, "LINKS": 0, "RECHTS": 0}
#     corridor_defined         = False
#     corridor_start           = None
#     corridor_end             = None
#     target_zone_enabled      = True
#     slowdown_state           = False
#     slowdown_trims_remaining = 0
#     orientation_update_in_progress = False
#     aruco_lost_since         = None
#     def try_send(cmd):
#         sender = _pico_sender
#         if sender is None: return False
#         if cmd in trim_counters:
#             if trim_counters[cmd] >= MAX_TRIM_COUNT: return False
#             sender.send(cmd); trim_counters[cmd] += 1
#             g["drone_trim_counters"] = dict(trim_counters); return True
#         else:
#             sender.send(cmd)
#             if cmd == "STOP":
#                 for k in trim_counters: trim_counters[k] = 0
#                 g["drone_trim_counters"] = dict(trim_counters)
#             return True
#     def emergency_stop():
#         sender = _pico_sender
#         if sender: sender.clear_queue(); sender.send("STOP")
#         for k in trim_counters: trim_counters[k] = 0
#         g["drone_trim_counters"] = dict(trim_counters)
#     while g.get("running", False):
#         time.sleep(0.02)
#         autonomous              = g.get("drone_autonomous", False)
#         drone_orientation_angle = g.get("drone_orientation_angle")
#         found_a                 = g.get("drone_aruco_pos")
#         found_b                 = g.get("drone_target_pos")
#         now                     = time.time()
#         if found_b is None:
#             corridor_defined = False; corridor_start = None; corridor_end = None
#         if not corridor_defined and found_a is not None and found_b is not None:
#             corridor_start = found_a; corridor_end = found_b; corridor_defined = True
#         if not autonomous:
#             target_zone_enabled = True; slowdown_state = False; slowdown_trims_remaining = 0; continue
#         if not (found_a and found_b and drone_orientation_angle is not None):
#             if found_a is None:
#                 if aruco_lost_since is None:
#                     aruco_lost_since = now; emergency_stop()
#                     g["drone_nav_cmd"] = "ARUCO VERLOREN – STOP"
#                 elif now - aruco_lost_since > COMMAND_INTERVAL * 3:
#                     emergency_stop(); aruco_lost_since = now
#             continue
#         if aruco_lost_since is not None: aruco_lost_since = None
#         if now - last_command_time < COMMAND_INTERVAL: continue
#         dx_img = found_b[0] - found_a[0]; dy_img = found_b[1] - found_a[1]
#         dist = np.hypot(dx_img, dy_img)
#         if dist < TARGET_REACHED_DIST:
#             emergency_stop(); g["drone_reached"]=True
#             with _lock: g["drone_autonomous"]=False
#             g["drone_nav_cmd"]="ZIEL ERREICHT -> STOP"; last_command_time=now; continue
#         if dist < SLOWDOWN_RADIUS:
#             if not slowdown_state: slowdown_state=True; slowdown_trims_remaining=SLOWDOWN_TRIMS
#             if slowdown_state and slowdown_trims_remaining<=0:
#                 emergency_stop(); last_command_time=now; slowdown_state=False
#                 g["drone_nav_cmd"]="SLOWDOWN -> STOP"; continue
#         else:
#             if slowdown_state: slowdown_state=False; slowdown_trims_remaining=0
#         # ALT: dynamische Korridorbreite
#         if corridor_defined:
#             c_dx=corridor_end[0]-corridor_start[0]; c_dy=corridor_end[1]-corridor_start[1]
#             _, c_lat = image_to_drone_coords(c_dx,c_dy,drone_orientation_angle) if drone_orientation_angle else (0,0)
#             dynamic_half_width = max(CORRIDOR_HALF_WIDTH, int(abs(c_lat)*0.55))
#             in_corridor = point_in_corridor(found_a, corridor_start, corridor_end, dynamic_half_width)
#             in_outer_zone = not in_corridor
#         else:
#             in_outer_zone = (dist < ORIENTATION_UPDATE_RADIUS)
#         if in_outer_zone and target_zone_enabled:
#             emergency_stop(); g["drone_orientation_angle"]=None
#             target_zone_enabled=False; orientation_update_in_progress=False
#             g["drone_nav_cmd"]="ZONE ERREICHT -> STOP & RESET ORIENT"; last_command_time=now; continue
#         x_dr,y_dr = image_to_drone_coords(dx_img,dy_img,drone_orientation_angle)
#         deadzone_pixels = DEADZONE_PIXELS
#         try:
#             mps = g.get("drone_marker_pixel_size")
#             if mps and mps > 0 and mps < PIXELS_MIN_MARKER:
#                 deadzone_pixels += int((PIXELS_MIN_MARKER - mps) / 4)
#         except: pass
#         y_check = -y_dr if INVERT_LATERAL else y_dr
#         lat_cmd=None; long_cmd=None
#         if y_check>deadzone_pixels: lat_cmd="RECHTS"
#         elif y_check<-deadzone_pixels: lat_cmd="LINKS"
#         if x_dr>deadzone_pixels: long_cmd="VOR"
#         elif x_dr<-deadzone_pixels: long_cmd="ZURUECK"
#         if lat_cmd and long_cmd:
#             if PRIORITIZE_BY_ANGLE:
#                 try:
#                     if abs(angle_wrap(np.arctan2(y_dr,x_dr)))>np.deg2rad(ANGLE_PRIORITY_THRESHOLD_DEG): long_cmd=None
#                 except: pass
#             elif PRIORITIZE_LATERAL:
#                 try:
#                     if abs(y_check)>abs(x_dr)*PRIORITIZE_LATERAL_THRESHOLD: long_cmd=None
#                 except: pass
#         sent=False
#         if lat_cmd:
#             if slowdown_state:
#                 if slowdown_trims_remaining>0:
#                     if try_send(lat_cmd): command_text=lat_cmd; slowdown_trims_remaining-=1; sent=True
#                 else: emergency_stop(); command_text="SLOWDOWN STOP"; sent=True
#             else:
#                 if try_send(lat_cmd): command_text=lat_cmd; sent=True
#         if long_cmd:
#             if slowdown_state:
#                 if slowdown_trims_remaining>0:
#                     if try_send(long_cmd): command_text=(command_text+" "+long_cmd).strip(); slowdown_trims_remaining-=1; sent=True
#                 else: emergency_stop(); command_text="SLOWDOWN STOP"; sent=True
#             else:
#                 if try_send(long_cmd): command_text=(command_text+" "+long_cmd).strip(); sent=True
#         if not lat_cmd and not long_cmd: emergency_stop(); command_text="STOP"; sent=True
#         if sent: last_command_time=now; g["drone_nav_cmd"]=command_text


def drone_nav_loop():
    """
    NEU: Navigations-Thread – Logik 1:1 vom Ideal (AUTONOME DRONE NO ARUCO 1.py).

    Änderungen gegenüber ALT:
      - Frame-Gating: navigiert nur wenn backend neuen Frame liefert (drone_frame_seq)
      - Feste Korridorbreite (CORRIDOR_HALF_WIDTH) statt dynamischer Skalierung
      - Kein komplexer ArUco-Verlust-Timer – stoppt sofort & natürlich
      - Snapshot aller g[]-Werte am Schleifenanfang → keine Race Conditions
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
    last_frame_seq              = -1     # NEU: Frame-Gating wie FRAME_DECIMATION im Ideal
    aruco_lost_count            = 0      # aufeinanderfolgende Frames ohne Marker-Erkennung
    brake_trims_sent            = 0      # bereits gesendete Gegentrims während ArUco-Verlust
    last_lat_cmd                = None   # letzter Lateral-Befehl (für Gegentrim-Berechnung)
    last_long_cmd               = None   # letzter Longitudinal-Befehl
    full_stopped                = False  # True nach dem Vollstopp bei ArUco-Verlust
    recovery_trims_sent_count   = 0      # Zähler für Recovery-Trims nach Vollstopp
    RECOVERY_TRIMS              = 7      # Anzahl Trims Richtung Ziel nach Vollstopp
    consecutive_stop_count      = 0      # Aufeinanderfolgende STOP-Befehle (max 2 senden)
    MAX_CONSECUTIVE_STOPS       = 2      # Drohne nicht mit STOPs fluten
    _OPPOSITE = {"VOR": "ZURUECK", "ZURUECK": "VOR", "RECHTS": "LINKS", "LINKS": "RECHTS"}

    def try_send(cmd):
        nonlocal consecutive_stop_count
        sender = _pico_sender
        if sender is None:
            return False
        if cmd in trim_counters:
            if trim_counters[cmd] >= MAX_TRIM_COUNT:
                print(f"[BLOCK] Max trims für {cmd} erreicht ({trim_counters[cmd]})")
                return False
            sender.send(cmd)
            trim_counters[cmd] += 1
            consecutive_stop_count = 0  # Echter Trim → STOP-Throttle zurücksetzen
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
        nonlocal consecutive_stop_count
        sender = _pico_sender
        if sender:
            sender.clear_queue()
            if consecutive_stop_count < MAX_CONSECUTIVE_STOPS:
                sender.send("STOP")
                consecutive_stop_count += 1
                print(f"[STOP] #{consecutive_stop_count}/{MAX_CONSECUTIVE_STOPS}")
            else:
                print("[STOP] Throttle aktiv – kein weiteres STOP gesendet")
        for k in trim_counters:
            trim_counters[k] = 0
        g["drone_trim_counters"] = dict(trim_counters)

    print("[DRONE-NAV] Navigations-Thread gestartet (Ideal-Logik).")

    while g.get("running", False):
        time.sleep(0.02)  # ~50 Hz polling

        # ── NEU: Frame-Gating – nur navigieren wenn backend einen neuen Frame hat ─
        current_frame_seq = g.get("drone_frame_seq", 0)
        if current_frame_seq == last_frame_seq:
            continue
        last_frame_seq = current_frame_seq

        # ── Snapshot aller Werte (atomar lesen, keine Race Conditions) ────────
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

        # ── Korridor beim ersten gemeinsamen Sichten definieren (wie Ideal) ───
        if not corridor_defined and found_a is not None and found_b is not None:
            corridor_start   = found_a
            corridor_end     = found_b
            corridor_defined = True
            print(f"[CORRIDOR] Definiert: {corridor_start} → {corridor_end}")

        # ── Stoppen wenn kein Autonom-Modus ───────────────────────────────────
        # Vollständiger State-Reset: Trim-Counter, Korridor, Brems-State, letzte
        # Befehle – damit jede neue Mission mit einer sauberen Basis startet.
        if not autonomous:
            target_zone_enabled      = True
            slowdown_state           = False
            slowdown_trims_remaining = 0
            aruco_lost_count         = 0
            brake_trims_sent         = 0
            last_lat_cmd             = None
            last_long_cmd            = None
            full_stopped             = False
            recovery_trims_sent_count = 0
            consecutive_stop_count   = 0
            corridor_defined         = False
            corridor_start           = None
            corridor_end             = None
            for k in trim_counters:
                trim_counters[k] = 0
            g["drone_trim_counters"] = dict(trim_counters)
            continue

        # ── Navigations-Voraussetzungen prüfen ────────────────────────────────
        # drone_aruco_visible: True nur wenn Marker im AKTUELLEN Frame erkannt.
        # Stufen:
        #  1–TOLERANCE Frames verloren → Navigation läuft mit Buffer-Werten weiter
        #  >TOLERANCE Frames           → Gegentrims zum sanften Abbremsen
        #  >TOLERANCE + BRAKE_TRIMS    → voller Stop als Sicherheitsnetz
        aruco_fresh = g.get("drone_aruco_visible", False)
        if aruco_fresh:
            aruco_lost_count           = 0
            brake_trims_sent           = 0
            full_stopped               = False
            recovery_trims_sent_count  = 0
        else:
            aruco_lost_count += 1

        if not (found_a and found_b and drone_orientation_angle is not None) or aruco_lost_count > ARUCO_NAV_LOST_TOLERANCE:
            if aruco_lost_count > ARUCO_NAV_LOST_TOLERANCE:
                if brake_trims_sent < ARUCO_BRAKE_TRIMS:
                    # Gegentrim: Drohne abbremsen statt hart zu stoppen
                    if now - last_command_time >= COMMAND_INTERVAL:
                        sender = _pico_sender
                        if sender:
                            sender.clear_queue()
                            brake_cmds = []
                            if last_lat_cmd and _OPPOSITE.get(last_lat_cmd):
                                sender.send(_OPPOSITE[last_lat_cmd])
                                brake_cmds.append(_OPPOSITE[last_lat_cmd])
                                consecutive_stop_count = 0
                            if last_long_cmd and _OPPOSITE.get(last_long_cmd):
                                sender.send(_OPPOSITE[last_long_cmd])
                                brake_cmds.append(_OPPOSITE[last_long_cmd])
                                consecutive_stop_count = 0
                            if not brake_cmds:
                                if consecutive_stop_count < MAX_CONSECUTIVE_STOPS:
                                    sender.send("STOP")
                                    consecutive_stop_count += 1
                                brake_cmds = ["STOP"]
                        brake_trims_sent += 1
                        last_command_time = now
                        g["drone_nav_cmd"] = f"BREMSEN {brake_trims_sent}/{ARUCO_BRAKE_TRIMS}: {' '.join(brake_cmds)}"
                        print(f"[BRAKE] ArUco weg ({aruco_lost_count}f), Gegentrim {brake_trims_sent}/{ARUCO_BRAKE_TRIMS}: {brake_cmds}")
                else:
                    if not full_stopped:
                        emergency_stop()
                        full_stopped = True
                        recovery_trims_sent_count = 0
                        last_command_time = now
                        g["drone_nav_cmd"] = "ARUCO VERLOREN – STOP"
                        print("[ARUCO] Vollstopp – starte Recovery-Trims")
                    elif recovery_trims_sent_count < RECOVERY_TRIMS:
                        if now - last_command_time >= COMMAND_INTERVAL:
                            sender = _pico_sender
                            if sender:
                                cmds = []
                                if last_long_cmd:
                                    sender.send(last_long_cmd)
                                    cmds.append(last_long_cmd)
                                    consecutive_stop_count = 0
                                if last_lat_cmd:
                                    sender.send(last_lat_cmd)
                                    cmds.append(last_lat_cmd)
                                    consecutive_stop_count = 0
                                if not cmds:
                                    if consecutive_stop_count < MAX_CONSECUTIVE_STOPS:
                                        sender.send("STOP")
                                        consecutive_stop_count += 1
                                    cmds = ["STOP"]
                            recovery_trims_sent_count += 1
                            last_command_time = now
                            g["drone_nav_cmd"] = f"RECOVERY {recovery_trims_sent_count}/{RECOVERY_TRIMS}: {' '.join(cmds)}"
                            print(f"[RECOVERY] Trim {recovery_trims_sent_count}/{RECOVERY_TRIMS}: {cmds}")
                    # else: halten – warte auf ArUco ohne weiteren Befehl
            continue

        # ── Zeitsteuerung ─────────────────────────────────────────────────────
        if now - last_command_time < COMMAND_INTERVAL:
            continue

        dx_img = found_b[0] - found_a[0]
        dy_img = found_b[1] - found_a[1]
        dist   = np.hypot(dx_img, dy_img)

        command_text = ""

        # ── FALL A: Ziel erreicht ─────────────────────────────────────────────
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

        # ── FALL B: Slowdown-Zone (nur aktiv wenn SLOWDOWN_TRIMS > 0) ──────────
        # WICHTIG: Bei SLOWDOWN_TRIMS=0 diesen Block überspringen – sonst
        # entsteht ein permanenter STOP-Loop im gesamten SLOWDOWN_RADIUS-Bereich!
        if SLOWDOWN_TRIMS > 0:
            if dist < SLOWDOWN_RADIUS:
                if not slowdown_state:
                    slowdown_state           = True
                    slowdown_trims_remaining = SLOWDOWN_TRIMS
                    print(f"[SLOWDOWN] Eingetreten – sende {slowdown_trims_remaining} Trims")
                if slowdown_state and slowdown_trims_remaining <= 0:
                    # Trims erschöpft → zurück zu normaler Navigation, KEIN STOP.
                    # Ein STOP hier würde den State zurücksetzen und die Zone im
                    # nächsten Frame sofort neu initialisieren → endloser Stop-Loop.
                    slowdown_state = False
                    print("[SLOWDOWN] Trims aufgebraucht – normale Navigation fortgesetzt")
            else:
                if slowdown_state:
                    slowdown_state           = False
                    slowdown_trims_remaining = 0

        # ── FALL C: Korridor / Orientierungs-Zone ─────────────────────────────
        # NEU: Feste Korridorbreite wie Ideal (keine dynamische Skalierung)
        if corridor_defined:
            in_corridor   = point_in_corridor(found_a, corridor_start, corridor_end, CORRIDOR_HALF_WIDTH)
            in_outer_zone = not in_corridor
        else:
            in_corridor   = True
            in_outer_zone = (dist < ORIENTATION_UPDATE_RADIUS)

        # Außenbereich-Check reaktivieren sobald Drohne zurück im Korridor
        if in_corridor and not target_zone_enabled:
            target_zone_enabled = True
            print(f"[CORRIDOR] Zurück im Korridor – Außenbereich-Check reaktiviert")

        if in_outer_zone and target_zone_enabled:
            command_text = "KORRIDOR VERLASSEN -> STOP"
            emergency_stop()
            last_command_time = now
            # Orientierung NICHT zurücksetzen – ursprüngliche Orientierung bleibt gültig,
            # damit die Drohne von hier aus mit derselben Referenz weiternavigiert.
            target_zone_enabled = False
            g["drone_nav_cmd"] = command_text
            print(f"[DRONE-NAV] {command_text} – Orientierung bleibt: {np.degrees(drone_orientation_angle):.1f}°")
            continue

        # ── Transformiere in Drohnen-Koordinaten ──────────────────────────────
        # Korridorzentrierung: Navigationsvektor Richtung Mittellinie biegen
        if corridor_defined and CORRIDOR_CENTERING_GAIN > 0:
            corr_x, corr_y = corridor_correction_vector(
                found_a, corridor_start, corridor_end, CORRIDOR_HALF_WIDTH)
            dx_img += corr_x * CORRIDOR_CENTERING_GAIN
            dy_img += corr_y * CORRIDOR_CENTERING_GAIN

        x_dr, y_dr = image_to_drone_coords(dx_img, dy_img, drone_orientation_angle)

        # ── Adaptive Deadzone (Marker-Größe) – wie Ideal ─────────────────────
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

        # ── Priorisierung: immer nur EIN Befehl pro Frame ─────────────────────
        # Verhindert diagonales Fliegen – Drohne korrigiert erst die dominante
        # Achse, bevor sie die andere bedient.
        if lat_cmd and long_cmd:
            if abs(y_check) > abs(x_dr):
                long_cmd = None   # Lateraler Fehler größer → nur seitlich korrigieren
            else:
                lat_cmd = None    # Longitudinaler Fehler größer → nur vor/zurück

        sent = False

        if lat_cmd:
            print(f"[NAV] lat={lat_cmd}  x_dr={x_dr:.1f}  y_dr={y_dr:.1f}")
            if slowdown_state and slowdown_trims_remaining > 0:
                if try_send(lat_cmd):
                    command_text = lat_cmd
                    slowdown_trims_remaining -= 1
                    sent = True
            else:
                if try_send(lat_cmd):
                    command_text = lat_cmd
                    sent = True

        if long_cmd:
            print(f"[NAV] long={long_cmd}  x_dr={x_dr:.1f}  y_dr={y_dr:.1f}")
            if slowdown_state and slowdown_trims_remaining > 0:
                if try_send(long_cmd):
                    command_text = (command_text + " " + long_cmd).strip()
                    slowdown_trims_remaining -= 1
                    sent = True
            else:
                if try_send(long_cmd):
                    command_text = (command_text + " " + long_cmd).strip()
                    sent = True

        if not sent:
            # Sicherheitsnetz: kein Befehl konnte gesendet werden (z.B. alle
            # try_send() durch MAX_TRIM_COUNT geblockt, oder keine Richtung nötig).
            # Ohne diesen STOP würde die Drohne ungebremst driften.
            emergency_stop()
            command_text = "STOP"
            sent = True

        if sent:
            last_command_time = now
            g["drone_nav_cmd"] = command_text
            # Letzte Richtungsbefehle merken – für Gegentrim bei ArUco-Verlust
            if lat_cmd:
                last_lat_cmd = lat_cmd
            if long_cmd:
                last_long_cmd = long_cmd

    print("[DRONE-NAV] Navigations-Thread beendet.")
