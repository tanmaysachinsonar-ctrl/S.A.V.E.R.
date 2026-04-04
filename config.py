"""
S.A.V.E.R – config.py
Konstanten, globaler Zustand (g), ArUco-Helfer, Konfig-Persistenz, Hilfsfunktionen.
Kein UI-Code. Wird von allen anderen Modulen importiert.
"""

import math
import os
import sys
import cv2
import threading
import time
import json
import socket
import numpy as np
from collections import deque
from datetime import datetime

# ── Alarm-Sound (Windows only) ────────────────────────────────────────────────
try:
    import winsound
    _HAS_WINSOUND = True
except ImportError:
    _HAS_WINSOUND = False

# ── Trainingsdaten-Collector (optional) ───────────────────────────────────────
try:
    # SAVER_SPLIT/ liegt in Programming/ → zwei Ebenen hoch = 2025_26/
    _parent_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    if _parent_dir not in sys.path:
        sys.path.insert(0, _parent_dir)
    import saver_training_collector as stc
    _HAS_STC = True
except Exception:
    _HAS_STC = False
    class stc:  # Dummy
        @staticmethod
        def init(*a, **kw): pass
        @staticmethod
        def record(*a, **kw): pass
        @staticmethod
        def label(*a, **kw): pass
        @staticmethod
        def stop(*a, **kw): pass

# Basis-Verzeichnis: Programming/ (eine Ebene über SAVER_SPLIT/)
_BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ═══════════════════════════════════════════════════════════════════════════════
# ─── DROHNEN-KONFIG ───────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
PICO_IP   = "192.168.43.21"
PICO_PORT = 12345

ARUCO_DICT_NAME      = "DICT_4X4_50"
MARKER_ID            = 23
DEADZONE_PIXELS      = 20
TARGET_REACHED_DIST  = 100
COMMAND_INTERVAL     = 0.5
PIXELS_MIN_MARKER    = 32

SLOWDOWN_RADIUS = int(TARGET_REACHED_DIST * 5)
SLOWDOWN_TRIMS  = 6

INVERT_LATERAL               = False
SOCKET_CONNECT_TIMEOUT       = 6.0
SOCKET_RECV_TIMEOUT          = 1.0
ORIENTATION_UPDATE_RADIUS    = int(TARGET_REACHED_DIST * 2.0)
ORIENT_EMA_ALPHA             = 0.25
MAX_TRIM_COUNT               = 15
CORRIDOR_HALF_WIDTH          = 70
PRIORITIZE_LATERAL           = True
PRIORITIZE_LATERAL_THRESHOLD = 1.2
PRIORITIZE_BY_ANGLE          = True
ANGLE_PRIORITY_THRESHOLD_DEG = 45

DICT_MAP = {
    "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":  cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100":  cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250":  cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}

# ═══════════════════════════════════════════════════════════════════════════════
# ─── S.A.V.E.R.-KONFIG ────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
_CFG_PATH            = os.path.join(_BASE_DIR, "saver_config.json")
_LOG_PATH            = os.path.join(_BASE_DIR, "saver_logbook.json")
_TG_SUBSCRIBERS_PATH = os.path.join(_BASE_DIR, "saver_tg_subscribers.json")
_log_lock            = threading.Lock()

TELEGRAM_TOKEN  = "8798686855:AAEcRjDz0kK8pB9jsIOMTfhoKBlXsHPndSg"
TELEGRAM_API    = f"https://api.telegram.org/bot{TELEGRAM_TOKEN}"
_tg_pending     = set()
TG_PASSWORD     = "tanmayistderbeste"
STREAM_PORT     = 8080

SENSITIVITY = {
    "Niedrig": dict(distT=15, frames=200, radius=5),
    "Mittel":  dict(distT=30, frames=150, radius=3),
    "Hoch":    dict(distT=50, frames=90,  radius=2),
}

AWARENESS_INTERVAL_SEC  = 300
AWARENESS_TIMEOUT_SEC   = 60
ALARM_AUTO_DISPATCH_SEC = 60

# ─── DESIGN ───────────────────────────────────────────────────────────────────
BG       = "#151620"
PANEL    = "#151620"
CARD     = "#141428"
GREEN    = "#00cc30"
RED      = "#ff0015"
ORANGE   = "#ff8800"
BLUE     = "#0088ff"
TEXT     = "#ccccee"
MUTED    = "#445566"
BORDER   = "#1a1a33"
LOGO_CLR = "#5a5a7a"
_LOGO_PATH = os.path.join(_BASE_DIR, "S.A.V.E.R. LOGO.png")

# ═══════════════════════════════════════════════════════════════════════════════
# ─── GLOBALER ZUSTAND ─────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
_lock = threading.Lock()
g = dict(
    state="SETUP",
    lat1=0.0, lon1=0.0, lat2=0.0, lon2=0.0,
    distT=30, frames=150, radius=3, cam=1,
    mold={}, counters={},
    marked=set(), locked=set(), notified=set(),
    tg_sent=False, tg_uid=0, rescue=False,
    alarm_pid=None, alarm_lat=0.0, alarm_lon=0.0,
    persons=0, alarms=0, t0=time.time(),
    frame=None, running=False,
    alert_cooldown_until=0.0,
    use_gps=False,
    alarm_pending=False,
    exclusion_zones=[],
    awareness_pending=False,
    awareness_deadline=0.0,
    awareness_alarm=False,
    alarm_triggered_at=0.0,
    auto_dispatched=False,
    person_sensitivity={},
    bboxes={},
    collect_training=False,
    # Drohnen-Status
    drone_autonomous=False,
    drone_orientation_angle=None,
    drone_aruco_pos=None,
    drone_target_pos=None,
    drone_pico_status="Nicht verbunden",
    drone_trim_counters={"VOR": 0, "ZURUECK": 0, "LINKS": 0, "RECHTS": 0},
    drone_reached=False,
    drone_nav_cmd="",
    drone_marker_pixel_size=None,
)

# ═══════════════════════════════════════════════════════════════════════════════
# ─── ARUCO HELPER ─────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def get_aruco_detector(dict_name=ARUCO_DICT_NAME):
    if dict_name not in DICT_MAP:
        raise ValueError(f"Unknown dictionary '{dict_name}'")
    dict_id = DICT_MAP[dict_name]
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    except Exception:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    try:
        params = cv2.aruco.DetectorParameters_create()
    except Exception:
        params = cv2.aruco.DetectorParameters()
    try:
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.adaptiveThreshConstant   = 7
        params.minMarkerPerimeterRate   = 0.02
        params.maxMarkerPerimeterRate   = 4.0
        params.polygonalApproxAccuracyRate    = 0.03
        params.cornerRefinementMethod         = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize        = 5
        params.cornerRefinementMaxIterations  = 30
        params.cornerRefinementMinAccuracy    = 0.1
        params.perspectiveRemovePixelPerCell  = 8
        params.perspectiveRemoveIgnoredMarginPerCell = 0.13
    except Exception:
        pass
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        has_detector = True
    except Exception:
        detector = None
        has_detector = False
    return detector, aruco_dict, params, has_detector


def detect_aruco(gray, detector, aruco_dict, parameters, has_detector):
    try:
        if has_detector and detector is not None:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    except Exception:
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        except Exception:
            return None, None
    return corners, ids


def marker_center_and_angle(corners):
    c = corners.reshape((4, 2))
    cx = float(np.mean(c[:, 0]))
    cy = float(np.mean(c[:, 1]))
    top_mid_x = (c[0, 0] + c[1, 0]) / 2.0
    top_mid_y = (c[0, 1] + c[1, 1]) / 2.0
    angle = np.arctan2(top_mid_y - cy, top_mid_x - cx)
    return (cx, cy), angle, (int(top_mid_x), int(top_mid_y))


def angle_wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def angle_diff(a, b):
    return angle_wrap(a - b)

def median_point(buf):
    if not buf:
        return None
    xs = [p[0] for p in buf]
    ys = [p[1] for p in buf]
    return (int(np.median(xs)), int(np.median(ys)))

def image_to_drone_coords(dx, dy, fixed_angle):
    ca = np.cos(-fixed_angle)
    sa = np.sin(-fixed_angle)
    x_d = ca * dx - sa * dy
    y_d = sa * dx + ca * dy
    return x_d, y_d

def point_to_segment_distance(pt, a, b):
    px, py = pt
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    l2 = vx*vx + vy*vy
    if l2 == 0:
        return np.hypot(px - ax, py - ay), 0.0
    t = ((px - ax)*vx + (py - ay)*vy) / l2
    t_c = max(0.0, min(1.0, t))
    dist = np.hypot(px - (ax + t_c*vx), py - (ay + t_c*vy))
    return dist, t_c

def point_in_corridor(pt, a, b, half_width):
    dist, t = point_to_segment_distance(pt, a, b)
    return dist <= half_width and 0.0 <= t <= 1.0


# ═══════════════════════════════════════════════════════════════════════════════
# ─── KONFIG-PERSISTENZ ────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def _load_config():
    try:
        with open(_CFG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None

def _save_config(cfg):
    try:
        with open(_CFG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except Exception:
        pass

def _load_log():
    try:
        with open(_LOG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []

def _save_log(entries):
    try:
        with open(_LOG_PATH, "w", encoding="utf-8") as f:
            json.dump(entries, f, indent=2, ensure_ascii=False)
    except Exception:
        pass

def _log_event(event_type, details="", person_id=None, lat=None, lon=None):
    entry = {
        "ts": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "type": event_type,
        "details": details,
    }
    if person_id is not None:
        entry["person_id"] = person_id
    if lat is not None and lon is not None and g.get("use_gps"):
        entry["gps"] = {"lat": round(lat, 6), "lon": round(lon, 6)}
    entry["persons_count"] = g.get("persons", 0)
    with _log_lock:
        entries = _load_log()
        entries.append(entry)
        _save_log(entries)

def _load_tg_subscribers():
    try:
        with open(_TG_SUBSCRIBERS_PATH, "r", encoding="utf-8") as f:
            return set(json.load(f))
    except Exception:
        return {"8671390625", "8752798680"}

def _save_tg_subscribers(subs):
    try:
        with open(_TG_SUBSCRIBERS_PATH, "w", encoding="utf-8") as f:
            json.dump(list(subs), f, indent=2, ensure_ascii=False)
    except Exception:
        pass

_tg_subscribers = _load_tg_subscribers()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── HILFSFUNKTIONEN ──────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def px2gps(x, y, w, h):
    return (
        g["lat1"] + y / h * (g["lat2"] - g["lat1"]),
        g["lon1"] + x / w * (g["lon2"] - g["lon1"]),
    )

def alarm_sound():
    alarm_path = r"G:\Meine Ablage\Jugend Forscht\S.A.V.E.R\2025_26\Programming\SAVER_SPLIT\alarm.mp3"
    
    # Versuche pygame.mixer (beste Option für zuverlässige Wiedergabe)
    try:
        import pygame
        pygame.mixer.init()
        pygame.mixer.music.load(alarm_path)
        print(f"[ALARM] pygame.mixer aktiviert: {alarm_path}")
        while g["state"] == "ALARM":
            if not pygame.mixer.music.get_busy():
                pygame.mixer.music.play()
            time.sleep(0.1)
        return
    except Exception as e1:
        print(f"[ALARM] pygame.mixer fehlgeschlagen: {e1}")
    
    # Fallback: Windows API über ctypes (direkt über Windows)
    try:
        import ctypes
        winmm = ctypes.windll.winmm
        print(f"[ALARM] Windows API (mciSendString) aktiviert: {alarm_path}")
        while g["state"] == "ALARM":
            try:
                winmm.mciSendStringW(f'open "{alarm_path}" type mpegvideo alias alarm', None, 0, 0)
                winmm.mciSendStringW('play alarm', None, 0, 0)
                time.sleep(5)  # Warte ~5 Sekunden für Wiedergabe
                winmm.mciSendStringW('close alarm', None, 0, 0)
            except Exception:
                pass
        return
    except Exception as e2:
        print(f"[ALARM] Windows API fehlgeschlagen: {e2}")
    
    # Fallback: os.startfile() - startet MP3 im Standard-Player
    try:
        print(f"[ALARM] os.startfile() aktiviert: {alarm_path}")
        while g["state"] == "ALARM":
            try:
                os.startfile(alarm_path)
                time.sleep(5)
            except Exception:
                pass
        return
    except Exception as e3:
        print(f"[ALARM] os.startfile() fehlgeschlagen: {e3}")
    
    # Letzter Fallback: altes Beep-System
    if _HAS_WINSOUND:
        print("[ALARM] Fallback auf Beep-System")
        try:
            while g["state"] == "ALARM":
                winsound.Beep(2200, 600)
                time.sleep(0.03)
                winsound.Beep(700, 600)
                time.sleep(0.03)
                winsound.Beep(2500, 400)
                time.sleep(0.03)
                winsound.Beep(900, 400)
                time.sleep(0.03)
        except Exception:
            pass

def _in_exclusion_zone(nx, ny):
    for (zx1, zy1, zx2, zy2) in g.get("exclusion_zones", []):
        if zx1 <= nx <= zx2 and zy1 <= ny <= zy2:
            return True
    return False

def _get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"
