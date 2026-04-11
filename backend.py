"""
S.A.V.E.R – backend.py
Detection-Loop (YOLO + ArUco Fusion), Telegram-Bot, MJPEG-Stream-Server,
Alarm-Logik (send_alert, confirm_rescue, reset_alarm), Eskalations-Thread.
"""

import math
import os
import threading
import time
import json
import cv2
import traceback
import numpy as np
import requests
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

import drone as _drone
from config import (
    g, _lock,
    ARUCO_DICT_NAME, MARKER_ID, ORIENT_EMA_ALPHA,
    TARGET_REACHED_DIST, SLOWDOWN_RADIUS, PIXELS_MIN_MARKER,
    TELEGRAM_API, STREAM_PORT,
    PICO_IP, PICO_PORT, SENSITIVITY,
    AWARENESS_INTERVAL_SEC, AWARENESS_TIMEOUT_SEC, ALARM_AUTO_DISPATCH_SEC,
    _tg_subscribers, _tg_pending, TG_PASSWORD,
    _log_event, _load_log, _save_log, _log_lock, _save_tg_subscribers,
    _load_config, _save_config,
    _get_local_ip, _in_exclusion_zone, px2gps, alarm_sound,
    get_aruco_detector, detect_aruco, marker_center_and_angle,
    angle_wrap, angle_diff, median_point,
    _HAS_STC, stc,
)

# ── Web-UI HTML laden ─────────────────────────────────────────────────────────
_WEB_UI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_ui.html")
def _load_web_ui():
    try:
        with open(_WEB_UI_PATH, "r", encoding="utf-8") as f:
            return f.read()
    except Exception:
        return "<h1>Fehler: web_ui.html nicht gefunden</h1>"


# ═══════════════════════════════════════════════════════════════════════════════
# ─── SYSTEM START / STOP ──────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def start_system(cfg):
    """Start all backend threads with given config dict."""
    cam_map = {"Hauptkamera": 0, "Kamera 2": 1, "Kamera 3": 2}
    sens_name = cfg.get("sens", "Mittel")
    p = SENSITIVITY.get(sens_name, SENSITIVITY["Mittel"])
    with _lock:
        g.update(
            lat1=float(cfg.get("lat1", 0)), lon1=float(cfg.get("lon1", 0)),
            lat2=float(cfg.get("lat2", 0)), lon2=float(cfg.get("lon2", 0)),
            cam=cam_map.get(cfg.get("cam", "Hauptkamera"), 0),
            distT=p["distT"], frames=p["frames"], radius=p["radius"],
            running=True, t0=time.time(), state="NORMAL",
            alarms=0, persons=0,
            use_gps=cfg.get("use_gps", False),
            collect_training=cfg.get("collect_training", False),
            auto_takeoff=cfg.get("auto_takeoff", True),
            exclusion_zones=[tuple(z) for z in cfg.get("exclusion_zones", [])],
        )
    g["_sensitivity_name"] = sens_name
    if g["collect_training"] and _HAS_STC:
        stc.init(camera=cfg.get("cam"), sensitivity=sens_name)
    if _drone._pico_sender is None:
        pico_ip = cfg.get("pico_ip", PICO_IP)
        _drone._pico_sender = _drone.PicoSender(pico_ip, PICO_PORT)
        print(f"[PICO] PicoSender gestartet → {pico_ip}:{PICO_PORT}")
    threading.Thread(target=detection_loop,   daemon=True).start()
    threading.Thread(target=_drone.drone_nav_loop, daemon=True).start()
    threading.Thread(target=tg_poller,         daemon=True).start()
    threading.Thread(target=escalation_loop,   daemon=True).start()
    _save_config(cfg)
    _log_event("SYSTEM_START", f"System gestartet – {cfg.get('cam')}, {sens_name}")


def stop_system():
    """Stop the system gracefully."""
    if g.get("running"):
        _log_event("SYSTEM_STOP", "System beendet")
    g["running"] = False
    if _drone._pico_sender:
        _drone._pico_sender.clear_queue()
        _drone._pico_sender.send("STOP")
    if g.get("collect_training") and _HAS_STC:
        stc.stop()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── HTTP SERVER (Web-UI + API + MJPEG) ───────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class _MJPEGHandler(BaseHTTPRequestHandler):

    # ── Hilfs-Methoden ────────────────────────────────────────────────────────
    def _json_response(self, data, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(json.dumps(data, default=str).encode())

    def _read_json_body(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            return json.loads(body) if body else {}
        except Exception:
            return {}

    # ── GET ────────────────────────────────────────────────────────────────────
    def do_GET(self):
        path = self.path.split("?")[0]

        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                # Warte bis System läuft (max 30s)
                for _ in range(300):
                    if g.get("running"):
                        break
                    time.sleep(0.1)
                while g.get("running"):
                    fr = g.get("frame")
                    if fr is not None:
                        ok, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            data = buf.tobytes()
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                            self.wfile.write(data)
                            self.wfile.write(b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        elif path == "/api/status":
            sender = _drone._pico_sender
            pico_status = sender.get_status() if sender else "–"
            aruco_pos = g.get("drone_aruco_pos")
            orient = g.get("drone_orientation_angle")
            fr = g.get("frame")
            frame_size = [int(fr.shape[1]), int(fr.shape[0])] if fr is not None else [1, 1]
            tg_note = g.get("tg_action_note")
            tg_text = None
            if tg_note:
                note_text, note_time = tg_note
                if time.time() - note_time < 5.0:
                    tg_text = note_text
                else:
                    g["tg_action_note"] = None
            mold_out = {}
            for tid, pos in g.get("mold", {}).items():
                mold_out[str(tid)] = [pos[0], pos[1]]
            self._json_response({
                "state":       g.get("state", "SETUP"),
                "running":     g.get("running", False),
                "persons":     g.get("persons", 0),
                "alarms":      g.get("alarms", 0),
                "sensitivity": g.get("_sensitivity_name", "Mittel"),
                "clock":       time.strftime("%H:%M:%S"),
                "use_gps":     g.get("use_gps", False),
                "alarm_pid":   g.get("alarm_pid"),
                "alarm_lat":   g.get("alarm_lat", 0.0),
                "alarm_lon":   g.get("alarm_lon", 0.0),
                "awareness_pending": g.get("awareness_pending", False),
                "awareness_alarm":   g.get("awareness_alarm", False),
                "tg_action_note":    tg_text,
                "marked":  list(g.get("marked", set())),
                "locked":  list(g.get("locked", set())),
                "mold":    mold_out,
                "frame_size": frame_size,
                "drone": {
                    "pico":              pico_status,
                    "autonomous":        g.get("drone_autonomous", False),
                    "reached":           g.get("drone_reached", False),
                    "aruco_pos":         list(aruco_pos) if aruco_pos else None,
                    "orientation_angle": float(np.degrees(orient)) if orient is not None else None,
                    "nav_cmd":           g.get("drone_nav_cmd", ""),
                    "aruco_visible":     g.get("drone_aruco_visible", False),
                },
            })

        elif path == "/api/config":
            cfg = _load_config()
            if cfg is None:
                cfg = {}
            if "pico_ip" not in cfg:
                cfg["pico_ip"] = PICO_IP
            self._json_response(cfg)

        elif path == "/api/log":
            with _log_lock:
                entries = _load_log()
            self._json_response(entries)

        elif path.startswith("/api/snapshot"):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            cam_idx = int(qs.get("cam", [g.get("cam", 0)])[0])
            # Versuche laufenden Frame
            fr = g.get("frame")
            if fr is not None:
                ok, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(buf.tobytes())
                    return
            # Kein laufender Frame → direkt von Kamera
            try:
                cap = cv2.VideoCapture(cam_idx)
                ret, frame = cap.read()
                cap.release()
                if ret:
                    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ok:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.end_headers()
                        self.wfile.write(buf.tobytes())
                        return
            except Exception:
                pass
            self.send_response(404)
            self.end_headers()

        # Legacy Telegram action links (backward compat)
        elif path.startswith("/action"):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            do = qs.get("do", [""])[0]
            if   do == "confirm":
                g["tg_action_note"] = ("📱 Rettung wurde per Handy bestätigt", time.time())
                threading.Thread(target=confirm_rescue, daemon=True).start()
            elif do == "false_alarm":
                g["tg_action_note"] = ("📱 Fehlalarm wurde per Handy markiert", time.time())
                threading.Thread(target=reset_alarm, args=(None, "❌ Fehlalarm – System bereit."), daemon=True).start()
            elif do == "done":
                g["tg_action_note"] = ("📱 Rettung wurde per Handy abgeschlossen", time.time())
                threading.Thread(target=reset_alarm, args=(None, "✅ Rettung abgeschlossen – System bereit!"), daemon=True).start()
            self._json_response({"ok": True})

        else:
            # Serve web UI (always reload for live editing)
            html_bytes = _load_web_ui().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html_bytes)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(html_bytes)

    # ── POST ──────────────────────────────────────────────────────────────────
    def do_POST(self):
        path = self.path.split("?")[0]
        body = self._read_json_body()

        if path == "/api/start":
            if g.get("running"):
                self._json_response({"ok": False, "error": "System läuft bereits"})
                return
            try:
                start_system(body)
                self._json_response({"ok": True})
            except Exception as e:
                self._json_response({"ok": False, "error": str(e)})

        elif path == "/api/stop":
            stop_system()
            self._json_response({"ok": True})

        elif path == "/api/action":
            action = body.get("action", "")
            if action == "confirm":
                threading.Thread(target=confirm_rescue, daemon=True).start()
            elif action == "false_alarm":
                threading.Thread(target=reset_alarm, args=(None, "❌ Fehlalarm – System bereit."), daemon=True).start()
            elif action == "done":
                threading.Thread(target=reset_alarm, args=(None, "✅ Rettung abgeschlossen – System bereit!"), daemon=True).start()
            elif action == "awareness":
                with _lock:
                    g["awareness_pending"] = False
                    g["awareness_alarm"]   = False
            self._json_response({"ok": True})

        elif path == "/api/click":
            xn = float(body.get("x", 0.5))
            yn = float(body.get("y", 0.5))
            fr = g.get("frame")
            if fr is not None:
                fh, fw = fr.shape[:2]
                fx = xn * fw
                fy = yn * fh
                mold = g.get("mold", {})
                best_d = float("inf")
                best_pid = None
                for tid, pos in mold.items():
                    d = math.hypot(fx - pos[0], fy - pos[1])
                    if d < g.get("radius", 3) * 40 and d < best_d:
                        best_d = d
                        best_pid = tid
                if best_pid is not None and best_pid not in g.get("marked", set()):
                    cur = g.get("person_sensitivity", {}).get(best_pid, 1.0)
                    self._json_response({"pid": best_pid, "sensitivity": cur})
                    return
            self._json_response({"pid": None})

        elif path == "/api/sensitivity":
            pid  = body.get("pid")
            mult = body.get("mult", 1.0)
            if pid is not None:
                sens = g.get("person_sensitivity", {})
                if mult == 1.0:
                    sens.pop(pid, None)
                else:
                    sens[pid] = mult
                g["person_sensitivity"] = sens
            self._json_response({"ok": True})

        elif path == "/api/confirm_person":
            pid = body.get("pid")
            if pid is not None:
                g["alarm_pid"] = pid
                threading.Thread(target=confirm_rescue, daemon=True).start()
            self._json_response({"ok": True})

        elif path == "/api/dismiss":
            pid = body.get("pid")
            if pid is not None:
                with _lock:
                    g["marked"].discard(pid)
                    g["counters"].pop(pid, None)
                    g["notified"].discard(pid)
                    g.setdefault("dismissed_cooldown", {})[pid] = time.time() + 30
                _log_event("FEHLALARM", f"Person #{pid} einzeln als Fehlalarm markiert", person_id=pid)
                if g.get("collect_training") and _HAS_STC:
                    stc.label("FALSE_ALARM", details=f"Person #{pid} (einzeln)")
                if not g["marked"] and g["state"] == "ALARM":
                    threading.Thread(target=reset_alarm, args=(None, "❌ Fehlalarm – System bereit."), daemon=True).start()
            self._json_response({"ok": True})

        elif path == "/api/settings":
            sens_name = body.get("sensitivity")
            if sens_name and sens_name in SENSITIVITY:
                p = SENSITIVITY[sens_name]
                with _lock:
                    g["distT"]  = p["distT"]
                    g["frames"] = p["frames"]
                    g["radius"] = p["radius"]
                g["_sensitivity_name"] = sens_name
            pico_ip = body.get("pico_ip")
            if pico_ip and isinstance(pico_ip, str):
                pico_ip = pico_ip.strip()
            if pico_ip and _drone._pico_sender and _drone._pico_sender.ip != pico_ip:
                _drone._pico_sender.ip = pico_ip
                with _drone._pico_sender.lock:
                    if _drone._pico_sender.sock:
                        try: _drone._pico_sender.sock.close()
                        except Exception: pass
                        _drone._pico_sender.sock = None
                        _drone._pico_sender.last_status = "Neue IP – verbinde..."
                print(f"[PICO] IP geändert → {pico_ip}")
            zones = body.get("exclusion_zones")
            if zones is not None or pico_ip or sens_name:
                cfg = _load_config() or {}
                if zones is not None:
                    with _lock:
                        g["exclusion_zones"] = [tuple(z) for z in zones]
                    cfg["exclusion_zones"] = [list(z) for z in zones]
                if sens_name:
                    cfg["sens"] = sens_name
                if pico_ip:
                    cfg["pico_ip"] = pico_ip
                _save_config(cfg)
            self._json_response({"ok": True})

        elif path == "/api/zones":
            zones = body.get("zones", [])
            with _lock:
                g["exclusion_zones"] = [tuple(z) for z in zones]
            cfg = _load_config() or {}
            cfg["exclusion_zones"] = [list(z) for z in zones]
            _save_config(cfg)
            self._json_response({"ok": True})

        elif path == "/api/log/clear":
            with _log_lock:
                _save_log([])
            self._json_response({"ok": True})

        elif path == "/api/drone/stop":
            if _drone._pico_sender:
                _drone._pico_sender.clear_queue()
                _drone._pico_sender.send("STOP")
            with _lock:
                g["drone_autonomous"] = False
                g["drone_nav_cmd"]    = "NOTFALL STOP"
            print("[DRONE] NOTFALL STOP via Web-UI")
            self._json_response({"ok": True})

        elif path == "/api/drone/aruco_reset":
            g["drone_orientation_angle"] = None
            self._json_response({"ok": True})

        elif path == "/api/shutdown":
            self._json_response({"ok": True})
            stop_system()
            threading.Thread(target=lambda: (time.sleep(0.5), os._exit(0)), daemon=True).start()

        else:
            self._json_response({"error": "Unknown endpoint"}, 404)

    def log_message(self, fmt, *args):
        pass


def _start_stream_server():
    try:
        ThreadingHTTPServer.allow_reuse_address = True
        ThreadingHTTPServer.daemon_threads = True
        server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), _MJPEGHandler)
        print(f"[SERVER] Web-UI läuft auf Port {STREAM_PORT}")
        server.serve_forever()
    except OSError as e:
        print(f"[SERVER] FEHLER: Port {STREAM_PORT} belegt! Alten Prozess beenden und neu starten. ({e})")
    except Exception as e:
        print(f"[SERVER] FEHLER: {e}")


# ═══════════════════════════════════════════════════════════════════════════════
# ─── TELEGRAM ─────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def _post(path, **kw):
    try:
        requests.post(f"{TELEGRAM_API}/{path}", timeout=8, **kw)
    except Exception:
        pass


def send_alert(pid, lat, lon):
    with _lock:
        if g["tg_sent"]:
            return
        g["tg_sent"] = True
        g["alarm_triggered_at"] = time.time()
        g["auto_dispatched"] = False
    _log_event("ALARM", f"Person #{pid} als ertrinkend erkannt", person_id=pid, lat=lat, lon=lon)
    if g.get("collect_training") and _HAS_STC:
        stc.label("ALARM", details=f"Person #{pid}, GPS={lat:.6f},{lon:.6f}")

    # ── DROHNE PRE-TRIM: Nav-Thread sofort starten damit Trims gesendet werden ────
    # Die Drohne richtet sich am Boden in die Zielrichtung aus, sodass sie beim
    # TAKEOFF sofort korrekt losfliegt. Trim-Counter werden frisch zurückgesetzt
    # damit das Budget für die echte Navigation voll verfügbar ist.
    with _lock:
        for k in g["drone_trim_counters"]:
            g["drone_trim_counters"][k] = 0
        g["drone_autonomous"] = True
        g["drone_reached"]    = False
        g["drone_nav_cmd"]    = ""
    print("[DRONE] Pre-Trim gestartet – Drohne richtet sich aus, warte auf TAKEOFF-Bestätigung.")
    # ── ──────────────────────────────────────────────────────────────────────

    stream_url = f"http://{_get_local_ip()}:{STREAM_PORT}"
    caption = (f"🚨 *S.A.V.E.R ALARM*\n"
               f"Person ertrinkend erkannt! (Person #{pid})\n"
               f"📹 Live-Video: {stream_url}\n")
    if g["use_gps"]:
        caption += f"📍 GPS: `{lat:.6f}, {lon:.6f}`"
    markup = json.dumps({"inline_keyboard": [[
        {"text": "✅ Rettung bestätigen", "callback_data": "ok"},
        {"text": "❌ Fehlalarm",          "callback_data": "false"},
    ]]})
    for cid in list(_tg_subscribers):
        _post("sendMessage", json={
            "chat_id": cid, "text": caption,
            "parse_mode": "Markdown", "reply_markup": json.loads(markup),
        })


def confirm_rescue(cid=None):
    """Rettung bestätigen – sendet TAKEOFF (Trims laufen bereits seit ALARM)."""
    with _lock:
        g["state"]  = "RESCUE"
        g["rescue"] = True
        g["locked"].update(g["marked"])
    _log_event("RETTUNG_BESTAETIGT", "Rettung wurde bestätigt", person_id=g.get("alarm_pid"))
    if g.get("collect_training") and _HAS_STC:
        stc.label("DROWNING_CONFIRMED", details=f"Person #{g.get('alarm_pid')}")

    # ── DROHNE TAKEOFF ──────────────────────────────────────────────────────────────────
    # drone_autonomous läuft bereits seit ALARM (Pre-Trim). Hier nur TAKEOFF.
    if g.get("auto_takeoff", True):
        sender = _drone._pico_sender
        if sender is not None:
            if g.get("drone_orientation_angle") is None:
                print("[DRONE] FEHLER: Kein ArUco-Marker sichtbar – TAKEOFF abgebrochen!")
                _log_event("SYSTEM_START", "Drohnen-TAKEOFF ABGEBROCHEN – keine ArUco-Orientierung!")
            else:
                sender.send("TAKEOFF")
                print("[DRONE] TAKEOFF angefordert – Drohne hebt ab!")
    else:
        print("[DRONE] Auto-Takeoff deaktiviert – Drohne bleibt am Boden.")
    # ── ──────────────────────────────────────────────────────────────────────

    if cid:
        _post("answerCallbackQuery", json={"callback_query_id": cid, "text": "✅ Rettung läuft!"})
    for chat_id in list(_tg_subscribers):
        _post("sendMessage", json={
            "chat_id": chat_id,
            "text": "✅ *Rettung läuft!*\nDrohne ist auf dem Weg!\nDrücke wenn Person gerettet:",
            "parse_mode": "Markdown",
            "reply_markup": {"inline_keyboard": [[
                {"text": "🏊 Rettung abgeschlossen", "callback_data": "done"}
            ]]},
        })


def reset_alarm(cid=None, note=""):
    was_rescue = g.get("rescue", False)
    pid = g.get("alarm_pid")

    # ── DROHNE STOPPEN ──────────────────────────────────────────────────────
    sender = _drone._pico_sender
    if sender is not None:
        sender.clear_queue()
        sender.send("STOP")
    with _lock:
        g["drone_autonomous"] = False
        g["drone_target_pos"] = None
        g["drone_nav_cmd"]    = ""
        tc = g.get("drone_trim_counters", {})
        for k in tc:
            tc[k] = 0
    # ── ──────────────────────────────────────────────────────────────────────

    with _lock:
        g.update(state="NORMAL", tg_sent=False, rescue=False, alarm_pid=None,
                 alert_cooldown_until=0.0, alarm_pending=False,
                 alarm_triggered_at=0.0, auto_dispatched=False)
        # Nur Counter der alarmierten Personen löschen – aktiv getrackte
        # Personen behalten ihren Fortschritts-Counter!
        for _cid in list(g["marked"]) + list(g["locked"]) + list(g["notified"]):
            g["counters"].pop(_cid, None)
        g["marked"].clear()
        g["locked"].clear()
        g["notified"].clear()
    if was_rescue:
        _log_event("RETTUNG_FERTIG", "Rettung abgeschlossen", person_id=pid)
    else:
        _log_event("FEHLALARM", "Alarm als Fehlalarm markiert", person_id=pid)
    if g.get("collect_training") and _HAS_STC:
        stc.label("RESCUE_DONE" if was_rescue else "FALSE_ALARM", details=f"Person #{pid}")
    if cid:
        _post("answerCallbackQuery", json={"callback_query_id": cid, "text": note})
    if note:
        for chat_id in list(_tg_subscribers):
            _post("sendMessage", json={"chat_id": chat_id, "text": note})


def tg_poller():
    while True:
        try:
            r = requests.get(f"{TELEGRAM_API}/getUpdates",
                             params={"offset": g["tg_uid"] + 1, "timeout": 30},
                             timeout=35).json()
            for u in r.get("result", []):
                g["tg_uid"] = u["update_id"]
                msg = u.get("message")
                if msg:
                    uid = str(msg["chat"]["id"])
                    txt = (msg.get("text") or "").strip()
                    if uid in _tg_subscribers:
                        pass
                    elif uid in _tg_pending:
                        if txt == TG_PASSWORD:
                            _tg_pending.discard(uid)
                            _tg_subscribers.add(uid)
                            _save_tg_subscribers(_tg_subscribers)
                            _post("sendMessage", json={"chat_id": uid,
                                "text": "✅ *Passwort korrekt!*\n🌊 Du erhältst ab jetzt alle S.A.V.E.R Alarme.",
                                "parse_mode": "Markdown"})
                        else:
                            _post("sendMessage", json={"chat_id": uid,
                                "text": "❌ Falsches Passwort. Bitte erneut versuchen:"})
                    else:
                        _tg_pending.add(uid)
                        _post("sendMessage", json={"chat_id": uid,
                            "text": "🔐 *S.A.V.E.R Zugangsschutz*\nBitte gib das Passwort ein:",
                            "parse_mode": "Markdown"})
                cb = u.get("callback_query")
                if cb:
                    cb_uid = str(cb["from"]["id"])
                    if cb_uid not in _tg_subscribers:
                        _tg_subscribers.add(cb_uid)
                        _save_tg_subscribers(_tg_subscribers)
                    d, cid = cb["data"], cb["id"]
                    if   d == "ok":
                        g["tg_action_note"] = ("📱 Rettung wurde per Handy bestätigt", time.time())
                        confirm_rescue(cid)
                    elif d == "false":
                        g["tg_action_note"] = ("📱 Fehlalarm wurde per Handy markiert", time.time())
                        reset_alarm(cid, "❌ Fehlalarm – System bereit.")
                    elif d == "done":
                        g["tg_action_note"] = ("📱 Rettung wurde per Handy abgeschlossen", time.time())
                        reset_alarm(cid, "✅ Rettung abgeschlossen – System bereit!")
                    elif d == "aware":
                        with _lock:
                            g["awareness_pending"] = False
                            g["awareness_alarm"]   = False
                        _post("answerCallbackQuery",
                              json={"callback_query_id": cid, "text": "✅ Anwesenheit bestätigt!"})
                        for chat_id in list(_tg_subscribers):
                            _post("sendMessage", json={"chat_id": chat_id,
                                "text": "✅ Anwesenheit bestätigt – System läuft weiter."})
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
# ─── ESKALATIONS-THREAD ───────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def escalation_loop():
    last_awareness = time.time()
    while g.get("running", False):
        now = time.time()
        if (g["state"] in ("NORMAL",) and not g["awareness_pending"] and
                now - last_awareness >= AWARENESS_INTERVAL_SEC):
            last_awareness = now
            with _lock:
                g["awareness_pending"] = True
                g["awareness_deadline"] = now + AWARENESS_TIMEOUT_SEC
                g["awareness_alarm"]    = False
            for cid in list(_tg_subscribers):
                _post("sendMessage", json={"chat_id": cid,
                    "text": f"🔔 *Anwesenheits-Check*\nBitte bestätige innerhalb von {AWARENESS_TIMEOUT_SEC} Sekunden.",
                    "parse_mode": "Markdown",
                    "reply_markup": {"inline_keyboard": [[{"text": "✅ Ich bin da", "callback_data": "aware"}]]}})
        if (g["awareness_pending"] and not g["awareness_alarm"] and now >= g["awareness_deadline"]):
            with _lock:
                g["awareness_alarm"] = True
            _log_event("AWARENESS_ALARM", "Rettungsschwimmer hat Anwesenheit nicht bestätigt")
            for cid in list(_tg_subscribers):
                _post("sendMessage", json={"chat_id": cid,
                    "text": "🚨 *ACHTUNG – Rettungsschwimmer nicht bestätigt!*\nBitte sofort prüfen!",
                    "parse_mode": "Markdown",
                    "reply_markup": {"inline_keyboard": [[{"text": "✅ Ich bin jetzt da", "callback_data": "aware"}]]}})
            threading.Thread(target=alarm_sound, daemon=True).start()
        # # ── Auto-Dispatch (aktuell deaktiviert) ──────────────────────────────
        # if (g["state"] == "ALARM" and g["alarm_triggered_at"] > 0 and
        #         not g["auto_dispatched"] and now - g["alarm_triggered_at"] >= ALARM_AUTO_DISPATCH_SEC):
        #     _log_event("AUTO_DISPATCH", f"Auto-Eskalation nach {ALARM_AUTO_DISPATCH_SEC}s", person_id=g.get("alarm_pid"))
        #     with _lock:
        #         g["auto_dispatched"] = True
        #     for cid in list(_tg_subscribers):
        #         _post("sendMessage", json={"chat_id": cid,
        #             "text": f"⚠️ *AUTO-ESKALATION*\nKeine Bestätigung seit {ALARM_AUTO_DISPATCH_SEC}s!\nRettung wird AUTOMATISCH bestätigt.",
        #             "parse_mode": "Markdown",
        #             "reply_markup": {"inline_keyboard": [[{"text": "❌ STOPP – Fehlalarm", "callback_data": "false"}]]}})
        #     confirm_rescue()
        if g["state"] in ("ALARM", "RESCUE"):
            last_awareness = now
        time.sleep(1)


# ═══════════════════════════════════════════════════════════════════════════════
# ─── KAMERA-READER (Threading – kein I/O-Blocking im Main Loop) ───────────────
# ═══════════════════════════════════════════════════════════════════════════════
class _CamReader:
    """Liest Kameraframes in einem eigenen Thread.
    read() liefert sofort den aktuellsten Frame – nie einen alten Puffer-Frame.
    Reconnect erfolgt automatisch nach 30 aufeinanderfolgenden Fehlern.
    """
    def __init__(self, index):
        self._index = index
        self._cap   = cv2.VideoCapture(index)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._frame = None
        self._ok    = False
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        fail = 0
        while True:
            ok, fr = self._cap.read()
            if ok:
                fail = 0
                with self._lock:
                    self._frame = fr
                    self._ok    = True
            else:
                fail += 1
                if fail > 30:
                    self._cap.release()
                    time.sleep(1.0)
                    self._cap = cv2.VideoCapture(self._index)
                    self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    fail = 0
                else:
                    time.sleep(0.02)

    def read(self):
        with self._lock:
            if not self._ok:
                return False, None
            return True, self._frame.copy()

    def release(self):
        self._cap.release()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── DETECTION LOOP (YOLO + ArUco FUSION) ─────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
def detection_loop():
    """
    Öffnet eine Kamera (g["cam"]).
    Führt YOLO-Erkennung (Personen/Ertrinken) UND
    ArUco-Erkennung (Drohnen-Position/Orientierung) durch.
    Schreibt Ergebnisse in g[].
    """
    model = YOLO("yolov8n-pose.pt")
    aruco_detector, aruco_dict, aruco_params, has_detector = get_aruco_detector(ARUCO_DICT_NAME)

    cam = _CamReader(g["cam"])

    found_a_buf = deque(maxlen=5)
    found_b_buf = deque(maxlen=5)
    aruco_lost_frames   = 0   # konsekutive Frames ohne ArUco-Erkennung
    ARUCO_LOST_THRESHOLD = 5  # ab hier Position auf None setzen

    _grace        = {}       # tid → konsekutive Frames seit letzter Erkennung
    _GRACE_FRAMES = 20       # Frames Gnadenfrist bevor Counter/Marked gelöscht
    _last_bbox    = {}       # tid → (x1, y1, x2, y2) letzte bekannte Bounding Box
    _ghost_pool   = {}       # tid → {"pos": [cx,cy], "counter": N, "sens": S, "marked": bool, "age": N}
    _yolo_last    = None     # Letztes YOLO-Ergebnis
    _yolo_skip    = 0        # Frame-Zähler für YOLO-Skip
    _YOLO_EVERY   = 2        # YOLO nur jeden N-ten Frame (flüssigere Anzeige)

    drone_path     = []
    corridor_points = None

    while g["running"]:
        ret, frame = cam.read()
        if not ret:
            time.sleep(0.01)
            continue

        fh, fw = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── 1. YOLO – Personen-Erkennung ──────────────────────────────────────
        # YOLO nur jeden _YOLO_EVERY-ten Frame – dazwischen letztes Ergebnis wiederverwenden
        _yolo_skip += 1
        if _yolo_skip >= _YOLO_EVERY:
            _yolo_skip = 0
            try:
                _yolo_last = model.track(source=frame, persist=True,
                                         conf=0.25, iou=0.45, verbose=False,
                                         tracker=os.path.join(os.path.dirname(os.path.abspath(__file__)), "bytetrack.yaml"))
            except Exception:
                time.sleep(0.02)
                continue
        results = _yolo_last or []

        if g.get("collect_training") and _HAS_STC:
            stc.record(results, fw, fh, _in_exclusion_zone)

        mnew = {}
        for r in results:
            for box in r.boxes:
                if box.id is None:
                    continue
                track_id = int(box.id[0])
                b  = box.xyxy[0]
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                if _in_exclusion_zone(cx / fw, cy / fh):
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (80, 80, 80), 1)
                    cv2.putText(frame, "ausserhalb", (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 80, 80), 1)
                    continue
                mnew[track_id] = [cx, cy]
                _last_bbox[track_id] = (x1, y1, x2, y2)
                track_idx = track_id
                la, lo = px2gps(cx, cy, fw, fh)
                drowning = track_idx in g["marked"]
                if drowning:
                    cv2.rectangle(frame, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (0, 0, 255), 4)
                    icx, icy = int(cx), int(cy)
                    cv2.line(frame, (0, icy), (fw, icy), (0, 0, 200), 1)
                    cv2.line(frame, (icx, 0), (icx, fh), (0, 0, 200), 1)
                    cl = 20
                    for (ex, ey, dx, dy) in [(x1, y1, 1, 1), (x2, y1, -1, 1),
                                              (x1, y2, 1, -1), (x2, y2, -1, -1)]:
                        cv2.line(frame, (ex, ey), (ex + dx * cl, ey), (0, 0, 255), 3)
                        cv2.line(frame, (ex, ey), (ex, ey + dy * cl), (0, 0, 255), 3)
                    lbl = "!! ERTRINKEN !!"
                    (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(frame, (x1, y1 - th - 16), (x1 + tw + 10, y1 - 4), (0, 0, 180), -1)
                    cv2.putText(frame, lbl, (x1 + 5, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    sens = g.get("person_sensitivity", {}).get(track_idx, 1.0)
                    box_color = (0, 200, 255) if sens < 1.0 else ((200, 200, 0) if sens > 1.0 else (0, 160, 0))
                    tag = "GENAU" if sens < 1.0 else ("LOCKER" if sens > 1.0 else "")
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 1)
                    secs = g["counters"].get(track_idx, 0) / 30
                    lbl_txt = f"{secs:.1f}s" + (f" [{tag}]" if tag else "")
                    cv2.putText(frame, lbl_txt, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1)

        # ── Grace Period + ID-Transfer: kurze YOLO-Aussetzer abdämpfen ─────
        # Personen die YOLO für wenige Frames verliert behalten ihre
        # letzte Position, Bounding Box, Counter und Status.
        # Wenn YOLO eine Person unter neuer ID wiederfindet, wird der
        # alte State (Counter, Sensitivity, Marked) auf die neue ID übertragen.

        _ID_TRANSFER_RADIUS = 80  # max Pixel-Abstand für ID-Transfer

        # 1. Neue IDs prüfen → ggf. State von ge-graceter Person übernehmen
        for tid, new_pos in list(mnew.items()):
            if tid not in g.get("mold", {}) and tid not in g["counters"]:
                # Neue Person → schauen ob eine ge-gracete Person in der Nähe ist
                best_old = None
                best_dist = _ID_TRANSFER_RADIUS
                for old_tid, grace_frames in list(_grace.items()):
                    if old_tid in mnew:
                        continue  # ist schon aktiv, kein Transfer nötig
                    old_pos = g.get("mold", {}).get(old_tid)
                    if old_pos is None:
                        continue
                    d = math.hypot(old_pos[0] - new_pos[0], old_pos[1] - new_pos[1])
                    if d < best_dist:
                        best_dist = d
                        best_old = old_tid
                # Auch Ghost Pool checken (kürzlich abgelaufene Grace)
                for old_tid, ghost in list(_ghost_pool.items()):
                    d = math.hypot(ghost["pos"][0] - new_pos[0], ghost["pos"][1] - new_pos[1])
                    if d < best_dist:
                        best_dist = d
                        best_old = old_tid
                if best_old is not None:
                    # State übertragen: alte ID → neue ID
                    old_counter = g["counters"].pop(best_old, 0)
                    if best_old in _ghost_pool:
                        old_counter = max(old_counter, _ghost_pool[best_old].get("counter", 0))
                        ghost_data = _ghost_pool.pop(best_old)
                        if ghost_data.get("marked") and best_old not in g.get("locked", set()):
                            g["marked"].add(tid)
                    g["counters"][tid] = old_counter
                    old_sens = g.get("person_sensitivity", {}).pop(best_old, None)
                    if old_sens is not None:
                        g.setdefault("person_sensitivity", {})[tid] = old_sens
                    if best_old in g["marked"]:
                        g["marked"].discard(best_old)
                        g["marked"].add(tid)
                    if best_old in g["notified"]:
                        g["notified"].discard(best_old)
                        g["notified"].add(tid)
                    if best_old in g.get("locked", set()):
                        g["locked"].discard(best_old)
                        g["locked"].add(tid)
                    if g.get("alarm_pid") == best_old:
                        g["alarm_pid"] = tid
                    # Grace + Bbox aufräumen
                    _grace.pop(best_old, None)
                    bbox = _last_bbox.pop(best_old, None)
                    if bbox:
                        _last_bbox[tid] = bbox
                    # Aus mold entfernen damit alte ID nicht weiter läuft
                    g.get("mold", {}).pop(best_old, None)

        # 2. Grace Period für verschwundene Personen
        for old_tid in list(g.get("mold", {}).keys()):
            if old_tid not in mnew:
                _grace[old_tid] = _grace.get(old_tid, 0) + 1
                if _grace[old_tid] <= _GRACE_FRAMES:
                    # Noch in Gnadenfrist → letzte Position + Box beibehalten
                    mnew[old_tid] = g["mold"][old_tid]
                    if old_tid in _last_bbox:
                        bx1, by1, bx2, by2 = _last_bbox[old_tid]
                        if old_tid in g["marked"]:
                            # Ertrinken-Markierung auch in Grace Period zeichnen
                            cv2.rectangle(frame, (bx1 - 4, by1 - 4), (bx2 + 4, by2 + 4), (0, 0, 255), 4)
                            icx, icy = int((bx1+bx2)/2), int((by1+by2)/2)
                            cv2.line(frame, (0, icy), (fw, icy), (0, 0, 200), 1)
                            cv2.line(frame, (icx, 0), (icx, fh), (0, 0, 200), 1)
                            cl = 20
                            for (ex, ey, dx, dy) in [(bx1, by1, 1, 1), (bx2, by1, -1, 1),
                                                      (bx1, by2, 1, -1), (bx2, by2, -1, -1)]:
                                cv2.line(frame, (ex, ey), (ex + dx * cl, ey), (0, 0, 255), 3)
                                cv2.line(frame, (ex, ey), (ex, ey + dy * cl), (0, 0, 255), 3)
                            lbl = "!! ERTRINKEN !!"
                            (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                            cv2.rectangle(frame, (bx1, by1 - th - 16), (bx1 + tw + 10, by1 - 4), (0, 0, 180), -1)
                            cv2.putText(frame, lbl, (bx1 + 5, by1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        else:
                            ghost_color = (100, 100, 100)
                            cv2.rectangle(frame, (bx1, by1), (bx2, by2), ghost_color, 1)
                            secs = g["counters"].get(old_tid, 0) / 30
                            cv2.putText(frame, f"{secs:.1f}s", (bx1, by1 - 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, ghost_color, 1)
                else:
                    # Gnadenfrist abgelaufen → in Ghost Pool für ID-Transfer
                    old_pos = g.get("mold", {}).get(old_tid)
                    if old_pos is not None:
                        _ghost_pool[old_tid] = {
                            "pos": old_pos,
                            "counter": g["counters"].get(old_tid, 0),
                            "marked": old_tid in g.get("marked", set()),
                            "age": 0,
                        }
                    _grace.pop(old_tid, None)
                    # Für markierte/gesperrte Personen bbox NICHT löschen – wird
                    # bis Rettungsabschluss an letzter Position gerendert
                    if old_tid not in g.get("marked", set()) and old_tid not in g.get("locked", set()):
                        _last_bbox.pop(old_tid, None)
                    g["counters"].pop(old_tid, None)
                    # Markierte Personen NICHT aus marked entfernen – bleiben
                    # markiert bis explizite Nutzer-Aktion (✅/❌)
                    if old_tid in g["notified"] and old_tid not in g["locked"] and old_tid not in g["marked"]:
                        g["notified"].discard(old_tid)
            else:
                _grace.pop(old_tid, None)  # wieder sichtbar → Gnadenfrist zurücksetzen

        # 3. Ghost Pool aufräumen (max 60 Frames nach Grace-Ablauf)
        for gid in list(_ghost_pool.keys()):
            _ghost_pool[gid]["age"] += 1
            if _ghost_pool[gid]["age"] > 60:
                _ghost_pool.pop(gid, None)

        # 4. Persistente Anzeige ertrinkender Personen die nicht mehr getrackt
        # werden – Bounding Box bleibt an letzter bekannter Position bis zur
        # expliziten Rettungsbestätigung (✅/❌)
        _all_drowning = g.get("marked", set()) | g.get("locked", set())
        for _pid in _all_drowning:
            if _pid not in mnew and _pid in _last_bbox:
                bx1, by1, bx2, by2 = _last_bbox[_pid]
                # Gestrichelter Rahmen + Fadenkreuz in dunklerem Rot ("zuletzt gesehen")
                cv2.rectangle(frame, (bx1 - 4, by1 - 4), (bx2 + 4, by2 + 4), (0, 0, 220), 3)
                icx, icy = int((bx1 + bx2) / 2), int((by1 + by2) / 2)
                cv2.line(frame, (0, icy), (fw, icy), (0, 0, 180), 1)
                cv2.line(frame, (icx, 0), (icx, fh), (0, 0, 180), 1)
                cl = 20
                for (ex, ey, dx, dy) in [(bx1, by1, 1, 1), (bx2, by1, -1, 1),
                                          (bx1, by2, 1, -1), (bx2, by2, -1, -1)]:
                    cv2.line(frame, (ex, ey), (ex + dx * cl, ey), (0, 0, 220), 3)
                    cv2.line(frame, (ex, ey), (ex, ey + dy * cl), (0, 0, 220), 3)
                lbl2 = "!! LETZTE POS !!"
                (tw2, th2), _ = cv2.getTextSize(lbl2, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                cv2.rectangle(frame, (bx1, by1 - th2 - 16), (bx1 + tw2 + 10, by1 - 4), (0, 0, 160), -1)
                cv2.putText(frame, lbl2, (bx1 + 5, by1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 255), 2)
                # Person an letzter Position in mold halten → Frontend-Seitenpanel bleibt sichtbar
                mnew[_pid] = [icx, icy]

        # ── Bewegungs-Tracking / Alarm auslösen ───────────────────────────────
        mold = g["mold"]

        for tid, new_pos in mnew.items():
            try:
                if tid in mold:
                    old_pos = mold[tid]
                    if math.hypot(old_pos[0] - new_pos[0], old_pos[1] - new_pos[1]) < g["distT"]:
                        g["counters"][tid] = g["counters"].get(tid, 0) + 1
                        threshold = g["frames"] * g.get("person_sensitivity", {}).get(tid, 1.0)
                        if g["counters"][tid] >= threshold:
                            _dc = g.get("dismissed_cooldown", {})
                            if tid in _dc and time.time() < _dc[tid]:
                                # Person wurde als Fehlalarm markiert – Cooldown läuft
                                g["counters"][tid] = 0
                            else:
                                _dc.pop(tid, None)  # abgelaufenen Eintrag bereinigen
                                g["marked"].add(tid)
                            trigger = False
                            if tid in g["marked"]:
                                with _lock:
                                    if g["state"] == "NORMAL" and not g["alarm_pending"]:
                                        g["state"]   = "ALARM"
                                        g["alarms"] += 1
                                        g["alert_cooldown_until"] = time.time() + 1.5
                                        trigger = True
                                    elif g["state"] in ("ALARM", "RESCUE") and tid not in g["notified"]:
                                        # Weitere Person erkannt während Alarm bereits läuft –
                                        # Ton auslösen aber keinen neuen Telegram-Alarm senden
                                        trigger = True
                            if trigger:
                                g["notified"].add(tid)
                                la, lo = px2gps(new_pos[0], new_pos[1], fw, fh)
                                if g["state"] == "NORMAL":
                                    # Erster Alarm: Telegram + Ton
                                    g.update(alarm_pid=tid, alarm_lat=la, alarm_lon=lo)
                                    threading.Thread(target=send_alert, args=(tid, la, lo), daemon=True).start()
                                threading.Thread(target=alarm_sound, daemon=True).start()
                    else:
                        # Nur Counter zurücksetzen; als ertrinkend markierte Personen
                        # bleiben IMMER markiert bis explizite Nutzer-Aktion (✅/❌)
                        if tid not in g["marked"]:
                            g["counters"][tid] = 0
                        if tid in g["notified"] and tid not in g["locked"] and tid not in g["marked"]:
                            g["notified"].discard(tid)
            except Exception:
                traceback.print_exc()

        # ── Ausschluss-Zonen zeichnen ─────────────────────────────────────────
        zones = g.get("exclusion_zones", [])
        if zones:
            overlay = frame.copy()
            for (nx1, ny1, nx2, ny2) in zones:
                cv2.rectangle(overlay, (int(nx1*fw), int(ny1*fh)),
                              (int(nx2*fw), int(ny2*fh)), (0, 0, 120), -1)
            frame = cv2.addWeighted(overlay, 0.25, frame, 0.75, 0)
            for (nx1, ny1, nx2, ny2) in zones:
                cv2.rectangle(frame, (int(nx1*fw), int(ny1*fh)),
                              (int(nx2*fw), int(ny2*fh)), (0, 0, 180), 1)

        g["mold"]    = mnew
        g["persons"] = len(mnew)
        g["bboxes"]  = {tid: (pos[0], pos[1]) for tid, pos in mnew.items()}

        # ── 2. ArUco – Drohnen-Position und Orientierung ──────────────────────
        corners, ids = detect_aruco(gray, aruco_detector, aruco_dict, aruco_params, has_detector)
        found_a_raw       = None
        current_marker_angle = None
        marker_front_pt   = None
        marker_pixel_size = None

        if ids is not None and len(ids) > 0:
            for i, mid in enumerate(ids.flatten()):
                if int(mid) == MARKER_ID:
                    c = np.array(corners[i], dtype=np.float32)
                    (mcx, mcy), current_marker_angle, marker_front_pt = marker_center_and_angle(c)
                    found_a_raw = (int(mcx), int(mcy))
                    corner_pts  = c.reshape((4, 2))
                    d01 = np.hypot(corner_pts[0, 0] - corner_pts[1, 0],
                                   corner_pts[0, 1] - corner_pts[1, 1])
                    d12 = np.hypot(corner_pts[1, 0] - corner_pts[2, 0],
                                   corner_pts[1, 1] - corner_pts[2, 1])
                    marker_pixel_size = max(d01, d12)

                    # Orientierung nur einmalig beim ersten Erkennen setzen (dann eingefroren)
                    if g.get("drone_orientation_angle") is None:
                        g["drone_orientation_angle"] = current_marker_angle
                        print(f"[ARUCO] Orientierung gesetzt (einmalig): {np.degrees(current_marker_angle):.1f}°")

                    # NEU: Anker-Offset-Infrastruktur wie Ideal (aktuell shift_px=0)
                    corner_pts_off = c.reshape((4, 2))
                    top_mid_x_off = (corner_pts_off[0, 0] + corner_pts_off[1, 0]) / 2.0
                    top_mid_y_off = (corner_pts_off[0, 1] + corner_pts_off[1, 1]) / 2.0
                    shift_px = 0.0  # kann später angepasst werden
                    vx_off = top_mid_x_off - mcx
                    vy_off = top_mid_y_off - mcy
                    vlen_off = np.hypot(vx_off, vy_off)
                    if vlen_off != 0.0:
                        found_a_raw = (int(mcx + (vx_off / vlen_off) * shift_px),
                                       int(mcy + (vy_off / vlen_off) * shift_px))

                    g["drone_marker_pixel_size"] = marker_pixel_size

                    # Zeichnen
                    for pt in corner_pts.astype(int):
                        cv2.circle(frame, tuple(pt), 3, (0, 255, 0), -1)
                    cv2.circle(frame, found_a_raw, 6, (0, 200, 0), -1)
                    cv2.putText(frame, f"DROHNE (ArUco {MARKER_ID})",
                                (found_a_raw[0] + 10, found_a_raw[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)
                    if marker_front_pt:
                        cv2.line(frame, found_a_raw, marker_front_pt, (128, 255, 128), 2)
                    break

        # Glättung ArUco-Position
        g["drone_aruco_visible"] = (found_a_raw is not None)  # aktueller Frame – kein Buffer-Lag
        if found_a_raw is not None:
            found_a_buf.append(found_a_raw)
            aruco_lost_frames = 0
        else:
            aruco_lost_frames += 1
            if aruco_lost_frames >= ARUCO_LOST_THRESHOLD:
                # Puffer leeren → Position wird None → Nav-Thread sendet STOP
                found_a_buf.clear()
        smoothed_a = median_point(found_a_buf) if found_a_buf else None
        g["drone_aruco_pos"] = smoothed_a

        # Drohnenpfad speichern
        if smoothed_a is not None:
            if not drone_path or smoothed_a != drone_path[-1]:
                drone_path.append(smoothed_a)
            if len(drone_path) > 100:
                drone_path = drone_path[-100:]

        # Fester Orientierungs-Pfeil (Magenta)
        drone_angle = g.get("drone_orientation_angle")
        if drone_angle is not None and smoothed_a is not None:
            arr_len = 80
            ex = int(smoothed_a[0] + arr_len * np.cos(drone_angle))
            ey = int(smoothed_a[1] + arr_len * np.sin(drone_angle))
            cv2.arrowedLine(frame, smoothed_a, (ex, ey), (255, 0, 255), 3, tipLength=0.3)

        # ── 3. Drohnen-Ziel = Position der ertrinkenden Person ────────────────
        alarm_pid = g.get("alarm_pid")
        if alarm_pid is not None and alarm_pid in mnew:
            tx = int(mnew[alarm_pid][0])
            ty = int(mnew[alarm_pid][1])
            found_b_raw = (tx, ty)
            found_b_buf.append(found_b_raw)
            smoothed_b = median_point(found_b_buf)
            if smoothed_b:
                g["drone_target_pos"] = smoothed_b
                cv2.circle(frame, smoothed_b, 10, (0, 0, 255), -1)
                cv2.circle(frame, smoothed_b, TARGET_REACHED_DIST, (0, 255, 0), 1)
                cv2.circle(frame, smoothed_b, SLOWDOWN_RADIUS, (0, 165, 255), 1)
                cv2.putText(frame, "DROHNEN-ZIEL",
                            (smoothed_b[0] + 14, smoothed_b[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                if smoothed_a is not None:
                    cv2.line(frame, smoothed_a, smoothed_b, (0, 200, 200), 2)
                    dist = int(np.hypot(smoothed_b[0] - smoothed_a[0],
                                        smoothed_b[1] - smoothed_a[1]))
                    cv2.putText(frame, f"Dist: {dist}px",
                                (10, fh - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 200), 2)
                if corridor_points is None and smoothed_a is not None:
                    corridor_points = (smoothed_a, smoothed_b)
        else:
            found_b_buf.clear()
            if g.get("state") == "NORMAL":
                g["drone_target_pos"] = None
            corridor_points = None

        # Korridor zeichnen
        if corridor_points is not None:
            start, end = corridor_points
            cv2.line(frame, start, end, (255, 255, 0), 1)
            # Korridorgrenzen (parallele Linien) wie Ideal
            sx, sy = start
            ex, ey = end
            vx_c = ex - sx
            vy_c = ey - sy
            vlen_c = np.hypot(vx_c, vy_c)
            if vlen_c > 0.0:
                px_c = -vy_c / vlen_c
                py_c = vx_c / vlen_c
                from config import CORRIDOR_HALF_WIDTH as _CHW
                offx = int(px_c * _CHW)
                offy = int(py_c * _CHW)
                cv2.line(frame, (sx + offx, sy + offy), (ex + offx, ey + offy), (0, 255, 255), 2)
                cv2.line(frame, (sx - offx, sy - offy), (ex - offx, ey - offy), (0, 255, 255), 2)
            cv2.putText(frame, "KORRIDOR", (start[0] + 10, start[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Drohnenpfad zeichnen
        if len(drone_path) > 1:
            for i in range(1, len(drone_path)):
                cv2.line(frame, drone_path[i-1], drone_path[i], (255, 128, 0), 2)
            cv2.circle(frame, drone_path[0], 6, (255, 128, 0), -1)
            cv2.putText(frame, "Pfad", (drone_path[0][0]+8, drone_path[0][1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 1)

        # Status-Overlay
        nav_cmd = g.get("drone_nav_cmd", "")
        if nav_cmd:
            cv2.putText(frame, f"DRONE CMD: {nav_cmd}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 120, 0), 2)
        if g.get("drone_autonomous") and not g.get("drone_orientation_angle"):
            cv2.putText(frame, "WARTE: Suche ArUco-Marker ...", (10, fh - 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
        if marker_pixel_size is not None and marker_pixel_size < PIXELS_MIN_MARKER:
            cv2.putText(frame, f"ArUco KLEIN ({int(marker_pixel_size)}px – Drohne zu weit?)",
                        (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        g["frame"] = frame.copy()

        # NEU: Frame-Sequenz inkrementieren → Nav-Thread navigiert nur bei neuem Frame
        g["drone_frame_seq"] = g.get("drone_frame_seq", 0) + 1

    cam.release()
