"""
S.A.V.E.R – drone_sim.py
Standalone-Simulation der autonomen Drohnennavigation.

Keine Kamera, keine Hardware benötigt.
Nutzt den echten drone_nav_loop aus drone.py via SimPicoSender.

Physik:
  - Trim-basiert: VOR/ZURÜCK → Nickwinkel kumuliert → Beschleunigung
  - RECHTS/LINKS → Rollwinkel kumuliert → Querbeschleunigung
  - STOP → beide Winkel auf 0 → Abbremsung durch Luftwiderstand
  - Drohnen-Heading bleibt während des Flugs konstant (kein Gieren)

Start: python drone_sim.py
"""

import math
import os
import random
import sys
import threading
import time
import tkinter as tk
import customtkinter as ctk
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import (
    g, _lock,
    TARGET_REACHED_DIST, SLOWDOWN_RADIUS, COMMAND_INTERVAL,
    CORRIDOR_HALF_WIDTH, DEADZONE_PIXELS,
    BG, PANEL, CARD, GREEN, RED, ORANGE, BLUE, TEXT, MUTED, BORDER,
    angle_wrap, angle_diff,
    image_to_drone_coords,
)
import drone as _drone

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ─── Physik-Parameter (live in der UI anpassbar) ──────────────────────────────
_SIM = {
    "ACCEL_SCALE": 900.0,    # px/s² pro Radian Neigung
    "DRAG":        1.8,      # 1/s – Luftwiderstand (höher = schneller abbremsen)
    "TRIM_STEP":   0.020,    # rad pro Trim-Befehl  (größer = stärkere Reaktion)
    "MAX_TRIM":    0.07,     # rad maximale Neigung  (Clamp)
    "NOISE_POS":   1.5,      # px Std-Abweichung der ArUco-Positionsmessung
    "NOISE_ANG":   0.010,    # rad Std-Abweichung der Orientierungsmessung
}

_DT       = 0.020   # Physik-Zeitschritt  (50 Hz)
CANVAS_W  = 720
CANVAS_H  = 480
POOL_PAD  = 32      # Canvas-Rand bis Becken-Kante (px)


# ═══════════════════════════════════════════════════════════════════════════════
# ─── PHYSIK-MODELL ────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class DronePhysics:
    """
    Trim-basiertes 2-D-Physikmodell der Drohne (Top-Down-Sicht).

    Koordinaten:  +x = rechts im Canvas, +y = unten im Canvas (Bildkoordinaten)
    Heading:      Winkel in Bild-rad (0 = Drohne zeigt nach rechts/+x)

    Beziehung Heading ↔ Weltbeschleunigung:
        ax = cos(heading)*ap − sin(heading)*ar     (VOR/ZURÜCK-Anteil)
        ay = sin(heading)*ap + cos(heading)*ar     (RECHTS/LINKS-Anteil)
    Damit ist die Physik exakt invers zu image_to_drone_coords() in config.py.
    """

    def __init__(self, px: float, py: float, heading: float = 0.0):
        self.px      = px
        self.py      = py
        self.vx      = 0.0
        self.vy      = 0.0
        self.pitch   = 0.0    # + = VOR,   – = ZURÜCK
        self.roll    = 0.0    # + = RECHTS, – = LINKS
        self.heading = heading
        self._dlock  = threading.Lock()
        self.trail: list[tuple[int, int]] = []

    def apply_command(self, cmd: str) -> None:
        with self._dlock:
            ts = _SIM["TRIM_STEP"]
            mt = _SIM["MAX_TRIM"]
            if   cmd == "VOR":     self.pitch = min(self.pitch + ts,  mt)
            elif cmd == "ZURUECK": self.pitch = max(self.pitch - ts, -mt)
            elif cmd == "RECHTS":  self.roll  = min(self.roll  + ts,  mt)
            elif cmd == "LINKS":   self.roll  = max(self.roll  - ts, -mt)
            elif cmd == "STOP":    self.pitch = 0.0;  self.roll = 0.0

    def step(self) -> None:
        with self._dlock:
            asc  = _SIM["ACCEL_SCALE"]
            drag = _SIM["DRAG"]
            ap   = math.sin(self.pitch) * asc
            ar   = math.sin(self.roll)  * asc
            ca   = math.cos(self.heading)
            sa   = math.sin(self.heading)
            ax   = ca * ap - sa * ar
            ay   = sa * ap + ca * ar
            # Euler-Integration mit linearem Luftwiderstand
            self.vx = self.vx * (1.0 - drag * _DT) + ax * _DT
            self.vy = self.vy * (1.0 - drag * _DT) + ay * _DT
            self.px += self.vx * _DT
            self.py += self.vy * _DT
            # Pool-Wände (elastische Reflexion mit Dämpfung)
            b = float(POOL_PAD)
            rx, ry = float(CANVAS_W) - b, float(CANVAS_H) - b
            if self.px < b:  self.px = b;  self.vx =  abs(self.vx) * 0.25
            if self.px > rx: self.px = rx; self.vx = -abs(self.vx) * 0.25
            if self.py < b:  self.py = b;  self.vy =  abs(self.vy) * 0.25
            if self.py > ry: self.py = ry; self.vy = -abs(self.vy) * 0.25
            # Trail speichern
            self.trail.append((int(self.px), int(self.py)))
            if len(self.trail) > 500:
                self.trail = self.trail[-500:]

    def get(self) -> dict:
        with self._dlock:
            return dict(
                px=self.px, py=self.py,
                vx=self.vx, vy=self.vy,
                pitch=self.pitch, roll=self.roll,
                heading=self.heading,
                speed=math.hypot(self.vx, self.vy),
            )

    def reset(self, px: float, py: float, heading: float = 0.0) -> None:
        with self._dlock:
            self.px = px; self.py = py
            self.vx = self.vy = self.pitch = self.roll = 0.0
            self.heading = heading
            self.trail.clear()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── SIMULIERTER PICO-SENDER ──────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class SimPicoSender:
    """
    Fängt alle Befehle von drone_nav_loop ab und leitet sie an DronePhysics.
    Zählt außerdem gesendete Befehle für die Statistik.
    """
    def __init__(self, physics: DronePhysics, stats: dict):
        self._phys  = physics
        self._stats = stats

    def send(self, cmd: str) -> None:
        self._phys.apply_command(cmd)
        self._stats["cmds"] = self._stats.get("cmds", 0) + 1
        self._stats["last_cmd"] = cmd

    def clear_queue(self) -> None:
        pass

    def get_status(self) -> str:
        return "SIM-Verbunden ✓"

    def queue_size(self) -> int:
        return 0

    def stop(self) -> None:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
# ─── HAUPTANWENDUNG ───────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
_DRONE_START = (180.0, 240.0)
_TARGET_DEF  = (560.0, 240.0)

# Szenarien: (Name, Ziel-px, Ziel-py, Heading-Grad)
_SCENARIOS = [
    ("🎯  3 Uhr  –  Geradeaus",          560, 240,  0),
    ("↗  1 Uhr  –  Vorne-links",          550,  90,  0),
    ("↘  5 Uhr  –  Vorne-rechts",         550, 395,  0),
    ("↙  7 Uhr  –  Hinten-rechts",        100, 380,  0),
    ("←  9 Uhr  –  Direkt hinten",         80, 240,  0),
    ("↖  11 Uhr –  Hinten-links",         100,  90,  0),
    ("🔄  Heading 90°, Ziel links",       560, 240, 90),
    ("🔄  Heading 180°, Ziel hinten",     560, 240,180),
]


class SimulationApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("S.A.V.E.R  DRONE  –  Navigations-Simulation")
        self.geometry("1200x700")
        self.configure(fg_color=BG)
        self.resizable(False, False)

        self._mission = False
        self._stats: dict = {"cmds": 0, "dist_min": float("inf"),
                             "arrived": False, "t0": 0.0}

        self._physics = DronePhysics(*_DRONE_START, heading=0.0)
        self._target  = list(_TARGET_DEF)

        # g[] initialisieren (minimale Felder für drone_nav_loop)
        g.update(
            running=True, state="NORMAL",
            drone_autonomous=False, drone_reached=False,
            drone_aruco_pos=None, drone_target_pos=None,
            drone_orientation_angle=None,
            drone_nav_cmd="", drone_pico_status="SIM",
            drone_trim_counters={}, drone_marker_pixel_size=None,
            alarm_pid=99, marked=set(), mold={},
            counters={}, notified={}, locked=set(),
        )

        # Simulierten Sender einhängen → drone_nav_loop nutzt ihn automatisch
        _drone._pico_sender = SimPicoSender(self._physics, self._stats)

        self._build_ui()

        # Echten Navigations-Thread starten (drone.py Code, unverändert)
        threading.Thread(target=_drone.drone_nav_loop, daemon=True).start()
        # Physik-Thread starten
        threading.Thread(target=self._sim_loop, daemon=True).start()
        # UI-Render starten
        self._draw()

    # ── UI Aufbau ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Canvas-Seite ──────────────────────────────────────────────────────
        left = ctk.CTkFrame(self, fg_color="transparent")
        left.pack(side="left", fill="both", expand=True, padx=(10, 4), pady=10)

        ctk.CTkLabel(
            left,
            text="Top-Down-Ansicht  ·  Linksklick = Ziel setzen  ·  Rechtsklick = Drohne teleportieren (nur bei Stop)",
            font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED,
        ).pack(anchor="w", pady=(0, 3))

        self._canvas = tk.Canvas(
            left, width=CANVAS_W, height=CANVAS_H,
            bg="#030b18", highlightthickness=1, highlightbackground="#1a2030",
        )
        self._canvas.pack()
        self._canvas.bind("<Button-1>", self._on_left_click)
        self._canvas.bind("<Button-3>", self._on_right_click)

        self._bar = ctk.CTkLabel(left, text="", font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED)
        self._bar.pack(anchor="w", pady=(4, 0))

        # ── Panel-Seite ───────────────────────────────────────────────────────
        right = ctk.CTkScrollableFrame(self, fg_color=PANEL, width=350, corner_radius=0)
        right.pack(side="right", fill="y", padx=(4, 10), pady=10)

        ctk.CTkLabel(right, text="🚁  Drohnen-Simulation",
                     font=ctk.CTkFont("Segoe UI", 20, "bold"), text_color=BLUE).pack(pady=(12, 2))
        ctk.CTkLabel(right, text="Echter drone_nav_loop  ·  Simulated Physics",
                     font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED).pack(pady=(0, 14))

        # Mission-Buttons
        self._section(right, "MISSION")
        row = ctk.CTkFrame(right, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 5))
        self._btn_start = ctk.CTkButton(
            row, text="▶  START", height=46, font=ctk.CTkFont("Segoe UI", 16, "bold"),
            fg_color=GREEN, hover_color="#00aa55", command=self._start_mission)
        self._btn_start.pack(side="left", expand=True, fill="x", padx=(0, 3))
        ctk.CTkButton(
            row, text="⏹  STOP", height=46, font=ctk.CTkFont("Segoe UI", 16, "bold"),
            fg_color="#660000", hover_color="#440000", command=self._stop_mission,
        ).pack(side="left", expand=True, fill="x")
        ctk.CTkButton(
            right, text="↺  Reset (Position + Trims)", height=32,
            fg_color=CARD, hover_color="#252540", text_color="#8899aa",
            command=self._reset,
        ).pack(fill="x", padx=8, pady=(0, 10))

        # Heading
        self._section(right, "DROHNEN-HEADING (Anfangsorientierung)")
        hrow = ctk.CTkFrame(right, fg_color="transparent")
        hrow.pack(fill="x", padx=8, pady=(0, 2))
        for lbl, deg in [("→ 0°", 0), ("↓ 90°", 90), ("← 180°", 180), ("↑ 270°", 270)]:
            ctk.CTkButton(
                hrow, text=lbl, width=72, height=30,
                fg_color=CARD, hover_color="#252540", text_color="#8899aa",
                font=ctk.CTkFont("Segoe UI", 12),
                command=lambda d=deg: self._set_heading(d),
            ).pack(side="left", padx=2)
        self._hdg_lbl = ctk.CTkLabel(right, text="Heading: 0°",
                                      font=ctk.CTkFont("Segoe UI", 11), text_color=TEXT)
        self._hdg_lbl.pack(anchor="w", padx=10, pady=(4, 10))

        # Szenarien
        self._section(right, "SZENARIEN  (Reset + Ziel setzen)")
        for name, tx, ty, hdg in _SCENARIOS:
            ctk.CTkButton(
                right, text=name, height=30,
                font=ctk.CTkFont("Segoe UI", 12),
                fg_color=CARD, hover_color="#252540", text_color=TEXT,
                command=lambda t=(tx, ty), h=hdg: self._load_scenario(t, h),
            ).pack(fill="x", padx=8, pady=2)

        # Telemetrie
        self._section(right, "TELEMETRIE (LIVE)")
        tcard = ctk.CTkFrame(right, fg_color=CARD, corner_radius=10)
        tcard.pack(fill="x", padx=8, pady=(0, 8))
        ti = ctk.CTkFrame(tcard, fg_color="transparent")
        ti.pack(fill="x", padx=10, pady=8)
        self._tele: dict[str, ctk.CTkLabel] = {}
        for key, label in [
            ("cmd",   "Nav-Befehl"),
            ("dist",  "Distanz zum Ziel"),
            ("speed", "Geschwindigkeit"),
            ("pitch", "Pitch (Nicken)"),
            ("roll",  "Roll (Rollen)"),
            ("vx",    "vx  (px/s)"),
            ("vy",    "vy  (px/s)"),
            ("pos",   "Position"),
        ]:
            r = ctk.CTkFrame(ti, fg_color="transparent")
            r.pack(fill="x", pady=2)
            ctk.CTkLabel(r, text=label, font=ctk.CTkFont("Segoe UI", 11),
                         text_color=MUTED, anchor="w", width=130).pack(side="left")
            lbl = ctk.CTkLabel(r, text="–", font=ctk.CTkFont("Segoe UI", 12, "bold"),
                               text_color=TEXT, anchor="e")
            lbl.pack(side="right")
            self._tele[key] = lbl

        # Trim-Visualisierung
        self._section(right, "TRIM-ZUSTAND")
        tbar = ctk.CTkFrame(right, fg_color=CARD, corner_radius=10)
        tbar.pack(fill="x", padx=8, pady=(0, 8))
        tb = ctk.CTkFrame(tbar, fg_color="transparent")
        tb.pack(fill="x", padx=10, pady=8)
        ctk.CTkLabel(tb, text="Nicken  (VOR ← 0 → ZURÜCK)",
                     font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED).pack(anchor="w")
        self._pitch_bar = ctk.CTkProgressBar(tb, height=12, corner_radius=4)
        self._pitch_bar.set(0.5)
        self._pitch_bar.pack(fill="x", pady=(2, 8))
        ctk.CTkLabel(tb, text="Rollen  (LINKS ← 0 → RECHTS)",
                     font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED).pack(anchor="w")
        self._roll_bar = ctk.CTkProgressBar(tb, height=12, corner_radius=4)
        self._roll_bar.set(0.5)
        self._roll_bar.pack(fill="x", pady=(2, 4))
        self._trim_val = ctk.CTkLabel(tb, text="Pitch: 0.000  Roll: 0.000",
                                       font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED)
        self._trim_val.pack(anchor="w", pady=(4, 0))

        # Physik-Slider
        self._section(right, "PHYSIK-PARAMETER  (live anpassbar)")
        pcard = ctk.CTkFrame(right, fg_color=CARD, corner_radius=10)
        pcard.pack(fill="x", padx=8, pady=(0, 8))
        pc = ctk.CTkFrame(pcard, fg_color="transparent")
        pc.pack(fill="x", padx=10, pady=8)
        for key, label, mn, mx in [
            ("ACCEL_SCALE", "Beschleunigung (px/s²/rad)", 100, 2500),
            ("DRAG",        "Luftwiderstand (1/s)",         0.3,  5.0),
            ("TRIM_STEP",   "Trim-Schritt (rad/Befehl)",   0.005, 0.06),
        ]:
            ctk.CTkLabel(pc, text=label, font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED).pack(anchor="w")
            srow = ctk.CTkFrame(pc, fg_color="transparent")
            srow.pack(fill="x", pady=(1, 8))
            val_lbl = ctk.CTkLabel(srow, text=f"{_SIM[key]:.3f}",
                                   font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color=TEXT, width=52)
            val_lbl.pack(side="right")
            s = ctk.CTkSlider(srow, from_=mn, to=mx,
                              command=lambda v, k=key, l=val_lbl: self._set_param(k, v, l))
            s.set(_SIM[key])
            s.pack(side="left", fill="x", expand=True, padx=(0, 4))

        # Statistik
        self._section(right, "MISSION-STATISTIK")
        scard = ctk.CTkFrame(right, fg_color=CARD, corner_radius=10)
        scard.pack(fill="x", padx=8, pady=(0, 14))
        sc = ctk.CTkFrame(scard, fg_color="transparent")
        sc.pack(fill="x", padx=10, pady=8)
        self._stat_lbl = ctk.CTkLabel(sc, text="–", font=ctk.CTkFont("Segoe UI", 12),
                                       text_color=TEXT, justify="left")
        self._stat_lbl.pack(anchor="w")

    def _section(self, parent, title: str) -> None:
        ctk.CTkLabel(parent, text=title, font=ctk.CTkFont("Segoe UI", 10, "bold"),
                     text_color="#445566").pack(anchor="w", padx=10, pady=(10, 2))
        ctk.CTkFrame(parent, height=1, fg_color="#1a1a3a").pack(fill="x", padx=8, pady=(0, 5))

    # ── Event-Handler ─────────────────────────────────────────────────────────
    def _on_left_click(self, event) -> None:
        self._target = [float(event.x), float(event.y)]
        g["drone_target_pos"] = (event.x, event.y)

    def _on_right_click(self, event) -> None:
        if not self._mission:
            hdg = self._physics.get()["heading"]
            self._physics.reset(event.x, event.y, hdg)

    def _set_heading(self, deg: int) -> None:
        rad = math.radians(deg)
        self._physics.heading = rad
        g["drone_orientation_angle"] = rad
        self._hdg_lbl.configure(text=f"Heading: {deg}°")

    def _set_param(self, key: str, val: float, lbl: ctk.CTkLabel) -> None:
        _SIM[key] = val
        lbl.configure(text=f"{val:.3f}" if val < 1 else f"{val:.1f}")

    def _load_scenario(self, target: tuple, heading_deg: int) -> None:
        self._stop_mission()
        rad = math.radians(heading_deg)
        self._target = list(target)
        self._physics.reset(*_DRONE_START, heading=rad)
        g.update(drone_target_pos=target, drone_autonomous=False,
                 drone_reached=False, drone_orientation_angle=rad,
                 drone_nav_cmd="")
        self._stats = {"cmds": 0, "dist_min": float("inf"), "arrived": False, "t0": 0.0}
        self._hdg_lbl.configure(text=f"Heading: {heading_deg}°")
        self._btn_start.configure(text="▶  START", fg_color=GREEN)

    def _start_mission(self) -> None:
        if not self._mission:
            self._mission = True
            self._stats   = {"cmds": 0, "dist_min": float("inf"),
                             "arrived": False, "t0": time.time()}
            g.update(drone_autonomous=True, drone_reached=False,
                     state="RESCUE", drone_nav_cmd="")
            self._btn_start.configure(text="▶  AKTIV", fg_color="#004422")

    def _stop_mission(self) -> None:
        self._mission = False
        g.update(drone_autonomous=False, drone_nav_cmd="GESTOPPT")
        if _drone._pico_sender:
            _drone._pico_sender.send("STOP")
        self._btn_start.configure(text="▶  START", fg_color=GREEN)

    def _reset(self) -> None:
        self._stop_mission()
        hdg = self._physics.get()["heading"]
        self._physics.reset(*_DRONE_START, heading=hdg)
        self._target = list(_TARGET_DEF)
        g.update(drone_autonomous=False, drone_reached=False,
                 drone_nav_cmd="", drone_orientation_angle=hdg,
                 drone_target_pos=tuple(_TARGET_DEF), state="NORMAL")
        self._stats = {"cmds": 0, "dist_min": float("inf"), "arrived": False, "t0": 0.0}

    # ── Simulations-Thread (50 Hz) ────────────────────────────────────────────
    def _sim_loop(self) -> None:
        """
        Läuft in eigenem Thread.
        Schritt: Physik → ArUco-Rauschen → g[] schreiben → Statistik.
        """
        while g.get("running", False):
            t0 = time.monotonic()

            self._physics.step()
            state = self._physics.get()

            # Simulierte ArUco-Messung (mit realistischem Positionsrauschen)
            noise = _SIM["NOISE_POS"]
            gx = int(state["px"] + random.gauss(0, noise))
            gy = int(state["py"] + random.gauss(0, noise))
            g["drone_aruco_pos"] = (gx, gy)

            # Orientierungswinkel mit EMA-Glättung (wie echte detection_loop)
            raw = state["heading"] + random.gauss(0, _SIM["NOISE_ANG"])
            prev = g.get("drone_orientation_angle")
            if prev is None:
                g["drone_orientation_angle"] = raw
            else:
                diff = angle_diff(raw, prev)
                g["drone_orientation_angle"] = angle_wrap(prev + 0.25 * diff)

            # Ziel immer aktuell halten
            g["drone_target_pos"] = (int(self._target[0]), int(self._target[1]))

            # Statistik: minimale Distanz + Ankunftserkennung
            dx = self._target[0] - state["px"]
            dy = self._target[1] - state["py"]
            dist = math.hypot(dx, dy)
            if dist < self._stats.get("dist_min", float("inf")):
                self._stats["dist_min"] = dist
            if dist < TARGET_REACHED_DIST and not self._stats.get("arrived"):
                self._stats["arrived"] = True
                t_start = self._stats.get("t0", time.time())
                self._stats["t_arrive"] = time.time() - t_start

            # 50 Hz einhalten
            elapsed = time.monotonic() - t0
            rest = _DT - elapsed
            if rest > 0:
                time.sleep(rest)

    # ── UI-Render (~30 Hz via after) ───────────────────────────────────────────
    def _draw(self) -> None:
        try:
            c     = self._canvas
            state = self._physics.get()
            px, py = state["px"], state["py"]
            tx, ty = self._target
            dist   = math.hypot(tx - px, ty - py)
            hdg    = state["heading"]
            nav    = g.get("drone_nav_cmd", "")
            reached = g.get("drone_reached", False)

            c.delete("all")

            # ── Pool-Hintergrund ──────────────────────────────────────────────
            p = POOL_PAD
            c.create_rectangle(p, p, CANVAS_W-p, CANVAS_H-p,
                               fill="#04101e", outline="#0d2840", width=2)
            # Wasser-Grid
            for x in range(p, CANVAS_W-p, 40):
                c.create_line(x, p, x, CANVAS_H-p, fill="#060f1e", width=1)
            for y in range(p, CANVAS_H-p, 40):
                c.create_line(p, y, CANVAS_W-p, y, fill="#060f1e", width=1)

            # ── Alarm-Zonen um Ziel ───────────────────────────────────────────
            sr = SLOWDOWN_RADIUS
            c.create_oval(tx-sr, ty-sr, tx+sr, ty+sr,
                          outline="#1a3300", width=1, dash=(4, 6))
            tr = TARGET_REACHED_DIST
            c.create_oval(tx-tr, ty-tr, tx+tr, ty+tr,
                          outline="#006622", width=2)

            # ── Korridor (Direkt-Linie Drohne → Ziel) ────────────────────────
            c.create_line(int(px), int(py), int(tx), int(ty),
                          fill="#1a2200", width=1, dash=(3, 8))

            # ── Ziel (Person) ─────────────────────────────────────────────────
            c.create_oval(int(tx)-16, int(ty)-16, int(tx)+16, int(ty)+16,
                          fill="#330000", outline="#ff2233", width=2)
            c.create_text(int(tx), int(ty), text="🧍", font=("Segoe UI", 13))
            c.create_text(int(tx), int(ty)+22, text="ZIEL",
                          fill="#ff4455", font=("Segoe UI", 8, "bold"))

            # ── Drohnen-Trail ─────────────────────────────────────────────────
            trail = self._physics.trail
            if len(trail) > 2:
                n = len(trail)
                for i in range(1, n):
                    a = i / n
                    r = int(220 * a); g2 = int(100 * a); b2 = 0
                    col = f"#{r:02x}{g2:02x}{b2:02x}"
                    c.create_line(trail[i-1][0], trail[i-1][1],
                                  trail[i][0],   trail[i][1],
                                  fill=col, width=1)

            # ── Deadzone-Kreis ────────────────────────────────────────────────
            dp = DEADZONE_PIXELS
            c.create_oval(int(px)-dp, int(py)-dp, int(px)+dp, int(py)+dp,
                          outline="#0d1a22", width=1, dash=(2, 5))

            # ── Drohne (Körper + Heading-Pfeil) ───────────────────────────────
            cos_h = math.cos(hdg); sin_h = math.sin(hdg)
            bdy = 14

            def rot(dx_, dy_):
                return (px + cos_h*dx_ - sin_h*dy_,
                        py + sin_h*dx_ + cos_h*dy_)

            corners = [rot(-bdy, -bdy), rot(bdy, -bdy),
                       rot(bdy,  bdy),  rot(-bdy,  bdy)]
            flat = [v for pt in corners for v in pt]
            c.create_polygon(flat, fill="#10104a", outline="#2244cc", width=2)

            # Heading-Pfeil (Vorne-Markierung)
            tip  = rot(bdy + 20,  0)
            alft = rot(bdy +  5, -8)
            argt = rot(bdy +  5,  8)
            c.create_polygon(
                [tip[0], tip[1], alft[0], alft[1], argt[0], argt[1]],
                fill="#3366ff", outline="#5588ff", width=1,
            )
            # ArUco-Muster im Zentrum
            c.create_rectangle(int(px)-7, int(py)-7, int(px)+7, int(py)+7,
                               fill="#ffffff", outline="#999999", width=1)
            c.create_line(int(px)-7, int(py)-7, int(px)+7, int(py)+7,
                          fill="#000000", width=1)
            c.create_line(int(px)+7, int(py)-7, int(px)-7, int(py)+7,
                          fill="#000000", width=1)

            # ── Geschwindigkeits-Vektor ────────────────────────────────────────
            spd = state["speed"]
            if spd > 3:
                scale = min(spd * 0.4, 60) / max(spd, 0.001)
                ex = int(px + state["vx"] * scale)
                ey = int(py + state["vy"] * scale)
                c.create_line(int(px), int(py), ex, ey,
                              fill="#0099ff", width=2, arrow=tk.LAST)

            # ── Distanz-Linie + Label ──────────────────────────────────────────
            c.create_line(int(px), int(py), int(tx), int(ty),
                          fill="#223344", width=1)
            mx_, my_ = (px+tx)/2, (py+ty)/2
            c.create_text(int(mx_), int(my_)-10, text=f"{int(dist)}px",
                          fill="#445566", font=("Segoe UI", 9))

            # ── Trim-Balken (Canvas-Ecke) ─────────────────────────────────────
            bx, by0 = CANVAS_W - 100, CANVAS_H - 80
            bw, bh  = 70, 8
            mt = _SIM["MAX_TRIM"]
            for label_, trim_, yo in [("NICK", state["pitch"], 0), ("ROLL", state["roll"], 20)]:
                cy_ = by0 + yo
                c.create_rectangle(bx, cy_, bx+bw, cy_+bh, fill="#0d1020", outline="#223344")
                c.create_line(bx + bw//2, cy_, bx + bw//2, cy_+bh, fill="#334455", width=1)
                ratio = max(-1.0, min(1.0, trim_ / mt))
                fill_w = int(ratio * bw/2)
                if fill_w != 0:
                    x0 = bx + bw//2
                    col = "#ff6600" if abs(ratio) > 0.6 else "#3355bb"
                    c.create_rectangle(x0, cy_, x0 + fill_w, cy_+bh, fill=col)
                c.create_text(bx - 4, cy_ + bh//2, text=label_, fill="#334455",
                              font=("Segoe UI", 7), anchor="e")

            # ── Status-Overlay (oben links) ───────────────────────────────────
            cmd_col = "#ffaa33" if nav else "#334455"
            if reached:
                c.create_text(12, 14, text="✅  ZIEL ERREICHT!", fill=GREEN,
                              font=("Segoe UI", 14, "bold"), anchor="w")
            else:
                c.create_text(12, 14, text=f"CMD: {nav or '–'}",
                              fill=cmd_col, font=("Segoe UI", 11, "bold"), anchor="w")
            c.create_text(12, 34,
                          text=f"Dist: {int(dist)}px  |  Speed: {spd:.1f}px/s",
                          fill="#445566", font=("Segoe UI", 10), anchor="w")

            if reached and self._mission:
                c.create_text(CANVAS_W//2, CANVAS_H//2,
                              text="✅  ZIEL ERREICHT!", fill=GREEN,
                              font=("Segoe UI", 24, "bold"))

            # ── Panel-Telemetrie ──────────────────────────────────────────────
            dist_col = (GREEN if dist < TARGET_REACHED_DIST
                        else ORANGE if dist < SLOWDOWN_RADIUS else TEXT)
            pch_deg = math.degrees(state["pitch"])
            rol_deg = math.degrees(state["roll"])
            pch_col = ORANGE if abs(state["pitch"]) > mt * 0.7 else TEXT
            rol_col = ORANGE if abs(state["roll"])  > mt * 0.7 else TEXT

            self._tele["cmd"].configure(text=nav or "–",
                                        text_color=ORANGE if nav else MUTED)
            self._tele["dist"].configure(text=f"{int(dist)} px",    text_color=dist_col)
            self._tele["speed"].configure(text=f"{spd:.1f} px/s")
            self._tele["pitch"].configure(text=f"{pch_deg:+.2f}°",  text_color=pch_col)
            self._tele["roll"].configure( text=f"{rol_deg:+.2f}°",  text_color=rol_col)
            self._tele["vx"].configure(   text=f"{state['vx']:+.1f}")
            self._tele["vy"].configure(   text=f"{state['vy']:+.1f}")
            self._tele["pos"].configure(  text=f"{int(px)}, {int(py)}")

            self._pitch_bar.set(max(0.0, min(1.0, 0.5 + state["pitch"] / mt * 0.5)))
            self._roll_bar.set( max(0.0, min(1.0, 0.5 + state["roll"]  / mt * 0.5)))
            self._trim_val.configure(
                text=f"Pitch: {state['pitch']:+.3f} rad  Roll: {state['roll']:+.3f} rad")

            hdg_deg = round(math.degrees(hdg)) % 360
            self._hdg_lbl.configure(text=f"Heading: {hdg_deg}°")

            self._bar.configure(
                text=f"Drohne ({int(px)}, {int(py)})  →  Ziel ({int(tx)}, {int(ty)})  "
                     f"| Dist: {int(dist)}px  | Mission: {'AKTIV' if self._mission else 'STOP'}")

            # Statistik
            d_min  = self._stats.get("dist_min", float("inf"))
            t_arr  = self._stats.get("t_arrive")
            n_cmds = self._stats.get("cmds", 0)
            stat   = (f"Min. Distanz:   {int(d_min) if d_min < 1e8 else '–'} px\n"
                      f"Ankunft:        {'✅  ' + f'{t_arr:.1f}s' if t_arr else 'noch nicht'}\n"
                      f"Befehle gesamt: {n_cmds}")
            self._stat_lbl.configure(text=stat)

        except Exception:
            import traceback
            traceback.print_exc()
        finally:
            self.after(33, self._draw)   # ~30 Hz


# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = SimulationApp()
    app.protocol("WM_DELETE_WINDOW", lambda: (g.update(running=False), app.destroy()))
    app.mainloop()
