"""
S.A.V.E.R – manual.py
Manuelle Drohnensteuerung via PC (Tastatur + Buttons).
Starte mit:  python manual.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import threading
import time
import tkinter as tk
import customtkinter as ctk

import drone as _drone
from config import (
    PICO_IP, PICO_PORT,
    BG, PANEL, CARD, GREEN, RED, ORANGE, BLUE, TEXT, MUTED, BORDER,
)

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ── Dummy g für PicoSender-Kompatibilität ─────────────────────────────────────
_g_dummy = {"drone_pico_status": "Nicht verbunden"}
import config
config.g.update(running=True)

# ── Kommando-Konstanten ───────────────────────────────────────────────────────
CMD_TAKEOFF = "TAKEOFF"
CMD_LAND    = "LAND"
CMD_STOP    = "STOP"
CMD_VOR     = "VOR"
CMD_ZURUECK = "ZURUECK"
CMD_LINKS   = "LINKS"
CMD_RECHTS  = "RECHTS"

# Tastaturbelegung → Kommando
KEY_MAP = {
    "w":     CMD_VOR,
    "s":     CMD_ZURUECK,
    "a":     CMD_LINKS,
    "d":     CMD_RECHTS,
    "space": CMD_STOP,
    "t":     CMD_TAKEOFF,
    "l":     CMD_LAND,
}

# ═══════════════════════════════════════════════════════════════════════════════
class ManualApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("S.A.V.E.R  –  Manuelle Drohnensteuerung")
        self.geometry("560x680")
        self.resizable(False, False)
        self.configure(fg_color=BG)
        self.protocol("WM_DELETE_WINDOW", self._quit)

        self._sender    = None
        self._last_cmd  = ""
        self._connected = False
        self._held_keys = set()   # momentan gedrückte Tasten
        self._hold_job  = None    # after()-ID für Wiederholungs-Loop

        self._build_ui()
        self._connect()
        self._tick()

        # Tastatur
        self.bind("<KeyPress>",   self._on_key_press)
        self.bind("<KeyRelease>", self._on_key_release)
        self.focus_set()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Header
        hdr = ctk.CTkFrame(self, height=54, fg_color=PANEL, corner_radius=0)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        ctk.CTkLabel(hdr, text="🚁  Manuelle Drohnensteuerung",
                     font=ctk.CTkFont("Segoe UI", 18, "bold"),
                     text_color=BLUE).pack(side="left", padx=18, pady=10)
        ctk.CTkFrame(self, height=1, fg_color=BORDER, corner_radius=0).pack(fill="x")

        # Verbindungsstatus
        status_card = ctk.CTkFrame(self, fg_color=CARD, corner_radius=14,
                                   border_width=1, border_color=BORDER)
        status_card.pack(fill="x", padx=18, pady=(14, 0))
        inner = ctk.CTkFrame(status_card, fg_color="transparent")
        inner.pack(fill="x", padx=16, pady=10)
        ctk.CTkLabel(inner, text="Pico-Verbindung",
                     font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED,
                     anchor="w").pack(side="left")
        self._lbl_conn = ctk.CTkLabel(inner, text="Verbinde …",
                                       font=ctk.CTkFont("Segoe UI", 13, "bold"),
                                       text_color=ORANGE, anchor="e")
        self._lbl_conn.pack(side="right")

        # Letztes Kommando
        cmd_card = ctk.CTkFrame(self, fg_color=CARD, corner_radius=14,
                                border_width=1, border_color=BORDER)
        cmd_card.pack(fill="x", padx=18, pady=(8, 0))
        cmd_inner = ctk.CTkFrame(cmd_card, fg_color="transparent")
        cmd_inner.pack(fill="x", padx=16, pady=10)
        ctk.CTkLabel(cmd_inner, text="Letztes Kommando",
                     font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED,
                     anchor="w").pack(side="left")
        self._lbl_cmd = ctk.CTkLabel(cmd_inner, text="–",
                                      font=ctk.CTkFont("Segoe UI", 20, "bold"),
                                      text_color=TEXT, anchor="e")
        self._lbl_cmd.pack(side="right")

        # Takeoff / Land / Stop
        top_row = ctk.CTkFrame(self, fg_color="transparent")
        top_row.pack(fill="x", padx=18, pady=(18, 0))
        ctk.CTkButton(top_row, text="▲  TAKEOFF  [T]",
                      font=ctk.CTkFont("Segoe UI", 15, "bold"), height=52, corner_radius=14,
                      fg_color="#004422", hover_color="#005533", text_color="#00ff88",
                      command=lambda: self._send(CMD_TAKEOFF)).pack(side="left", expand=True,
                                                                     fill="x", padx=(0, 6))
        ctk.CTkButton(top_row, text="▼  LAND  [L]",
                      font=ctk.CTkFont("Segoe UI", 15, "bold"), height=52, corner_radius=14,
                      fg_color="#333300", hover_color="#444400", text_color="#ffee00",
                      command=lambda: self._send(CMD_LAND)).pack(side="left", expand=True,
                                                                  fill="x", padx=(6, 0))

        ctk.CTkButton(self, text="⏹  STOP  [Space]",
                      font=ctk.CTkFont("Segoe UI", 16, "bold"), height=52, corner_radius=14,
                      fg_color="#550000", hover_color="#770000", text_color="#ff4444",
                      command=lambda: self._send(CMD_STOP)).pack(fill="x", padx=18, pady=(10, 0))

        # Richtungstasten-Pad
        ctk.CTkLabel(self, text="RICHTUNG  [W A S D]",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color="#556688").pack(pady=(22, 4))

        pad = ctk.CTkFrame(self, fg_color="transparent")
        pad.pack()

        btn_cfg = dict(width=90, height=90, corner_radius=16,
                       font=ctk.CTkFont("Segoe UI", 20, "bold"),
                       fg_color=CARD, hover_color="#252550",
                       border_width=2, border_color=BORDER)

        # Reihe 1: VOR
        row1 = ctk.CTkFrame(pad, fg_color="transparent")
        row1.pack()
        ctk.CTkFrame(row1, width=90, height=90, fg_color="transparent").pack(side="left", padx=5, pady=5)
        ctk.CTkButton(row1, text="▲\nVOR", text_color=TEXT,
                      command=lambda: self._send(CMD_VOR), **btn_cfg).pack(side="left", padx=5, pady=5)
        ctk.CTkFrame(row1, width=90, height=90, fg_color="transparent").pack(side="left", padx=5, pady=5)

        # Reihe 2: LINKS / STOP / RECHTS
        row2 = ctk.CTkFrame(pad, fg_color="transparent")
        row2.pack()
        ctk.CTkButton(row2, text="◀\nLINKS", text_color=TEXT,
                      command=lambda: self._send(CMD_LINKS), **btn_cfg).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(row2, text="⏹\nSTOP",
                      fg_color="#330000", hover_color="#550000",
                      text_color="#ff4444", corner_radius=16,
                      border_width=2, border_color=BORDER,
                      width=90, height=90,
                      font=ctk.CTkFont("Segoe UI", 16, "bold"),
                      command=lambda: self._send(CMD_STOP)).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(row2, text="▶\nRECHTS", text_color=TEXT,
                      command=lambda: self._send(CMD_RECHTS), **btn_cfg).pack(side="left", padx=5, pady=5)

        # Reihe 3: ZURUECK
        row3 = ctk.CTkFrame(pad, fg_color="transparent")
        row3.pack()
        ctk.CTkFrame(row3, width=90, height=90, fg_color="transparent").pack(side="left", padx=5, pady=5)
        ctk.CTkButton(row3, text="▼\nZURÜCK", text_color=TEXT,
                      command=lambda: self._send(CMD_ZURUECK), **btn_cfg).pack(side="left", padx=5, pady=5)
        ctk.CTkFrame(row3, width=90, height=90, fg_color="transparent").pack(side="left", padx=5, pady=5)

        # Tastatur-Legende
        ctk.CTkFrame(self, height=1, fg_color=BORDER).pack(fill="x", padx=18, pady=(18, 0))
        legend = ctk.CTkFrame(self, fg_color="transparent")
        legend.pack(pady=10)
        ctk.CTkLabel(legend,
                     text="W = VOR   S = ZURÜCK   A = LINKS   D = RECHTS\n"
                          "T = TAKEOFF   L = LAND   SPACE = STOP",
                     font=ctk.CTkFont("Segoe UI", 11), text_color="#445566",
                     justify="center").pack()

    # ── Verbindung ────────────────────────────────────────────────────────────
    def _connect(self):
        self._sender = _drone.PicoSender(PICO_IP, PICO_PORT)
        _drone._pico_sender = self._sender

    # ── Kommando senden ───────────────────────────────────────────────────────
    def _send(self, cmd):
        if self._sender:
            self._sender.send(cmd)
        self._last_cmd = cmd
        self._lbl_cmd.configure(text=cmd, text_color=self._cmd_color(cmd))

    def _cmd_color(self, cmd):
        return {
            CMD_TAKEOFF: "#00ff88",
            CMD_LAND:    "#ffee00",
            CMD_STOP:    "#ff4444",
            CMD_VOR:     BLUE,
            CMD_ZURUECK: BLUE,
            CMD_LINKS:   BLUE,
            CMD_RECHTS:  BLUE,
        }.get(cmd, TEXT)

    # ── Tastatur-Handling ─────────────────────────────────────────────────────
    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in self._held_keys:
            return  # Auto-Repeat des OS ignorieren
        self._held_keys.add(key)
        cmd = KEY_MAP.get(key)
        if cmd:
            self._send(cmd)
            # Beim Gedrückthalten alle 400 ms wiederholen
            if key not in ("t", "l"):   # TAKEOFF/LAND nicht wiederholen
                self._schedule_hold(cmd)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self._held_keys.discard(key)
        cmd = KEY_MAP.get(key)
        if cmd and cmd not in (CMD_STOP, CMD_TAKEOFF, CMD_LAND, CMD_STOP):
            # Richtungstasten loslassen → STOP
            self._send(CMD_STOP)
        if self._hold_job:
            self.after_cancel(self._hold_job)
            self._hold_job = None

    def _schedule_hold(self, cmd):
        if self._hold_job:
            self.after_cancel(self._hold_job)
        def _repeat():
            if cmd in [KEY_MAP.get(k) for k in self._held_keys]:
                self._send(cmd)
                self._hold_job = self.after(350, _repeat)
            else:
                self._hold_job = None
        self._hold_job = self.after(400, _repeat)

    # ── Tick – Verbindungsstatus aktualisieren ────────────────────────────────
    def _tick(self):
        try:
            if self._sender:
                status = self._sender.get_status()
                connected = "Verbunden" in status
                if connected != self._connected:
                    self._connected = connected
                self._lbl_conn.configure(
                    text=status[:34],
                    text_color=GREEN if connected else ORANGE)
        except Exception:
            pass
        self.after(500, self._tick)

    # ── Beenden ───────────────────────────────────────────────────────────────
    def _quit(self):
        try:
            if self._sender:
                self._sender.clear_queue()
                self._sender.send(CMD_STOP)
                time.sleep(0.2)
                self._sender.stop()
        except Exception:
            pass
        config.g["running"] = False
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = ManualApp()
    app.mainloop()
