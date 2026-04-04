"""
S.A.V.E.R – main.py
Gesamte UI (customtkinter), Entry Point.
Starte mit:  python main.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import math
import threading
import time
import traceback
import cv2
import numpy as np
import tkinter as tk
import customtkinter as ctk
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageTk
from datetime import datetime

import drone as _drone
import backend
from config import (
    g, _lock,
    PICO_IP, PICO_PORT,
    SENSITIVITY,
    _load_config, _save_config,
    _log_event, _load_log, _save_log, _log_lock,
    _tg_subscribers,
    BG, PANEL, CARD, GREEN, RED, ORANGE, BLUE, TEXT, MUTED, BORDER, LOGO_CLR, _LOGO_PATH,
    _HAS_STC, stc,
)

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ─── EVENT STYLE ──────────────────────────────────────────────────────────────
_EVT_STYLE = {
    "ALARM":               {"color": "#ff2233", "icon": "🚨", "label": "Alarm"},
    "RETTUNG_BESTAETIGT":  {"color": "#ff8800", "icon": "🚁", "label": "Rettung bestätigt"},
    "FEHLALARM":           {"color": "#888888", "icon": "❌", "label": "Fehlalarm"},
    "RETTUNG_FERTIG":      {"color": "#00cc66", "icon": "✅", "label": "Rettung abgeschlossen"},
    "SYSTEM_START":        {"color": "#0088ff", "icon": "▶",  "label": "System gestartet"},
    "SYSTEM_STOP":         {"color": "#556677", "icon": "⏹",  "label": "System gestoppt"},
}


# ═══════════════════════════════════════════════════════════════════════════════
# ─── LOGBUCH ──────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class LogbookViewer(ctk.CTkToplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("📋  S.A.V.E.R Logbuch")
        self.geometry("720x700")
        self.configure(fg_color=BG)
        self.transient(parent)
        self.grab_set()
        self._entries = _load_log()

        header = ctk.CTkFrame(self, height=56, fg_color=PANEL)
        header.pack(fill="x")
        header.pack_propagate(False)
        ctk.CTkLabel(header, text="📋  Logbuch", font=ctk.CTkFont("Segoe UI", 22, "bold"),
                     text_color=BLUE).pack(side="left", padx=18, pady=10)
        self._filter_v = ctk.StringVar(value="Alle")
        ctk.CTkOptionMenu(header, variable=self._filter_v,
                          values=["Alle","Alarme","Fehlalarme","Rettungen","System"],
                          width=160, height=32, font=ctk.CTkFont("Segoe UI", 12),
                          fg_color="#1a1a35", button_color="#252550",
                          button_hover_color="#303065", dropdown_fg_color="#1a1a35",
                          command=lambda _: self._render()).pack(side="right", padx=18, pady=12)
        ctk.CTkLabel(header, text="Filter:", font=ctk.CTkFont("Segoe UI", 12),
                     text_color=MUTED).pack(side="right")

        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        self._summary_frame = ctk.CTkFrame(self, fg_color=CARD, corner_radius=0)
        self._summary_frame.pack(fill="x")
        self._summary_inner = ctk.CTkFrame(self._summary_frame, fg_color="transparent")
        self._summary_inner.pack(fill="x", padx=18, pady=12)
        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        self._scroll = ctk.CTkScrollableFrame(self, fg_color=BG, corner_radius=0)
        self._scroll.pack(fill="both", expand=True)
        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        footer = ctk.CTkFrame(self, height=50, fg_color=PANEL)
        footer.pack(fill="x")
        footer.pack_propagate(False)
        self._count_lbl = ctk.CTkLabel(footer, text="", font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED)
        self._count_lbl.pack(side="left", padx=18)
        ctk.CTkButton(footer, text="📥  CSV Export", width=130, height=34,
                      font=ctk.CTkFont("Segoe UI", 13, "bold"),
                      fg_color="#1a1a40", hover_color="#252560", text_color="#8899cc",
                      command=self._export_csv).pack(side="right", padx=(6, 18), pady=8)
        ctk.CTkButton(footer, text="🗑  Logbuch leeren", width=140, height=34,
                      font=ctk.CTkFont("Segoe UI", 13),
                      fg_color="#440000", hover_color="#660000", text_color="#cc6666",
                      command=self._clear_log).pack(side="right", pady=8)
        self._render()

    def _build_summary(self):
        for w in self._summary_inner.winfo_children():
            w.destroy()
        today = datetime.now().strftime("%Y-%m-%d")
        today_e = [e for e in self._entries if e["ts"].startswith(today)]
        n_alarm  = sum(1 for e in today_e if e["type"] == "ALARM")
        n_false  = sum(1 for e in today_e if e["type"] == "FEHLALARM")
        n_rescue = sum(1 for e in today_e if e["type"] == "RETTUNG_FERTIG")
        title_row = ctk.CTkFrame(self._summary_inner, fg_color="transparent")
        title_row.pack(fill="x")
        ctk.CTkLabel(title_row, text=f"Heute  ({datetime.now().strftime('%d.%m.%Y')})",
                     font=ctk.CTkFont("Segoe UI", 14, "bold"), text_color=TEXT).pack(side="left")
        ctk.CTkLabel(title_row, text=f"{len(today_e)} Ereignisse",
                     font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED).pack(side="right")
        stats_row = ctk.CTkFrame(self._summary_inner, fg_color="transparent")
        stats_row.pack(fill="x", pady=(8, 0))
        for count, label, color in [(n_alarm,"Alarme","#ff2233"),(n_false,"Fehlalarme","#888888"),(n_rescue,"Rettungen","#00cc66")]:
            box = ctk.CTkFrame(stats_row, fg_color="#0e0e20", corner_radius=10,
                               border_width=1, border_color="#1a1a33")
            box.pack(side="left", expand=True, fill="x", padx=4)
            ctk.CTkLabel(box, text=str(count), font=ctk.CTkFont("Segoe UI", 28, "bold"),
                         text_color=color).pack(pady=(8, 0))
            ctk.CTkLabel(box, text=label, font=ctk.CTkFont("Segoe UI", 11),
                         text_color=MUTED).pack(pady=(0, 8))
        if n_alarm > 0:
            rate = n_false / n_alarm * 100
            rate_color = "#00cc66" if rate < 30 else ("#ff8800" if rate < 60 else "#ff2233")
            ctk.CTkLabel(self._summary_inner, text=f"Fehlalarm-Quote: {rate:.0f}%",
                         font=ctk.CTkFont("Segoe UI", 12, "bold"), text_color=rate_color).pack(anchor="w", pady=(8, 0))

    def _filtered(self):
        filt = self._filter_v.get()
        if filt == "Alle": return self._entries
        mapping = {
            "Alarme":    {"ALARM"},
            "Fehlalarme": {"FEHLALARM"},
            "Rettungen": {"RETTUNG_BESTAETIGT", "RETTUNG_FERTIG"},
            "System":    {"SYSTEM_START", "SYSTEM_STOP"},
        }
        allowed = mapping.get(filt, set())
        return [e for e in self._entries if e["type"] in allowed]

    def _render(self):
        for w in self._scroll.winfo_children():
            w.destroy()
        self._build_summary()
        entries = self._filtered()
        self._count_lbl.configure(text=f"{len(entries)} von {len(self._entries)} Einträgen")
        if not entries:
            ctk.CTkLabel(self._scroll, text="Keine Einträge vorhanden",
                         font=ctk.CTkFont("Segoe UI", 16), text_color=MUTED).pack(pady=40)
            return
        last_date = None
        for entry in reversed(entries):
            ts_str    = entry.get("ts", "")
            date_part = ts_str[:10]
            time_part = ts_str[11:19] if len(ts_str) >= 19 else ts_str[11:]
            if date_part != last_date:
                last_date = date_part
                try:
                    dt = datetime.strptime(date_part, "%Y-%m-%d")
                    nice_date = dt.strftime("%A, %d. %B %Y")
                except Exception:
                    nice_date = date_part
                sep = ctk.CTkFrame(self._scroll, fg_color="transparent")
                sep.pack(fill="x", padx=14, pady=(14, 4))
                ctk.CTkFrame(sep, height=1, fg_color="#1a1a3a").pack(fill="x", side="top")
                ctk.CTkLabel(sep, text=nice_date, font=ctk.CTkFont("Segoe UI", 11, "bold"),
                             text_color="#556688").pack(anchor="w", pady=(4, 0))
            evt   = entry.get("type", "")
            style = _EVT_STYLE.get(evt, {"color": MUTED, "icon": "•", "label": evt})
            card  = ctk.CTkFrame(self._scroll, fg_color=CARD, corner_radius=12,
                                 border_width=1, border_color="#1a1a33")
            card.pack(fill="x", padx=14, pady=3)
            inner = ctk.CTkFrame(card, fg_color="transparent")
            inner.pack(fill="x", padx=14, pady=10)
            left  = ctk.CTkFrame(inner, fg_color="transparent", width=80)
            left.pack(side="left")
            left.pack_propagate(False)
            ctk.CTkLabel(left, text=style["icon"], font=ctk.CTkFont("Segoe UI", 20)).pack(anchor="w")
            ctk.CTkLabel(left, text=time_part, font=ctk.CTkFont("Segoe UI", 13, "bold"),
                         text_color=TEXT).pack(anchor="w")
            mid = ctk.CTkFrame(inner, fg_color="transparent")
            mid.pack(side="left", fill="x", expand=True, padx=(8, 0))
            ctk.CTkLabel(mid, text=style["label"], font=ctk.CTkFont("Segoe UI", 14, "bold"),
                         text_color=style["color"]).pack(anchor="w")
            detail = entry.get("details", "")
            if detail:
                ctk.CTkLabel(mid, text=detail, font=ctk.CTkFont("Segoe UI", 12),
                             text_color="#778899").pack(anchor="w")
            right = ctk.CTkFrame(inner, fg_color="transparent")
            right.pack(side="right")
            pid = entry.get("person_id")
            if pid is not None:
                ctk.CTkLabel(right, text=f"Person #{pid}", font=ctk.CTkFont("Segoe UI", 11, "bold"),
                             text_color="#8899aa").pack(anchor="e")
            gps = entry.get("gps")
            if gps:
                ctk.CTkLabel(right, text=f"📍 {gps['lat']:.5f}, {gps['lon']:.5f}",
                             font=ctk.CTkFont("Segoe UI", 10), text_color="#556677").pack(anchor="e")
            pc = entry.get("persons_count")
            if pc is not None:
                ctk.CTkLabel(right, text=f"👥 {pc} Personen",
                             font=ctk.CTkFont("Segoe UI", 10), text_color="#445566").pack(anchor="e")

    def _export_csv(self):
        try:
            from tkinter import filedialog
            path = filedialog.asksaveasfilename(
                parent=self, title="Logbuch exportieren",
                defaultextension=".csv",
                filetypes=[("CSV-Datei", "*.csv"), ("Alle Dateien", "*.*")],
                initialfile=f"saver_logbuch_{datetime.now().strftime('%Y%m%d')}.csv")
            if not path: return
            with open(path, "w", encoding="utf-8-sig") as f:
                f.write("Zeitstempel;Typ;Details;Person-ID;GPS-Lat;GPS-Lon;Personen im Bild\n")
                for e in self._entries:
                    gps = e.get("gps", {})
                    f.write(f"{e.get('ts','')};{e.get('type','')};{e.get('details','')};"
                            f"{e.get('person_id','')};{gps.get('lat','')};{gps.get('lon','')};"
                            f"{e.get('persons_count','')}\n")
        except Exception:
            traceback.print_exc()

    def _clear_log(self):
        confirm = ctk.CTkToplevel(self)
        confirm.title("Logbuch leeren?")
        confirm.geometry("360x160")
        confirm.configure(fg_color=BG)
        confirm.transient(self)
        confirm.grab_set()
        ctk.CTkLabel(confirm, text="⚠  Gesamtes Logbuch löschen?",
                     font=ctk.CTkFont("Segoe UI", 16, "bold"), text_color="#ff8800").pack(pady=(20, 4))
        ctk.CTkLabel(confirm, text="Diese Aktion kann nicht rückgängig gemacht werden.",
                     font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED).pack()
        row = ctk.CTkFrame(confirm, fg_color="transparent")
        row.pack(pady=16)
        def do_clear():
            with _log_lock: _save_log([])
            self._entries = []
            self._render()
            confirm.grab_release()
            confirm.destroy()
        ctk.CTkButton(row, text="Ja, löschen", width=120, height=36,
                      fg_color="#660000", hover_color="#880000", text_color="#ffffff",
                      command=do_clear).pack(side="left", padx=6)
        ctk.CTkButton(row, text="Abbrechen", width=120, height=36,
                      fg_color=CARD, hover_color="#1e1e3a", text_color="#8899aa",
                      command=lambda: (confirm.grab_release(), confirm.destroy())).pack(side="left", padx=6)


# ═══════════════════════════════════════════════════════════════════════════════
# ─── STATISTIK-DASHBOARD ──────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class StatsDashboard(ctk.CTkToplevel):
    _RANGES = ["Heute", "7 Tage", "30 Tage", "Gesamt"]

    def __init__(self, parent):
        super().__init__(parent)
        self.title("📊  S.A.V.E.R Statistik")
        self.geometry("880x780")
        self.configure(fg_color=BG)
        self.transient(parent)
        self.grab_set()
        self._all_entries = _load_log()

        hdr = ctk.CTkFrame(self, height=56, fg_color=PANEL)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        ctk.CTkLabel(hdr, text="📊  Statistik-Dashboard", font=ctk.CTkFont("Segoe UI", 22, "bold"),
                     text_color=BLUE).pack(side="left", padx=18, pady=10)
        self._range_v = ctk.StringVar(value="7 Tage")
        ctk.CTkOptionMenu(hdr, variable=self._range_v, values=self._RANGES, width=140, height=32,
                          font=ctk.CTkFont("Segoe UI", 12), fg_color="#1a1a35", button_color="#252550",
                          button_hover_color="#303065", dropdown_fg_color="#1a1a35",
                          command=lambda _: self._refresh()).pack(side="right", padx=18, pady=12)
        ctk.CTkLabel(hdr, text="Zeitraum:", font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED).pack(side="right")
        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        self._kpi_frame = ctk.CTkFrame(self, fg_color=CARD, corner_radius=0)
        self._kpi_frame.pack(fill="x")
        self._kpi_inner = ctk.CTkFrame(self._kpi_frame, fg_color="transparent")
        self._kpi_inner.pack(fill="x", padx=18, pady=14)
        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        self._scroll = ctk.CTkScrollableFrame(self, fg_color=BG, corner_radius=0)
        self._scroll.pack(fill="both", expand=True)
        ctk.CTkFrame(self, height=1, fg_color="#1a1a3a").pack(fill="x")
        footer = ctk.CTkFrame(self, height=50, fg_color=PANEL)
        footer.pack(fill="x")
        footer.pack_propagate(False)
        ctk.CTkButton(footer, text="🗑  Statistik zurücksetzen", width=200, height=34,
                      font=ctk.CTkFont("Segoe UI", 13), fg_color="#440000", hover_color="#660000",
                      text_color="#cc6666", command=self._reset_stats).pack(side="right", padx=18, pady=8)
        self._refresh()

    def _filtered(self):
        import datetime as dt_module
        rng = self._range_v.get()
        now = datetime.now()
        entries = self._all_entries
        if rng == "Heute":
            tag = now.strftime("%Y-%m-%d")
            entries = [e for e in entries if e.get("ts", "").startswith(tag)]
        elif rng == "7 Tage":
            cutoff = (now - dt_module.timedelta(days=7)).strftime("%Y-%m-%d %H:%M:%S")
            entries = [e for e in entries if e.get("ts", "") >= cutoff]
        elif rng == "30 Tage":
            cutoff = (now - dt_module.timedelta(days=30)).strftime("%Y-%m-%d %H:%M:%S")
            entries = [e for e in entries if e.get("ts", "") >= cutoff]
        return entries

    def _refresh(self):
        for w in self._kpi_inner.winfo_children(): w.destroy()
        for w in self._scroll.winfo_children():    w.destroy()
        entries     = self._filtered()
        n_alarm     = sum(1 for e in entries if e["type"] == "ALARM")
        n_false     = sum(1 for e in entries if e["type"] == "FEHLALARM")
        n_rescue    = sum(1 for e in entries if e["type"] == "RETTUNG_FERTIG")
        n_confirmed = sum(1 for e in entries if e["type"] == "RETTUNG_BESTAETIGT")
        persons_list = [e.get("persons_count", 0) for e in entries
                        if e["type"] == "ALARM" and e.get("persons_count")]
        avg_persons = (sum(persons_list) / len(persons_list)) if persons_list else 0
        starts = sorted([e["ts"] for e in self._all_entries if e["type"] == "SYSTEM_START"])
        stops  = sorted([e["ts"] for e in self._all_entries if e["type"] == "SYSTEM_STOP"])
        total_hours = 0.0
        for s in starts:
            s_dt = datetime.strptime(s, "%Y-%m-%d %H:%M:%S")
            matching_stop = None
            for st in stops:
                st_dt = datetime.strptime(st, "%Y-%m-%d %H:%M:%S")
                if st_dt > s_dt:
                    matching_stop = st_dt
                    break
            if matching_stop:
                total_hours += (matching_stop - s_dt).total_seconds() / 3600
            else:
                total_hours += (datetime.now() - s_dt).total_seconds() / 3600
        false_rate = (n_false / n_alarm * 100) if n_alarm > 0 else 0
        rate_color = "#00cc66" if false_rate < 30 else (ORANGE if false_rate < 60 else RED)
        kpis = [
            (str(n_alarm),           "Alarme",          RED),
            (f"{false_rate:.0f}%",   "Fehlalarm-Quote", rate_color),
            (str(n_rescue),          "Rettungen",        GREEN),
            (f"{avg_persons:.1f}",   "⌀ Personen",       BLUE),
            (f"{total_hours:.1f}h",  "Betriebszeit",     "#8888cc"),
        ]
        for val, label, color in kpis:
            box = ctk.CTkFrame(self._kpi_inner, fg_color="#0e0e20", corner_radius=12,
                               border_width=1, border_color="#1a1a33")
            box.pack(side="left", expand=True, fill="x", padx=4)
            ctk.CTkLabel(box, text=val, font=ctk.CTkFont("Segoe UI", 30, "bold"), text_color=color).pack(pady=(10, 0))
            ctk.CTkLabel(box, text=label, font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED).pack(pady=(0, 10))
        self._draw_bar_chart(entries)
        self._draw_pie_chart(n_alarm, n_false, n_rescue, n_confirmed)

    def _draw_bar_chart(self, entries):
        from collections import Counter
        alarm_e = [e for e in entries if e["type"] in ("ALARM", "FEHLALARM", "RETTUNG_FERTIG")]
        if not alarm_e:
            ctk.CTkLabel(self._scroll, text="Keine Alarm-Daten vorhanden",
                         font=ctk.CTkFont("Segoe UI", 14), text_color=MUTED).pack(pady=20)
            return
        counts = Counter(e["ts"][:10] for e in alarm_e)
        dates  = sorted(counts.keys())
        vals   = [counts[d] for d in dates]
        fig, ax = plt.subplots(figsize=(7, 3), facecolor="#0e0e20")
        ax.set_facecolor("#0e0e20")
        ax.bar(range(len(dates)), vals, color=RED, alpha=0.8)
        ax.set_xticks(range(len(dates)))
        ax.set_xticklabels([d[5:] for d in dates], rotation=45, ha="right", color="#778899", fontsize=8)
        ax.tick_params(colors="#778899")
        for spine in ax.spines.values(): spine.set_edgecolor("#1a1a3a")
        ax.set_title("Alarme pro Tag", color=TEXT, fontsize=11)
        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, master=self._scroll)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="x", padx=14, pady=8)
        plt.close(fig)

    def _draw_pie_chart(self, n_alarm, n_false, n_rescue, n_confirmed):
        if n_alarm + n_false + n_rescue == 0:
            return
        labels  = ["Alarme", "Fehlalarme", "Rettungen bestätigt", "Rettungen fertig"]
        vals    = [max(0, n_alarm - n_false), n_false, n_confirmed, n_rescue]
        colors  = [RED, "#555555", ORANGE, GREEN]
        non_zero = [(l, v, c) for l, v, c in zip(labels, vals, colors) if v > 0]
        if not non_zero: return
        labels2, vals2, colors2 = zip(*non_zero)
        fig, ax = plt.subplots(figsize=(5, 3), facecolor="#0e0e20")
        ax.set_facecolor("#0e0e20")
        ax.pie(vals2, labels=labels2, colors=colors2, autopct="%1.0f%%",
               textprops={"color": TEXT, "fontsize": 9})
        ax.set_title("Alarm-Verteilung", color=TEXT, fontsize=11)
        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, master=self._scroll)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="x", padx=14, pady=8)
        plt.close(fig)

    def _reset_stats(self):
        confirm = ctk.CTkToplevel(self)
        confirm.title("Statistik zurücksetzen?")
        confirm.geometry("360x160")
        confirm.configure(fg_color=BG)
        confirm.transient(self)
        confirm.grab_set()
        ctk.CTkLabel(confirm, text="⚠  Gesamte Statistik löschen?",
                     font=ctk.CTkFont("Segoe UI", 16, "bold"), text_color="#ff8800").pack(pady=(20, 4))
        ctk.CTkLabel(confirm, text="Diese Aktion kann nicht rückgängig gemacht werden.",
                     font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED).pack()
        row = ctk.CTkFrame(confirm, fg_color="transparent")
        row.pack(pady=16)
        def do_reset():
            with _log_lock: _save_log([])
            self._all_entries = []
            self._refresh()
            confirm.grab_release()
            confirm.destroy()
        ctk.CTkButton(row, text="Ja, löschen", width=120, height=36,
                      fg_color="#660000", hover_color="#880000", text_color="#ffffff",
                      command=do_reset).pack(side="left", padx=6)
        ctk.CTkButton(row, text="Abbrechen", width=120, height=36,
                      fg_color=CARD, hover_color="#1e1e3a", text_color="#8899aa",
                      command=lambda: (confirm.grab_release(), confirm.destroy())).pack(side="left", padx=6)


# ═══════════════════════════════════════════════════════════════════════════════
# ─── AUSSCHLUSS-ZONEN-EDITOR ──────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class ExclusionZoneEditor(ctk.CTkToplevel):
    def __init__(self, parent, cam_index=0, zones=None, on_save=None, still_frame=None):
        super().__init__(parent)
        self.title("🎯  Nichtwasserbereiche markieren")
        self.geometry("960x620")
        self.configure(fg_color=BG)
        self.transient(parent)
        self.grab_set()
        self._zones    = [list(z) for z in (zones or [])]
        self._on_save  = on_save
        self._drawing  = False
        self._start_x  = 0
        self._start_y  = 0
        self._current_rect = None
        self._cap      = None
        self._running  = True
        self._img_x = 0; self._img_y = 0; self._img_w = 1; self._img_h = 1
        self._still = still_frame.copy() if still_frame is not None else None

        header = ctk.CTkFrame(self, height=50, fg_color=PANEL)
        header.pack(fill="x")
        header.pack_propagate(False)
        ctk.CTkLabel(header, text="🎯  Nichtwasserbereiche markieren",
                     font=ctk.CTkFont("Segoe UI", 18, "bold"), text_color=BLUE).pack(side="left", padx=16, pady=10)
        ctk.CTkLabel(self,
                     text="Ziehe Rechtecke über Bereiche die NICHT überwacht werden sollen.  Rechtsklick = Zone löschen.",
                     font=ctk.CTkFont("Segoe UI", 12), text_color="#778899", wraplength=900).pack(pady=8)
        self._canvas = tk.Canvas(self, bg="#030310", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        self._canvas.bind("<ButtonPress-1>",   self._on_press)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<ButtonPress-3>",   self._on_right_click)
        bot = ctk.CTkFrame(self, fg_color="transparent")
        bot.pack(fill="x", padx=10, pady=(0, 10))
        self._zone_count_lbl = ctk.CTkLabel(bot, text=self._zone_text(),
                                             font=ctk.CTkFont("Segoe UI", 13), text_color=MUTED)
        self._zone_count_lbl.pack(side="left")
        ctk.CTkButton(bot, text="Alle löschen", width=120, height=36,
                      fg_color="#440000", hover_color="#660000", text_color="#ffffff",
                      command=self._clear_zones).pack(side="left", padx=10)
        ctk.CTkButton(bot, text="✓  Fertig", width=160, height=44,
                      font=ctk.CTkFont("Segoe UI", 16, "bold"),
                      fg_color=GREEN, hover_color="#00aa55", text_color="#ffffff",
                      command=self._save_and_close).pack(side="right")
        self.protocol("WM_DELETE_WINDOW", self._save_and_close)
        if self._still is None:
            idx = cam_index if isinstance(cam_index, int) else {"Hauptkamera": 0, "Kamera 2": 1, "Kamera 3": 2}.get(cam_index, 0)
            self._cap = cv2.VideoCapture(idx)
        self.after(100, self._update_frame)

    def _zone_text(self):
        n = len(self._zones)
        return f"{n} Zone(n) definiert" if n else "Keine Zonen definiert"

    def _get_frame(self):
        if self._still is not None: return self._still.copy()
        if self._cap is not None and self._cap.isOpened():
            ret, fr = self._cap.read()
            if ret: return fr
        return None

    def _update_frame(self):
        if not self._running: return
        frame = self._get_frame()
        if frame is None:
            if not hasattr(self, "_no_cam"):
                self._no_cam = True
                self._canvas.create_text(480, 300, text="⚠ Kamera nicht verfügbar",
                                         fill="#ff5555", font=("Segoe UI", 18))
            self.after(500, self._update_frame)
            return
        fh, fw = frame.shape[:2]
        if self._zones:
            overlay = frame.copy()
            for z in self._zones:
                cv2.rectangle(overlay, (int(z[0]*fw), int(z[1]*fh)),
                              (int(z[2]*fw), int(z[3]*fh)), (0, 0, 150), -1)
            frame = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)
            for z in self._zones:
                cv2.rectangle(frame, (int(z[0]*fw), int(z[1]*fh)),
                              (int(z[2]*fw), int(z[3]*fh)), (0, 0, 255), 2)
                cv2.putText(frame, "KEINE ERKENNUNG",
                            (int(z[0]*fw)+5, int(z[1]*fh)+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        if cw > 10 and ch > 10:
            sc = min(cw/fw, ch/fh)
            nw, nh = max(1, int(fw*sc)), max(1, int(fh*sc))
            self._img_x = (cw-nw)//2
            self._img_y = (ch-nh)//2
            self._img_w = nw
            self._img_h = nh
            resized = cv2.resize(frame, (nw, nh))
            img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)))
            self._canvas.delete("all")
            self._canvas.create_image(self._img_x, self._img_y, anchor="nw", image=img)
            self._canvas._img = img
            if self._drawing and self._current_rect:
                x1, y1, x2, y2 = self._current_rect
                self._canvas.create_rectangle(x1, y1, x2, y2, outline="#ff4444", width=2, dash=(5, 3))
        self.after(50 if self._still is None else 200, self._update_frame)

    def _canvas_to_norm(self, cx, cy):
        rx = (cx - self._img_x) / max(1, self._img_w)
        ry = (cy - self._img_y) / max(1, self._img_h)
        return max(0.0, min(1.0, rx)), max(0.0, min(1.0, ry))

    def _on_press(self, event):
        self._drawing = True; self._start_x = event.x; self._start_y = event.y; self._current_rect = None

    def _on_drag(self, event):
        if self._drawing: self._current_rect = (self._start_x, self._start_y, event.x, event.y)

    def _on_release(self, event):
        if not self._drawing: return
        self._drawing = False
        nx1, ny1 = self._canvas_to_norm(self._start_x, self._start_y)
        nx2, ny2 = self._canvas_to_norm(event.x, event.y)
        zx1, zx2 = min(nx1, nx2), max(nx1, nx2)
        zy1, zy2 = min(ny1, ny2), max(ny1, ny2)
        if (zx2-zx1) > 0.02 and (zy2-zy1) > 0.02:
            self._zones.append([zx1, zy1, zx2, zy2])
            self._zone_count_lbl.configure(text=self._zone_text())
        self._current_rect = None

    def _on_right_click(self, event):
        nx, ny = self._canvas_to_norm(event.x, event.y)
        for i, z in enumerate(self._zones):
            if z[0] <= nx <= z[2] and z[1] <= ny <= z[3]:
                self._zones.pop(i)
                self._zone_count_lbl.configure(text=self._zone_text())
                break

    def _clear_zones(self):
        self._zones.clear()
        self._zone_count_lbl.configure(text=self._zone_text())

    def _save_and_close(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        if self._on_save:
            self._on_save([tuple(z) for z in self._zones])
        self.grab_release()
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── HAUPTANWENDUNG ───────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
class SAVERApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("S.A.V.E.R DRONE  –  Smart Aerial View for Emergency Rescue")
        self.geometry("1400x860")
        self.minsize(1100, 700)
        self.configure(fg_color=BG)
        self.protocol("WM_DELETE_WINDOW", self._quit)
        self._blink         = False
        self._blink_counter = 0
        self._prev_state    = None
        self._sens          = "Mittel"
        self._pulse_on      = True
        self._settings_open = False
        self._exclusion_zones = []
        self._build_home()

    # ── CONFIG HELPERS ────────────────────────────────────────────────────────
    def _cfg_to_dict(self):
        return {
            "cam":  self._cam_v.get(),
            "sens": self._sens_v.get(),
            "use_gps": self._gps_v.get(),
            "lat1": self._lat1.get(), "lon1": self._lon1.get(),
            "lat2": self._lat2.get(), "lon2": self._lon2.get(),
            "exclusion_zones": [list(z) for z in self._exclusion_zones],
        }

    def _apply_cfg(self, cfg):
        self._cam_v.set(cfg.get("cam", "Hauptkamera"))
        self._sens_v.set(cfg.get("sens", "Mittel"))
        self._gps_v.set(cfg.get("use_gps", False))
        for key in ("lat1", "lon1", "lat2", "lon2"):
            entry = getattr(self, f"_{key}")
            entry.delete(0, "end")
            val = cfg.get(key, "")
            if val: entry.insert(0, str(val))
        self._exclusion_zones = [tuple(z) for z in cfg.get("exclusion_zones", [])]
        if hasattr(self, "_zone_lbl"):
            try:
                count = len(self._exclusion_zones)
                self._zone_lbl.configure(
                    text=f"{count} Nichtwasserbereich(e) definiert  ✓" if count > 0 else "Keine Zonen definiert",
                    text_color=GREEN if count > 0 else MUTED)
            except Exception: pass
        self._toggle_gps()

    def _open_zone_editor(self):
        cam_map = {"Hauptkamera": 0, "Kamera 2": 1, "Kamera 3": 2}
        still   = g.get("frame").copy() if g.get("running") and g.get("frame") is not None else None
        cam_name = "Hauptkamera"
        if hasattr(self, "_cam_v"):
            try: cam_name = self._cam_v.get()
            except Exception: pass
        else:
            cfg = _load_config() or {}
            cam_name = cfg.get("cam", cam_name)
        ExclusionZoneEditor(self, cam_index=cam_map.get(cam_name, 0),
                            zones=self._exclusion_zones, on_save=self._on_zones_saved,
                            still_frame=still)

    def _on_zones_saved(self, zones):
        self._exclusion_zones = zones
        if hasattr(self, "_zone_lbl"):
            try:
                count = len(zones)
                self._zone_lbl.configure(
                    text=f"{count} Nichtwasserbereich(e) definiert  ✓" if count > 0 else "Keine Zonen definiert",
                    text_color=GREEN if count > 0 else MUTED)
            except Exception: pass
        if g.get("running"):
            with _lock: g["exclusion_zones"] = [tuple(z) for z in zones]
            cfg = _load_config() or {}
            cfg["exclusion_zones"] = [list(z) for z in zones]
            _save_config(cfg)

    # ── HOME SCREEN ───────────────────────────────────────────────────────────
    def _build_home(self):
        self._clear()
        saved = _load_config()
        wrap  = ctk.CTkFrame(self, fg_color="transparent")
        wrap.place(relx=0.5, rely=0.5, anchor="center")
        try:
            _logo_img = Image.open(_LOGO_PATH)
            _w, _h = _logo_img.size
            _scale  = min(520/_w, 180/_h)
            _logo_img = _logo_img.resize((int(_w*_scale), int(_h*_scale)), Image.LANCZOS)
            self._logo_photo = ctk.CTkImage(light_image=_logo_img, dark_image=_logo_img,
                                            size=(int(_w*_scale), int(_h*_scale)))
            ctk.CTkLabel(wrap, image=self._logo_photo, text="").pack(pady=(0, 6))
        except Exception:
            ctk.CTkLabel(wrap, text="S . A . V . E . R .",
                         font=ctk.CTkFont("Segoe UI Light", 52), text_color=LOGO_CLR).pack()
        ctk.CTkLabel(wrap, text="Smart Aerial View for Emergency Rescue  ·  DRONE EDITION",
                     font=ctk.CTkFont("Segoe UI Light", 15), text_color=LOGO_CLR).pack(pady=(0, 44))
        if saved:
            info_card = ctk.CTkFrame(wrap, corner_radius=16, fg_color=CARD,
                                     border_width=1, border_color="#1e1e3a")
            info_card.pack(fill="x", pady=(0, 8))
            inner = ctk.CTkFrame(info_card, fg_color="transparent")
            inner.pack(padx=24, pady=16)
            ctk.CTkLabel(inner, text="Letzte Konfiguration:",
                         font=ctk.CTkFont("Segoe UI", 12), text_color="#556688").pack(anchor="w")
            ctk.CTkLabel(inner,
                         text=f"{saved.get('cam','Hauptkamera')}  ·  {saved.get('sens','Mittel')}  ·  {'GPS aktiv' if saved.get('use_gps') else 'Ohne GPS'}",
                         font=ctk.CTkFont("Segoe UI", 15, "bold"), text_color=TEXT).pack(anchor="w", pady=(4, 0))
            ctk.CTkButton(wrap, text="▶   STARTEN",
                          font=ctk.CTkFont("Segoe UI", 26, "bold"), height=80, corner_radius=18,
                          fg_color=GREEN, hover_color="#00aa55", text_color="#ffffff",
                          command=lambda: self._quick_start(saved)).pack(fill="x", pady=(10, 10))
            ctk.CTkButton(wrap, text="⚙   Einstellungen ändern",
                          font=ctk.CTkFont("Segoe UI", 16), height=50, corner_radius=14,
                          fg_color=CARD, hover_color="#1e1e3a", text_color="#8899aa",
                          border_width=1, border_color="#2a2a44",
                          command=self._build_setup).pack(fill="x")
        else:
            ctk.CTkLabel(wrap, text="Noch keine Konfiguration gespeichert",
                         font=ctk.CTkFont("Segoe UI", 14), text_color="#556677").pack(pady=(0, 16))
            ctk.CTkButton(wrap, text="⚙   Konfiguration einrichten",
                          font=ctk.CTkFont("Segoe UI", 22, "bold"), height=72, corner_radius=18,
                          fg_color=GREEN, hover_color="#00aa55", text_color="#ffffff",
                          command=self._build_setup).pack(fill="x")

    def _quick_start(self, cfg):
        use_gps = cfg.get("use_gps", False)
        la1 = lo1 = la2 = lo2 = 0.0
        if use_gps:
            try:
                la1 = float(cfg.get("lat1", 0)); lo1 = float(cfg.get("lon1", 0))
                la2 = float(cfg.get("lat2", 0)); lo2 = float(cfg.get("lon2", 0))
            except (ValueError, TypeError):
                self._build_setup(); return
        cam_map = {"Hauptkamera": 0, "Kamera 2": 1, "Kamera 3": 2}
        p = SENSITIVITY[cfg.get("sens", "Mittel")]
        self._sens = cfg.get("sens", "Mittel")
        self._exclusion_zones = [tuple(z) for z in cfg.get("exclusion_zones", [])]
        with _lock:
            g.update(lat1=la1, lon1=lo1, lat2=la2, lon2=lo2,
                     cam=cam_map.get(cfg.get("cam", "Hauptkamera"), 0),
                     distT=p["distT"], frames=p["frames"], radius=p["radius"],
                     running=True, t0=time.time(), state="NORMAL",
                     alarms=0, persons=0, use_gps=use_gps,
                     exclusion_zones=self._exclusion_zones)
        self._start_all_threads()
        _log_event("SYSTEM_START", f"Schnellstart – {cfg.get('cam','Hauptkamera')}, {cfg.get('sens','Mittel')}")
        self._build_main()

    # ── SETUP SCREEN ──────────────────────────────────────────────────────────
    def _build_setup(self):
        self._clear()
        wrap = ctk.CTkFrame(self, fg_color="transparent")
        wrap.place(relx=0.5, rely=0.5, anchor="center")
        ctk.CTkLabel(wrap, text="⚙  Einstellungen",
                     font=ctk.CTkFont("Segoe UI", 36, "bold"), text_color=BLUE).pack()
        ctk.CTkLabel(wrap, text="Konfiguration wird automatisch gespeichert",
                     font=ctk.CTkFont("Segoe UI", 13), text_color="#556677").pack(pady=(0, 28))
        card = ctk.CTkFrame(wrap, corner_radius=20, fg_color=CARD,
                            border_width=1, border_color="#1e1e3a", width=560)
        card.pack(fill="x")
        inner = ctk.CTkFrame(card, fg_color="transparent")
        inner.pack(fill="x", padx=32, pady=28)

        # Kamera
        ctk.CTkLabel(inner, text="KAMERA", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w")
        ctk.CTkFrame(inner, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 12))
        cam_row = ctk.CTkFrame(inner, fg_color="transparent")
        cam_row.pack(fill="x", pady=(0, 20))
        ctk.CTkLabel(cam_row, text="Videoquelle", font=ctk.CTkFont("Segoe UI", 14),
                     text_color=TEXT, width=120, anchor="w").pack(side="left")
        self._cam_v = ctk.StringVar(value="Hauptkamera")
        ctk.CTkOptionMenu(cam_row, variable=self._cam_v,
                          values=["Hauptkamera","Kamera 2","Kamera 3"],
                          width=240, height=36, font=ctk.CTkFont("Segoe UI", 13),
                          fg_color="#1a1a35", button_color="#252550",
                          button_hover_color="#303065", dropdown_fg_color="#1a1a35").pack(side="left")

        # Überwachungsbereich
        ctk.CTkFrame(inner, height=8, fg_color="transparent").pack()
        ctk.CTkLabel(inner, text="ÜBERWACHUNGSBEREICH", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w")
        ctk.CTkFrame(inner, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 10))
        ctk.CTkLabel(inner,
                     text="Markiere Bereiche die NICHT überwacht werden sollen\n(z.B. Beckenrand, Liegewiese, Gebäude)",
                     font=ctk.CTkFont("Segoe UI", 12), text_color="#667788", justify="left").pack(anchor="w", pady=(0, 8))
        zone_row = ctk.CTkFrame(inner, fg_color="transparent")
        zone_row.pack(fill="x", pady=(0, 8))
        ctk.CTkButton(zone_row, text="📷  Kameravorschau öffnen",
                      font=ctk.CTkFont("Segoe UI", 14, "bold"), height=42, corner_radius=12,
                      fg_color="#1a1a40", hover_color="#252560", text_color="#8899cc",
                      command=self._open_zone_editor).pack(side="left")
        self._zone_lbl = ctk.CTkLabel(zone_row, text="Keine Zonen definiert",
                                       font=ctk.CTkFont("Segoe UI", 12), text_color=MUTED)
        self._zone_lbl.pack(side="left", padx=14)

        # GPS
        ctk.CTkLabel(inner, text="GPS-KOORDINATEN  (optional)",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w", pady=(8, 0))
        ctk.CTkFrame(inner, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 10))
        self._gps_v = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(inner, text="GPS-Koordinaten für Standort-Erkennung verwenden",
                        variable=self._gps_v, font=ctk.CTkFont("Segoe UI", 13),
                        text_color="#778899", command=self._toggle_gps).pack(anchor="w", pady=(0, 8))
        self._collect_v = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(inner, text="Trainingsdaten sammeln (Pose-Keypoints für KI)",
                        variable=self._collect_v, font=ctk.CTkFont("Segoe UI", 13),
                        text_color="#778899").pack(anchor="w", pady=(0, 8))
        self._gps_frame = ctk.CTkFrame(inner, fg_color="transparent")
        for lbl, attr_lat, attr_lon in [("Oben-Links","_lat1","_lon1"),("Unten-Rechts","_lat2","_lon2")]:
            r = ctk.CTkFrame(self._gps_frame, fg_color="transparent")
            r.pack(fill="x", pady=3)
            ctk.CTkLabel(r, text=lbl, width=100, anchor="w", font=ctk.CTkFont("Segoe UI", 13), text_color="#667788").pack(side="left")
            ctk.CTkLabel(r, text="Lat", text_color=MUTED, font=ctk.CTkFont("Segoe UI", 11)).pack(side="left", padx=(4, 2))
            e1 = ctk.CTkEntry(r, placeholder_text="48.123456", width=140, height=32,
                              font=ctk.CTkFont("Segoe UI", 13), fg_color="#0e0e20", border_color="#252545")
            e1.pack(side="left", padx=(0, 10))
            setattr(self, attr_lat, e1)
            ctk.CTkLabel(r, text="Lon", text_color=MUTED, font=ctk.CTkFont("Segoe UI", 11)).pack(side="left", padx=(0, 2))
            e2 = ctk.CTkEntry(r, placeholder_text="11.567890", width=140, height=32,
                              font=ctk.CTkFont("Segoe UI", 13), fg_color="#0e0e20", border_color="#252545")
            e2.pack(side="left")
            setattr(self, attr_lon, e2)

        # Empfindlichkeit
        ctk.CTkFrame(inner, height=8, fg_color="transparent").pack()
        ctk.CTkLabel(inner, text="ALARM-EMPFINDLICHKEIT",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w")
        ctk.CTkFrame(inner, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 12))
        sens_row = ctk.CTkFrame(inner, fg_color="transparent")
        sens_row.pack(fill="x", pady=(0, 6))
        self._sens_v = ctk.StringVar(value="Mittel")
        for s, desc in [("Niedrig","Weniger Fehlalarme"),("Mittel","Empfohlen  ★"),("Hoch","Maximale Sicherheit")]:
            bf = ctk.CTkFrame(sens_row, fg_color="transparent")
            bf.pack(side="left", expand=True, fill="x", padx=4)
            ctk.CTkRadioButton(bf, text=s, variable=self._sens_v, value=s,
                               font=ctk.CTkFont("Segoe UI", 14, "bold"), text_color=TEXT,
                               fg_color=GREEN, border_color="#334466", hover_color="#225544").pack(anchor="w")
            ctk.CTkLabel(bf, text=desc, text_color="#556677", font=ctk.CTkFont("Segoe UI", 11)).pack(anchor="w", padx=(24, 0))

        saved = _load_config()
        if saved: self._apply_cfg(saved)

        self._err = ctk.CTkLabel(wrap, text="", text_color="#ff5555", font=ctk.CTkFont("Segoe UI", 13))
        self._err.pack(pady=(14, 0))
        btn_row = ctk.CTkFrame(wrap, fg_color="transparent")
        btn_row.pack(fill="x", pady=(14, 0))
        ctk.CTkButton(btn_row, text="▶   SPEICHERN & STARTEN",
                      font=ctk.CTkFont("Segoe UI", 20, "bold"), height=68, corner_radius=16,
                      fg_color=GREEN, hover_color="#00aa55", text_color="#ffffff",
                      command=self._start).pack(fill="x")
        ctk.CTkButton(wrap, text="←  Zurück",
                      font=ctk.CTkFont("Segoe UI", 14), height=40, corner_radius=12,
                      fg_color="transparent", hover_color="#1a1a33", text_color="#667788",
                      command=self._build_home).pack(fill="x", pady=(8, 0))

    def _toggle_gps(self):
        if self._gps_v.get():
            self._gps_frame.pack(fill="x", pady=(0, 8))
        else:
            self._gps_frame.pack_forget()

    def _start(self):
        use_gps = self._gps_v.get()
        la1 = lo1 = la2 = lo2 = 0.0
        if use_gps:
            try:
                la1 = float(self._lat1.get()); lo1 = float(self._lon1.get())
                la2 = float(self._lat2.get()); lo2 = float(self._lon2.get())
            except ValueError:
                self._err.configure(text="⚠️  Bitte alle GPS-Felder korrekt ausfüllen  (Beispiel: 48.123456)")
                return
        _save_config(self._cfg_to_dict())
        cam_map = {"Hauptkamera": 0, "Kamera 2": 1, "Kamera 3": 2}
        p = SENSITIVITY[self._sens_v.get()]
        self._sens = self._sens_v.get()
        with _lock:
            g.update(lat1=la1, lon1=lo1, lat2=la2, lon2=lo2,
                     cam=cam_map.get(self._cam_v.get(), 0),
                     distT=p["distT"], frames=p["frames"], radius=p["radius"],
                     running=True, t0=time.time(), state="NORMAL",
                     alarms=0, persons=0, use_gps=use_gps,
                     collect_training=self._collect_v.get(),
                     exclusion_zones=[tuple(z) for z in self._exclusion_zones])
        if g["collect_training"] and _HAS_STC:
            stc.init(camera=self._cam_v.get(), sensitivity=self._sens_v.get())
        self._start_all_threads()
        _log_event("SYSTEM_START", f"System gestartet – {self._cam_v.get()}, {self._sens_v.get()}")
        self._build_main()

    def _start_all_threads(self):
        if _drone._pico_sender is None:
            _drone._pico_sender = _drone.PicoSender(PICO_IP, PICO_PORT)
            print(f"[PICO] PicoSender gestartet → {PICO_IP}:{PICO_PORT}")
        threading.Thread(target=backend.detection_loop,       daemon=True).start()
        threading.Thread(target=_drone.drone_nav_loop,        daemon=True).start()
        threading.Thread(target=backend.tg_poller,            daemon=True).start()
        threading.Thread(target=backend._start_stream_server, daemon=True).start()
        threading.Thread(target=backend.escalation_loop,      daemon=True).start()

    # ── LOGBUCH / STATISTIK / EINSTELLUNGEN ───────────────────────────────────
    def _open_logbook(self): LogbookViewer(self)
    def _open_stats(self):   StatsDashboard(self)

    def _open_settings(self):
        if self._settings_open: return
        self._settings_open = True
        self._settings_win = ctk.CTkToplevel(self)
        self._settings_win.title("Einstellungen")
        self._settings_win.geometry("500x540")
        self._settings_win.configure(fg_color=BG)
        self._settings_win.transient(self)
        self._settings_win.grab_set()
        self._settings_win.protocol("WM_DELETE_WINDOW", self._close_settings)
        wrap = ctk.CTkFrame(self._settings_win, fg_color="transparent")
        wrap.pack(fill="both", expand=True, padx=24, pady=20)
        ctk.CTkLabel(wrap, text="⚙  Einstellungen",
                     font=ctk.CTkFont("Segoe UI", 24, "bold"), text_color=BLUE).pack(anchor="w")
        ctk.CTkLabel(wrap, text="Änderungen werden sofort übernommen",
                     font=ctk.CTkFont("Segoe UI", 12), text_color="#556677").pack(anchor="w", pady=(2, 18))
        ctk.CTkLabel(wrap, text="ALARM-EMPFINDLICHKEIT",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w")
        ctk.CTkFrame(wrap, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 12))
        self._set_sens_v = ctk.StringVar(value=self._sens)
        sens_row = ctk.CTkFrame(wrap, fg_color="transparent")
        sens_row.pack(fill="x", pady=(0, 18))
        for s, desc in [("Niedrig","Weniger Fehlalarme"),("Mittel","Empfohlen  ★"),("Hoch","Maximale Sicherheit")]:
            bf = ctk.CTkFrame(sens_row, fg_color="transparent")
            bf.pack(side="left", expand=True, fill="x", padx=4)
            ctk.CTkRadioButton(bf, text=s, variable=self._set_sens_v, value=s,
                               font=ctk.CTkFont("Segoe UI", 14, "bold"), text_color=TEXT,
                               fg_color=GREEN, border_color="#334466", hover_color="#225544").pack(anchor="w")
            ctk.CTkLabel(bf, text=desc, text_color="#556677",
                         font=ctk.CTkFont("Segoe UI", 11)).pack(anchor="w", padx=(24, 0))
        ctk.CTkLabel(wrap, text="KAMERA", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w", pady=(8, 0))
        ctk.CTkFrame(wrap, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(4, 10))
        saved = _load_config() or {}
        self._set_cam_v = ctk.StringVar(value=saved.get("cam", "Hauptkamera"))
        ctk.CTkOptionMenu(wrap, variable=self._set_cam_v,
                          values=["Hauptkamera","Kamera 2","Kamera 3"],
                          width=240, height=36, font=ctk.CTkFont("Segoe UI", 13),
                          fg_color="#1a1a35", button_color="#252550",
                          button_hover_color="#303065", dropdown_fg_color="#1a1a35").pack(anchor="w", pady=(0, 6))
        ctk.CTkLabel(wrap, text="Kamera-Wechsel wird beim nächsten Neustart aktiv",
                     font=ctk.CTkFont("Segoe UI", 11), text_color="#445566").pack(anchor="w")
        ctk.CTkFrame(wrap, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(18, 12))
        ctk.CTkLabel(wrap, text="ÜBERWACHUNGSBEREICH",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#556688").pack(anchor="w")
        ctk.CTkButton(wrap, text="📷  Nichtwasserbereiche bearbeiten",
                      font=ctk.CTkFont("Segoe UI", 13, "bold"), height=40, corner_radius=12,
                      fg_color="#1a1a40", hover_color="#252560", text_color="#8899cc",
                      command=self._open_zone_editor).pack(anchor="w", pady=(8, 0))
        _zc = len(self._exclusion_zones)
        ctk.CTkLabel(wrap,
                     text=f"{_zc} Nichtwasserbereich(e) definiert  ✓" if _zc > 0 else "Keine Zonen definiert",
                     font=ctk.CTkFont("Segoe UI", 11),
                     text_color=GREEN if _zc > 0 else MUTED).pack(anchor="w", pady=(4, 0))
        ctk.CTkFrame(wrap, height=1, fg_color="transparent").pack(fill="both", expand=True)
        ctk.CTkButton(wrap, text="✓   Übernehmen & Schließen",
                      font=ctk.CTkFont("Segoe UI", 16, "bold"), height=50, corner_radius=14,
                      fg_color=GREEN, hover_color="#00aa55", text_color="#ffffff",
                      command=self._apply_settings).pack(fill="x")

    def _apply_settings(self):
        new_sens = self._set_sens_v.get()
        p = SENSITIVITY[new_sens]
        with _lock: g.update(distT=p["distT"], frames=p["frames"], radius=p["radius"])
        self._sens = new_sens
        self._s_sens.configure(text=new_sens)
        cfg = _load_config() or {}
        cfg["sens"] = new_sens
        cfg["cam"]  = self._set_cam_v.get()
        _save_config(cfg)
        self._close_settings()

    def _close_settings(self):
        self._settings_open = False
        self._settings_win.grab_release()
        self._settings_win.destroy()

    # ── MAIN SCREEN ───────────────────────────────────────────────────────────
    def _build_main(self):
        self._clear()
        self._person_btns = {}
        self._anchors     = {}
        self._sens_popup  = None

        # TOP BAR
        top = ctk.CTkFrame(self, height=56, corner_radius=0, fg_color=PANEL)
        top.pack(fill="x")
        top.pack_propagate(False)
        ctk.CTkFrame(self, height=1, corner_radius=0, fg_color="#1a1a3a").pack(fill="x")
        ctk.CTkLabel(top, text="S . A . V . E . R .  DRONE",
                     font=ctk.CTkFont("Segoe UI Light", 18), text_color=LOGO_CLR).pack(side="left", padx=18, pady=10)
        self._badge = ctk.CTkLabel(top, text="●  BEREIT",
                                   font=ctk.CTkFont("Segoe UI", 14, "bold"),
                                   text_color=GREEN, fg_color="#0a2218", corner_radius=10, padx=16, pady=6)
        self._badge.pack(side="left", padx=10)
        self._clock = ctk.CTkLabel(top, text="", font=ctk.CTkFont("Segoe UI", 13), text_color="#556677")
        self._clock.pack(side="right", padx=18)
        ctk.CTkButton(top, text="⚙", width=40, height=36, font=ctk.CTkFont("Segoe UI", 18),
                      fg_color="transparent", hover_color="#1a1a33", text_color="#667788",
                      command=self._open_settings).pack(side="right", padx=(0, 6))
        ctk.CTkButton(top, text="📋", width=40, height=36, font=ctk.CTkFont("Segoe UI", 18),
                      fg_color="transparent", hover_color="#1a1a33", text_color="#667788",
                      command=self._open_logbook).pack(side="right", padx=(0, 2))
        ctk.CTkButton(top, text="📊", width=40, height=36, font=ctk.CTkFont("Segoe UI", 18),
                      fg_color="transparent", hover_color="#1a1a33", text_color="#667788",
                      command=self._open_stats).pack(side="right", padx=(0, 2))

        # CONTENT
        content = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        content.pack(fill="both", expand=True)

        side = ctk.CTkFrame(content, corner_radius=0, fg_color=PANEL, width=240)
        side.pack(side="right", fill="y")
        side.pack_propagate(False)
        ctk.CTkFrame(content, width=1, corner_radius=0, fg_color="#1a1a3a").pack(side="right", fill="y")

        vid_outer = ctk.CTkFrame(content, corner_radius=0, fg_color="#030310")
        vid_outer.pack(side="left", fill="both", expand=True)
        self._vid_outer = vid_outer

        self._vid = tk.Label(vid_outer, bg="#030310")
        self._vid.pack(fill="both", expand=True)
        self._vid.bind("<Button-1>", self._on_vid_click)

        self._vid_ph = ctk.CTkLabel(vid_outer, text="📷  Kamera wird gestartet …",
                                     font=ctk.CTkFont("Segoe UI", 20), text_color="#252540")
        self._vid_ph.place(relx=0.5, rely=0.5, anchor="center")

        self._banner = ctk.CTkFrame(vid_outer, corner_radius=0, fg_color="#2a0000")
        self._banner_lbl = ctk.CTkLabel(self._banner, text="",
                                         font=ctk.CTkFont("Segoe UI", 18, "bold"), text_color="#ff3333")
        self._banner_lbl.place(relx=0.5, rely=0.5, anchor="center")

        # SIDE PANEL
        side_inner = ctk.CTkFrame(side, fg_color="transparent")
        side_inner.pack(fill="both", expand=True, padx=12, pady=14)

        self._status_frame = ctk.CTkFrame(side_inner, corner_radius=14, fg_color="#0a2218",
                                           border_width=1, border_color="#1a3a2a")
        self._status_frame.pack(fill="x", pady=(0, 10))
        self._status_lbl = ctk.CTkLabel(self._status_frame, text="●  BEREIT",
                                         font=ctk.CTkFont("Segoe UI", 18, "bold"), text_color=GREEN)
        self._status_lbl.pack(pady=12)

        p_card = ctk.CTkFrame(side_inner, corner_radius=14, fg_color=CARD, border_width=1, border_color=BORDER)
        p_card.pack(fill="x", pady=(0, 8))
        ctk.CTkLabel(p_card, text="PERSONEN IM BILD", font=ctk.CTkFont("Segoe UI", 10, "bold"), text_color="#556688").pack(pady=(10, 2))
        self._s_persons = ctk.CTkLabel(p_card, text="0", font=ctk.CTkFont("Segoe UI", 44, "bold"), text_color=TEXT)
        self._s_persons.pack(pady=(0, 10))

        a_card = ctk.CTkFrame(side_inner, corner_radius=14, fg_color=CARD, border_width=1, border_color=BORDER)
        a_card.pack(fill="x", pady=(0, 8))
        a_row = ctk.CTkFrame(a_card, fg_color="transparent")
        a_row.pack(fill="x", padx=14, pady=10)
        ctk.CTkLabel(a_row, text="Alarme", font=ctk.CTkFont("Segoe UI", 13), text_color="#667788", anchor="w").pack(side="left")
        self._s_alarms = ctk.CTkLabel(a_row, text="0", font=ctk.CTkFont("Segoe UI", 20, "bold"), text_color=TEXT, anchor="e")
        self._s_alarms.pack(side="right")

        e_card = ctk.CTkFrame(side_inner, corner_radius=14, fg_color=CARD, border_width=1, border_color=BORDER)
        e_card.pack(fill="x", pady=(0, 8))
        e_row = ctk.CTkFrame(e_card, fg_color="transparent")
        e_row.pack(fill="x", padx=14, pady=10)
        ctk.CTkLabel(e_row, text="Empfindlichkeit", font=ctk.CTkFont("Segoe UI", 13), text_color="#667788", anchor="w").pack(side="left")
        self._s_sens = ctk.CTkLabel(e_row, text=self._sens, font=ctk.CTkFont("Segoe UI", 14, "bold"), text_color=GREEN, anchor="e")
        self._s_sens.pack(side="right")

        # DROHNEN-STATUS
        ctk.CTkFrame(side_inner, height=1, fg_color="#1a1a3a").pack(fill="x", pady=(6, 8))
        ctk.CTkLabel(side_inner, text="🚁  DROHNE", font=ctk.CTkFont("Segoe UI", 10, "bold"), text_color="#556688").pack(anchor="w")

        d_card = ctk.CTkFrame(side_inner, corner_radius=14, fg_color=CARD, border_width=1, border_color=BORDER)
        d_card.pack(fill="x", pady=(4, 6))
        d_inner = ctk.CTkFrame(d_card, fg_color="transparent")
        d_inner.pack(fill="x", padx=12, pady=8)

        dr_row = ctk.CTkFrame(d_inner, fg_color="transparent")
        dr_row.pack(fill="x", pady=(0, 4))
        ctk.CTkLabel(dr_row, text="Pico", font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED, anchor="w").pack(side="left")
        self._s_pico = ctk.CTkLabel(dr_row, text="–", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#8899aa", anchor="e")
        self._s_pico.pack(side="right")

        dm_row = ctk.CTkFrame(d_inner, fg_color="transparent")
        dm_row.pack(fill="x", pady=(0, 4))
        ctk.CTkLabel(dm_row, text="Modus", font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED, anchor="w").pack(side="left")
        self._s_drone_mode = ctk.CTkLabel(dm_row, text="BEREIT", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color=GREEN, anchor="e")
        self._s_drone_mode.pack(side="right")

        da_row = ctk.CTkFrame(d_inner, fg_color="transparent")
        da_row.pack(fill="x", pady=(0, 4))
        ctk.CTkLabel(da_row, text="ArUco", font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED, anchor="w").pack(side="left")
        self._s_aruco = ctk.CTkLabel(da_row, text="–", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color=MUTED, anchor="e")
        self._s_aruco.pack(side="right")

        dn_row = ctk.CTkFrame(d_inner, fg_color="transparent")
        dn_row.pack(fill="x")
        ctk.CTkLabel(dn_row, text="Cmd", font=ctk.CTkFont("Segoe UI", 11), text_color=MUTED, anchor="w").pack(side="left")
        self._s_nav_cmd = ctk.CTkLabel(dn_row, text="–", font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color=MUTED, anchor="e")
        self._s_nav_cmd.pack(side="right")

        ctk.CTkButton(side_inner, text="🛑  DROHNE STOPP",
                      font=ctk.CTkFont("Segoe UI", 13, "bold"), height=38, corner_radius=10,
                      fg_color="#440000", hover_color="#660000", text_color="#ff8888",
                      command=self._drone_emergency_stop).pack(fill="x", pady=(4, 0))
        ctk.CTkButton(side_inner, text="↺  ArUco Reset",
                      font=ctk.CTkFont("Segoe UI", 11), height=30, corner_radius=8,
                      fg_color="#1a1a35", hover_color="#252555", text_color="#6677aa",
                      command=lambda: g.update(drone_orientation_angle=None)).pack(fill="x", pady=(4, 0))

        tg_row = ctk.CTkFrame(side_inner, fg_color="transparent")
        tg_row.pack(fill="x", pady=(10, 0))
        ctk.CTkLabel(tg_row, text="Telegram  ✓", font=ctk.CTkFont("Segoe UI", 11), text_color="#3a6644").pack(side="left")

        # BOTTOM BAR
        self._bot = ctk.CTkFrame(self, height=90, corner_radius=0, fg_color=PANEL)
        ctk.CTkFrame(self, height=1, corner_radius=0, fg_color="#1a1a3a").pack(fill="x")
        self._bot.pack(fill="x")
        self._bot.pack_propagate(False)

        self._bot_txt = ctk.CTkLabel(self._bot,
            text="✅  System aktiv  –  Alle Personen werden überwacht",
            font=ctk.CTkFont("Segoe UI", 16), text_color=GREEN)
        self._bot_txt.place(relx=0.5, rely=0.5, anchor="center")

        self._btn_yes = ctk.CTkButton(self._bot,
            text="✅   RETTUNG BESTÄTIGEN  –  Drohne startet automatisch",
            font=ctk.CTkFont("Segoe UI", 18, "bold"), height=62, corner_radius=14,
            fg_color="#005522", hover_color="#003311", text_color="#ffffff",
            command=lambda: threading.Thread(target=backend.confirm_rescue, daemon=True).start())

        self._btn_no = ctk.CTkButton(self._bot,
            text="❌   FEHLALARM",
            font=ctk.CTkFont("Segoe UI", 18, "bold"), height=62, corner_radius=14,
            fg_color="#660000", hover_color="#440000", text_color="#ffffff",
            command=lambda: threading.Thread(target=backend.reset_alarm, daemon=True).start())

        self._btn_done = ctk.CTkButton(self._bot,
            text="🏊   RETTUNG ABGESCHLOSSEN  –  System zurücksetzen",
            font=ctk.CTkFont("Segoe UI", 18, "bold"), height=62, corner_radius=14,
            fg_color="#775500", hover_color="#553300", text_color="#ffffff",
            command=lambda: threading.Thread(target=backend.reset_alarm,
                                             args=(None, "✅ Rettung abgeschlossen – System bereit!"),
                                             daemon=True).start())

        def _confirm_awareness():
            with _lock:
                g["awareness_pending"] = False
                g["awareness_alarm"]   = False

        self._btn_aware = ctk.CTkButton(self._bot,
            text="✅   ICH BIN DA  –  Anwesenheit bestätigen",
            font=ctk.CTkFont("Segoe UI", 18, "bold"), height=62, corner_radius=14,
            fg_color="#005522", hover_color="#003311", text_color="#ffffff",
            command=_confirm_awareness)

        self._update()

    # ── NOTFALL-STOP ──────────────────────────────────────────────────────────
    def _drone_emergency_stop(self):
        if _drone._pico_sender:
            _drone._pico_sender.clear_queue()
            _drone._pico_sender.send("STOP")
        with _lock:
            g["drone_autonomous"] = False
            g["drone_nav_cmd"]    = "NOTFALL STOP"
        print("[DRONE] NOTFALL STOP via UI")

    # ── UPDATE LOOP ───────────────────────────────────────────────────────────
    def _update(self):
        try:
            state = g["state"]
            self._clock.configure(text=time.strftime("%H:%M:%S"))
            self._s_persons.configure(text=str(g["persons"]))
            self._s_alarms.configure(text=str(g["alarms"]))

            # Video-Frame anzeigen
            try:
                fr = g.get("frame")
                if fr is not None:
                    self._vid_ph.place_forget()
                    vw = self._vid.winfo_width()
                    vh = self._vid.winfo_height()
                    if vw > 10 and vh > 10:
                        fh_, fw_ = fr.shape[:2]
                        if fw_ > 0 and fh_ > 0:
                            sc = min(vw/fw_, vh/fh_)
                            nw, nh = max(1, int(fw_*sc)), max(1, int(fh_*sc))
                            img = ImageTk.PhotoImage(Image.fromarray(
                                cv2.cvtColor(cv2.resize(fr, (nw, nh)), cv2.COLOR_BGR2RGB)))
                            self._vid.configure(image=img)
                            self._vid.image = img
            except Exception:
                pass

            # Drohnen-Status aktualisieren
            try:
                sender      = _drone._pico_sender
                pico_status = sender.get_status() if sender else "–"
                self._s_pico.configure(
                    text=pico_status[:22],
                    text_color=GREEN if "Verbunden" in pico_status else "#888888")
                is_auto = g.get("drone_autonomous", False)
                reached = g.get("drone_reached", False)
                if reached:
                    drone_mode_txt, drone_mode_col = "ZIEL ERREICHT", GREEN
                elif is_auto:
                    drone_mode_txt, drone_mode_col = "AUTONOM ✈", ORANGE
                else:
                    drone_mode_txt, drone_mode_col = "BEREIT", "#6677aa"
                self._s_drone_mode.configure(text=drone_mode_txt, text_color=drone_mode_col)
                aruco_pos = g.get("drone_aruco_pos")
                aruco_ang = g.get("drone_orientation_angle")
                if aruco_pos and aruco_ang is not None:
                    self._s_aruco.configure(
                        text=f"{aruco_pos[0]},{aruco_pos[1]} | {np.degrees(aruco_ang):.0f}°",
                        text_color=GREEN)
                else:
                    self._s_aruco.configure(text="Nicht sichtbar", text_color=MUTED)
                nav_cmd = g.get("drone_nav_cmd", "")
                self._s_nav_cmd.configure(text=nav_cmd[:16] if nav_cmd else "–",
                                           text_color=ORANGE if nav_cmd else MUTED)
            except Exception:
                pass

            self._update_person_overlays()

            # State-Wechsel
            if state != self._prev_state:
                self._prev_state = state
                if   state == "NORMAL": self._show_normal()
                elif state == "ALARM":  self._enter_alarm()
                elif state == "RESCUE": self._show_rescue()

            # Alarm-Blinken
            if state == "ALARM":
                # Sicherstellen, dass Alarm-Buttons IMMER sichtbar sind
                if not self._btn_yes.winfo_ismapped():
                    self._enter_alarm()
                self._blink_counter += 1
                if self._blink_counter % 10 == 0:
                    self._blink = not self._blink
                    self.configure(fg_color="#1a0005" if self._blink else BG)
                    self._bot.configure(fg_color="#2a0008" if self._blink else "#1a0005")

            # Sicherstellen, dass Rettung-Button IMMER sichtbar ist
            if state == "RESCUE":
                if not self._btn_done.winfo_ismapped():
                    self._show_rescue()

            # Awareness-Meldung
            if g.get("awareness_pending") and not g.get("awareness_alarm") and state == "NORMAL":
                self._bot_txt.place(relx=0.5, rely=0.5, anchor="center")
                self._bot_txt.configure(text="🔔  Bitte Anwesenheit bestätigen!", text_color=ORANGE)
                self._btn_aware.place(relx=0.5, rely=0.5, anchor="center", relwidth=0.5)
            elif state == "NORMAL" and not g.get("awareness_pending"):
                if not self._btn_yes.winfo_ismapped():
                    pass

        except Exception:
            traceback.print_exc()
        finally:
            self.after(30, self._update)

    # ── STATE-ANZEIGEN ────────────────────────────────────────────────────────
    def _show_normal(self):
        self._blink = False
        self._blink_counter = 0
        self.configure(fg_color=BG)
        self._badge.configure(text="●  BEREIT", text_color=GREEN, fg_color="#0a2218")
        self._status_frame.configure(fg_color="#0a2218", border_color="#1a3a2a")
        self._status_lbl.configure(text="●  BEREIT", text_color=GREEN)
        self._banner.place_forget()
        self._bot.configure(fg_color=PANEL)
        self._bot_txt.place(relx=0.5, rely=0.5, anchor="center")
        self._bot_txt.configure(text="✅  System aktiv  –  Alle Personen werden überwacht", text_color=GREEN)
        self._btn_yes.place_forget()
        self._btn_no.place_forget()
        self._btn_done.place_forget()
        self._btn_aware.place_forget()
        self._clear_person_btns()

    def _enter_alarm(self):
        self._blink = False
        self._blink_counter = 0
        self.configure(fg_color=BG)
        pid = g.get("alarm_pid", "?")
        self._badge.configure(text="🚨  ALARM", text_color="#ffffff", fg_color="#550000")
        self._status_frame.configure(fg_color="#2a0008", border_color="#550011")
        self._status_lbl.configure(text="🚨  ALARM", text_color="#ff3333")
        self._banner.configure(fg_color="#2a0000")
        self._banner.place(x=0, y=0, relwidth=1, height=60)
        self._banner.lift()
        if g["use_gps"]:
            btxt = f"🚨  ALARM  –  Person #{pid}  –  GPS {g.get('alarm_lat',0.0):.5f}, {g.get('alarm_lon',0.0):.5f}  –  SOFORT HANDELN!"
        else:
            btxt = f"🚨  ALARM  –  Person #{pid}  –  SOFORT HANDELN!"
        self._banner_lbl.configure(text=btxt, text_color="#ff3333")
        self._bot.configure(fg_color="#1a0005")
        self._bot_txt.place_forget()
        self._btn_yes.place(relx=0.28, rely=0.5, anchor="center", relwidth=0.46)
        self._btn_no.place( relx=0.76, rely=0.5, anchor="center", relwidth=0.38)
        self._btn_done.place_forget()

    def _show_rescue(self):
        self._blink = False
        self._blink_counter = 0
        self.configure(fg_color=BG)
        self._badge.configure(text="🏊  RETTUNG LÄUFT", text_color="#ffffff", fg_color="#553300")
        self._status_frame.configure(fg_color="#1e1200", border_color="#443300")
        self._status_lbl.configure(text="🏊  RETTUNG LÄUFT", text_color=ORANGE)
        self._banner.configure(fg_color="#1e1000")
        self._banner.place(x=0, y=0, relwidth=1, height=60)
        self._banner.lift()
        self._banner_lbl.configure(
            text="🏊  RETTUNG LÄUFT  –  Drohne fliegt zur Person  –  Drücke unten wenn die Person gerettet ist",
            text_color=ORANGE)
        self._bot.configure(fg_color="#1a1000")
        self._bot_txt.place_forget()
        self._btn_yes.place_forget()
        self._btn_no.place_forget()
        self._btn_done.place(relx=0.5, rely=0.5, anchor="center", relwidth=0.7)
        self._clear_person_btns()

    # ── PERSON-OVERLAY-BUTTONS ────────────────────────────────────────────────
    def _update_person_overlays(self):
        if not hasattr(self, "_vid_outer"): return
        marked = set(g.get("marked", set()))
        state  = g.get("state")
        fr     = g.get("frame")
        mold   = g.get("mold", {})
        if state != "ALARM" or fr is None:
            if self._person_btns: self._clear_person_btns()
            return
        fh_, fw_ = fr.shape[:2]
        vw = self._vid.winfo_width()
        vh = self._vid.winfo_height()
        if vw <= 10 or vh <= 10 or fw_ <= 0 or fh_ <= 0: return
        sc    = min(vw/fw_, vh/fh_)
        nw    = int(fw_*sc)
        nh    = int(fh_*sc)
        x_off = (vw - nw) // 2
        y_off = (vh - nh) // 2
        for pid in list(self._person_btns):
            if pid not in marked:
                self._person_btns[pid].destroy()
                del self._person_btns[pid]
                self._anchors.pop(pid, None)
        for pid in marked:
            if pid in mold:
                cx, cy = mold[pid]
                ncx = cx / fw_
                ncy = cy / fh_
                if pid not in self._anchors:
                    self._anchors[pid] = (ncx, ncy)
                else:
                    ax, ay = self._anchors[pid]
                    if math.hypot(ncx-ax, ncy-ay) > 0.15:
                        self._anchors[pid] = (ncx, ncy)
            elif pid not in self._anchors:
                # Person nicht sichtbar und kein Anker → mittig platzieren
                self._anchors[pid] = (0.5, 0.4)
            ax, ay = self._anchors[pid]
            px = x_off + int(ax*nw)
            py = y_off + int(ay*nh) - 70
            py = max(64, py)
            if pid not in self._person_btns:
                panel = ctk.CTkFrame(self._vid_outer, fg_color="#0d0d22", corner_radius=10,
                                     border_width=2, border_color="#ff2233")
                ctk.CTkLabel(panel, text=f"Person #{pid}",
                             font=ctk.CTkFont("Segoe UI", 11, "bold"), text_color="#ff6666").pack(pady=(4, 2))
                row = ctk.CTkFrame(panel, fg_color="transparent")
                row.pack(padx=4, pady=(0, 4))
                ctk.CTkButton(row, text="✅", width=40, height=30, corner_radius=6,
                              fg_color="#005522", hover_color="#003311",
                              font=ctk.CTkFont(size=16),
                              command=lambda p=pid: self._confirm_person(p)).pack(side="left", padx=2)
                ctk.CTkButton(row, text="❌", width=40, height=30, corner_radius=6,
                              fg_color="#660000", hover_color="#440000",
                              font=ctk.CTkFont(size=16),
                              command=lambda p=pid: self._dismiss_person(p)).pack(side="left", padx=2)
                self._person_btns[pid] = panel
            self._person_btns[pid].place(x=px-55, y=py)
            self._person_btns[pid].lift()

    def _clear_person_btns(self):
        for pid in list(self._person_btns):
            self._person_btns[pid].destroy()
        self._person_btns.clear()
        self._anchors.clear()

    def _confirm_person(self, pid):
        if self._sens_popup is not None:
            self._sens_popup.destroy()
            self._sens_popup = None
        g["alarm_pid"] = pid
        threading.Thread(target=backend.confirm_rescue, daemon=True).start()

    def _dismiss_person(self, pid):
        if self._sens_popup is not None:
            self._sens_popup.destroy()
            self._sens_popup = None
        with _lock:
            g["marked"].discard(pid)
            g["counters"].pop(pid, None)
            g["notified"].discard(pid)
        if pid in self._person_btns:
            self._person_btns[pid].destroy()
            del self._person_btns[pid]
        self._anchors.pop(pid, None)
        _log_event("FEHLALARM", f"Person #{pid} einzeln als Fehlalarm markiert", person_id=pid)
        if g.get("collect_training") and _HAS_STC:
            stc.label("FALSE_ALARM", details=f"Person #{pid} (einzeln)")
        if not g["marked"] and g["state"] == "ALARM":
            with _lock:
                g.update(state="NORMAL", tg_sent=False, alarm_pid=None,
                         alert_cooldown_until=0.0, alarm_pending=False,
                         alarm_triggered_at=0.0, auto_dispatched=False)
                g["locked"].clear()
            for chat_id in list(_tg_subscribers):
                backend._post("sendMessage", json={"chat_id": chat_id,
                    "text": "❌ Fehlalarm – System bereit."})

    # ── VIDEO-KLICK → PERSON-SENSITIVITY ─────────────────────────────────────
    def _on_vid_click(self, event):
        if self._sens_popup is not None:
            self._sens_popup.destroy()
            self._sens_popup = None
        fr = g.get("frame")
        if fr is None: return
        fh_, fw_ = fr.shape[:2]
        vw = self._vid.winfo_width()
        vh = self._vid.winfo_height()
        if vw <= 10 or vh <= 10 or fw_ <= 0 or fh_ <= 0: return
        sc    = min(vw/fw_, vh/fh_)
        nw    = int(fw_*sc)
        nh    = int(fh_*sc)
        x_off = (vw-nw)//2
        y_off = (vh-nh)//2
        fx = (event.x - x_off) / sc
        fy = (event.y - y_off) / sc
        if fx < 0 or fy < 0 or fx > fw_ or fy > fh_: return
        mold     = g.get("mold", {})
        best_d   = float("inf")
        best_pid = None
        for tid, pos in mold.items():
            d = math.hypot(fx-pos[0], fy-pos[1])
            if d < g.get("radius", 3)*40 and d < best_d:
                best_d   = d
                best_pid = tid
        if best_pid is None: return
        if best_pid in g.get("marked", set()): return
        self._show_sensitivity_popup(best_pid, event.x, event.y)

    def _show_sensitivity_popup(self, pid, px, py):
        if self._sens_popup is not None: self._sens_popup.destroy()
        cur     = g.get("person_sensitivity", {}).get(pid, 1.0)
        panel   = ctk.CTkFrame(self._vid_outer, fg_color="#0d0d22", corner_radius=10,
                               border_width=2, border_color=BLUE)
        ctk.CTkLabel(panel, text=f"Person #{pid}",
                     font=ctk.CTkFont("Segoe UI", 12, "bold"), text_color=TEXT).pack(pady=(6, 2))
        cur_txt = "Normal" if cur == 1.0 else ("Genauer" if cur < 1.0 else "Weniger")
        ctk.CTkLabel(panel, text=f"Aktuell: {cur_txt}",
                     font=ctk.CTkFont("Segoe UI", 10), text_color=MUTED).pack(pady=(0, 4))
        def _set(mult):
            sens = g.get("person_sensitivity", {})
            if mult == 1.0: sens.pop(pid, None)
            else: sens[pid] = mult
            g["person_sensitivity"] = sens
            if self._sens_popup: self._sens_popup.destroy(); self._sens_popup = None
        ctk.CTkButton(panel, text="🔍  Genauer beobachten",
                      font=ctk.CTkFont("Segoe UI", 12), height=30,
                      fg_color="#004466", hover_color="#003355",
                      command=lambda: _set(0.5)).pack(fill="x", padx=6, pady=1)
        ctk.CTkButton(panel, text="😌  Weniger beobachten",
                      font=ctk.CTkFont("Segoe UI", 12), height=30,
                      fg_color="#444400", hover_color="#333300",
                      command=lambda: _set(2.0)).pack(fill="x", padx=6, pady=1)
        ctk.CTkButton(panel, text="↩  Normal",
                      font=ctk.CTkFont("Segoe UI", 12), height=30,
                      fg_color="#1a1a40", hover_color="#252560",
                      command=lambda: _set(1.0)).pack(fill="x", padx=6, pady=1)
        ctk.CTkButton(panel, text="✕  Schließen",
                      font=ctk.CTkFont("Segoe UI", 11), height=26,
                      fg_color="transparent", hover_color="#1a1a33", text_color=MUTED,
                      command=lambda: (panel.destroy(), setattr(self, "_sens_popup", None))).pack(fill="x", padx=6, pady=(1, 4))
        x_pos = max(0, min(px-70, self._vid.winfo_width()-150))
        y_pos = max(0, min(py-10, self._vid.winfo_height()-200))
        panel.place(x=x_pos, y=y_pos)
        panel.lift()
        self._sens_popup = panel

    # ── UTILS ─────────────────────────────────────────────────────────────────
    def _clear(self):
        for w in self.winfo_children():
            w.destroy()

    def _quit(self):
        if g.get("running"):
            _log_event("SYSTEM_STOP", "System beendet")
        g["running"] = False
        if _drone._pico_sender:
            _drone._pico_sender.clear_queue()
            _drone._pico_sender.send("STOP")
            time.sleep(0.3)
            _drone._pico_sender.stop()
        if g.get("collect_training") and _HAS_STC:
            stc.stop()
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
# ─── ENTRY POINT ──────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = SAVERApp()
    app.mainloop()
