# S.A.V.E.R. — Smart Aerial View for Emergency Rescue

Autonomes Drohnen-Rettungssystem zur Erkennung ertrinkender Personen via Kamera + KI (YOLOv8 Pose).  
Jugend Forscht 2025/26.

---

## Aktuelle Version — Web-UI (`main` Branch) ✅

Die Haupt- und Standardversion. Läuft als lokaler Python-Server, die Oberfläche ist eine Webseite im Browser.

### Dateien
| Datei | Beschreibung |
|---|---|
| `main.py` | Einstiegspunkt – startet HTTP-Server |
| `backend.py` | Erkennungs-Loop (YOLO + ArUco), Alarm-Logik, Telegram-Bot, MJPEG-Stream |
| `drone.py` | PicoSender (TCP) + autonomer Navigations-Thread |
| `config.py` | Konstanten, globaler Zustand, Konfig-Persistenz, ArUco-Helfer |
| `web_ui.html` | Browser-UI (Setup, Live-Stream, Alarm-Buttons, Logbuch, Statistik) |
| `main_tkinter_backup.py` | Ältere tkinter-Desktop-UI (Backup, nicht aktiv genutzt) |

### Starten
```bash
python main.py
```
Dann im Browser: `http://localhost:8080`

### Voraussetzungen
```
pip install ultralytics opencv-python requests
```

---

## Alte Version — tkinter-Desktop-UI (`old-version` Branch)

Ursprüngliche Version mit tkinter-Fenster statt Browser.  
Branch: `old-version` — funktioniert, wird aber nicht mehr weiterentwickelt.

---

## Branches
| Branch | Inhalt |
|---|---|
| `main` | **Standard** – Web-UI-Version (aktuell, empfohlen) |
| `old-version` | Ältere tkinter-Version (stabil, kein Weiterentwicklung) |

## Tags
| Tag | Beschreibung |
|---|---|
| `v2.1-stable` | Web-Version: Pico-IP per UI konfigurierbar, Live-Zonen-Editor, alle Kernfunktionen stabil |

