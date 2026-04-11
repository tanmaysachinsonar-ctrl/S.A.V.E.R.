"""
S.A.V.E.R â€“ main.py
Headless Entry-Point: Startet den Web-Server und Ã¶ffnet den Browser.
Die gesamte UI lÃ¤uft als Webseite unter http://localhost:8080
Starte mit:  python main.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import threading
import time
import webbrowser
import signal

import backend
from config import g, STREAM_PORT, _get_local_ip


def main():
    print("=" * 60)
    print("  S.A.V.E.R DRONE â€“ Smart Aerial View for Emergency Rescue")
    print("=" * 60)

    # Alten Prozess auf Port freigeben
    import socket as _sock
    import subprocess as _sub
    _test = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
    _test.settimeout(0.3)
    if _test.connect_ex(("127.0.0.1", STREAM_PORT)) == 0:
        print(f"  [WARNUNG] Port {STREAM_PORT} belegt – beende alten Prozess...")
        try:
            _r = _sub.run(
                ["netstat", "-ano"],
                capture_output=True, text=True
            )
            _my_pid = str(os.getpid())
            for _line in _r.stdout.splitlines():
                if f":{STREAM_PORT}" in _line and "LISTENING" in _line:
                    _pid = _line.split()[-1]
                    if _pid != _my_pid:
                        _sub.run(["taskkill", "/PID", _pid, "/F"],
                                 capture_output=True)
        except Exception as _e:
            print(f"  [WARNUNG] Konnte alten Prozess nicht beenden: {_e}")
        time.sleep(1.0)
    _test.close()

    # Web-Server starten
    threading.Thread(target=backend._start_stream_server, daemon=True).start()
    time.sleep(0.5)

    url = f"http://localhost:{STREAM_PORT}"
    lan = f"http://{_get_local_ip()}:{STREAM_PORT}"
    print(f"\n  Web-UI:      {url}")
    print(f"  LAN-Zugang:  {lan}")
    print(f"\n  Zum Beenden: Strg+C drÃ¼cken\n")

    # Browser Ã¶ffnen
    webbrowser.open(url)

    # Warte auf Strg+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SAVER] Beende System...")
        backend.stop_system()
        print("[SAVER] Beendet.")


if __name__ == "__main__":
    main()
