# main.py -- Pico W: Trim TCP server mit default high-Z (circuit offen)
# und optionaler Feedback-Überprüfung nach jedem Puls.
# CONFIGURE WIFI_SSID / WIFI_PASS and CHANNEL SETTINGS below.
import network, socket, time
from machine import Pin

# ---------- KONFIG ----------
WIFI_SSID = "AndroidAP2B9B"
WIFI_PASS = "OK-hotspot"
PORT = 12345

# Channel definitions:
# - pin: GP pin used to DRIVE the contact (will be set to output only while pulsing)
# - active_level: 0 -> active LOW (drive 0 to activate), 1 -> active HIGH (drive 1 to activate)
# - feedback_pin: optional GP pin configured as input with pull-up/down to READ whether contact is closed by controller (requires wiring)
# - normally_active: True -> Pin bleibt dauerhaft auf active_level (z.B. TAKEOFF-Taster der immer gedrückt ist)
CHANNELS = {
    "LINKS":   {"pin": 15, "active_level": 0, "feedback_pin": None},
    "RECHTS":  {"pin": 7,  "active_level": 0, "feedback_pin": None},
    "VOR":     {"pin": 2,  "active_level": 0, "feedback_pin": None},
    "ZURUECK": {"pin": 4,  "active_level": 0, "feedback_pin": None},
    # STOP now acts like trims: action on press (active LOW)
    "STOP":    {"pin": 16, "active_level": 0, "feedback_pin": None},
    # TAKEOFF: normally_active=True -> Pin ist dauerhaft HIGH (Taste gedrückt),
    # beim Befehl wird kurz losgelassen (LOW) und wieder gedrückt (HIGH)
    "TAKEOFF": {"pin": 20, "active_level": 1, "feedback_pin": None, "normally_active": True},
    "LAND":    {"pin": 26, "active_level": 0, "feedback_pin": None},
}

# Pulszeiten (ms) - anpassbar
PULSE_ON_MS = 60
PULSE_OFF_MS = 60
INTER_PULSE_MS = 80
# ----------------------------

# Helper: create objects for channel control and feedback
chan_ctrl = {}
chan_fb = {}

for name, cfg in CHANNELS.items():
    p = cfg["pin"]
    if p is not None:
        if cfg.get("normally_active", False):
            # Normally-active Pins: sofort als Output auf active_level setzen
            chan_ctrl[name] = Pin(p, Pin.OUT)
            chan_ctrl[name].value(cfg["active_level"])
        else:
            # Start in high-Z: choose pull direction according to active_level:
            # - active_level==0 -> use pull-up so idle is HIGH (open), activation pulls LOW
            # - active_level==1 -> use pull-down so idle is LOW, activation drives HIGH
            try:
                if cfg.get("active_level", 0) == 0:
                    chan_ctrl[name] = Pin(p, Pin.IN, Pin.PULL_UP)
                else:
                    try:
                        chan_ctrl[name] = Pin(p, Pin.IN, Pin.PULL_DOWN)
                    except Exception:
                        chan_ctrl[name] = Pin(p, Pin.IN)
            except Exception:
                chan_ctrl[name] = Pin(p, Pin.IN)
    else:
        chan_ctrl[name] = None

    fb = cfg.get("feedback_pin")
    if fb is not None:
        try:
            chan_fb[name] = Pin(fb, Pin.IN, Pin.PULL_UP)
        except Exception:
            chan_fb[name] = Pin(fb, Pin.IN)
    else:
        chan_fb[name] = None

# internal trim counters (just for status / STOP logic)
x_trim = 0
y_trim = 0

def set_high_z(name):
    """Set control pin to high-Z (input) with appropriate pull according to active_level.
    Normally-active Pins werden stattdessen auf active_level zurückgesetzt."""
    ctrl = chan_ctrl.get(name)
    if ctrl is None:
        return
    # Normally-active: nicht high-Z, sondern zurück auf active_level
    if CHANNELS[name].get("normally_active", False):
        set_output_level(name, CHANNELS[name]["active_level"])
        return
    pnum = CHANNELS[name]["pin"]
    try:
        if CHANNELS[name].get("active_level", 0) == 0:
            chan_ctrl[name] = Pin(pnum, Pin.IN, Pin.PULL_UP)
        else:
            try:
                chan_ctrl[name] = Pin(pnum, Pin.IN, Pin.PULL_DOWN)
            except Exception:
                chan_ctrl[name] = Pin(pnum, Pin.IN)
    except Exception:
        try:
            chan_ctrl[name] = Pin(pnum, Pin.IN)
        except Exception:
            pass

def set_output_level(name, level):
    """Set control pin as output and drive to level (0 or 1)."""
    ctrl = chan_ctrl.get(name)
    if ctrl is None:
        return False
    pnum = CHANNELS[name]["pin"]
    try:
        chan_ctrl[name] = Pin(pnum, Pin.OUT)
        chan_ctrl[name].value(level)
        return True
    except Exception:
        return False

def read_feedback(name):
    """Read feedback pin if configured. Returns None if no feedback pin."""
    fb = chan_fb.get(name)
    if fb is None:
        return None
    try:
        return fb.value()  # with pull-up: 1 = open, 0 = closed to GND
    except Exception:
        return None

def pulse_channel(name, count=1):
    """Perform count pulses on channel 'name'.
    Normal: set output to active_level, wait, release to high-Z.
    Normally-active: kurz LOSLASSEN (inactive), dann wieder DRÜCKEN (active)."""
    global x_trim, y_trim
    if name not in CHANNELS:
        return False, "UNKNOWN_CHANNEL"
    ctrl_cfg = CHANNELS[name]
    if ctrl_cfg["pin"] is None:
        return False, "NO_PIN"
    active_level = ctrl_cfg.get("active_level", 0)
    normally_active = ctrl_cfg.get("normally_active", False)

    success = True
    last_fb = None
    for i in range(count):
        if normally_active:
            # Kurz LOSLASSEN (inactive) dann wieder DRÜCKEN (active)
            release_level = 1 - active_level
            ok = set_output_level(name, release_level)  # loslassen
            if not ok:
                success = False
                break
            time.sleep_ms(PULSE_ON_MS)
            set_output_level(name, active_level)         # wieder drücken
            time.sleep_ms(PULSE_OFF_MS)
        else:
            # Normal: kurz DRÜCKEN dann LOSLASSEN (high-Z)
            drive_level = 0 if active_level == 0 else 1
            ok = set_output_level(name, drive_level)
            if not ok:
                success = False
                break
            time.sleep_ms(PULSE_ON_MS)
            set_high_z(name)
            time.sleep_ms(PULSE_OFF_MS)
        time.sleep_ms(INTER_PULSE_MS)
        # after pulse, if feedback available, sample it
        fb = read_feedback(name)
        last_fb = fb
    # update internal trim counters for status (only for trim channels)
    if name == "LINKS":
        x_trim -= count
    elif name == "RECHTS":
        x_trim += count
    elif name == "VOR":
        y_trim += count
    elif name == "ZURUECK":
        y_trim -= count
    return success, last_fb

def stop_and_neutralize():
    """Legacy: neutralize trims by pulsing opposing channels.
       We keep the function for compatibility but STOP command no longer calls this:
       physical STOP pin (GPIO16) handles motor cut; only internal counters are reset on STOP.
    """
    global x_trim, y_trim
    # Horizontal
    if x_trim > 0:
        if "LINKS" in CHANNELS and CHANNELS["LINKS"]["pin"] is not None:
            pulse_channel("LINKS", x_trim)
            x_trim = 0
    elif x_trim < 0:
        if "RECHTS" in CHANNELS and CHANNELS["RECHTS"]["pin"] is not None:
            pulse_channel("RECHTS", -x_trim)
            x_trim = 0
    # Vertical
    if y_trim > 0:
        if "ZURUECK" in CHANNELS and CHANNELS["ZURUECK"]["pin"] is not None:
            pulse_channel("ZURUECK", y_trim)
            y_trim = 0
    elif y_trim < 0:
        if "VOR" in CHANNELS and CHANNELS["VOR"]["pin"] is not None:
            pulse_channel("VOR", -y_trim)
            y_trim = 0
    return True

def status_str():
    return "OK x_trim=%d y_trim=%d" % (x_trim, y_trim)

# ---------- WLAN: STA then AP fallback ----------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Versuche STA-Verbindung...")
wlan.connect(WIFI_SSID, WIFI_PASS)
t0 = time.time()
while not wlan.isconnected() and time.time()-t0 < 25:
    time.sleep(0.5)

if wlan.isconnected():
    ip = wlan.ifconfig()[0]
    print("STA verbunden. ifconfig:", wlan.ifconfig())
else:
    print("STA fehlgeschlagen -> starte AP-Fallback")
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid="PICO_AP_TEST", authmode=0)
    t0 = time.time()
    while not ap.active() and time.time()-t0 < 5:
        time.sleep(0.2)
    ip = ap.ifconfig()[0]
    print("AP aktiv. ifconfig:", ap.ifconfig())
print("Server listening on", ip, "port", PORT)

# ---------- TCP Server ----------
addr = socket.getaddrinfo("0.0.0.0", PORT)[0][-1]
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(addr)
s.listen(1)

while True:
    try:
        cl, remote = s.accept()
        print("Client verbunden von", remote)
        cl_file = cl.makefile('rwb', 0)
        while True:
            line = cl_file.readline()
            if not line:
                break
            try:
                text = line.decode().strip()
            except:
                text = str(line)
            if not text:
                continue
            print("Empfangen:", text)
            parts = text.split()
            if len(parts) == 0:
                cl.send(b"ERR empty\n"); continue
            cmd = parts[0].upper()
            cnt = 0
            if len(parts) > 1:
                try:
                    cnt = int(parts[1])
                except:
                    cnt = 0
            if cnt == 0 and cmd not in ("STOP",):
                cnt = 1

            # Special handling for TAKEOFF
            if cmd == "TAKEOFF":
                ok, fb = pulse_channel("TAKEOFF", cnt)
                if ok:
                    if fb is None:
                        reply = (status_str() + "\n").encode()
                    else:
                        fb_text = "FB_CLOSED" if fb == 0 else "FB_OPEN"
                        reply = ((status_str() + " " + fb_text + "\n").encode())
                else:
                    reply = ("ERR pin_action_failed %s\n" % cmd).encode()

            elif cmd == "STOP":
                # Pulse the physical STOP pin to cut motors immediately.
                # Do NOT individually neutralize trim pins here — the physical STOP handles it.
                if "STOP" in CHANNELS and CHANNELS["STOP"]["pin"] is not None:
                    pulse_channel("STOP", 1)
                # Reset internal counters only
                x_trim = 0
                y_trim = 0
                reply = (status_str() + "\n").encode()

            elif cmd == "LAND":
                # LAND reserved for later - not implemented yet
                reply = ("ERR not_implemented LAND\n").encode()

            elif cmd in CHANNELS:
                ok, fb = pulse_channel(cmd, cnt)
                if ok:
                    # include feedback info if available
                    if fb is None:
                        reply = (status_str() + "\n").encode()
                    else:
                        # fb: 1 means open (pull-up reads 1), 0 means closed
                        fb_text = "FB_CLOSED" if fb == 0 else "FB_OPEN"
                        reply = ((status_str() + " " + fb_text + "\n").encode())
                else:
                    reply = ("ERR pin_action_failed %s\n" % cmd).encode()
            else:
                reply = ("ERR unknown_command %s\n" % cmd).encode()

            try:
                cl.send(reply)
            except:
                pass
        cl.close()
        print("Client getrennt")
    except Exception as e:
        print("Server-Error:", e)
        time.sleep(1)
