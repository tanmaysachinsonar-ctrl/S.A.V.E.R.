# Drone Trim Control Documentation

## Was ist Trim-Kontrolle? (What is Trim Control?)

**Trim** ist eine Feinabstimmungsfunktion bei Drohnen, die es ermöglicht, die neutrale Position der Steuerung anzupassen. Dies kompensiert Ungleichgewichte, Fertigungstoleranzen oder Umweltfaktoren, die dazu führen können, dass die Drohne auch ohne Steuereingaben driftet.

**Trim** is a fine-tuning function in drones that allows adjusting the neutral position of controls. This compensates for imbalances, manufacturing tolerances, or environmental factors that can cause the drone to drift even without control inputs.

## Wie funktioniert Trim? (How Does Trim Work?)

### Grundprinzip (Basic Principle)

Wenn eine Drohne schwebt, sollte sie theoretisch an derselben Position bleiben, wenn alle Steuersticks in neutraler Position sind (Mitte). In der Praxis können jedoch verschiedene Faktoren dazu führen, dass die Drohne driftet:

When a drone hovers, it should theoretically remain in the same position when all control sticks are in neutral position (center). In practice, however, various factors can cause the drone to drift:

1. **Ungleiche Motorleistung** - Motors may not be perfectly matched
2. **Schwerpunktverschiebung** - Center of gravity offset
3. **Aerodynamische Asymmetrien** - Aerodynamic asymmetries
4. **Windeinflüsse** - Wind influences
5. **Sensor-Kalibrierung** - Sensor calibration issues

Trim fügt einen kleinen konstanten Offset zu den Steuersignalen hinzu, um diese Abweichungen zu korrigieren.

Trim adds a small constant offset to the control signals to correct these deviations.

## Trim-Achsen (Trim Axes)

Eine Drohne hat typischerweise vier Trim-Einstellungen:

A drone typically has four trim settings:

### 1. **Pitch Trim (Nick-Trim)**
- Steuert Vorwärts-/Rückwärtsdrift
- Controls forward/backward drift
- Positiver Wert: Drohne neigt sich nach vorne
- Positive value: Drone tilts forward
- Negativer Wert: Drohne neigt sich nach hinten
- Negative value: Drone tilts backward

### 2. **Roll Trim (Roll-Trim)**
- Steuert Links-/Rechtsdrift
- Controls left/right drift
- Positiver Wert: Drohne neigt sich nach rechts
- Positive value: Drone tilts right
- Negativer Wert: Drohne neigt sich nach links
- Negative value: Drone tilts left

### 3. **Yaw Trim (Gier-Trim)**
- Steuert Rotationsdrift
- Controls rotational drift
- Positiver Wert: Drohne dreht sich nach rechts
- Positive value: Drone rotates right
- Negativer Wert: Drohne dreht sich nach links
- Negative value: Drone rotates left

### 4. **Throttle Trim (Gas-Trim)**
- Steuert vertikale Drift (Höhe)
- Controls vertical drift (altitude)
- Positiver Wert: Drohne steigt
- Positive value: Drone climbs
- Negativer Wert: Drohne sinkt
- Negative value: Drone descends

## Trim einstellen (Setting Trim)

### Methode 1: Manuelle Einstellung (Manual Adjustment)

1. **Starten Sie die Drohne** und lassen Sie sie schweben
   - Start the drone and let it hover

2. **Beobachten Sie die Drift-Richtung**
   - Observe the drift direction

3. **Passen Sie den Trim an:**
   - Adjust the trim:
   - Driftet nach vorne → Pitch Trim verringern (rückwärts trimmen)
   - Drifts forward → Decrease pitch trim (trim backward)
   - Driftet nach rechts → Roll Trim verringern (links trimmen)
   - Drifts right → Decrease roll trim (trim left)
   - Dreht sich → Yaw Trim anpassen
   - Rotates → Adjust yaw trim

4. **Kleine Schritte verwenden**
   - Use small increments
   - Typisch: 1-5 Einheiten pro Anpassung
   - Typical: 1-5 units per adjustment

5. **Testen und wiederholen**
   - Test and repeat
   - Wiederholen Sie den Vorgang, bis die Drohne stabil schwebt
   - Repeat the process until the drone hovers stably

### Methode 2: Automatische Kalibrierung (Automatic Calibration)

Moderne Drohnen können Trim automatisch kalibrieren:

Modern drones can calibrate trim automatically:

1. Aktivieren Sie den Kalibrierungsmodus
   - Activate calibration mode

2. Lassen Sie die Drohne für 10-30 Sekunden schweben
   - Let the drone hover for 10-30 seconds

3. Das System misst die Drift und passt den Trim automatisch an
   - The system measures drift and adjusts trim automatically

## Trim in der Praxis (Trim in Practice)

### Wann sollte Trim verwendet werden? (When to Use Trim?)

✅ **Verwenden Sie Trim für:**
- Use trim for:
  - Kleine konstante Abweichungen
  - Small constant deviations
  - Systematische Drift in eine Richtung
  - Systematic drift in one direction
  - Nach dem Austausch von Komponenten
  - After replacing components

❌ **Verwenden Sie KEINEN Trim für:**
- Do NOT use trim for:
  - Starken Wind (verwenden Sie aktive Steuerung)
  - Strong wind (use active control)
  - Große Instabilitäten (überprüfen Sie Hardware)
  - Major instabilities (check hardware)
  - Temporäre Störungen
  - Temporary disturbances

### Trim-Grenzen (Trim Limits)

- Typischer Bereich: -100 bis +100 Einheiten
- Typical range: -100 to +100 units
- Wenn mehr als ±30 Einheiten benötigt werden, überprüfen Sie:
- If more than ±30 units are needed, check:
  - Mechanische Ausrichtung
  - Mechanical alignment
  - Schwerpunkt
  - Center of gravity
  - Motor-/Propeller-Zustand
  - Motor/propeller condition

## Code-Beispiel (Code Example)

```python
from drone_trim_control import TrimControl, DroneController

# Erstellen Sie einen Controller
# Create a controller
drone = DroneController()

# Drohne bewaffnen
# Arm the drone
drone.arm()

# Beobachten Sie Drift und passen Sie an
# Observe drift and adjust
# Beispiel: Drohne driftet nach vorne und rechts
# Example: Drone drifts forward and right
drone.trim.adjust_pitch(-5)  # Trimme rückwärts
drone.trim.adjust_roll(-3)   # Trimme links

# Steuerbefehle mit angewendetem Trim senden
# Send control commands with trim applied
drone.send_controls(pitch=0, roll=0, yaw=0, throttle=50)

# Alle Trim-Werte abrufen
# Get all trim values
trim_values = drone.trim.get_trim_values()
print(trim_values)

# Trim zurücksetzen
# Reset trim
drone.trim.reset_all()
```

## Best Practices

1. **Kleine Anpassungen vornehmen**
   - Make small adjustments
   - Ändern Sie den Trim schrittweise um 1-5 Einheiten
   - Change trim incrementally by 1-5 units

2. **Eine Achse nach der anderen**
   - One axis at a time
   - Passen Sie nur eine Achse an, bevor Sie zur nächsten übergehen
   - Adjust only one axis before moving to the next

3. **In ruhigen Bedingungen trimmen**
   - Trim in calm conditions
   - Vermeiden Sie Wind für beste Ergebnisse
   - Avoid wind for best results

4. **Trim-Einstellungen speichern**
   - Save trim settings
   - Dokumentieren Sie Trim-Werte für verschiedene Konfigurationen
   - Document trim values for different configurations

5. **Regelmäßig überprüfen**
   - Check regularly
   - Trim kann sich mit der Zeit ändern (Verschleiß, Akkuzustand)
   - Trim can change over time (wear, battery state)

## Fehlerbehebung (Troubleshooting)

### Problem: Drohne driftet trotz Trim
**Solution:** 
- Überprüfen Sie die Propeller auf Beschädigungen
- Check propellers for damage
- Kalibrieren Sie die IMU/Gyroskope neu
- Recalibrate IMU/gyroscopes
- Prüfen Sie den Schwerpunkt
- Check center of gravity

### Problem: Trim funktioniert nicht
**Solution:**
- Überprüfen Sie die Firmware-Einstellungen
- Check firmware settings
- Stellen Sie sicher, dass Trim aktiviert ist
- Ensure trim is enabled
- Überprüfen Sie die Trim-Grenzen
- Check trim limits

### Problem: Zu viel Trim erforderlich
**Solution:**
- Dies deutet auf ein mechanisches Problem hin
- This indicates a mechanical problem
- Überprüfen Sie die Motor-/Propeller-Montage
- Check motor/propeller mounting
- Prüfen Sie die Rahmenausrichtung
- Check frame alignment

## Technische Details (Technical Details)

### Implementierung (Implementation)

Die Trim-Kontrolle funktioniert durch Addition eines Offset-Wertes zu den Steuereingaben:

Trim control works by adding an offset value to control inputs:

```
Finale_Steuerung = Pilot_Eingabe + Trim_Wert
Final_Control = Pilot_Input + Trim_Value
```

### Trim-Persistenz (Trim Persistence)

In einer Produktionsumgebung sollten Trim-Werte:
- Im EEPROM/Flash-Speicher gespeichert werden
- Beim Neustart wiederhergestellt werden
- Pro Flugmodus konfigurierbar sein

In a production environment, trim values should:
- Be stored in EEPROM/Flash memory
- Be restored on reboot
- Be configurable per flight mode

## API-Referenz (API Reference)

### TrimControl Klasse

#### Eigenschaften (Properties)
- `pitch_trim`: Pitch trim value (-100 to 100)
- `roll_trim`: Roll trim value (-100 to 100)
- `yaw_trim`: Yaw trim value (-100 to 100)
- `throttle_trim`: Throttle trim value (-100 to 100)

#### Methoden (Methods)
- `adjust_pitch(amount)`: Adjust pitch trim
- `adjust_roll(amount)`: Adjust roll trim
- `adjust_yaw(amount)`: Adjust yaw trim
- `adjust_throttle(amount)`: Adjust throttle trim
- `reset_all()`: Reset all trim to neutral
- `apply_trim(pitch, roll, yaw, throttle)`: Apply trim to inputs
- `get_trim_values()`: Get all trim values as dict
- `set_trim_values(**kwargs)`: Set multiple trim values

## Zusammenfassung (Summary)

Trim-Kontrolle ist ein wesentliches Werkzeug für stabilen Drohnenflug. Durch kleine Anpassungen der neutralen Position können Piloten sicherstellen, dass ihre Drohne ohne ständige Korrekturen schwebt. Dies verbessert die Flugqualität und reduziert die Ermüdung des Piloten.

Trim control is an essential tool for stable drone flight. By making small adjustments to the neutral position, pilots can ensure their drone hovers without constant corrections. This improves flight quality and reduces pilot fatigue.

---

**Hinweis:** Dieses Dokument beschreibt die grundlegenden Konzepte der Drohnen-Trim-Kontrolle. Spezifische Implementierungsdetails können je nach Drohnenmodell und Firmware variieren.

**Note:** This document describes the basic concepts of drone trim control. Specific implementation details may vary depending on drone model and firmware.
