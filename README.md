# S.A.V.E.R.
Everything for S.A.V.E.R.

## Drone Trim Control System

This repository implements a comprehensive drone trim control system. Trim control is essential for maintaining stable drone flight by compensating for hardware imbalances, manufacturing tolerances, and environmental factors.

### Features

- **Complete Trim Control Implementation**: Manage pitch, roll, yaw, and throttle trim adjustments
- **Safety Limits**: Automatic clamping to prevent excessive trim values
- **Easy Integration**: Simple API for integrating into existing drone control systems
- **Comprehensive Documentation**: Available in both English and German

### Quick Start

```python
from drone_trim_control import DroneController

# Create a drone controller with trim support
drone = DroneController()
drone.arm()

# Adjust trim to compensate for drift
drone.trim.adjust_pitch(-5)  # Compensate for forward drift
drone.trim.adjust_roll(-3)   # Compensate for right drift

# Send control commands (trim is automatically applied)
drone.send_controls(pitch=0, roll=0, yaw=0, throttle=50)
```

### Files

- **`drone_trim_control.py`** - Main implementation of trim control system
- **`TRIM_CONTROL_DOCUMENTATION.md`** - Comprehensive documentation (English/German)
- **`example_trim_usage.py`** - Interactive examples and demonstrations
- **`test_drone_trim_control.py`** - Unit tests for trim control functionality

### Running Examples

```bash
# Run the interactive demonstration
python example_trim_usage.py

# Run the basic demonstration
python drone_trim_control.py

# Run unit tests
python test_drone_trim_control.py
```

### What is Trim Control?

Trim is a fine-tuning function that adjusts the neutral position of drone controls. When a drone hovers with the control sticks centered, it should remain stationary. However, factors like:

- Unequal motor performance
- Center of gravity offset
- Aerodynamic asymmetries
- Wind influences
- Sensor calibration drift

...can cause the drone to drift. Trim adds small constant offsets to compensate for these factors, allowing stable flight without constant manual correction.

### Documentation

For detailed information about trim control theory, usage, and best practices, see:
- **[TRIM_CONTROL_DOCUMENTATION.md](TRIM_CONTROL_DOCUMENTATION.md)** - Complete guide in English and German

### Testing

The implementation includes comprehensive unit tests covering:
- Trim value setting and adjustment
- Limit enforcement
- Trim application to control inputs
- Reset functionality
- Integration with drone controller

Run tests with:
```bash
python test_drone_trim_control.py
```

### License

This project is part of S.A.V.E.R.
