"""
Interactive example demonstrating drone trim control.

This script provides an interactive demonstration of how to use
trim control in a practical scenario.
"""

from drone_trim_control import TrimControl, DroneController


def print_separator():
    """Print a visual separator."""
    print("\n" + "=" * 60 + "\n")


def demonstrate_basic_trim():
    """Demonstrate basic trim adjustment."""
    print("DEMO 1: Basic Trim Adjustment")
    print_separator()
    
    trim = TrimControl()
    
    print("Starting with neutral trim:")
    print(trim)
    print()
    
    print("Adjusting pitch trim by -5 (backward):")
    trim.adjust_pitch(-5)
    print(f"Pitch trim: {trim.pitch_trim}")
    print()
    
    print("Adjusting roll trim by 3 (right):")
    trim.adjust_roll(3)
    print(f"Roll trim: {trim.roll_trim}")
    print()
    
    print("Final trim state:")
    print(trim)


def demonstrate_trim_application():
    """Demonstrate how trim affects control inputs."""
    print("DEMO 2: Trim Application to Control Inputs")
    print_separator()
    
    trim = TrimControl()
    
    # Set some trim values
    trim.set_trim_values(pitch=-5, roll=3, yaw=1)
    
    print("Current trim settings:")
    print(f"  Pitch: {trim.pitch_trim}")
    print(f"  Roll: {trim.roll_trim}")
    print(f"  Yaw: {trim.yaw_trim}")
    print(f"  Throttle: {trim.throttle_trim}")
    print()
    
    # Neutral stick position
    print("Control Input: Pitch=0, Roll=0, Yaw=0, Throttle=50")
    result = trim.apply_trim(0, 0, 0, 50)
    print(f"After Trim:    Pitch={result[0]}, Roll={result[1]}, "
          f"Yaw={result[2]}, Throttle={result[3]}")
    print()
    
    # With pilot input
    print("Control Input: Pitch=20, Roll=-10, Yaw=5, Throttle=60")
    result = trim.apply_trim(20, -10, 5, 60)
    print(f"After Trim:    Pitch={result[0]}, Roll={result[1]}, "
          f"Yaw={result[2]}, Throttle={result[3]}")


def demonstrate_drift_correction():
    """Demonstrate correcting drone drift with trim."""
    print("DEMO 3: Correcting Drone Drift")
    print_separator()
    
    drone = DroneController()
    drone.arm()
    
    print("Scenario: Drone is hovering but drifting")
    print()
    
    print("Step 1: Observe drift")
    print("  - Drone drifts forward slowly")
    print("  - Drone also drifts slightly to the right")
    print()
    
    print("Step 2: Apply pitch trim correction")
    print("  - Adjusting pitch trim by -5 to counteract forward drift")
    drone.trim.adjust_pitch(-5)
    print(f"  - New pitch trim: {drone.trim.pitch_trim}")
    print()
    
    print("Step 3: Apply roll trim correction")
    print("  - Adjusting roll trim by -3 to counteract right drift")
    drone.trim.adjust_roll(-3)
    print(f"  - New roll trim: {drone.trim.roll_trim}")
    print()
    
    print("Step 4: Test hover with corrected trim")
    print("  - Sending neutral controls (hover command)")
    result = drone.send_controls(0, 0, 0, 50)
    print(f"  - Actual values sent to motors: Pitch={result[0]}, Roll={result[1]}")
    print()
    
    print("Step 5: Fine-tune if needed")
    print("  - Still drifting slightly forward...")
    drone.trim.adjust_pitch(-2)
    print(f"  - Adjusted pitch trim to: {drone.trim.pitch_trim}")
    result = drone.send_controls(0, 0, 0, 50)
    print(f"  - New motor values: Pitch={result[0]}, Roll={result[1]}")
    print()
    
    print("Result: Drone now hovers stably!")


def demonstrate_trim_limits():
    """Demonstrate trim value limits."""
    print("DEMO 4: Trim Value Limits")
    print_separator()
    
    trim = TrimControl()
    
    print("Attempting to set pitch trim to 150 (exceeds limit of 100):")
    trim.pitch_trim = 150
    print(f"Actual pitch trim: {trim.pitch_trim}")
    print()
    
    print("Attempting to set roll trim to -150 (exceeds limit of -100):")
    trim.roll_trim = -150
    print(f"Actual roll trim: {trim.roll_trim}")
    print()
    
    print("Note: If you need more than ±100 trim, check your hardware!")
    print("  - Propeller balance")
    print("  - Motor alignment")
    print("  - Center of gravity")


def demonstrate_trim_reset():
    """Demonstrate resetting trim values."""
    print("DEMO 5: Resetting Trim")
    print_separator()
    
    trim = TrimControl()
    
    # Set various trim values
    trim.set_trim_values(pitch=-7, roll=5, yaw=-2, throttle=3)
    print("Current trim values:")
    print(trim.get_trim_values())
    print()
    
    print("Resetting pitch trim only:")
    trim.reset_pitch()
    print(trim.get_trim_values())
    print()
    
    print("Resetting all trim values:")
    trim.reset_all()
    print(trim.get_trim_values())


def demonstrate_real_world_scenario():
    """Demonstrate a realistic flight scenario."""
    print("DEMO 6: Real-World Flight Scenario")
    print_separator()
    
    drone = DroneController()
    
    print("Pre-flight:")
    print("  1. Check drone hardware ✓")
    print("  2. Calibrate sensors ✓")
    print("  3. Reset trim to neutral")
    drone.trim.reset_all()
    print(f"     Trim: {drone.trim.get_trim_values()}")
    print()
    
    print("Flight:")
    drone.arm()
    print("  - Drone armed")
    print("  - Taking off...")
    drone.send_controls(0, 0, 0, 60)
    print()
    
    print("  - Hovering at 2 meters")
    print("  - Observing behavior...")
    drone.send_controls(0, 0, 0, 50)
    print()
    
    print("  - Drift detected: Forward and right")
    print("  - Applying trim corrections...")
    drone.trim.adjust_pitch(-6)
    drone.trim.adjust_roll(-4)
    print(f"     Updated trim: {drone.trim.get_trim_values()}")
    print()
    
    print("  - Testing hover again...")
    drone.send_controls(0, 0, 0, 50)
    print("  - Stable! ✓")
    print()
    
    print("  - Flying forward (pilot input)...")
    result = drone.send_controls(30, 0, 0, 55)
    print(f"     Command sent: Pitch={result[0]}, Roll={result[1]}")
    print()
    
    print("  - Returning to hover...")
    drone.send_controls(0, 0, 0, 50)
    print()
    
    print("Landing:")
    print("  - Descending...")
    drone.send_controls(0, 0, 0, 30)
    print("  - Landed safely")
    drone.disarm()
    print("  - Drone disarmed")
    print()
    
    print(f"Final trim values saved: {drone.trim.get_trim_values()}")
    print("(These can be loaded for next flight)")


def main():
    """Run all demonstrations."""
    print("\n")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║         DRONE TRIM CONTROL - Interactive Examples         ║")
    print("╚════════════════════════════════════════════════════════════╝")
    
    try:
        demonstrate_basic_trim()
        print_separator()
        
        demonstrate_trim_application()
        print_separator()
        
        demonstrate_drift_correction()
        print_separator()
        
        demonstrate_trim_limits()
        print_separator()
        
        demonstrate_trim_reset()
        print_separator()
        
        demonstrate_real_world_scenario()
        print_separator()
        
        print("\n✓ All demonstrations completed successfully!\n")
        print("For more information, see:")
        print("  - TRIM_CONTROL_DOCUMENTATION.md (detailed guide)")
        print("  - drone_trim_control.py (source code)")
        print("  - test_drone_trim_control.py (unit tests)")
        print()
        
    except Exception as e:
        print(f"\n✗ Error during demonstration: {e}\n")
        raise


if __name__ == "__main__":
    main()
