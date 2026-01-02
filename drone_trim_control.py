"""
Drone Trim Control Module

This module implements trim control for drones. Trim is used to fine-tune
the neutral position of a drone's controls to compensate for imbalances,
manufacturing tolerances, or environmental factors.

Trim adjustments allow the drone to maintain stable flight without constant
manual correction from the pilot.
"""


class TrimControl:
    """
    Manages trim adjustments for drone flight controls.
    
    Trim values are typically small adjustments (usually -100 to +100)
    that are added to the main control inputs to compensate for drift.
    """
    
    def __init__(self):
        """Initialize trim control with neutral values."""
        self._trim_pitch = 0.0  # Forward/backward trim
        self._trim_roll = 0.0   # Left/right trim
        self._trim_yaw = 0.0    # Rotation trim
        self._trim_throttle = 0.0  # Vertical trim
        
        # Trim limits to prevent excessive adjustment
        self._trim_limit = 100.0
        
    @property
    def pitch_trim(self):
        """Get current pitch trim value."""
        return self._trim_pitch
    
    @pitch_trim.setter
    def pitch_trim(self, value):
        """Set pitch trim value with limit checking."""
        self._trim_pitch = self._clamp(value)
    
    @property
    def roll_trim(self):
        """Get current roll trim value."""
        return self._trim_roll
    
    @roll_trim.setter
    def roll_trim(self, value):
        """Set roll trim value with limit checking."""
        self._trim_roll = self._clamp(value)
    
    @property
    def yaw_trim(self):
        """Get current yaw trim value."""
        return self._trim_yaw
    
    @yaw_trim.setter
    def yaw_trim(self, value):
        """Set yaw trim value with limit checking."""
        self._trim_yaw = self._clamp(value)
    
    @property
    def throttle_trim(self):
        """Get current throttle trim value."""
        return self._trim_throttle
    
    @throttle_trim.setter
    def throttle_trim(self, value):
        """Set throttle trim value with limit checking."""
        self._trim_throttle = self._clamp(value)
    
    def _clamp(self, value):
        """Clamp a value within trim limits."""
        return max(-self._trim_limit, min(self._trim_limit, value))
    
    def adjust_pitch(self, amount):
        """
        Adjust pitch trim by a specified amount.
        
        Args:
            amount: Increment/decrement value for pitch trim
        """
        self.pitch_trim = self._trim_pitch + amount
    
    def adjust_roll(self, amount):
        """
        Adjust roll trim by a specified amount.
        
        Args:
            amount: Increment/decrement value for roll trim
        """
        self.roll_trim = self._trim_roll + amount
    
    def adjust_yaw(self, amount):
        """
        Adjust yaw trim by a specified amount.
        
        Args:
            amount: Increment/decrement value for yaw trim
        """
        self.yaw_trim = self._trim_yaw + amount
    
    def adjust_throttle(self, amount):
        """
        Adjust throttle trim by a specified amount.
        
        Args:
            amount: Increment/decrement value for throttle trim
        """
        self.throttle_trim = self._trim_throttle + amount
    
    def reset_all(self):
        """Reset all trim values to neutral (0)."""
        self._trim_pitch = 0.0
        self._trim_roll = 0.0
        self._trim_yaw = 0.0
        self._trim_throttle = 0.0
    
    def reset_pitch(self):
        """Reset pitch trim to neutral."""
        self._trim_pitch = 0.0
    
    def reset_roll(self):
        """Reset roll trim to neutral."""
        self._trim_roll = 0.0
    
    def reset_yaw(self):
        """Reset yaw trim to neutral."""
        self._trim_yaw = 0.0
    
    def reset_throttle(self):
        """Reset throttle trim to neutral."""
        self._trim_throttle = 0.0
    
    def apply_trim(self, pitch, roll, yaw, throttle):
        """
        Apply trim adjustments to control inputs.
        
        Args:
            pitch: Raw pitch control input
            roll: Raw roll control input
            yaw: Raw yaw control input
            throttle: Raw throttle control input
            
        Returns:
            tuple: (trimmed_pitch, trimmed_roll, trimmed_yaw, trimmed_throttle)
        """
        trimmed_pitch = pitch + self._trim_pitch
        trimmed_roll = roll + self._trim_roll
        trimmed_yaw = yaw + self._trim_yaw
        trimmed_throttle = throttle + self._trim_throttle
        
        return (trimmed_pitch, trimmed_roll, trimmed_yaw, trimmed_throttle)
    
    def get_trim_values(self):
        """
        Get all current trim values.
        
        Returns:
            dict: Dictionary containing all trim values
        """
        return {
            'pitch': self._trim_pitch,
            'roll': self._trim_roll,
            'yaw': self._trim_yaw,
            'throttle': self._trim_throttle
        }
    
    def set_trim_values(self, pitch=None, roll=None, yaw=None, throttle=None):
        """
        Set multiple trim values at once.
        
        Args:
            pitch: Pitch trim value (optional)
            roll: Roll trim value (optional)
            yaw: Yaw trim value (optional)
            throttle: Throttle trim value (optional)
        """
        if pitch is not None:
            self.pitch_trim = pitch
        if roll is not None:
            self.roll_trim = roll
        if yaw is not None:
            self.yaw_trim = yaw
        if throttle is not None:
            self.throttle_trim = throttle
    
    def __str__(self):
        """String representation of current trim state."""
        return (f"TrimControl(pitch={self._trim_pitch}, roll={self._trim_roll}, "
                f"yaw={self._trim_yaw}, throttle={self._trim_throttle})")
    
    def __repr__(self):
        """Detailed representation of trim control."""
        return self.__str__()


class DroneController:
    """
    Example drone controller that uses trim control.
    
    This demonstrates how trim control would be integrated into
    a complete drone control system.
    """
    
    def __init__(self):
        """Initialize drone controller with trim control."""
        self.trim = TrimControl()
        self._armed = False
    
    def arm(self):
        """Arm the drone for flight."""
        self._armed = True
        print("Drone armed")
    
    def disarm(self):
        """Disarm the drone."""
        self._armed = False
        print("Drone disarmed")
    
    def send_controls(self, pitch, roll, yaw, throttle):
        """
        Send control commands to the drone with trim applied.
        
        Args:
            pitch: Pitch control input (-100 to 100)
            roll: Roll control input (-100 to 100)
            yaw: Yaw control input (-100 to 100)
            throttle: Throttle control input (0 to 100)
            
        Returns:
            tuple: Final control values sent to motors after trim
        """
        if not self._armed:
            print("Warning: Drone not armed!")
            return (0, 0, 0, 0)
        
        # Apply trim adjustments
        trimmed_values = self.trim.apply_trim(pitch, roll, yaw, throttle)
        
        # In a real implementation, these would be sent to motor controllers
        print(f"Sending controls: Pitch={trimmed_values[0]:.1f}, "
              f"Roll={trimmed_values[1]:.1f}, Yaw={trimmed_values[2]:.1f}, "
              f"Throttle={trimmed_values[3]:.1f}")
        
        return trimmed_values
    
    def calibrate_trim(self):
        """
        Perform automatic trim calibration.
        
        In a real implementation, this would:
        1. Hover the drone
        2. Measure drift over time
        3. Automatically adjust trim to compensate
        """
        print("Starting trim calibration...")
        print("Hover the drone and observe drift direction")
        print("Trim will be adjusted automatically")
        # Simulation of calibration
        print("Calibration complete")


def demonstrate_trim_control():
    """
    Demonstration of drone trim control functionality.
    """
    print("=== Drone Trim Control Demonstration ===\n")
    
    # Create a drone controller with trim
    drone = DroneController()
    
    # Display initial trim values
    print("Initial trim values:")
    print(drone.trim)
    print()
    
    # Arm the drone
    drone.arm()
    print()
    
    # Send controls without trim
    print("Sending controls without trim adjustments:")
    drone.send_controls(pitch=0, roll=0, yaw=0, throttle=50)
    print()
    
    # Adjust trim to compensate for drift
    print("Drone is drifting forward, adjusting pitch trim backward...")
    drone.trim.adjust_pitch(-5)
    
    print("Drone is drifting right, adjusting roll trim left...")
    drone.trim.adjust_roll(-3)
    print()
    
    # Show updated trim
    print("Updated trim values:")
    print(drone.trim)
    print()
    
    # Send same controls with trim applied
    print("Sending same controls with trim adjustments:")
    drone.send_controls(pitch=0, roll=0, yaw=0, throttle=50)
    print()
    
    # Fine-tune trim
    print("Fine-tuning trim values...")
    drone.trim.set_trim_values(pitch=-7, roll=-3, yaw=1)
    print(drone.trim)
    print()
    
    # Test with pilot input
    print("Pilot moves stick forward (pitch=20):")
    drone.send_controls(pitch=20, roll=0, yaw=0, throttle=60)
    print()
    
    # Reset trim
    print("Resetting all trim values...")
    drone.trim.reset_all()
    print(drone.trim)
    print()
    
    # Disarm
    drone.disarm()
    
    print("\n=== Demonstration Complete ===")


if __name__ == "__main__":
    demonstrate_trim_control()
