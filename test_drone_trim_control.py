"""
Unit tests for drone trim control functionality.
"""

import unittest
from drone_trim_control import TrimControl, DroneController


class TestTrimControl(unittest.TestCase):
    """Test cases for TrimControl class."""
    
    def setUp(self):
        """Set up test fixture."""
        self.trim = TrimControl()
    
    def test_initial_trim_values_are_zero(self):
        """Test that trim values are initialized to zero."""
        self.assertEqual(self.trim.pitch_trim, 0.0)
        self.assertEqual(self.trim.roll_trim, 0.0)
        self.assertEqual(self.trim.yaw_trim, 0.0)
        self.assertEqual(self.trim.throttle_trim, 0.0)
    
    def test_set_pitch_trim(self):
        """Test setting pitch trim value."""
        self.trim.pitch_trim = 10.0
        self.assertEqual(self.trim.pitch_trim, 10.0)
    
    def test_set_roll_trim(self):
        """Test setting roll trim value."""
        self.trim.roll_trim = -15.0
        self.assertEqual(self.trim.roll_trim, -15.0)
    
    def test_set_yaw_trim(self):
        """Test setting yaw trim value."""
        self.trim.yaw_trim = 5.0
        self.assertEqual(self.trim.yaw_trim, 5.0)
    
    def test_set_throttle_trim(self):
        """Test setting throttle trim value."""
        self.trim.throttle_trim = 20.0
        self.assertEqual(self.trim.throttle_trim, 20.0)
    
    def test_trim_limits_positive(self):
        """Test that trim values are limited to maximum."""
        self.trim.pitch_trim = 150.0
        self.assertEqual(self.trim.pitch_trim, 100.0)
    
    def test_trim_limits_negative(self):
        """Test that trim values are limited to minimum."""
        self.trim.roll_trim = -150.0
        self.assertEqual(self.trim.roll_trim, -100.0)
    
    def test_adjust_pitch(self):
        """Test pitch trim adjustment."""
        self.trim.adjust_pitch(5.0)
        self.assertEqual(self.trim.pitch_trim, 5.0)
        self.trim.adjust_pitch(-3.0)
        self.assertEqual(self.trim.pitch_trim, 2.0)
    
    def test_adjust_roll(self):
        """Test roll trim adjustment."""
        self.trim.adjust_roll(10.0)
        self.assertEqual(self.trim.roll_trim, 10.0)
        self.trim.adjust_roll(-5.0)
        self.assertEqual(self.trim.roll_trim, 5.0)
    
    def test_adjust_yaw(self):
        """Test yaw trim adjustment."""
        self.trim.adjust_yaw(-7.0)
        self.assertEqual(self.trim.yaw_trim, -7.0)
        self.trim.adjust_yaw(2.0)
        self.assertEqual(self.trim.yaw_trim, -5.0)
    
    def test_adjust_throttle(self):
        """Test throttle trim adjustment."""
        self.trim.adjust_throttle(3.0)
        self.assertEqual(self.trim.throttle_trim, 3.0)
        self.trim.adjust_throttle(4.0)
        self.assertEqual(self.trim.throttle_trim, 7.0)
    
    def test_adjust_with_limits(self):
        """Test that adjustments respect limits."""
        self.trim.pitch_trim = 95.0
        self.trim.adjust_pitch(10.0)
        self.assertEqual(self.trim.pitch_trim, 100.0)
    
    def test_reset_all(self):
        """Test resetting all trim values."""
        self.trim.pitch_trim = 10.0
        self.trim.roll_trim = -5.0
        self.trim.yaw_trim = 7.0
        self.trim.throttle_trim = -3.0
        
        self.trim.reset_all()
        
        self.assertEqual(self.trim.pitch_trim, 0.0)
        self.assertEqual(self.trim.roll_trim, 0.0)
        self.assertEqual(self.trim.yaw_trim, 0.0)
        self.assertEqual(self.trim.throttle_trim, 0.0)
    
    def test_reset_pitch(self):
        """Test resetting pitch trim only."""
        self.trim.pitch_trim = 10.0
        self.trim.roll_trim = -5.0
        self.trim.reset_pitch()
        
        self.assertEqual(self.trim.pitch_trim, 0.0)
        self.assertEqual(self.trim.roll_trim, -5.0)
    
    def test_reset_roll(self):
        """Test resetting roll trim only."""
        self.trim.pitch_trim = 10.0
        self.trim.roll_trim = -5.0
        self.trim.reset_roll()
        
        self.assertEqual(self.trim.pitch_trim, 10.0)
        self.assertEqual(self.trim.roll_trim, 0.0)
    
    def test_reset_yaw(self):
        """Test resetting yaw trim only."""
        self.trim.yaw_trim = 7.0
        self.trim.throttle_trim = -3.0
        self.trim.reset_yaw()
        
        self.assertEqual(self.trim.yaw_trim, 0.0)
        self.assertEqual(self.trim.throttle_trim, -3.0)
    
    def test_reset_throttle(self):
        """Test resetting throttle trim only."""
        self.trim.yaw_trim = 7.0
        self.trim.throttle_trim = -3.0
        self.trim.reset_throttle()
        
        self.assertEqual(self.trim.yaw_trim, 7.0)
        self.assertEqual(self.trim.throttle_trim, 0.0)
    
    def test_apply_trim_no_adjustment(self):
        """Test applying trim with neutral values."""
        result = self.trim.apply_trim(50, 30, -10, 75)
        self.assertEqual(result, (50, 30, -10, 75))
    
    def test_apply_trim_with_adjustments(self):
        """Test applying trim with non-zero values."""
        self.trim.pitch_trim = -5.0
        self.trim.roll_trim = 3.0
        self.trim.yaw_trim = -2.0
        self.trim.throttle_trim = 1.0
        
        result = self.trim.apply_trim(50, 30, -10, 75)
        self.assertEqual(result, (45.0, 33.0, -12.0, 76.0))
    
    def test_apply_trim_with_zero_inputs(self):
        """Test applying trim to neutral stick positions."""
        self.trim.pitch_trim = -7.0
        self.trim.roll_trim = 4.0
        self.trim.yaw_trim = 1.0
        self.trim.throttle_trim = -2.0
        
        result = self.trim.apply_trim(0, 0, 0, 0)
        self.assertEqual(result, (-7.0, 4.0, 1.0, -2.0))
    
    def test_get_trim_values(self):
        """Test getting all trim values as dictionary."""
        self.trim.pitch_trim = 5.0
        self.trim.roll_trim = -3.0
        self.trim.yaw_trim = 2.0
        self.trim.throttle_trim = -1.0
        
        values = self.trim.get_trim_values()
        
        self.assertEqual(values['pitch'], 5.0)
        self.assertEqual(values['roll'], -3.0)
        self.assertEqual(values['yaw'], 2.0)
        self.assertEqual(values['throttle'], -1.0)
    
    def test_set_trim_values_all(self):
        """Test setting all trim values at once."""
        self.trim.set_trim_values(pitch=10.0, roll=-5.0, yaw=3.0, throttle=7.0)
        
        self.assertEqual(self.trim.pitch_trim, 10.0)
        self.assertEqual(self.trim.roll_trim, -5.0)
        self.assertEqual(self.trim.yaw_trim, 3.0)
        self.assertEqual(self.trim.throttle_trim, 7.0)
    
    def test_set_trim_values_partial(self):
        """Test setting only some trim values."""
        self.trim.pitch_trim = 10.0
        self.trim.roll_trim = -5.0
        
        self.trim.set_trim_values(pitch=15.0, yaw=3.0)
        
        self.assertEqual(self.trim.pitch_trim, 15.0)
        self.assertEqual(self.trim.roll_trim, -5.0)  # Unchanged
        self.assertEqual(self.trim.yaw_trim, 3.0)
        self.assertEqual(self.trim.throttle_trim, 0.0)  # Unchanged
    
    def test_str_representation(self):
        """Test string representation of TrimControl."""
        self.trim.pitch_trim = 5.0
        self.trim.roll_trim = -3.0
        
        result = str(self.trim)
        
        self.assertIn("5.0", result)
        self.assertIn("-3.0", result)
        self.assertIn("TrimControl", result)


class TestDroneController(unittest.TestCase):
    """Test cases for DroneController class."""
    
    def setUp(self):
        """Set up test fixture."""
        self.drone = DroneController()
    
    def test_initial_state_disarmed(self):
        """Test that drone starts disarmed."""
        self.assertFalse(self.drone._armed)
    
    def test_arm_drone(self):
        """Test arming the drone."""
        self.drone.arm()
        self.assertTrue(self.drone._armed)
    
    def test_disarm_drone(self):
        """Test disarming the drone."""
        self.drone.arm()
        self.drone.disarm()
        self.assertFalse(self.drone._armed)
    
    def test_send_controls_when_disarmed(self):
        """Test that controls return zeros when disarmed."""
        result = self.drone.send_controls(50, 30, -10, 75)
        self.assertEqual(result, (0, 0, 0, 0))
    
    def test_send_controls_when_armed(self):
        """Test sending controls when armed."""
        self.drone.arm()
        result = self.drone.send_controls(50, 30, -10, 75)
        self.assertEqual(result, (50, 30, -10, 75))
    
    def test_send_controls_with_trim(self):
        """Test that trim is applied to controls."""
        self.drone.arm()
        self.drone.trim.pitch_trim = -5.0
        self.drone.trim.roll_trim = 3.0
        
        result = self.drone.send_controls(50, 30, -10, 75)
        self.assertEqual(result, (45.0, 33.0, -10.0, 75.0))
    
    def test_controller_has_trim(self):
        """Test that controller has TrimControl instance."""
        self.assertIsInstance(self.drone.trim, TrimControl)


class TestTrimControlIntegration(unittest.TestCase):
    """Integration tests for complete trim control workflow."""
    
    def test_complete_trim_workflow(self):
        """Test a complete trim adjustment workflow."""
        # Create drone controller
        drone = DroneController()
        
        # Arm drone
        drone.arm()
        
        # Simulate drift detection and correction
        # Drone drifts forward (needs negative pitch trim)
        drone.trim.adjust_pitch(-5)
        
        # Drone drifts right (needs negative roll trim)
        drone.trim.adjust_roll(-3)
        
        # Send neutral controls
        result = drone.send_controls(0, 0, 0, 50)
        
        # Verify trim is applied
        self.assertEqual(result, (-5.0, -3.0, 0.0, 50.0))
        
        # Fine tune
        drone.trim.adjust_pitch(-2)
        result = drone.send_controls(0, 0, 0, 50)
        self.assertEqual(result, (-7.0, -3.0, 0.0, 50.0))
        
        # Reset and verify
        drone.trim.reset_all()
        result = drone.send_controls(0, 0, 0, 50)
        self.assertEqual(result, (0.0, 0.0, 0.0, 50.0))
    
    def test_trim_with_pilot_input(self):
        """Test that trim works correctly with pilot input."""
        drone = DroneController()
        drone.arm()
        
        # Set trim
        drone.trim.set_trim_values(pitch=-5, roll=3)
        
        # Pilot moves forward
        result = drone.send_controls(pitch=20, roll=0, yaw=0, throttle=60)
        
        # Trim is added to pilot input
        self.assertEqual(result, (15.0, 3.0, 0.0, 60.0))


def run_tests():
    """Run all tests and display results."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestTrimControl))
    suite.addTests(loader.loadTestsFromTestCase(TestDroneController))
    suite.addTests(loader.loadTestsFromTestCase(TestTrimControlIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Return success status
    return result.wasSuccessful()


if __name__ == "__main__":
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
