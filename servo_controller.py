#!/usr/bin/env python

import numpy as np
from config import PID_PARAMS, TURN_CONFIG, STEERING_CONFIG


class ServoController:
    """Handles servo control and PID-based steering."""
    
    def __init__(self, rocky_borg):
        """Initialize servo controller with RockyBorg instance."""
        self.RB = rocky_borg
        
        # PID parameters
        self.Kp = PID_PARAMS['Kp']
        self.Ki = PID_PARAMS['Ki']
        self.Kd = PID_PARAMS['Kd']
        
        # PID memory
        self.previous_error = 0.0
        self.integral = 0.0
        
        # Steering memory
        self.previous_angle = 0.0
        self.angle_history = []
    
    def calculate_pid_steering(self, line_center, curve_gain):
        """Calculate steering angle using PID control."""
        # Calculate error with curve gain influence
        error = line_center + curve_gain * 0.1
        
        # PID calculations
        self.integral += error
        derivative = error - self.previous_error
        pid_output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error
        
        # Clip output to valid range
        steer_angle = np.clip(pid_output, -1.0, 1.0)
        
        # Apply smoothing
        steer_angle = (STEERING_CONFIG['smoothing_factor'] * self.previous_angle + 
                      (1 - STEERING_CONFIG['smoothing_factor']) * steer_angle)
        self.previous_angle = steer_angle
        
        return steer_angle
    
    def apply_turn_boost(self, steer_angle, curve_gain, line_center):
        """Apply intelligent boost for turns."""
        abs_angle = abs(steer_angle)
        abs_curve = abs(curve_gain)
        abs_center = abs(line_center)
        
        # Detect if we're in a turn
        turn_detected = (
            abs_angle > TURN_CONFIG['turn_angle_threshold'] or
            abs_curve > TURN_CONFIG['curve_threshold'] or
            abs_center > TURN_CONFIG['center_offset_threshold']
        )
        
        if turn_detected:
            # Strong boost for turns
            if abs_angle > 0.1:
                boost_factor = (TURN_CONFIG['boost_factor_base'] + 
                              (abs_angle * TURN_CONFIG['boost_factor_scale']))
                boosted_angle = steer_angle * boost_factor
                return np.clip(boosted_angle, -1.0, 1.0)
            else:
                return steer_angle * TURN_CONFIG['min_turn_boost']
        else:
            # Light steering for straight lines
            return steer_angle * TURN_CONFIG['straight_line_factor']
    
    def is_sharp_turn(self, curve_gain, line_center):
        """Detect sharp turns that need extra aggressive steering."""
        # Add current angle to history
        self.angle_history.append(abs(curve_gain))
        if len(self.angle_history) > TURN_CONFIG['angle_history_size']:
            self.angle_history.pop(0)
        
        # Calculate average curve
        avg_curve = sum(self.angle_history) / len(self.angle_history)
        
        # Sharp turn detection
        return (avg_curve > TURN_CONFIG['sharp_turn_threshold'] or 
                abs(line_center) > 0.6)
    
    def set_steering(self, line_center, curve_gain):
        """Calculate and set steering angle."""
        # Calculate base steering with PID
        steer_angle = self.calculate_pid_steering(line_center, curve_gain)
        
        # Apply turn boost
        steer_angle = self.apply_turn_boost(steer_angle, curve_gain, line_center)
        
        # Extra boost for sharp turns
        if self.is_sharp_turn(curve_gain, line_center):
            steer_angle = steer_angle * TURN_CONFIG['sharp_turn_boost']
            steer_angle = np.clip(steer_angle, -1.0, 1.0)
        
        # Set servo position
        self.RB.SetServoPosition(steer_angle)
        
        return steer_angle
    
    def set_angle(self, angle):
        """Directly set servo angle."""
        self.RB.SetServoPosition(angle)
    
    def go_straight(self):
        """Set servo to straight position."""
        self.set_angle(0)
    
    def reset_pid(self):
        """Reset PID controller state."""
        self.previous_error = 0.0
        self.integral = 0.0