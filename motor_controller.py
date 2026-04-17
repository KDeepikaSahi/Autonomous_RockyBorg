#!/usr/bin/env python

import sys
sys.path.insert(0, '/home/admin/project/rockyborg/')
from RockyBorg import RockyBorg
from config import MOTOR_CONFIG


class MotorController:
    """Handles RockyBorg motor control operations."""
    
    def __init__(self):
        """Initialize the RockyBorg motor controller."""
        self.RB = RockyBorg()
        self.RB.Init()
        self.RB.SetCommsFailsafe(False)
        self.RB.SetMotorsEnabled(True)
        self.motor_invert = MOTOR_CONFIG['MOTOR_INVERT']
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds directly."""
        self.RB.SetMotor1(left_speed)
        self.RB.SetMotor2(right_speed * self.motor_invert)
    
    def set_speed(self, speed):
        """Set both motors to the same speed (forward motion)."""
        self.set_motor_speeds(speed, speed)
    
    def set_speed_based_on_steering(self, steer_angle):
        """Adjust motor speed based on steering angle."""
        if abs(steer_angle) > MOTOR_CONFIG['TURN_SPEED_THRESHOLD']:
            motor_speed = MOTOR_CONFIG['TURN_SPEED']
        else:
            # Scale speed based on steering angle
            max_speed = MOTOR_CONFIG['MAX_SPEED']
            min_speed = MOTOR_CONFIG['MIN_SPEED']
            speed_scale = 1.0 - min(abs(steer_angle), 1.0)
            motor_speed = min_speed + (max_speed - min_speed) * speed_scale
        
        self.set_speed(motor_speed)
    
    def search_mode(self):
        """Set motors to search mode (slower speed)."""
        search_speed = MOTOR_CONFIG['BASE_SPEED'] * MOTOR_CONFIG['SEARCH_SPEED_FACTOR']
        self.set_speed(search_speed)
    
    def stop(self):
        """Stop all motors."""
        self.RB.MotorsOff()
    
    def __del__(self):
        """Ensure motors are stopped on cleanup."""
        self.stop()