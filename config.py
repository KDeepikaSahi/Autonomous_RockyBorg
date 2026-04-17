#!/usr/bin/env python
"""Configuration parameters for the line-following robot."""

# PID constants for servo control
PID_PARAMS = {
    'Kp': 0.4,      # previous 0.5
    'Ki': 0.02,     # previous 0.0
    'Kd': 0.3       # previous 0.35
}

# motor control parameters
MOTOR_CONFIG = {
    'BASE_SPEED': 0.15,
    'MOTOR_INVERT': -1,
    'TURN_SPEED': 0.18,
    'MIN_SPEED': 0.12,
    'MAX_SPEED': 0.18,
    'SEARCH_SPEED_FACTOR': 0.5
}

# camera configuration
CAMERA_CONFIG = {
    'resolution': (800, 400),
    'controls': {
        "AwbEnable": True,
        "AeEnable": True,
        "ExposureTime": 30000,
        "AnalogueGain": 4.0,
        "FrameRate": 30
    },
    'startup_delay': 2
}

# ROI parameters
ROI_CONFIG = {
    'main_roi_width': 0.8,
    'main_roi_height': 0.6,
    'main_roi_y_offset': 0.4,
    'side_crop': 0.05,
    'final_crop': 0.3,
    'scan_factors': [0.3, 0.45, 0.6],  # Multiple scan distances
    'scan_height': 0.15
}

# Image processing 
IMAGE_PROCESSING = {
    'clahe_clip_limit': 3.0,
    'clahe_grid_size': (8, 8),
    'gaussian_blur_size': (11, 11),
    'binary_threshold': 180,         #190 previous but 180 more robust in indoor lighting
    'min_contour_area': 100,
    'scan_min_contour_area': 50
}

# pixel fill parameters for junction
JUNCTION_CONFIG = {
    'plus_junction_threshold': 0.30,  # ROI fill percentage
    'blind_frames': 30,               # always divided into 2 (half for frozen angle and half for blind straight)
    'frozen_angle_frames': 10,  
    'white_surround_margin': 10,
    'white_surround_threshold': 0.1
}

# waypoint and trajectory parameters
WAYPOINT_CONFIG = {
    'max_history': 8,
    'trajectory_tolerance': 0.3,
    'confidence_threshold': 0.3,
    'line_memory_weight': 0.7,
    'trajectory_weight_high_conf': 0.7,
    'trajectory_weight_low_conf': 0.3
}

# Turn detection and boost parameters
TURN_CONFIG = {
    'turn_angle_threshold': 0.15,
    'curve_threshold': 0.3,
    'center_offset_threshold': 0.4,
    'sharp_turn_threshold': 0.5,
    'boost_factor_base': 1.5,
    'boost_factor_scale': 0.8,
    'min_turn_boost': 1.3,
    'straight_line_factor': 0.8,
    'sharp_turn_boost': 1.4,
    'angle_history_size': 5
}

STEERING_CONFIG = {
    'smoothing_factor': 0.8,
    'turn_speed_threshold': 0.4
}