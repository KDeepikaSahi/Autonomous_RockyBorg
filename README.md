# Autonomous_RockyBorg

A black line-following robot implementation using Raspberry Pi, PiCamera2, with servo steering control. The robot can follow black lines, detect cross junctions.

## Features
- **PID-controlled servo steering** for precise line following
- **Pixel based Cross junction detection** with hardcoded navigation
- **CLAHE image enhancement** for better line visibility

## Hardware Used

| Component              | Description                         |
|------------------------|-------------------------------------|
| Raspberry Pi 3B+       | Main controller                     |
| RockyBorg PCB          | Motor + servo control via I2C       |
| 2x DC Motors           | Rear drive motors                   |
| 1x Servo Motor         | Steering                            |
| Pi Camera2             | Line detection using OpenCV         |
| Power Supply           | 8x AAA batteries                    |

## Software Features

- Manual control via RockyBorg GUI or joystick
- Line Following using camera and OpenCV
- Remote access via `connect.raspberrypi.com`

## Project Structure

```
main/
├── README.md
├── main.py                 # Main control loop and robot coordination
├── config.py               # Configuration parameters and constants
├── servo_controller.py     # Servo PWM control and PID steering logic
├── vision_processor.py     # Camera handling and computer vision processing
└── motor_controller.py     # RockyBorg motor control interface
```

## Acknowledgements
- PiBorg for RockyBorg library (https://github.com/piborg/RockyBorg)
- OpenCV tutorials: TechWithTim
- Raspberry Pi community
- VisionRace Masters (https://github.com/CRM-UAM/VisionRace/tree/master)
- Robot Positioning (x-IMU) (https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU/tree/master)
- Line Following Robot OpenCV (https://www.youtube.com/watch?v=ZC4VUt1I5FI)
- Line Tracking (https://github.com/fustyles/Arduino/blob/master/ESP32-CAM_Car/ESP32-CAM_CAR_TrackColorline_Tracking.js/ESP32-CAM_CAR_TrackColorline_Tracking.js.ino)
- https://github.com/abaeyens/image-processing/tree/master/RCJ_2014
- https://www.youtube.com/watch?v=o2ul4KrLT-s
