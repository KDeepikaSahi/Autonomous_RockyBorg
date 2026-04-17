#!/usr/bin/env python

import cv2
import time
from motor_controller import MotorController
from servo_controller import ServoController
from vision_processor import VisionProcessor
from config import MOTOR_CONFIG, JUNCTION_CONFIG, STEERING_CONFIG


class LineFollowingRobot:
    
    def __init__(self):
        print("Initializing robot systems")
        self.motor_controller = MotorController()
        self.servo_controller = ServoController(self.motor_controller.RB)
        self.vision_processor = VisionProcessor()
        
        # Junction state start cleaner
        self.junction_state = {
            'blind_frames': 0,
            'frozen_angle': 0.0,
            'in_junction': False
        }
        
        print("Robot initialization complete!")
    
    def handle_junction_blind_mode(self, frame):
        """Handle blind navigation through PLUS junctions."""
        if self.junction_state['blind_frames'] > JUNCTION_CONFIG['frozen_angle_frames']:
            # Phase 1: Maintain frozen steering angle
            self.servo_controller.set_angle(self.junction_state['frozen_angle'])
            status_text = (f"BLIND FROZEN ANGLE: {self.junction_state['frozen_angle']:.2f} "
                          f"({self.junction_state['blind_frames']})")
        else:
            # Phase 2: Go straight
            self.servo_controller.go_straight()
            status_text = f"BLIND STRAIGHT: {self.junction_state['blind_frames']}"
        
        # Maintain speed during blind mode
        self.motor_controller.set_speed(MOTOR_CONFIG['BASE_SPEED'])
        self.junction_state['blind_frames'] -= 1
        
        # Reset junction state when blind sequence completes
        if self.junction_state['blind_frames'] == 0:
            self.junction_state['in_junction'] = False
            print("Junction sequence completed - resuming line following")
        
        # Display status
        cv2.putText(frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return status_text
    
    def handle_plus_junction(self, line_info):
        """Handle PLUS junction detection."""
        if line_info['is_plus_junction'] and not self.junction_state['in_junction']:
            print(f"PLUS Junction detected! ROI fill: {line_info['roi_fill']:.1%}")
            
            # Capture current steering angle for phase 1
            self.junction_state['frozen_angle'] = self.servo_controller.previous_angle
            self.junction_state['blind_frames'] = JUNCTION_CONFIG['blind_frames']
            self.junction_state['in_junction'] = True
            
            # Override line detection to go straight
            line_info['found'] = True
            line_info['center'] = 0.0
            line_info['curve'] = 0.0
            
            # Update waypoint history for straight trajectory
            self.vision_processor._update_waypoint_history(0.0)
    
    def process_frame(self, frame):
        """Process single frame and return control decisions."""
        # Extract and process ROI
        processed_frame = self.vision_processor.process_roi(frame)
        
        # Detect line and junctions
        line_info = self.vision_processor.detect_line(processed_frame)
        
        # Handle PLUS junction if detected
        self.handle_plus_junction(line_info)
        
        return processed_frame, line_info
    
    def execute_line_following(self, line_info):
        """Execute line following behavior based on detection results."""
        if line_info['found']:
            # Calculate and apply steering
            steer_angle = self.servo_controller.set_steering(
                line_info['center'], 
                line_info['curve']
            )
            
            # Adjust motor speed based on steering
            self.motor_controller.set_speed_based_on_steering(steer_angle)
            
            # Status message
            _, confidence = self.vision_processor._predict_line_trajectory()
            
            if line_info['is_plus_junction'] and self.junction_state['in_junction']:
                status = (f"PLUS JUNCTION - Starting blind sequence "
                         f"(ROI: {line_info['roi_fill']:.1%})")
            else:
                status = f"STEERING: {steer_angle:.2f} CONF: {confidence:.2f}"
        else:
            # Search mode
            self.motor_controller.search_mode()
            self.servo_controller.go_straight()
            status = f"SEARCHING (ROI: {line_info['roi_fill']:.1%})"
        
        return status
    
    def run(self):
        """Main control loop."""
        try:
            print("Line following with PLUS junction detection started.")
            print("Press 'q' to exit.")
            
            while True:
                # Capture frame
                frame = self.vision_processor.capture_frame()
                
                # Check if in blind junction mode
                if self.junction_state['blind_frames'] > 0:
                    status = self.handle_junction_blind_mode(frame)
                    
                    # Show limited view during blind mode
                    display_frame = frame[int(frame.shape[0]*0.4):, :]
                    cv2.imshow("Processing View", display_frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                
                # Normal processing
                processed_frame, line_info = self.process_frame(frame)
                
                # Execute line following
                status = self.execute_line_following(line_info)
                
                # Update display
                self.vision_processor.draw_line_info(processed_frame, line_info, status)
                print(status)
                
                # Show debug windows
                cv2.imshow("Processing View", processed_frame)
                cv2.imshow("Binary", line_info['binary'])
                
                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources."""
        print("\nShutting down robot systems...")
        self.motor_controller.stop()
        self.vision_processor.cleanup()
        print("Shutdown complete")


def main():
    """Entry point for the line-following robot."""
    robot = LineFollowingRobot()
    robot.run()


if __name__ == "__main__":
    main()