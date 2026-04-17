#!/usr/bin/env python

import cv2
import numpy as np
import time
from picamera2 import Picamera2
from config import (CAMERA_CONFIG, ROI_CONFIG, IMAGE_PROCESSING, 
                   JUNCTION_CONFIG, WAYPOINT_CONFIG)


class VisionProcessor:
    
    def __init__(self):
        #start camera
        self.picam2 = Picamera2()
        self._setup_camera()
        
        # Waypoint tracking
        self.waypoint_history = []
        self.previous_line_position = None
        
    def _setup_camera(self):
        #camera config
        config = self.picam2.create_preview_configuration(
            main={"size": CAMERA_CONFIG['resolution']},
            controls=CAMERA_CONFIG['controls']
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(CAMERA_CONFIG['startup_delay'])
    
    def capture_frame(self):
        #conversion to RGB or BGR
        frame = self.picam2.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    def process_roi(self, frame):
        """Extract and process main ROI."""
        h, w = frame.shape[:2]
        roi_w = int(w * ROI_CONFIG['main_roi_width'])
        roi_h = int(h * ROI_CONFIG['main_roi_height'])
        roi_x = int((w - roi_w) / 2)
        roi_y = int(h * ROI_CONFIG['main_roi_y_offset'])
        
        cropped = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # Enhance image
        enhanced = self._enhance_image(cropped)
        
        # Progressive narrowing
        side_crop = int(cropped.shape[1] * ROI_CONFIG['side_crop'])
        narrow_frame = enhanced[:, side_crop:-side_crop]
        temp_frame = narrow_frame[narrow_frame.shape[0]//2:, :]
        final_frame = temp_frame[int(temp_frame.shape[0]*ROI_CONFIG['final_crop']):, :]
        final_side_crop = int(final_frame.shape[1] * ROI_CONFIG['side_crop'])
        extreme_narrow_frame = final_frame[:, final_side_crop:-final_side_crop]
        
        return extreme_narrow_frame
    
    def _enhance_image(self, image):
        """All about CLAHE"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(
            clipLimit=IMAGE_PROCESSING['clahe_clip_limit'],
            tileGridSize=IMAGE_PROCESSING['clahe_grid_size']
        )
        limg = clahe.apply(l)
        enhanced = cv2.merge([limg, a, b])
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
    
    def detect_line(self, frame):
        """Main line detection function."""
        # Convert to binary
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, IMAGE_PROCESSING['gaussian_blur_size'], 0)
        _, binary = cv2.threshold(blurred, IMAGE_PROCESSING['binary_threshold'], 
                                 255, cv2.THRESH_BINARY_INV)
        
        # Check for PLUS junction
        roi_fill_percentage = np.sum(binary == 255) / binary.size
        is_plus_junction = roi_fill_percentage >= JUNCTION_CONFIG['plus_junction_threshold']
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get trajectory prediction
        expected_pos, confidence = self._predict_line_trajectory()
        
        # Filter and find best contour
        valid_contours = self._filter_contours(contours, frame, binary.shape, 
                                              expected_pos, confidence)
        
        line_info = {
            'found': False,
            'center': 0,
            'curve': 0,
            'contour': None,
            'is_plus_junction': is_plus_junction,
            'roi_fill': roi_fill_percentage,
            'binary': binary
        }
        
        if valid_contours:
            best_contour = self._find_best_contour(valid_contours, binary.shape, 
                                                   expected_pos, confidence)
            if best_contour is not None:
                line_info['found'] = True
                line_info['contour'] = best_contour
                
                # Calculate line parameters
                [vx, vy, x0, y0] = cv2.fitLine(best_contour, cv2.DIST_L2, 0, 0.01, 0.01)
                line_info['center'] = (float(x0[0]) / binary.shape[1]) * 2 - 1
                line_info['curve'] = np.clip(float(vx[0]) * 2.0, -1.0, 1.0)
                
                # Update tracking
                self._update_waypoint_history(line_info['center'])
                self.previous_line_position = line_info['center']
                
        elif self.previous_line_position is not None:
            # Use memory when no line found
            line_info['found'] = True
            line_info['center'] = self.previous_line_position * WAYPOINT_CONFIG['line_memory_weight']
            line_info['curve'] = 0
        
        return line_info
    
    def _filter_contours(self, contours, image, binary_shape, expected_pos, confidence):
        """Filter contours by size and surrounding color."""
        valid_contours = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > IMAGE_PROCESSING['min_contour_area'] and \
               self._is_surrounded_by_white(cnt, image):
                valid_contours.append(cnt)
        
        # Apply cross-junction filtering if confident
        if valid_contours and confidence > 0.4:
            filtered = self._filter_cross_junction_lines(
                valid_contours, binary_shape, expected_pos, confidence
            )
            if filtered:
                valid_contours = filtered
        
        return valid_contours
    
    def _is_surrounded_by_white(self, contour, image):
        """Check if contour is surrounded by white/light colors."""
        x, y, w, h = cv2.boundingRect(contour)
        margin = JUNCTION_CONFIG['white_surround_margin']
        
        x1 = max(0, x - margin)
        y1 = max(0, y - margin)
        x2 = min(image.shape[1], x + w + margin)
        y2 = min(image.shape[0], y + h + margin)
        
        surrounding = image[y1:y2, x1:x2]
        
        if len(surrounding.shape) == 3:
            surrounding_gray = cv2.cvtColor(surrounding, cv2.COLOR_BGR2GRAY)
        else:
            surrounding_gray = surrounding
        
        white_pixels = np.sum(surrounding_gray > 200)
        white_percentage = white_pixels / surrounding_gray.size
        
        return white_percentage > JUNCTION_CONFIG['white_surround_threshold']
    
    def _filter_cross_junction_lines(self, contours, binary_shape, expected_pos, confidence):
        """Filter out perpendicular lines from cross junctions."""
        valid_lines = []
        
        for contour in contours:
            [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            line_center = (float(x0[0]) / binary_shape[1]) * 2 - 1
            line_angle = np.arctan2(float(vy[0]), float(vx[0])) * 180 / np.pi
            
            abs_angle = abs(line_angle)
            is_roughly_horizontal = abs_angle < 45 or abs_angle > 135
            
            matches_trajectory = self._is_line_consistent_with_trajectory(
                line_center, expected_pos, confidence
            )
            
            if is_roughly_horizontal and matches_trajectory:
                valid_lines.append(contour)
            elif confidence < 0.4 and is_roughly_horizontal:
                valid_lines.append(contour)
        
        return valid_lines
    
    def _find_best_contour(self, contours, binary_shape, expected_pos, confidence):
        """Find best contour based on trajectory and size."""
        best_contour = None
        best_score = -1
        
        for contour in contours:
            [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            contour_center = (float(x0[0]) / binary_shape[1]) * 2 - 1
            
            trajectory_score = 1.0 - min(1.0, abs(contour_center - expected_pos))
            size_score = min(1.0, cv2.contourArea(contour) / 1000.0)
            
            if confidence > 0.5:
                total_score = (WAYPOINT_CONFIG['trajectory_weight_high_conf'] * trajectory_score + 
                             (1 - WAYPOINT_CONFIG['trajectory_weight_high_conf']) * size_score)
            else:
                total_score = (WAYPOINT_CONFIG['trajectory_weight_low_conf'] * trajectory_score + 
                             (1 - WAYPOINT_CONFIG['trajectory_weight_low_conf']) * size_score)
            
            if total_score > best_score:
                best_score = total_score
                best_contour = contour
        
        return best_contour
    
    def _predict_line_trajectory(self):
        """Predict where line should be based on history."""
        if len(self.waypoint_history) < 3:
            return 0.0, 0.0
        
        recent_waypoints = self.waypoint_history[-4:]
        x_positions = [float(wp[0]) for wp in recent_waypoints]
        
        if len(x_positions) >= 2:
            recent_movement = x_positions[-1] - x_positions[-2]
            expected_position = x_positions[-1] + recent_movement
            
            if len(x_positions) >= 3:
                movements = [x_positions[i] - x_positions[i-1] for i in range(1, len(x_positions))]
                movement_variance = np.var(movements) if len(movements) > 1 else 0
                confidence = max(0, 1.0 - movement_variance * 5)
            else:
                confidence = 0.5
            
            return float(expected_position), float(confidence)
        
        return 0.0, 0.0
    
    def _is_line_consistent_with_trajectory(self, line_center, expected_pos, confidence):
        """Check if detected line matches expected trajectory."""
        if confidence < WAYPOINT_CONFIG['confidence_threshold']:
            return True
        
        adjusted_tolerance = WAYPOINT_CONFIG['trajectory_tolerance'] * (1.0 + confidence)
        distance_from_expected = abs(line_center - expected_pos)
        
        return distance_from_expected <= adjusted_tolerance
    
    def _update_waypoint_history(self, line_center):
        """Update waypoint history with new position."""
        current_time = time.time()
        self.waypoint_history.append((float(line_center), current_time))
        if len(self.waypoint_history) > WAYPOINT_CONFIG['max_history']:
            self.waypoint_history.pop(0)
    
    def detect_multiple_scan_lines(self, frame):
        """Detect lines at multiple distances for waypoint validation."""
        h, w = frame.shape[:2]
        scan_results = []
        
        for i, scan_factor in enumerate(ROI_CONFIG['scan_factors']):
            roi_h = int(h * ROI_CONFIG['scan_height'])
            roi_y = int(h * scan_factor)
            
            if roi_y + roi_h >= h:
                continue
            
            roi_w = int(w * 1)
            roi_x = int((w - roi_w) / 2)
            scan_roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            
            # Process scan ROI
            enhanced = self._enhance_image(scan_roi)
            side_crop = int(scan_roi.shape[1] * ROI_CONFIG['side_crop'])
            narrow_frame = enhanced[:, side_crop:-side_crop]
            
            gray = cv2.cvtColor(narrow_frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, IMAGE_PROCESSING['gaussian_blur_size'], 0)
            _, binary = cv2.threshold(blurred, IMAGE_PROCESSING['binary_threshold'], 
                                    255, cv2.THRESH_BINARY_INV)
            
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            valid_contours = []
            for cnt in contours:
                if cv2.contourArea(cnt) > IMAGE_PROCESSING['scan_min_contour_area']:
                    valid_contours.append(cnt)
            
            if valid_contours:
                largest = max(valid_contours, key=cv2.contourArea)
                [vx, vy, x0, y0] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
                line_center = (float(x0[0]) / binary.shape[1]) * 2 - 1
                scan_results.append((line_center, scan_factor, i))
        
        return scan_results
    
    def draw_line_info(self, frame, line_info, status_text):
        """Draw debugging information on frame."""
        if line_info['found'] and line_info['contour'] is not None:
            x, y, w, h = cv2.boundingRect(line_info['contour'])
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        
        # Draw trajectory prediction if available
        expected_pos, confidence = self._predict_line_trajectory()
        if confidence > WAYPOINT_CONFIG['confidence_threshold']:
            pred_x = int((expected_pos + 1) / 2 * frame.shape[1])
            cv2.circle(frame, (pred_x, 10), 5, (255, 0, 255), -1)
        
        cv2.putText(frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    def cleanup(self):
        """Stop camera and clean up."""
        self.picam2.stop()
        cv2.destroyAllWindows()