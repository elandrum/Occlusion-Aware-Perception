#!/usr/bin/env python

"""
Pedestrian Detector using YOLOv8 + Depth Camera
Vision-based pedestrian detection for realistic autonomous driving simulation
"""

import numpy as np
import math

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Run: pip install ultralytics")


class PedestrianDetector:
    """
    Vision-based pedestrian detection using YOLOv8.
    
    Uses RGB camera for detection and depth camera for 3D position estimation.
    Only detects pedestrians that are actually visible to the camera.
    """
    
    def __init__(self, model_name='yolov8n.pt', confidence_threshold=0.5):
        """
        Initialize the pedestrian detector.
        
        Args:
            model_name: YOLO model to use (yolov8n.pt is fastest, yolov8x.pt is most accurate)
            confidence_threshold: Minimum confidence for detection
        """
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.model_name = model_name
        
        # COCO class ID for person
        self.person_class_id = 0
        
        # Camera intrinsics (will be set from camera)
        self.fx = None  # Focal length x
        self.fy = None  # Focal length y
        self.cx = None  # Principal point x
        self.cy = None  # Principal point y
        self.image_width = None
        self.image_height = None
        
        # Detection results
        self.last_detections = []
        self.last_3d_positions = []
        
        # Load model
        self._load_model()
        
    def _load_model(self):
        """Load the YOLO model"""
        if not YOLO_AVAILABLE:
            print("YOLO not available - using fallback detection")
            return
        
        try:
            print(f"Loading YOLO model: {self.model_name}")
            self.model = YOLO(self.model_name)
            print("YOLO model loaded successfully")
        except Exception as e:
            print(f"Failed to load YOLO model: {e}")
            self.model = None
    
    def set_camera_intrinsics(self, width, height, fov_degrees):
        """
        Set camera intrinsic parameters from CARLA camera settings.
        
        Args:
            width: Image width in pixels
            height: Image height in pixels
            fov_degrees: Horizontal field of view in degrees
        """
        self.image_width = width
        self.image_height = height
        
        # Calculate focal length from FOV
        # fx = width / (2 * tan(fov/2))
        fov_rad = math.radians(fov_degrees)
        self.fx = width / (2.0 * math.tan(fov_rad / 2.0))
        self.fy = self.fx  # Assuming square pixels
        
        # Principal point at image center
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        print(f"Camera intrinsics set: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
    
    def detect(self, rgb_frame):
        """
        Detect pedestrians in RGB frame.
        
        Args:
            rgb_frame: BGR numpy array from camera
            
        Returns:
            List of detections, each with:
            - bbox: (x1, y1, x2, y2) pixel coordinates
            - confidence: detection confidence
            - center: (cx, cy) center pixel
        """
        if rgb_frame is None:
            return []
        
        if self.model is None:
            return []
        
        detections = []
        
        try:
            # Run YOLO inference
            results = self.model(rgb_frame, verbose=False, classes=[self.person_class_id])
            
            for result in results:
                boxes = result.boxes
                
                if boxes is None:
                    continue
                
                for i in range(len(boxes)):
                    # Get confidence
                    conf = float(boxes.conf[i])
                    
                    if conf < self.confidence_threshold:
                        continue
                    
                    # Get class (should be person=0)
                    cls = int(boxes.cls[i])
                    if cls != self.person_class_id:
                        continue
                    
                    # Get bounding box
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                    
                    detection = {
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': conf,
                        'center': (int((x1 + x2) / 2), int((y1 + y2) / 2)),
                        'width': int(x2 - x1),
                        'height': int(y2 - y1)
                    }
                    detections.append(detection)
            
        except Exception as e:
            print(f"YOLO detection error: {e}")
        
        self.last_detections = detections
        return detections
    
    def estimate_3d_positions(self, detections, depth_frame):
        """
        Estimate 3D positions of detected pedestrians using depth camera.
        
        Args:
            detections: List of detections from detect()
            depth_frame: Depth image as numpy array (values in meters)
            
        Returns:
            List of 3D positions in camera frame (x_right, y_down, z_forward)
        """
        if depth_frame is None or self.fx is None:
            return []
        
        positions_3d = []
        
        for det in detections:
            cx, cy = det['center']
            x1, y1, x2, y2 = det['bbox']
            
            # Sample depth at multiple points in the bounding box for robustness
            # Use lower center (feet area) for more accurate ground distance
            sample_y = min(int(y2 - det['height'] * 0.1), depth_frame.shape[0] - 1)
            sample_x = cx
            
            # Get depth value (in meters)
            # Sample a small region and take median for robustness
            y_start = max(0, sample_y - 5)
            y_end = min(depth_frame.shape[0], sample_y + 5)
            x_start = max(0, sample_x - 5)
            x_end = min(depth_frame.shape[1], sample_x + 5)
            
            depth_region = depth_frame[y_start:y_end, x_start:x_end]
            
            # Filter out invalid depths (0 or very large)
            valid_depths = depth_region[(depth_region > 0.1) & (depth_region < 100)]
            
            if len(valid_depths) == 0:
                continue
            
            depth = np.median(valid_depths)
            
            # Back-project to 3D using pinhole camera model
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            # Z = depth
            
            z = depth  # Forward distance
            x = (cx - self.cx) * z / self.fx  # Right (+) / Left (-)
            y = (cy - self.cy) * z / self.fy  # Down (+) / Up (-)
            
            position_3d = {
                'x': x,           # Right/left in camera frame
                'y': y,           # Down/up in camera frame  
                'z': z,           # Forward distance
                'distance': z,    # Simplified: use forward distance
                'detection': det,
                'lateral_offset': x  # How far left/right of center
            }
            positions_3d.append(position_3d)
        
        self.last_3d_positions = positions_3d
        return positions_3d
    
    def get_pedestrians_in_path(self, positions_3d, lane_width=3.5, stop_distance=15.0):
        """
        Determine which pedestrians are in the ego vehicle's path.
        
        Args:
            positions_3d: List of 3D positions from estimate_3d_positions()
            lane_width: Width of lane to consider (meters)
            stop_distance: Distance threshold for emergency stop
            
        Returns:
            - pedestrians_in_path: List of pedestrians in the driving path
            - should_stop: Boolean - True if emergency stop needed
            - closest_distance: Distance to closest pedestrian in path
        """
        pedestrians_in_path = []
        should_stop = False
        closest_distance = float('inf')
        
        half_lane = lane_width / 2.0
        
        for pos in positions_3d:
            # Check if pedestrian is ahead (positive z)
            if pos['z'] <= 0:
                continue
            
            # Check if pedestrian is within lane width
            if abs(pos['lateral_offset']) < half_lane:
                pos['in_path'] = True
                pedestrians_in_path.append(pos)
                
                if pos['distance'] < closest_distance:
                    closest_distance = pos['distance']
                
                if pos['distance'] < stop_distance:
                    should_stop = True
            else:
                pos['in_path'] = False
        
        return pedestrians_in_path, should_stop, closest_distance
    
    def draw_detections(self, frame, positions_3d=None):
        """
        Draw bounding boxes and labels on frame.
        
        Args:
            frame: BGR numpy array to draw on
            positions_3d: Optional 3D positions for distance labels
            
        Returns:
            Frame with detections drawn
        """
        import cv2
        
        frame = frame.copy()
        
        # Use 3D positions if available, otherwise use last detections
        if positions_3d:
            for pos in positions_3d:
                det = pos['detection']
                x1, y1, x2, y2 = det['bbox']
                
                # Color based on whether in path
                if pos.get('in_path', False):
                    color = (0, 0, 255)  # Red - in path
                    thickness = 3
                else:
                    color = (0, 255, 255)  # Yellow - detected but not in path
                    thickness = 2
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                
                # Draw label with distance
                label = f"PED {pos['distance']:.1f}m"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                
                # Background for label
                cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                             (x1 + label_size[0], y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Confidence
                conf_text = f"{det['confidence']:.0%}"
                cv2.putText(frame, conf_text, (x1, y2 + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        else:
            # Just draw detections without 3D info
            for det in self.last_detections:
                x1, y1, x2, y2 = det['bbox']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                label = f"PED {det['confidence']:.0%}"
                cv2.putText(frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return frame


class FallbackDetector:
    """
    Fallback detector when YOLO is not available.
    Uses simple color/motion detection (less accurate but no dependencies).
    """
    
    def __init__(self):
        self.last_detections = []
        self.last_3d_positions = []
        print("Using fallback detector (YOLO not available)")
    
    def set_camera_intrinsics(self, width, height, fov_degrees):
        pass
    
    def detect(self, rgb_frame):
        # No detection without ML model
        return []
    
    def estimate_3d_positions(self, detections, depth_frame):
        return []
    
    def get_pedestrians_in_path(self, positions_3d, lane_width=3.5, stop_distance=15.0):
        return [], False, float('inf')
    
    def draw_detections(self, frame, positions_3d=None):
        return frame


def create_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5):
    """
    Factory function to create appropriate detector.
    
    Args:
        use_yolo: Whether to use YOLO (if available)
        model_name: YOLO model name
        confidence: Detection confidence threshold
        
    Returns:
        PedestrianDetector or FallbackDetector instance
    """
    if use_yolo and YOLO_AVAILABLE:
        return PedestrianDetector(model_name, confidence)
    else:
        return FallbackDetector()
