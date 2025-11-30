#!/usr/bin/env python

"""
Vehicle Detector using YOLOv8 + Depth Camera
Vision-based vehicle detection for oncoming traffic detection in autonomous driving simulation
"""

import numpy as np
import math

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Run: pip install ultralytics")


# COCO class IDs for vehicles
VEHICLE_CLASS_IDS = {
    'car': 2,
    'motorcycle': 3,
    'bus': 5,
    'truck': 7,
}


class VehicleDetector:
    """
    Vision-based vehicle detection using YOLOv8.
    
    Uses RGB camera for detection and depth camera for 3D position estimation.
    Only detects vehicles that are actually visible to the camera.
    Designed for detecting oncoming traffic in intersection scenarios.
    """
    
    def __init__(self, model_name='yolov8n.pt', confidence_threshold=0.5):
        """
        Initialize the vehicle detector.
        
        Args:
            model_name: YOLO model to use (yolov8n.pt is fastest, yolov8x.pt is most accurate)
            confidence_threshold: Minimum confidence for detection
        """
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.model_name = model_name
        
        # COCO class IDs for vehicles
        self.vehicle_class_ids = list(VEHICLE_CLASS_IDS.values())
        
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
            print("YOLO not available - vehicle detection disabled")
            return
        
        try:
            print(f"Loading YOLO model for vehicle detection: {self.model_name}")
            self.model = YOLO(self.model_name)
            print("YOLO model loaded successfully for vehicle detection")
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
        fov_rad = math.radians(fov_degrees)
        self.fx = width / (2.0 * math.tan(fov_rad / 2.0))
        self.fy = self.fx  # Assuming square pixels
        
        # Principal point at image center
        self.cx = width / 2.0
        self.cy = height / 2.0
    
    def detect(self, rgb_frame):
        """
        Detect vehicles in RGB frame.
        
        Args:
            rgb_frame: BGR numpy array from camera
            
        Returns:
            List of detections, each with:
            - bbox: (x1, y1, x2, y2) pixel coordinates
            - confidence: detection confidence
            - center: (cx, cy) center pixel
            - class_id: COCO class ID
            - class_name: vehicle type name
        """
        if rgb_frame is None or self.model is None:
            return []
        
        detections = []
        
        try:
            # Run YOLO inference on vehicle classes only
            results = self.model(rgb_frame, verbose=False, classes=self.vehicle_class_ids)
            
            for result in results:
                boxes = result.boxes
                
                if boxes is None:
                    continue
                
                for i in range(len(boxes)):
                    conf = float(boxes.conf[i])
                    
                    if conf < self.confidence_threshold:
                        continue
                    
                    cls = int(boxes.cls[i])
                    
                    # Get vehicle type name
                    class_name = 'vehicle'
                    for name, cid in VEHICLE_CLASS_IDS.items():
                        if cid == cls:
                            class_name = name
                            break
                    
                    # Get bounding box
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                    
                    detection = {
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': conf,
                        'center': (int((x1 + x2) / 2), int((y1 + y2) / 2)),
                        'width': int(x2 - x1),
                        'height': int(y2 - y1),
                        'class_id': cls,
                        'class_name': class_name
                    }
                    detections.append(detection)
            
        except Exception as e:
            print(f"YOLO vehicle detection error: {e}")
        
        self.last_detections = detections
        return detections
    
    def estimate_3d_positions(self, detections, depth_frame):
        """
        Estimate 3D positions of detected vehicles using depth camera.
        
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
            
            # Sample depth at center of bounding box for vehicles
            sample_y = cy
            sample_x = cx
            
            # Get depth value (in meters)
            # Sample a small region and take median for robustness
            y_start = max(0, sample_y - 10)
            y_end = min(depth_frame.shape[0], sample_y + 10)
            x_start = max(0, sample_x - 10)
            x_end = min(depth_frame.shape[1], sample_x + 10)
            
            depth_region = depth_frame[y_start:y_end, x_start:x_end]
            
            # Filter out invalid depths (0 or very large)
            valid_depths = depth_region[(depth_region > 0.1) & (depth_region < 150)]
            
            if len(valid_depths) == 0:
                continue
            
            depth = np.median(valid_depths)
            
            # Back-project to 3D using pinhole camera model
            z = depth  # Forward distance
            x = (cx - self.cx) * z / self.fx  # Right (+) / Left (-)
            y = (cy - self.cy) * z / self.fy  # Down (+) / Up (-)
            
            position_3d = {
                'x': x,
                'y': y,
                'z': z,
                'distance': z,
                'detection': det,
                'lateral_offset': x,
                'class_name': det.get('class_name', 'vehicle'),
                'class_id': det.get('class_id', 2)
            }
            positions_3d.append(position_3d)
        
        self.last_3d_positions = positions_3d
        return positions_3d
    
    def get_vehicles_in_path(self, positions_3d, lane_width=4.0, stop_distance=20.0):
        """
        Determine which vehicles are in the ego vehicle's same lane.
        
        Args:
            positions_3d: List of 3D positions from estimate_3d_positions()
            lane_width: Width of lane to consider (meters)
            stop_distance: Distance threshold for emergency stop
            
        Returns:
            - vehicles_in_path: List of vehicles in the driving path
            - should_stop: Boolean - True if emergency stop needed
            - closest_distance: Distance to closest vehicle in path
        """
        vehicles_in_path = []
        should_stop = False
        closest_distance = float('inf')
        
        half_lane = lane_width / 2.0
        
        for pos in positions_3d:
            # Check if vehicle is ahead (positive z)
            if pos['z'] <= 0:
                continue
            
            # Check if vehicle is within lane width
            if abs(pos['lateral_offset']) < half_lane:
                pos['in_path'] = True
                pos['threat_type'] = 'same_lane'
                vehicles_in_path.append(pos)
                
                if pos['distance'] < closest_distance:
                    closest_distance = pos['distance']
                
                if pos['distance'] < stop_distance:
                    should_stop = True
            else:
                pos['in_path'] = False
        
        return vehicles_in_path, should_stop, closest_distance
    
    def get_oncoming_vehicles(self, positions_3d, lateral_range=(-10.0, -2.0), stop_distance=25.0):
        """
        Detect oncoming vehicles (for left turn scenarios).
        
        Args:
            positions_3d: List of 3D positions from estimate_3d_positions()
            lateral_range: (min, max) lateral offset range for oncoming lane
                          Negative values = left side of ego
            stop_distance: Distance threshold for emergency stop
            
        Returns:
            - oncoming_vehicles: List of oncoming vehicles
            - should_stop: Boolean - True if emergency stop needed
            - closest_distance: Distance to closest oncoming vehicle
        """
        oncoming_vehicles = []
        should_stop = False
        closest_distance = float('inf')
        
        for pos in positions_3d:
            # Check if vehicle is ahead (positive z)
            if pos['z'] <= 0:
                continue
            
            lateral = pos['lateral_offset']
            distance = pos['distance']
            
            # Check if in oncoming lane (left side)
            if lateral_range[0] <= lateral <= lateral_range[1]:
                pos['in_path'] = True
                pos['threat_type'] = 'oncoming'
                oncoming_vehicles.append(pos)
                
                if distance < closest_distance:
                    closest_distance = distance
                
                if distance < stop_distance:
                    should_stop = True
        
        return oncoming_vehicles, should_stop, closest_distance
    
    def get_all_vehicle_threats(self, positions_3d, lane_width=4.0, same_lane_stop=20.0,
                                 check_oncoming=True, oncoming_lateral_range=(-10.0, -2.0),
                                 oncoming_stop_distance=25.0):
        """
        Get all vehicle threats in one call.
        
        Returns:
            dict with keys:
            - vehicles_in_path: List of vehicles in same lane
            - oncoming_vehicles: List of oncoming vehicles
            - should_stop_same_lane: Boolean for same lane emergency
            - should_stop_oncoming: Boolean for oncoming emergency
            - should_stop: Boolean for any emergency
            - closest_same_lane_distance: Closest same lane vehicle distance
            - closest_oncoming_distance: Closest oncoming vehicle distance
        """
        # Get same lane threats
        same_lane, same_stop, same_dist = self.get_vehicles_in_path(
            positions_3d, lane_width, same_lane_stop
        )
        
        # Get oncoming threats
        oncoming = []
        oncoming_stop = False
        oncoming_dist = float('inf')
        
        if check_oncoming:
            oncoming, oncoming_stop, oncoming_dist = self.get_oncoming_vehicles(
                positions_3d, oncoming_lateral_range, oncoming_stop_distance
            )
        
        return {
            'vehicles_in_path': same_lane,
            'oncoming_vehicles': oncoming,
            'should_stop_same_lane': same_stop,
            'should_stop_oncoming': oncoming_stop,
            'should_stop': same_stop or oncoming_stop,
            'closest_same_lane_distance': same_dist,
            'closest_oncoming_distance': oncoming_dist
        }
    
    def draw_detections(self, frame, positions_3d=None):
        """
        Draw bounding boxes and labels on frame for vehicles.
        
        Args:
            frame: BGR numpy array to draw on
            positions_3d: Optional 3D positions for distance labels
            
        Returns:
            Frame with detections drawn
        """
        import cv2
        
        frame = frame.copy()
        
        if positions_3d:
            for pos in positions_3d:
                det = pos['detection']
                x1, y1, x2, y2 = det['bbox']
                threat_type = pos.get('threat_type', '')
                
                # Color based on threat type
                if pos.get('in_path', False):
                    if threat_type == 'oncoming':
                        color = (0, 0, 255)  # Red - oncoming vehicle DANGER
                    else:
                        color = (0, 100, 255)  # Orange - same lane vehicle
                    thickness = 3
                else:
                    color = (255, 255, 0)  # Cyan - vehicle not in path
                    thickness = 2
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                
                # Draw label
                if threat_type == 'oncoming':
                    label = f"ONCOMING {pos['distance']:.1f}m"
                else:
                    label = f"{det['class_name'].upper()} {pos['distance']:.1f}m"
                    
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
                color = (255, 255, 0)  # Cyan
                label = f"{det['class_name'].upper()} {det['confidence']:.0%}"
                    
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return frame


class FallbackVehicleDetector:
    """
    Fallback detector when YOLO is not available.
    """
    
    def __init__(self):
        self.last_detections = []
        self.last_3d_positions = []
        print("Using fallback vehicle detector (YOLO not available)")
    
    def set_camera_intrinsics(self, width, height, fov_degrees):
        pass
    
    def detect(self, rgb_frame):
        return []
    
    def estimate_3d_positions(self, detections, depth_frame):
        return []
    
    def get_vehicles_in_path(self, positions_3d, lane_width=4.0, stop_distance=20.0):
        return [], False, float('inf')
    
    def get_oncoming_vehicles(self, positions_3d, lateral_range=(-10.0, -2.0), stop_distance=25.0):
        return [], False, float('inf')
    
    def get_all_vehicle_threats(self, positions_3d, **kwargs):
        return {
            'vehicles_in_path': [],
            'oncoming_vehicles': [],
            'should_stop_same_lane': False,
            'should_stop_oncoming': False,
            'should_stop': False,
            'closest_same_lane_distance': float('inf'),
            'closest_oncoming_distance': float('inf')
        }
    
    def draw_detections(self, frame, positions_3d=None):
        return frame


def create_vehicle_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5):
    """
    Factory function to create appropriate vehicle detector.
    
    Args:
        use_yolo: Whether to use YOLO (if available)
        model_name: YOLO model name
        confidence: Detection confidence threshold
        
    Returns:
        VehicleDetector or FallbackVehicleDetector instance
    """
    if use_yolo and YOLO_AVAILABLE:
        return VehicleDetector(model_name, confidence)
    else:
        return FallbackVehicleDetector()
