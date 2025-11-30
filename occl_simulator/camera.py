#!/usr/bin/env python

"""
Camera Manager - Handles RGB and Depth camera sensors
Provides live frames for vision-based pedestrian detection
"""

import carla
import os
import cv2
import numpy as np
from datetime import datetime


class CameraManager:
    """Manages RGB and Depth camera sensors for ego vehicle with HUD overlay"""

    def __init__(self, world, output_dir='output', save_frames=True):
        self.world = world
        self.camera = None
        self.depth_camera = None  # Depth camera for 3D position estimation
        self.ego_vehicle = None  # Reference to ego vehicle for speed display

        self.output_dir = output_dir
        self.save_frames = save_frames

        # Live frame buffers
        self.latest_frame = None       # RGB numpy array (BGR)
        self.latest_frame_raw = None   # RGB without HUD overlay
        self.latest_depth = None       # Depth numpy array (meters)

        # Standard camera settings
        self.image_width = 960
        self.image_height = 540
        self.fov = 110  # Field of view in degrees
        self.fps = 20   # 20 FPS camera

        # For optional frame saving
        self.image_dir = None
        self.frame_count = 0
        self.frame_timestamps = []

        # For optional direct camera-only video
        self.video_writer = None
        self.video_path = None
        
        # HUD settings
        self.show_hud = True
        self.hud_metrics = {}  # Dictionary to store metrics for display
        
        # Pedestrian detection overlay
        self.detection_overlay = None  # Will be set by controller

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    # -----------------------------------------------------------
    # Attach cameras to ego vehicle
    # -----------------------------------------------------------
    def setup_camera(self, vehicle):
        self.ego_vehicle = vehicle  # Store reference for speed display
        
        blueprint_library = self.world.get_blueprint_library()
        
        # ----- RGB Camera -----
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.image_width))
        camera_bp.set_attribute('image_size_y', str(self.image_height))
        camera_bp.set_attribute('fov', str(self.fov))
        camera_bp.set_attribute('sensor_tick', '0.05')  # 20Hz

        # Camera transform: front, slightly raised
        camera_transform = carla.Transform(
            carla.Location(x=2.5, z=1.0),
            carla.Rotation(pitch=0.0)
        )

        self.camera = self.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle
        )
        
        # ----- Depth Camera (same position as RGB) -----
        depth_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(self.image_width))
        depth_bp.set_attribute('image_size_y', str(self.image_height))
        depth_bp.set_attribute('fov', str(self.fov))
        depth_bp.set_attribute('sensor_tick', '0.05')  # 20Hz
        
        self.depth_camera = self.world.spawn_actor(
            depth_bp,
            camera_transform,
            attach_to=vehicle
        )

        # Timestamp for folders
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.save_frames:
            # Folder for raw PNG frames
            self.image_dir = os.path.join(self.output_dir, f"frames_{timestamp}")
            os.makedirs(self.image_dir)

            # Direct video writer for raw camera feed
            self.video_path = os.path.join(self.output_dir, f"video_{timestamp}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                self.video_path, fourcc, self.fps,
                (self.image_width, self.image_height)
            )

        # Start listening to camera feeds
        self.camera.listen(self._on_camera_update)
        self.depth_camera.listen(self._on_depth_update)

        print("RGB Camera attached.")
        print("Depth Camera attached.")
        if self.save_frames:
            print(f"Saving raw CARLA frames to: {self.image_dir}")
            print(f"Saving raw video to: {self.video_path}")
    
    def get_camera_intrinsics(self):
        """Return camera intrinsic parameters for 3D projection"""
        return {
            'width': self.image_width,
            'height': self.image_height,
            'fov': self.fov
        }
    
    def get_raw_frame(self):
        """Get the latest RGB frame without HUD overlay (for YOLO detection)"""
        return self.latest_frame_raw
    
    def get_depth_frame(self):
        """Get the latest depth frame (values in meters)"""
        return self.latest_depth
    
    def set_detection_overlay(self, overlay_frame):
        """Set a frame with detection boxes to be composited onto HUD"""
        self.detection_overlay = overlay_frame

    # -----------------------------------------------------------
    # Depth camera callback
    # -----------------------------------------------------------
    def _on_depth_update(self, image):
        """Process depth camera frame"""
        # Convert CARLA depth image to meters
        # CARLA encodes depth as (R + G*256 + B*256*256) / (256^3 - 1) * 1000
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        
        # Convert BGRA to depth in meters
        # Normalized depth = (R + G*256 + B*256*256) / (256^3 - 1)
        # Depth in meters = normalized * 1000
        array = array.astype(np.float32)
        depth = (array[:, :, 2] + array[:, :, 1] * 256.0 + array[:, :, 0] * 256.0 * 256.0)
        depth = depth / (256.0 * 256.0 * 256.0 - 1.0)
        depth = depth * 1000.0  # Convert to meters
        
        self.latest_depth = depth

    # -----------------------------------------------------------
    # HUD Rendering Methods
    # -----------------------------------------------------------
    def update_metrics(self, metrics_dict):
        """Update HUD metrics from controller
        
        Args:
            metrics_dict: Dictionary with keys like 'speed_kmh', 'risk_level', etc.
        """
        self.hud_metrics.update(metrics_dict)
    
    def _get_ego_speed(self):
        """Get current ego vehicle speed in km/h"""
        if self.ego_vehicle is None:
            return 0.0
        velocity = self.ego_vehicle.get_velocity()
        speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return speed_ms * 3.6  # Convert to km/h
    
    def _render_hud(self, frame):
        """Render HUD overlay on frame
        
        Args:
            frame: BGR numpy array
            
        Returns:
            frame with HUD overlay
        """
        if not self.show_hud:
            return frame
        
        frame = frame.copy()
        h, w = frame.shape[:2]
        
        # Get speed (either from metrics or directly from vehicle)
        speed_kmh = self.hud_metrics.get('speed_kmh', self._get_ego_speed())
        
        # --- Speed Display (Top-Left) ---
        # Background box
        cv2.rectangle(frame, (10, 10), (200, 80), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (200, 80), (255, 255, 255), 2)
        
        # Speed value
        speed_text = f"{speed_kmh:.1f}"
        cv2.putText(frame, speed_text, (20, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        cv2.putText(frame, "km/h", (140, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # --- Risk Level Display (Top-Right, if available) ---
        risk_level = self.hud_metrics.get('risk_level')
        if risk_level is not None:
            risk_color = self._get_risk_color(risk_level)
            risk_text = f"Risk: {risk_level:.0%}"
            
            # Background box
            cv2.rectangle(frame, (w-210, 10), (w-10, 80), (0, 0, 0), -1)
            cv2.rectangle(frame, (w-210, 10), (w-10, 80), risk_color, 2)
            
            cv2.putText(frame, risk_text, (w-200, 55), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, risk_color, 2)
        
        # --- Controller Mode (Bottom-Left, if available) ---
        mode = self.hud_metrics.get('mode')
        if mode:
            cv2.rectangle(frame, (10, h-50), (250, h-10), (0, 0, 0), -1)
            cv2.putText(frame, mode, (20, h-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # --- Target Speed (Below main speed, if different) ---
        target_speed = self.hud_metrics.get('target_speed_kmh')
        if target_speed is not None and abs(target_speed - speed_kmh) > 1.0:
            cv2.putText(frame, f"Target: {target_speed:.0f} km/h", (20, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
        
        # --- Pedestrian Alert (Center, if detected) ---
        ped_in_path = self.hud_metrics.get('pedestrian_in_path', False)
        ped_distance = self.hud_metrics.get('pedestrian_distance')
        num_peds = self.hud_metrics.get('num_pedestrians', 0)
        
        if ped_in_path:
            # EMERGENCY ALERT - Pedestrian in path
            alert_text = "PEDESTRIAN!"
            alert_color = (0, 0, 255)  # Red
            
            # Large flashing alert box in center-top
            box_w, box_h = 300, 60
            box_x = (w - box_w) // 2
            box_y = 90
            
            cv2.rectangle(frame, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 0, 0), -1)
            cv2.rectangle(frame, (box_x, box_y), (box_x + box_w, box_y + box_h), alert_color, 3)
            
            cv2.putText(frame, alert_text, (box_x + 30, box_y + 42), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, alert_color, 3)
            
            if ped_distance is not None:
                dist_text = f"{ped_distance:.1f}m"
                cv2.putText(frame, dist_text, (box_x + 220, box_y + 42), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        elif num_peds > 0 and ped_distance is not None:
            # Caution - pedestrians nearby but not directly in path
            caution_text = f"PEDS: {num_peds} ({ped_distance:.0f}m)"
            cv2.putText(frame, caution_text, (w//2 - 80, 110), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return frame
    
    def _get_risk_color(self, risk_level):
        """Get color based on risk level (green -> yellow -> red)"""
        if risk_level < 0.3:
            return (0, 255, 0)  # Green - safe
        elif risk_level < 0.6:
            return (0, 255, 255)  # Yellow - caution
        elif risk_level < 0.85:
            return (0, 165, 255)  # Orange - danger
        else:
            return (0, 0, 255)  # Red - emergency

    # -----------------------------------------------------------
    # Camera callback: convert to numpy and store
    # -----------------------------------------------------------
    def _on_camera_update(self, image):
        """Receive live frames from CARLA"""

        # Convert raw BGRA → BGR numpy
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # BGR

        # Store raw frame (without HUD) for YOLO detection
        self.latest_frame_raw = array.copy()
        
        # If we have detection overlay, use that instead of raw frame
        if self.detection_overlay is not None:
            array = self.detection_overlay.copy()
            self.detection_overlay = None  # Clear after use

        # Apply HUD overlay
        array_with_hud = self._render_hud(array)

        # Store live frame for display (with HUD)
        self.latest_frame = array_with_hud.copy()

        # Optionally write PNG & raw camera video (with HUD)
        if self.save_frames:
            # Save frame with HUD
            img_path = os.path.join(self.image_dir, f"frame_{self.frame_count:06d}.png")
            cv2.imwrite(img_path, array_with_hud)

            if self.video_writer is not None:
                self.video_writer.write(array_with_hud)

        self.frame_count += 1
        self.frame_timestamps.append(image.timestamp)

    # -----------------------------------------------------------
    # Optional: compile PNG frames into a video
    # -----------------------------------------------------------
    def compile_video(self):
        if not self.save_frames:
            print("Saving frames disabled.")
            return None

        if self.image_dir is None or self.frame_count == 0:
            print("No frames to compile.")
            return None

        print("\nCompiling video from raw frames...")

        images = sorted([
            os.path.join(self.image_dir, f)
            for f in os.listdir(self.image_dir)
            if f.endswith(".png")
        ])

        if not images:
            print("No PNG frames found.")
            return None

        frame = cv2.imread(images[0])
        h, w, c = frame.shape

        output_path = os.path.join(
            self.output_dir,
            f"compiled_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        )

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")

        # Estimate FPS by timestamps
        if len(self.frame_timestamps) > 1:
            total_time = self.frame_timestamps[-1] - self.frame_timestamps[0]
            fps = len(images) / total_time
        else:
            fps = self.fps

        writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

        for idx, img_path in enumerate(images):
            frame = cv2.imread(img_path)
            writer.write(frame)
            if idx % 30 == 0:
                print(f"Writing {idx}/{len(images)} frames...")

        writer.release()
        print(f"Compiled video saved to {output_path}")
        return output_path

    # -----------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------
    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            print("\nRGB Camera destroyed.")
        
        if self.depth_camera is not None:
            self.depth_camera.stop()
            self.depth_camera.destroy()
            print("Depth Camera destroyed.")

        if self.video_writer is not None:
            self.video_writer.release()
            print(f"Camera-only video saved to: {self.video_path}")


def main():
    print("CameraManager module. Import and use in scenarios.")


if __name__ == '__main__':
    main()

