#!/usr/bin/env python

"""
Camera Manager - Handles camera sensor and provides live frames
Compatible with the Combined Grid + Camera Video Controller
"""

import carla
import os
import cv2
import numpy as np
from datetime import datetime


class CameraManager:
    """Manages camera sensor for ego vehicle and provides live frames with HUD overlay"""

    def __init__(self, world, output_dir='output', save_frames=True):
        self.world = world
        self.camera = None
        self.ego_vehicle = None  # Reference to ego vehicle for speed display

        self.output_dir = output_dir
        self.save_frames = save_frames

        # Live frame buffer for controller
        self.latest_frame = None   # numpy array (BGR)

        # Standard camera settings
        self.image_width = 960
        self.image_height = 540
        self.fps = 20  # 20 FPS camera

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

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    # -----------------------------------------------------------
    # Attach camera to ego vehicle
    # -----------------------------------------------------------
    def setup_camera(self, vehicle):
        self.ego_vehicle = vehicle  # Store reference for speed display
        
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')

        # Camera attributes
        camera_bp.set_attribute('image_size_x', str(self.image_width))
        camera_bp.set_attribute('image_size_y', str(self.image_height))
        camera_bp.set_attribute('fov', '110')
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

        self.camera.listen(self._on_camera_update)

        print("Camera attached.")
        if self.save_frames:
            print(f"Saving raw CARLA frames to: {self.image_dir}")
            print(f"Saving raw video to: {self.video_path}")

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

        # Apply HUD overlay
        array_with_hud = self._render_hud(array)

        # Store live frame for controller (with HUD)
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
            print("\nCamera destroyed.")

        if self.video_writer is not None:
            self.video_writer.release()
            print(f"Camera-only video saved to: {self.video_path}")


def main():
    print("CameraManager module. Import and use in scenarios.")


if __name__ == '__main__':
    main()

