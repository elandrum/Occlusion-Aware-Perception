#!/usr/bin/env python

"""
Camera Manager - Handles camera sensor and image capture
"""

import carla
import os
import cv2
import numpy as np
from datetime import datetime


class CameraManager:
    """Manages camera sensor for ego vehicle and video recording"""
    
    def __init__(self, world, output_dir='output'):
        self.world = world
        self.camera = None
        self.output_dir = output_dir
        self.image_dir = None
        self.frame_count = 0
        self.image_width = 960
        self.image_height = 540
        self.fps = 20  # Higher frame rate for smoother video
        self.frame_timestamps = []  # Track actual frame times
        self.video_writer = None  # Direct video writer
        self.video_path = None
        
        # Create output directory
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    
    def setup_camera(self, vehicle):
        """Attach camera sensor to vehicle"""
        # Get camera blueprint
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        
        # Set camera attributes
        camera_bp.set_attribute('image_size_x', str(self.image_width))
        camera_bp.set_attribute('image_size_y', str(self.image_height))
        camera_bp.set_attribute('fov', '110')
        camera_bp.set_attribute('sensor_tick', '0.05')  # Capture every 0.05 seconds (20 FPS)
        
        # Camera transform (front of vehicle)
        camera_transform = carla.Transform(
            carla.Location(x=2.5, z=1.0),  # Front bumper, slightly elevated
            carla.Rotation(pitch=0.0)
        )
        
        # Spawn camera
        self.camera = self.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle
        )
        
        # Create session directory for images
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.image_dir = os.path.join(self.output_dir, f'frames_{timestamp}')
        os.makedirs(self.image_dir)
        
        # Setup direct video writer
        self.video_path = os.path.join(self.output_dir, f'video_{timestamp}.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            self.video_path, 
            fourcc, 
            self.fps, 
            (self.image_width, self.image_height)
        )
        
        # Start listening
        self.camera.listen(self._on_camera_update)
        
        print(f"Camera attached to vehicle")
        print(f"Saving frames to: {self.image_dir}")
        print(f"Saving video to: {self.video_path}")
        print(f"Resolution: {self.image_width}x{self.image_height}")
    
    def _on_camera_update(self, image):
        """Callback when camera captures an image"""
        # Convert CARLA image to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))  # BGRA format
        array = array[:, :, :3]  # Remove alpha channel, keep BGR
        
        # Write frame directly to video
        if self.video_writer is not None:
            self.video_writer.write(array)
        
        # Also save individual frame as PNG
        image_path = os.path.join(self.image_dir, f'frame_{self.frame_count:06d}.png')
        image.save_to_disk(image_path)
        
        # Track timestamp
        self.frame_timestamps.append(image.timestamp)
        self.frame_count += 1
    
    def compile_video(self, output_filename=None):
        """Compile saved images into video"""
        if self.image_dir is None or self.frame_count == 0:
            print("No frames to compile!")
            return None
        
        # Generate output filename
        if output_filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_filename = f'video_{timestamp}.mp4'
        
        output_path = os.path.join(self.output_dir, output_filename)
        
        print(f"\nCompiling video from {self.frame_count} frames...")
        print(f"Output: {output_path}")
        
        # Get list of images
        images = sorted([
            os.path.join(self.image_dir, img)
            for img in os.listdir(self.image_dir)
            if img.endswith('.png')
        ])
        
        if not images:
            print("No images found!")
            return None
        
        # Read first image to get dimensions
        frame = cv2.imread(images[0])
        height, width, layers = frame.shape
        
        # Calculate actual FPS from timestamps
        actual_fps = self.fps  # Default
        if len(self.frame_timestamps) > 1:
            total_time = self.frame_timestamps[-1] - self.frame_timestamps[0]
            actual_fps = (len(self.frame_timestamps) - 1) / total_time
            print(f"Actual capture rate: {actual_fps:.2f} fps")
            print(f"Total duration: {total_time:.2f} seconds")
        
        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(output_path, fourcc, actual_fps, (width, height))
        
        # Write frames to video
        for i, image_path in enumerate(images):
            frame = cv2.imread(image_path)
            video.write(frame)
            
            # Progress indicator
            if (i + 1) % 30 == 0:
                print(f"Processed {i + 1}/{len(images)} frames...")
        
        video.release()
        
        print(f"Video compiled successfully!")
        print(f"Total frames: {len(images)}")
        print(f"Duration: {len(images)/actual_fps:.2f} seconds")
        print(f"Video saved to: {output_path}")
        
        return output_path
    
    def get_frame_count(self):
        """Get number of frames captured"""
        return self.frame_count
    
    def destroy(self):
        """Clean up camera sensor"""
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            print(f"\nCamera destroyed. Total frames captured: {self.frame_count}")
        
        # Close video writer
        if self.video_writer is not None:
            self.video_writer.release()
            
            # Calculate actual duration
            if len(self.frame_timestamps) > 1:
                total_time = self.frame_timestamps[-1] - self.frame_timestamps[0]
                actual_fps = (len(self.frame_timestamps) - 1) / total_time
                print(f"\nVideo Recording Complete:")
                print(f"  File: {self.video_path}")
                print(f"  Frames: {self.frame_count}")
                print(f"  Actual FPS: {actual_fps:.2f}")
                print(f"  Duration: {total_time:.2f} seconds")


def main():
    """Test camera manager"""
    print("CameraManager is a module to be used with scenarios")
    print("Import it in your scenario and use setup_camera() to attach to vehicle")


if __name__ == '__main__':
    main()
