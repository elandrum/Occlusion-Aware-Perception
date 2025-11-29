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
    """Manages camera sensor for ego vehicle and provides live frames"""

    def __init__(self, world, output_dir='output', save_frames=True):
        self.world = world
        self.camera = None

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

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    # -----------------------------------------------------------
    # Attach camera to ego vehicle
    # -----------------------------------------------------------
    def setup_camera(self, vehicle):
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
    # Camera callback: convert to numpy and store
    # -----------------------------------------------------------
    def _on_camera_update(self, image):
        """Receive live frames from CARLA"""

        # Convert raw BGRA → BGR numpy
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # BGR

        # Store live frame for controller
        self.latest_frame = array.copy()

        # Optionally write PNG & raw camera video
        if self.save_frames:
            img_path = os.path.join(self.image_dir, f"frame_{self.frame_count:06d}.png")
            image.save_to_disk(img_path)

            if self.video_writer is not None:
                self.video_writer.write(array)

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

