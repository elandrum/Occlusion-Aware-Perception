import carla
import math
import numpy as np
import cv2
import os
from datetime import datetime


class VehicleController:
    def __init__(self, world):
        self.world = world
        self.ego_vehicle = None
        self.ego_speed_kmh = 30
        self.ego_control = carla.VehicleControl()
        self.ego_moving = False
        self.occluder_actors = None
        # ----- Occlusion grid settings -----
        self.grid_world_size = 20.0      # meters (width/height of square around ego)
        self.grid_cells = 40             # 40 x 40 cells
        self.ray_step = 0.5              # meters per sample along ray
        self.max_range = self.grid_world_size / 2.0
        self.pedestrian_actors = [] 

        # Video writer
        os.makedirs("output", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.grid_video_path = f"output/grid_{timestamp}.mp4"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.frame_size_px = 400  # 400x400 output
        self.grid_writer = cv2.VideoWriter(
            self.grid_video_path,
            fourcc,
            20,                        # fps
            (self.frame_size_px, self.frame_size_px),
        )
        self.grid_frame_count = 0

    def set_occluders(self, actors):
        """Set list of CARLA actors that should be treated as occluders."""
        self.occluder_actors = list(actors) if actors is not None else []
        
    def set_pedestrians(self, actors):
        """Set pedestrians whose positions should be drawn on the occlusion grid."""
        self.pedestrian_actors = list(actors) if actors is not None else []

    def _get_blocking_rects(self):
        """
        Return blockers as axis-aligned rectangles in world XY:
            (min_x, max_x, min_y, max_y)

        Uses the actors provided via set_occluders(). If none are set,
        it falls back to "all vehicles except ego" so old behavior still works.
        """
        rects = []

        # Prefer explicitly provided occluders
        actors = self.occluder_actors
        if actors is None and (self.world is not None):
            actors = self.world.get_actors().filter("vehicle.*")

        ego_loc = None
        if self.ego_vehicle is not None:
            ego_loc = self.ego_vehicle.get_location()

        for a in actors:
            # Ignore ego if it sneaks into the list
            if self.ego_vehicle is not None and a.id == self.ego_vehicle.id:
                continue

            if not hasattr(a, "bounding_box"):
                continue  # e.g. pedestrian, light, etc.

            bb = a.bounding_box
            tf = a.get_transform()

            center = tf.transform(bb.location)

            pad = 1.1
            half_x = bb.extent.x * pad
            half_y = bb.extent.y * pad

            min_x = center.x - half_x
            max_x = center.x + half_x
            min_y = center.y - half_y
            max_y = center.y + half_y

            # If ego is inside this rectangle, skip it to avoid "everything red"
            if ego_loc is not None:
                if (min_x <= ego_loc.x <= max_x) and (min_y <= ego_loc.y <= max_y):
                    continue

            rects.append((min_x, max_x, min_y, max_y))

        return rects

    def _compute_occlusion_grid(self):
        """
        Returns a (H, W) array with:
          0 = free
          1 = occluded
        in an ego-centric top-down frame.
        """
        H = W = self.grid_cells
        grid = np.zeros((H, W), dtype=np.uint8)

        if self.ego_vehicle is None:
            return grid  # all zeros

        ego_tf = self.ego_vehicle.get_transform()
        ego_loc = ego_tf.location
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()

        # Rect blockers instead of circles
        blockers = self._get_blocking_rects()

        half_world = self.grid_world_size / 2.0
        cell_size = self.grid_world_size / self.grid_cells

        cx = W / 2.0
        cy = H / 2.0

        for iy in range(H):
            for ix in range(W):
                # cell center in ego-local frame (x: right, y: forward)
                x_local = (ix + 0.5 - cx) * cell_size
                y_local = (cy - (iy + 0.5)) * cell_size

                dist = math.sqrt(x_local * x_local + y_local * y_local)

                # Skip outside range
                if dist > self.max_range or dist < 1e-2:
                    continue

                dx_local = x_local / dist
                dy_local = y_local / dist

                # ego-local → world direction
                dir_world_x = dx_local * right.x + dy_local * fwd.x
                dir_world_y = dx_local * right.y + dy_local * fwd.y

                occluded = False

                # Start a bit away from ego to avoid self-blocking
                s = 1.0
                while s <= dist:
                    wx = ego_loc.x + s * dir_world_x
                    wy = ego_loc.y + s * dir_world_y

                    # Check against all rectangular blockers
                    for (min_x, max_x, min_y, max_y) in blockers:
                        if (min_x <= wx <= max_x) and (min_y <= wy <= max_y):
                            occluded = True
                            break

                    if occluded:
                        break

                    s += self.ray_step

                if occluded:
                    grid[iy, ix] = 1

        return grid



    # -------------------------------------------------------
    # Rendering occlusion grid to an image
    # -------------------------------------------------------

    def _render_occlusion_grid(self):
        """
        Compute occlusion grid and convert to a 400x400 RGB image.
        free  -> dark gray
        occ   -> red
        ego   -> green dot at center
        """
        grid = self._compute_occlusion_grid()
        H, W = grid.shape
        img = np.zeros((self.frame_size_px, self.frame_size_px, 3), dtype=np.uint8)

        # map grid cells to pixels
        scale = self.frame_size_px / self.grid_cells

        # draw cells
        for iy in range(H):
            for ix in range(W):
                y0 = int(iy * scale)
                y1 = int((iy + 1) * scale)
                x0 = int(ix * scale)
                x1 = int((ix + 1) * scale)

                if grid[iy, ix] == 1:
                    color = (0, 0, 255)      # occluded: red
                else:
                    color = (50, 50, 50)     # free: dark gray

                img[y0:y1, x0:x1] = color

        # draw ego at center
        center_px = self.frame_size_px // 2
        cv2.circle(img, (center_px, center_px), 6, (0, 255, 0), -1)

        # write video frame
        if self.grid_writer is not None:
            self.grid_writer.write(img)
            self.grid_frame_count += 1

    # -------------------------------------------------------
    # Basic speed controller
    # -------------------------------------------------------

    def set_ego_vehicle(self, vehicle, speed_kmh=30):
        self.ego_vehicle = vehicle
        self.ego_speed_kmh = speed_kmh
        print(f"Ego vehicle control initialized at {speed_kmh} km/h")

    def start_movement(self):
        self.ego_moving = True
        print("\nEgo vehicle starting to move...")

    def stop_movement(self):
        self.ego_moving = False
        self.ego_control.throttle = 0.0
        self.ego_control.brake = 1.0
        if self.ego_vehicle:
            self.ego_vehicle.apply_control(self.ego_control)
        print("\nEgo vehicle stopped")

    def update(self, *_, **__):
        """
        Update ego vehicle and record occlusion grid frame.
        """
        # always render grid if we know ego pose
        if self.ego_vehicle is not None:
            self._render_occlusion_grid()

        if not self.ego_moving or not self.ego_vehicle:
            return

        target_speed_ms = self.ego_speed_kmh / 3.6
        vel = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

        if current_speed < target_speed_ms:
            self.ego_control.throttle = 0.7
            self.ego_control.brake = 0.0
        else:
            self.ego_control.throttle = 0.3
            self.ego_control.brake = 0.0

        self.ego_control.steer = 0.0
        self.ego_vehicle.apply_control(self.ego_control)

    def destroy(self):
        if self.grid_writer is not None:
            self.grid_writer.release()
            print(
                f"[Occlusion Grid Video] Saved {self.grid_frame_count} frames to "
                f"{self.grid_video_path}"
            )
