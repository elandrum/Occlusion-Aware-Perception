import carla
import math
import numpy as np
import cv2
import os
from datetime import datetime


class VehicleController:
    """
    VehicleController

    Modes:
      - "naive": basic cruise controller (ignores occlusion)
      - "occl":  occlusion-aware controller
                 * Slows down when a lot of space ahead is occluded
                 * Performs hard brake if a large occluded region is very close
    """

    def __init__(self, world, mode: str = "naive"):
        self.world = world
        self.mode = mode

        # Ego state
        self.ego_vehicle = None
        self.ego_speed_kmh = 25.0
        self.ego_control = carla.VehicleControl()
        self.ego_moving = False

        # Occluder actors (vehicles / trucks, etc.)
        self.occluder_actors = None

        # ----- Occlusion grid settings -----
        self.grid_world_size = 20.0      # meters (width/height of square around ego)
        self.grid_cells = 40             # 40 x 40 grid
        self.ray_step = 0.5              # meters per sample along ray
        self.max_range = self.grid_world_size / 2.0

        # Video writer for occlusion grid
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

        print(f"[Controller] Initialized in mode '{self.mode}'")
        print(f"[Controller] Occlusion grid video -> {self.grid_video_path}")

    # -------------------------------------------------------------------------
    # Configuration setters
    # -------------------------------------------------------------------------

    def set_mode(self, mode: str):
        """Switch between 'naive' and 'occl' at runtime if needed."""
        print(f"[Controller] Switching mode to '{mode}'")
        self.mode = mode

    def set_occluders(self, actors):
        """Set list of CARLA actors that should be treated as occluders."""
        self.occluder_actors = list(actors) if actors is not None else []

    def set_ego_vehicle(self, vehicle, speed_kmh: float = 30.0):
        """Attach controller to ego vehicle and desired cruise speed."""
        self.ego_vehicle = vehicle
        self.ego_speed_kmh = float(speed_kmh)
        print(f"[Controller] Ego vehicle control initialized at {speed_kmh} km/h")

    # -------------------------------------------------------------------------
    # Occlusion helpers
    # -------------------------------------------------------------------------

    def _get_blocking_rects(self):
        """
        Return blockers as axis-aligned rectangles in world XY:
            (min_x, max_x, min_y, max_y)

        Uses the actors provided via set_occluders(). If none are set,
        it falls back to "all vehicles except ego".
        """
        rects = []

        # Prefer explicitly provided occluders
        actors = self.occluder_actors
        if actors is None and (self.world is not None):
            actors = self.world.get_actors().filter("vehicle.*")

        if not actors:
            return rects

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

            # Slight padding to be safe
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

        blockers = self._get_blocking_rects()

        cell_size = self.grid_world_size / self.grid_cells
        cx = W / 2.0
        cy = H / 2.0

        for iy in range(H):
            for ix in range(W):
                # cell center in ego-local frame (x: right, y: forward)
                x_local = (ix + 0.5 - cx) * cell_size
                y_local = (cy - (iy + 0.5)) * cell_size  # positive y_local = forward

                dist = math.sqrt(x_local * x_local + y_local * y_local)

                # Skip outside range or too close to ego
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

    def _compute_front_occlusion_metrics(self, grid: np.ndarray):
        """
        Compute simple metrics from the occlusion grid:

        Returns:
            front_occ_frac: fraction of cells ahead that are occluded (0..1)
            near_blocked:   True if there is an occluded cell very close ahead
        """
        H, W = grid.shape
        cy = H // 2

        # "Ahead" = rows above center (smaller iy), and central columns
        front_y0 = 0
        front_y1 = cy
        front_x0 = int(W * 0.25)
        front_x1 = int(W * 0.75)

        front_roi = grid[front_y0:front_y1, front_x0:front_x1]
        if front_roi.size == 0:
            front_occ_frac = 0.0
        else:
            front_occ_frac = float(front_roi.mean())  # since 0/1

        # "Very close ahead" band: just in front of ego
        near_y0 = int(cy - H * 0.15)
        near_y1 = cy
        near_x0 = front_x0
        near_x1 = front_x1

        near_y0 = max(0, near_y0)
        near_y1 = max(near_y0 + 1, near_y1)

        near_roi = grid[near_y0:near_y1, near_x0:near_x1]
        near_blocked = bool(near_roi.max() > 0) if near_roi.size > 0 else False

        return front_occ_frac, near_blocked

    def _project_pedestrians_to_grid(self, pedestrian_locations):
        """
        Map pedestrian world positions into grid cell indices.

        Returns:
            List of (ix, iy) tuples where each tuple is a grid cell containing
            a pedestrian (in ego-centric coordinates).
        """
        ped_cells = []

        if self.ego_vehicle is None or not pedestrian_locations:
            return ped_cells

        H = W = self.grid_cells
        cell_size = self.grid_world_size / self.grid_cells
        max_r = self.max_range

        ego_tf = self.ego_vehicle.get_transform()
        ego_loc = ego_tf.location
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()

        cx = W / 2.0
        cy = H / 2.0

        for ped in pedestrian_locations:
            # Normalise to a carla.Location
            try:
                if hasattr(ped, "get_location"):
                    loc = ped.get_location()
                elif hasattr(ped, "location"):
                    loc = ped.location
                else:
                    continue
            except RuntimeError:
                # Actor can disappear between ticks
                continue

            dx = loc.x - ego_loc.x
            dy = loc.y - ego_loc.y

            # Ego-local coordinates (x: right, y: forward), ignore z
            x_local = dx * right.x + dy * right.y
            y_local = dx * fwd.x + dy * fwd.y

            dist = math.hypot(x_local, y_local)
            if dist < 1e-2 or dist > max_r:
                # Too close to distinguish or outside the grid range
                continue

            # Invert the mapping used in _compute_occlusion_grid:
            # x_local = (ix + 0.5 - cx) * cell_size
            # y_local = (cy - (iy + 0.5)) * cell_size
            ix_f = x_local / cell_size + cx
            iy_f = cy - (y_local / cell_size)

            ix = int(ix_f)
            iy = int(iy_f)

            if 0 <= ix < W and 0 <= iy < H:
                ped_cells.append((ix, iy))

        return ped_cells

       # -------------------------------------------------------------------------
    # Rendering occlusion grid to an image
    # -------------------------------------------------------------------------

    def _render_occlusion_grid(self, grid: np.ndarray, ped_cells=None):
        """
        Convert occlusion grid to a 400x400 RGB image.

        free  -> dark gray
        occ   -> red
        ego   -> green dot at center
        ped   -> blue dot(s) at projected positions
        """
        H, W = grid.shape
        img = np.zeros((self.frame_size_px, self.frame_size_px, 3), dtype=np.uint8)

        # map grid cells to pixels
        scale = self.frame_size_px / self.grid_cells

        # draw cells
        for iy in range(H):
            for ix in range(W):
                x0 = int(ix * scale)
                y0 = int(iy * scale)
                x1 = int((ix + 1) * scale)
                y1 = int((iy + 1) * scale)

                if grid[iy, ix] == 1:
                    color = (0, 0, 255)       # occluded: red (BGR)
                else:
                    color = (50, 50, 50)      # free: dark gray

                img[y0:y1, x0:x1] = color

        # overlay pedestrians (after cells so they sit on top)
        if ped_cells:
            for (ix, iy) in ped_cells:
                if 0 <= ix < W and 0 <= iy < H:
                    cx_px = int((ix + 0.5) * scale)
                    cy_px = int((iy + 0.5) * scale)
                    # blue pedestrian marker
                    cv2.circle(img, (cx_px, cy_px), 5, (255, 0, 0), -1)

        # draw ego at center (on top of everything else)
        center_px = self.frame_size_px // 2
        cv2.circle(img, (center_px, center_px), 6, (0, 255, 0), -1)

        # write video frame
        if self.grid_writer is not None:
            self.grid_writer.write(img)
            self.grid_frame_count += 1

    # -------------------------------------------------------------------------
    # Motion control
    # -------------------------------------------------------------------------

    def start_movement(self):
        self.ego_moving = True
        print("\n[Controller] Ego vehicle starting to move...")

    def stop_movement(self):
        self.ego_moving = False
        self.ego_control.throttle = 0.0
        self.ego_control.brake = 1.0
        if self.ego_vehicle:
            self.ego_vehicle.apply_control(self.ego_control)
        print("\n[Controller] Ego vehicle stopped")

    def _naive_speed_control(self, base_target_speed_ms: float, current_speed: float):
        """
        Very simple P-like speed controller for naive mode.
        """
        # Basic proportional control on speed error
        error = base_target_speed_ms - current_speed

        # kp chosen so we don't slam max throttle all the time
        kp = 0.15
        throttle_cmd = kp * error

        throttle_cmd = max(0.0, min(1.0, throttle_cmd))

        # No braking in naive mode unless we overshoot a lot
        if error < -1.0:
            # a bit too fast -> light brake
            self.ego_control.brake = min(0.4, -0.1 * error)
            self.ego_control.throttle = 0.0
        else:
            self.ego_control.brake = 0.0
            self.ego_control.throttle = throttle_cmd
        # -------------------------------------------------------------------------
    # Pedestrian hazard metrics (for controller logic)
    # -------------------------------------------------------------------------

    def _compute_pedestrian_hazards(self, pedestrian_locations):
        """
        Compute simple hazard features from pedestrian positions in ego-local frame.

        Returns a dict with:
            has_ped_ahead: bool
            ped_in_near_zone: bool     # e.g. < 12 m ahead in lane
            ped_in_stop_zone: bool     # e.g. < 6 m ahead in lane
            nearest_ped_dist: float or None
        """
        hazards = {
            "has_ped_ahead": False,
            "ped_in_near_zone": False,
            "ped_in_stop_zone": False,
            "nearest_ped_dist": None,
        }

        if self.ego_vehicle is None or not pedestrian_locations:
            return hazards

        # Tunable parameters
        lane_half_width = 2.5   # meters from lane center to consider "in lane"
        near_dist = 18.0        # m: start slowing aggressively
        stop_dist = 10.0         # m: hard brake zone
        max_r = self.max_range  # reuse grid range

        ego_tf = self.ego_vehicle.get_transform()
        ego_loc = ego_tf.location
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()

        nearest_dist = None

        for ped in pedestrian_locations:
            try:
                if hasattr(ped, "get_location"):
                    loc = ped.get_location()
                elif hasattr(ped, "location"):
                    loc = ped.location
                else:
                    continue
            except RuntimeError:
                # Actor may have been destroyed
                continue

            dx = loc.x - ego_loc.x
            dy = loc.y - ego_loc.y

            # Ego-local coordinates (x: right, y: forward)
            x_local = dx * right.x + dy * right.y
            y_local = dx * fwd.x + dy * fwd.y

            # Ignore pedestrians behind or way outside the grid range
            if y_local <= 0:
                continue

            dist = math.hypot(x_local, y_local)
            if dist > max_r:
                continue

            # Check if they are roughly in the lane
            if abs(x_local) > lane_half_width:
                continue

            # Now we have a ped in front, in lane
            hazards["has_ped_ahead"] = True

            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist

            if dist <= near_dist:
                hazards["ped_in_near_zone"] = True
            if dist <= stop_dist:
                hazards["ped_in_stop_zone"] = True

        hazards["nearest_ped_dist"] = nearest_dist
        return hazards

    def _occl_speed_control(self, base_target_speed_ms, current_speed, grid, ped_hazards=None):
        front_occ_frac, near_blocked = self._compute_front_occlusion_metrics(grid)

        # Unpack pedestrian hazard info
        has_ped_ahead = False
        ped_in_near_zone = False
        ped_in_stop_zone = False
        nearest_ped_dist = None

        if ped_hazards:
            has_ped_ahead   = ped_hazards.get("has_ped_ahead", False)
            ped_in_near_zone = ped_hazards.get("ped_in_near_zone", False)
            ped_in_stop_zone = ped_hazards.get("ped_in_stop_zone", False)
            nearest_ped_dist = ped_hazards.get("nearest_ped_dist", None)

        # -------------------------
        # 1) Base occlusion-based speed scaling
        # -------------------------
        min_scale = 0.3  # never go below 30% of base due to occlusion alone
        speed_scale = max(min_scale, 1.0 - 0.7 * front_occ_frac)
        target_speed_ms = base_target_speed_ms * speed_scale

        # -------------------------
        # 2) Pedestrian-aware speed caps
        # -------------------------
        if has_ped_ahead and nearest_ped_dist is not None:
            # Simple distance-based cap:
            #   - Far (> 15m): at most ~5 m/s
            #   - Close: taper down to ~1 m/s
            ped_cap = max(1.0, min(5.0, nearest_ped_dist / 3.0))
            target_speed_ms = min(target_speed_ms, ped_cap)

        # -------------------------
        # 3) Emergency braking logic
        # -------------------------

        # If there is a very close pedestrian in lane, always brake hard.
        # Or: ped in near zone + high occlusion ahead → be conservative.
        if ped_in_stop_zone or (ped_in_near_zone and front_occ_frac > 0.4) or (near_blocked and front_occ_frac > 0.6):
            self.ego_control.throttle = 0.0
            self.ego_control.brake = 1.0

            # Latch stop once we've basically come to a halt
            if current_speed < 0.2:
                self.ego_moving = False
            return

        # -------------------------
        # 4) P-like speed control toward the (occlusion + ped) target speed
        # -------------------------
        error = target_speed_ms - current_speed
        kp = 0.3

        throttle_cmd = kp * error
        throttle_cmd = max(0.0, min(1.0, throttle_cmd))

        brake_cmd = 0.0
        if error < -0.5:
            # If we’re more than ~0.5 m/s over target, brake proportional
            brake_cmd = min(1.0, -error)
            throttle_cmd = 0.0

        self.ego_control.throttle = throttle_cmd
        self.ego_control.brake = brake_cmd

    def update(self, pedestrian_locations=None):
        if self.ego_vehicle is None:
            return

        # 1) Grid
        grid = self._compute_occlusion_grid()

        # 2) Pedestrians: projection for viz + metrics for control
        ped_cells = self._project_pedestrians_to_grid(pedestrian_locations)
        ped_hazards = self._compute_pedestrian_hazards(pedestrian_locations)

        self._render_occlusion_grid(grid, ped_cells)

        # 3) If we’re not moving yet, just visualize
        if not self.ego_moving:
            return

        base_target_speed_ms = self.ego_speed_kmh / 3.6
        vel = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

        if self.mode == "occl":
            self._occl_speed_control(base_target_speed_ms, current_speed, grid, ped_hazards)
        else:
            self._naive_speed_control(base_target_speed_ms, current_speed)

        self.ego_control.steer = 0.0
        self.ego_vehicle.apply_control(self.ego_control)

    # -------------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------------

    def destroy(self):
        if self.grid_writer is not None:
            self.grid_writer.release()
            print(
                f"[Occlusion Grid Video] Saved {self.grid_frame_count} frames to "
                f"{self.grid_video_path}"
            )
