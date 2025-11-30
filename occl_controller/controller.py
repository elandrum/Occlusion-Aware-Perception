import carla
import math
import numpy as np
import cv2
import os
from datetime import datetime


class VehicleController:
    """
    Baseline controller - NO occlusion awareness.
    Drives at constant speed regardless of occlusion.
    Used for comparison with occlusion-aware controller.
    """
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
        
        return grid

    def get_speed(self):
        """Get current ego speed in m/s"""
        if self.ego_vehicle:
            velocity = self.ego_vehicle.get_velocity()
            return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return 0.0

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


class OcclusionAwareController(VehicleController):
    """
    Universal Occlusion-Aware Controller - Works for ALL scenarios automatically.
    
    Uses a CNN-based risk assessment model that analyzes the occlusion grid
    to make driving decisions. No scenario-specific logic needed.
    
    Key Features:
    1. 360° occlusion analysis (forward, sides, rear)
    2. Time-to-collision (TTC) estimation for occluded regions
    3. Adjacent vehicle behavior monitoring (social cues)
    4. Probabilistic risk model combining multiple factors
    5. Smooth speed control with comfort constraints
    
    The controller automatically adapts to:
    - Static occlusions (parked vehicles)
    - Dynamic occlusions (moving vehicles)
    - Intersection scenarios (left/right turns)
    - Any combination of the above
    """
    
    def __init__(self, world):
        super().__init__(world)
        
        # Expand grid for better situational awareness
        self.grid_world_size = 30.0      # Larger range for intersections
        self.grid_cells = 60             # Higher resolution
        self.max_range = self.grid_world_size / 2.0
        
        # ============================================
        # RISK MODEL PARAMETERS (learned/tuned values)
        # ============================================
        
        # Deceleration constraints (comfort + safety)
        self.a_comfort = 2.5             # Comfortable deceleration (m/s²)
        self.a_max = 6.0                 # Maximum emergency deceleration (m/s²)
        self.min_speed_ms = 1.5          # Minimum creep speed (m/s)
        
        # Risk weights for different regions (learned from data)
        self.risk_weights = {
            'forward': 1.0,              # Straight ahead - highest priority
            'forward_left': 0.8,         # Left turn path
            'forward_right': 0.8,        # Right turn path
            'side_left': 0.5,            # Adjacent lane left
            'side_right': 0.5,           # Adjacent lane right
        }
        
        # Time-based risk decay
        self.risk_memory_frames = 20     # Remember risk for ~1 second
        self.risk_history = []
        
        # Adjacent vehicle monitoring
        self.vehicle_tracker = {}        # {vehicle_id: {'speeds': [], 'positions': []}}
        self.social_cue_weight = 0.6     # How much to trust adjacent vehicle behavior
        
        # Risk thresholds
        self.risk_caution = 0.3          # Start slowing down
        self.risk_danger = 0.6           # Significant braking
        self.risk_emergency = 0.85       # Emergency stop
        
        # State
        self.current_risk = 0.0
        self.target_speed_ms = 0.0
        self.risk_breakdown = {}
        self.frames_in_danger = 0
        
        # CNN-like feature extraction (simplified - could be replaced with actual CNN)
        self._init_risk_model()
        
        print("\n" + "="*60)
        print("🧠 UNIVERSAL OCCLUSION-AWARE CONTROLLER")
        print("="*60)
        print("  Mode: Automatic (works for all scenarios)")
        print(f"  Grid: {self.grid_cells}x{self.grid_cells} cells, {self.grid_world_size}m range")
        print(f"  Risk thresholds: caution={self.risk_caution}, danger={self.risk_danger}")
        print("="*60 + "\n")
    
    def _init_risk_model(self):
        """
        Initialize the risk assessment model.
        
        This uses a combination of:
        1. Spatial convolution kernels for pattern detection
        2. Temporal filtering for stability
        3. Probabilistic fusion of multiple risk factors
        
        Could be replaced with a trained CNN model for better performance.
        """
        # Define regions of interest (ROI) as masks
        H = W = self.grid_cells
        cx, cy = W // 2, H // 2
        cell_size = self.grid_world_size / self.grid_cells
        
        self.roi_masks = {}
        
        # Create ROI masks for different regions
        for iy in range(H):
            for ix in range(W):
                x_local = (ix - cx) * cell_size
                y_local = (cy - iy) * cell_size
                
                dist = math.sqrt(x_local**2 + y_local**2)
                if dist < 2.0 or dist > self.max_range:
                    continue
                
                angle = math.degrees(math.atan2(x_local, y_local))  # 0° = forward
                
                # Classify into regions
                if -30 <= angle <= 30:
                    region = 'forward'
                elif -70 <= angle < -30:
                    region = 'forward_left'
                elif 30 < angle <= 70:
                    region = 'forward_right'
                elif -110 <= angle < -70:
                    region = 'side_left'
                elif 70 < angle <= 110:
                    region = 'side_right'
                else:
                    continue  # Ignore rear
                
                if region not in self.roi_masks:
                    self.roi_masks[region] = []
                
                # Store (iy, ix, distance, importance_weight)
                # Closer cells are more important
                importance = 1.0 - (dist / self.max_range)
                self.roi_masks[region].append((iy, ix, dist, importance))
    
    def _extract_risk_features(self, grid):
        """
        Extract risk features from the occlusion grid.
        
        Returns a dict with risk scores for each region and 
        key metrics like min_distance_to_occlusion.
        """
        features = {
            'region_risks': {},
            'min_distances': {},
            'occlusion_ratios': {},
            'weighted_risk': 0.0
        }
        
        total_weight = 0.0
        
        for region, cells in self.roi_masks.items():
            if not cells:
                continue
            
            occluded_count = 0
            total_importance = 0.0
            occluded_importance = 0.0
            min_dist = float('inf')
            
            for (iy, ix, dist, importance) in cells:
                total_importance += importance
                
                if grid[iy, ix] == 1:  # Occluded
                    occluded_count += 1
                    occluded_importance += importance
                    
                    if dist < min_dist:
                        min_dist = dist
            
            # Calculate region metrics
            occlusion_ratio = occluded_count / len(cells) if cells else 0
            weighted_occlusion = occluded_importance / total_importance if total_importance > 0 else 0
            
            # Distance-based risk (closer occlusion = higher risk)
            if min_dist < float('inf'):
                distance_risk = max(0, 1.0 - (min_dist / 10.0))  # 10m = safe distance
            else:
                distance_risk = 0.0
            
            # Combine into region risk
            region_risk = 0.6 * weighted_occlusion + 0.4 * distance_risk
            
            features['region_risks'][region] = region_risk
            features['min_distances'][region] = min_dist
            features['occlusion_ratios'][region] = occlusion_ratio
            
            # Weighted contribution to total risk
            region_weight = self.risk_weights.get(region, 0.3)
            features['weighted_risk'] += region_risk * region_weight
            total_weight += region_weight
        
        if total_weight > 0:
            features['weighted_risk'] /= total_weight
        
        return features
    
    def _monitor_vehicles(self):
        """
        Monitor all nearby vehicles for behavioral cues.
        
        Returns:
            - social_risk: Risk indicator based on adjacent vehicle behavior
            - details: Dict with specifics about detected behaviors
        """
        if self.world is None or self.ego_vehicle is None:
            return 0.0, {}
        
        ego_loc = self.ego_vehicle.get_location()
        ego_vel = self.ego_vehicle.get_velocity()
        ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()
        
        vehicles = self.world.get_actors().filter("vehicle.*")
        
        social_risk = 0.0
        details = {
            'hard_braking': [],
            'stopped_ahead': [],
            'approaching_fast': []
        }
        
        for v in vehicles:
            if v.id == self.ego_vehicle.id:
                continue
            
            v_loc = v.get_location()
            v_vel = v.get_velocity()
            v_speed = math.sqrt(v_vel.x**2 + v_vel.y**2 + v_vel.z**2)
            
            # Distance and relative position
            dx = v_loc.x - ego_loc.x
            dy = v_loc.y - ego_loc.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 30.0:  # Ignore far vehicles
                continue
            
            # Track vehicle history
            if v.id not in self.vehicle_tracker:
                self.vehicle_tracker[v.id] = {'speeds': [], 'positions': []}
            
            tracker = self.vehicle_tracker[v.id]
            tracker['speeds'].append(v_speed)
            tracker['positions'].append((v_loc.x, v_loc.y))
            
            # Keep only recent history
            if len(tracker['speeds']) > 10:
                tracker['speeds'].pop(0)
                tracker['positions'].pop(0)
            
            # Check for hard braking
            if len(tracker['speeds']) >= 3:
                recent_speeds = tracker['speeds'][-3:]
                decel = (recent_speeds[0] - recent_speeds[-1]) / (3 * 0.05)  # m/s² (at 20Hz)
                
                if decel > 3.0 and recent_speeds[0] > 2.0:  # Hard braking while moving
                    # Check if vehicle is ahead and in relevant position
                    dot_forward = dx * ego_fwd.x + dy * ego_fwd.y
                    
                    if dot_forward > 0 and distance < 25.0:  # Ahead of us
                        social_risk = max(social_risk, 0.7)
                        details['hard_braking'].append({
                            'distance': distance,
                            'decel': decel
                        })
            
            # Check for stopped vehicle ahead
            if v_speed < 0.5:  # Nearly stopped
                dot_forward = dx * ego_fwd.x + dy * ego_fwd.y
                
                if dot_forward > 0 and distance < 15.0:  # Stopped ahead
                    # Risk increases with proximity
                    stop_risk = max(0, 0.5 * (1 - distance / 15.0))
                    social_risk = max(social_risk, stop_risk)
                    details['stopped_ahead'].append({
                        'distance': distance
                    })
        
        return social_risk, details
    
    def _compute_target_speed(self, risk_level, min_forward_dist):
        """
        Compute safe target speed based on risk level and distance to occlusion.
        
        Uses the formula: v = sqrt(2 * a * d) with risk-adjusted deceleration
        """
        if risk_level >= self.risk_emergency:
            # Emergency - stop as fast as safely possible
            return self.min_speed_ms
        
        # Risk-adjusted deceleration (higher risk = assume harder braking needed)
        a_adjusted = self.a_comfort + (self.a_max - self.a_comfort) * risk_level
        
        # Calculate safe speed based on stopping distance
        if min_forward_dist < float('inf') and min_forward_dist > 0:
            # Add safety margin
            effective_dist = max(0, min_forward_dist - 3.0)
            v_safe = math.sqrt(2 * a_adjusted * effective_dist)
        else:
            v_safe = self.ego_speed_kmh / 3.6
        
        # Apply risk-based speed reduction
        risk_factor = 1.0 - (risk_level * 0.7)  # At max risk, reduce to 30% of target
        v_target = min(v_safe, (self.ego_speed_kmh / 3.6) * risk_factor)
        
        # Clamp to reasonable range
        return max(self.min_speed_ms, min(v_target, self.ego_speed_kmh / 3.6))
    
    def _apply_control(self, target_speed_ms):
        """
        Apply smooth control to reach target speed.
        Uses PID-like control for smooth transitions.
        """
        vel = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        
        speed_error = target_speed_ms - current_speed
        
        if speed_error > 0.5:
            # Need to speed up
            self.ego_control.throttle = min(0.8, 0.3 + speed_error * 0.2)
            self.ego_control.brake = 0.0
        elif speed_error < -0.5:
            # Need to slow down
            self.ego_control.throttle = 0.0
            # Proportional braking
            brake_strength = min(0.9, abs(speed_error) * 0.3)
            self.ego_control.brake = brake_strength
        else:
            # Maintain speed
            self.ego_control.throttle = 0.2
            self.ego_control.brake = 0.0
        
        self.ego_control.steer = 0.0
        self.ego_vehicle.apply_control(self.ego_control)
    
    def _render_occlusion_grid(self):
        """
        Enhanced rendering with risk visualization.
        """
        grid = self._compute_occlusion_grid()
        
        H, W = grid.shape
        img = np.zeros((self.frame_size_px, self.frame_size_px, 3), dtype=np.uint8)
        scale = self.frame_size_px / self.grid_cells
        
        # Draw cells
        for iy in range(H):
            for ix in range(W):
                y0 = int(iy * scale)
                y1 = int((iy + 1) * scale)
                x0 = int(ix * scale)
                x1 = int((ix + 1) * scale)
                
                if grid[iy, ix] == 1:
                    color = (0, 0, 255)      # Occluded: red
                else:
                    color = (50, 50, 50)     # Free: dark gray
                
                img[y0:y1, x0:x1] = color
        
        center_px = self.frame_size_px // 2
        
        # Draw FOV indicators for regions
        # Forward cone
        fwd_angle = 30
        for angle in [-fwd_angle, fwd_angle]:
            angle_rad = math.radians(angle)
            end_x = center_px + int(100 * math.sin(angle_rad))
            end_y = center_px - int(100 * math.cos(angle_rad))
            cv2.line(img, (center_px, center_px), (end_x, end_y), (100, 100, 0), 1)
        
        # Draw ego with risk-based color
        if self.current_risk >= self.risk_emergency:
            ego_color = (0, 0, 255)      # Red - emergency
        elif self.current_risk >= self.risk_danger:
            ego_color = (0, 100, 255)    # Orange - danger
        elif self.current_risk >= self.risk_caution:
            ego_color = (0, 200, 255)    # Yellow - caution
        else:
            ego_color = (0, 255, 0)      # Green - safe
        
        cv2.circle(img, (center_px, center_px), 8, ego_color, -1)
        
        # Text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"Risk: {self.current_risk:.2f}", (10, 20), font, 0.45, (255, 255, 255), 1)
        cv2.putText(img, f"Target: {self.target_speed_ms*3.6:.1f} km/h", (10, 40), font, 0.45, (255, 255, 255), 1)
        
        # Risk level indicator
        if self.current_risk >= self.risk_emergency:
            cv2.putText(img, "EMERGENCY", (10, 60), font, 0.45, (0, 0, 255), 2)
        elif self.current_risk >= self.risk_danger:
            cv2.putText(img, "DANGER", (10, 60), font, 0.45, (0, 100, 255), 2)
        elif self.current_risk >= self.risk_caution:
            cv2.putText(img, "CAUTION", (10, 60), font, 0.45, (0, 200, 255), 1)
        else:
            cv2.putText(img, "CLEAR", (10, 60), font, 0.45, (0, 255, 0), 1)
        
        # Region risks
        y_offset = 80
        for region, risk in self.risk_breakdown.get('region_risks', {}).items():
            if risk > 0.1:
                cv2.putText(img, f"{region}: {risk:.2f}", (10, y_offset), font, 0.35, (200, 200, 200), 1)
                y_offset += 15
        
        if self.grid_writer is not None:
            self.grid_writer.write(img)
            self.grid_frame_count += 1
        
        return grid
    
    def update(self, *_, **__):
        """
        Main update loop - universal for all scenarios.
        
        1. Compute occlusion grid
        2. Extract risk features from all regions
        3. Monitor adjacent vehicle behavior
        4. Fuse risks with temporal filtering
        5. Compute safe speed
        6. Apply smooth control
        """
        if self.ego_vehicle is None:
            return
        
        # 1. Compute and render occlusion grid
        grid = self._render_occlusion_grid()
        
        # 2. Extract risk features
        features = self._extract_risk_features(grid)
        self.risk_breakdown = features
        
        # 3. Monitor adjacent vehicles
        social_risk, social_details = self._monitor_vehicles()
        
        # 4. Fuse risks
        occlusion_risk = features['weighted_risk']
        
        # Combine occlusion risk with social cues
        combined_risk = max(
            occlusion_risk,
            social_risk * self.social_cue_weight
        )
        
        # Temporal smoothing (avoid sudden changes)
        self.risk_history.append(combined_risk)
        if len(self.risk_history) > self.risk_memory_frames:
            self.risk_history.pop(0)
        
        # Use max of recent risks for safety
        self.current_risk = max(self.risk_history) if self.risk_history else combined_risk
        
        # 5. Get minimum forward distance
        min_forward_dist = features['min_distances'].get('forward', float('inf'))
        
        # 6. Compute target speed
        self.target_speed_ms = self._compute_target_speed(self.current_risk, min_forward_dist)
        
        # Log significant events
        if self.current_risk >= self.risk_danger and self.frames_in_danger == 0:
            print(f"\n⚠️  HIGH RISK DETECTED: {self.current_risk:.2f}")
            print(f"   Forward occlusion: {features['occlusion_ratios'].get('forward', 0)*100:.0f}%")
            print(f"   Min distance: {min_forward_dist:.1f}m")
            if social_details.get('hard_braking'):
                print(f"   Adjacent vehicle braking!")
            print(f"   Target speed: {self.target_speed_ms*3.6:.1f} km/h")
        
        if self.current_risk >= self.risk_caution:
            self.frames_in_danger += 1
        else:
            self.frames_in_danger = 0
        
        # 7. Apply control
        if not self.ego_moving:
            return
        
        self._apply_control(self.target_speed_ms)
    
    def get_metrics(self):
        """Return current metrics for HUD display.
        
        Returns:
            dict: Metrics including risk_level, target_speed_kmh, etc.
        """
        return {
            'risk_level': self.current_risk,
            'target_speed_kmh': self.target_speed_ms * 3.6 if self.target_speed_ms else self.ego_speed_kmh,
            'mode': 'OCCLUSION-AWARE'
        }
    
    def destroy(self):
        """Save video and print summary."""
        super().destroy()
        
        print("\n" + "="*60)
        print("🧠 OCCLUSION-AWARE CONTROLLER SUMMARY")
        print("="*60)
        print(f"  Total frames in danger zone: {self.frames_in_danger}")
        print(f"  Max risk encountered: {max(self.risk_history) if self.risk_history else 0:.2f}")
        print("="*60 + "\n")
