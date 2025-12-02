#!/usr/bin/env python

"""
Scenario 4: Red-Light Runner Hidden Behind Trucks (Left Turn Occlusion)
Setup: Two trucks moving ahead of ego (one in each lane), blocking view of oncoming traffic
       Ego wants to turn left at intersection
       Red-light runner approaching from opposite direction in leftmost oncoming lane
Logic: Trucks block ego's view, ego turns left, collides with hidden red-light runner
       Occlusion-aware ego should proceed cautiously when view is blocked
"""

import carla
import math

from .config import ScenarioConfig
from .actors_static import SceneryManager
from .actors_peds import PedestrianController
from .turn_controller import EgoTurnController


class TurningVehicleController:
    """Controller for vehicles that need to make turns at intersections"""
    
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicles = []
        
    def spawn_moving_vehicles(self, vehicle_config):
        """Spawn moving vehicles from configuration"""
        if 'moving_vehicles' not in vehicle_config:
            print("\nNo moving vehicles found in config.")
            return
        
        print("\nSpawning moving vehicles for intersection scenario...")
        
        for i, veh_config in enumerate(vehicle_config['moving_vehicles']):
            veh_type = veh_config.get('type', 'vehicle.audi.a2')
            
            try:
                veh_bp = self.blueprint_library.find(veh_type)
            except:
                print(f"Warning: Vehicle type '{veh_type}' not found, using default")
                veh_bp = self.blueprint_library.filter('vehicle.audi.a2')[0]
            
            # Set color for red-light runner to make it visible
            if veh_config.get('id') == 'red_light_runner':
                if veh_bp.has_attribute('color'):
                    veh_bp.set_attribute('color', '255,0,0')  # Red color
            
            location = carla.Location(
                x=veh_config['location']['x'],
                y=veh_config['location']['y'],
                z=veh_config['location']['z']
            )
            rotation = carla.Rotation(
                pitch=veh_config['rotation']['pitch'],
                yaw=veh_config['rotation']['yaw'],
                roll=veh_config['rotation']['roll']
            )
            spawn_point = carla.Transform(location, rotation)
            
            try:
                vehicle = self.world.spawn_actor(veh_bp, spawn_point)
                vehicle.set_simulate_physics(True)
                
                vehicle_data = {
                    'actor': vehicle,
                    'id': veh_config.get('id', f'vehicle_{i}'),
                    'speed_kmh': veh_config.get('speed_kmh', 26),
                    'behavior': veh_config.get('behavior', 'straight'),
                    'is_moving': False,
                    'control': carla.VehicleControl()
                }
                self.vehicles.append(vehicle_data)
                
                veh_id = vehicle_data['id']
                print(f"Vehicle '{veh_id}' spawned at ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})")
                
            except Exception as e:
                print(f"Failed to spawn vehicle {i+1}: {e}")
    
    def start_movement(self):
        """Start all moving vehicles"""
        for veh_data in self.vehicles:
            veh_data['is_moving'] = True
        print(f"\n{len(self.vehicles)} vehicle(s) started moving")
    
    def update(self):
        """Update all moving vehicles - maintain speed in their direction"""
        for veh_data in self.vehicles:
            if not veh_data['is_moving']:
                continue
            
            vehicle = veh_data['actor']
            control = veh_data['control']
            
            # Maintain speed
            target_speed_ms = veh_data['speed_kmh'] / 3.6
            velocity = vehicle.get_velocity()
            current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            speed_diff = target_speed_ms - current_speed
            
            if speed_diff > 5.0:
                # Far below target - full throttle
                control.throttle = 1.0
                control.brake = 0.0
            elif speed_diff > 1.0:
                # Below target - high throttle
                control.throttle = 0.8
                control.brake = 0.0
            elif speed_diff > 0:
                # Slightly below - moderate throttle
                control.throttle = 0.5
                control.brake = 0.0
            else:
                # At or above target - maintain
                control.throttle = 0.3
                control.brake = 0.0
            
            control.steer = 0.0  # Go straight
            vehicle.apply_control(control)
    
    def get_vehicle_by_id(self, vehicle_id):
        """Get vehicle data by ID"""
        for veh_data in self.vehicles:
            if veh_data['id'] == vehicle_id:
                return veh_data
        return None
    
    def get_vehicle_speeds(self):
        """Get current speeds of all vehicles"""
        speeds = {}
        for veh_data in self.vehicles:
            velocity = veh_data['actor'].get_velocity()
            speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speeds[veh_data['id']] = speed_ms * 3.6
        return speeds
    
    def destroy_all(self):
        """Destroy all vehicles"""
        print("Destroying scenario vehicles...")
        for veh_data in self.vehicles:
            if veh_data['actor'].is_alive:
                veh_data['actor'].destroy()


# EgoTurnController is now imported from turn_controller.py for modularity


class OcclusionAwareTurnController(EgoTurnController):
    """
    Occlusion-Aware Turn Controller for left turns at intersections.
    
    Extends EgoTurnController with:
    1. Occlusion grid computation (ray-casting)
    2. Left-side occlusion analysis (for oncoming traffic)
    3. Safe turn decision making - only proceed when view is clear
    4. Video output of occlusion grid
    """
    
    def __init__(self, world):
        super().__init__(world)
        
        # Import needed for occlusion grid
        import numpy as np
        import cv2
        import os
        from datetime import datetime
        
        self.np = np
        self.cv2 = cv2
        
        # Occlusion grid settings
        self.grid_world_size = 30.0      # Larger for intersection (meters)
        self.grid_cells = 40             # 40 x 40 cells
        self.ray_step = 0.5              # meters per sample
        self.max_range = self.grid_world_size / 2.0
        
        # Video writer for occlusion grid
        os.makedirs("output", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.grid_video_path = f"output/grid_turn_{timestamp}.mp4"
        
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.frame_size_px = 400
        self.grid_writer = cv2.VideoWriter(
            self.grid_video_path,
            fourcc,
            20,
            (self.frame_size_px, self.frame_size_px),
        )
        self.grid_frame_count = 0
        
        # Occlusion-aware parameters
        self.left_occlusion_threshold = 0.3  # If >30% of left view occluded, be cautious
        self.min_turn_speed_kmh = 8.0        # Slow speed when turning with occlusion
        self.safe_turn_speed_kmh = 15.0      # Normal turn speed
        
        # State
        self.is_cautious_mode = False
        self.left_occlusion_ratio = 0.0
        
        print("\n" + "="*50)
        print("🛡️  OCCLUSION-AWARE TURN CONTROLLER ACTIVE")
        print("="*50)
        print(f"  Left occlusion threshold: {self.left_occlusion_threshold*100:.0f}%")
        print(f"  Cautious turn speed: {self.min_turn_speed_kmh} km/h")
        print("="*50 + "\n")
    
    def _get_blocking_rects(self):
        """Get bounding boxes of all vehicles (except ego) as blockers"""
        rects = []
        
        if self.world is None:
            return rects
        
        vehicles = self.world.get_actors().filter("vehicle.*")
        ego_loc = self.ego_vehicle.get_location() if self.ego_vehicle else None
        
        for v in vehicles:
            if self.ego_vehicle and v.id == self.ego_vehicle.id:
                continue
            
            if not hasattr(v, "bounding_box"):
                continue
            
            bb = v.bounding_box
            tf = v.get_transform()
            center = tf.transform(bb.location)
            
            pad = 1.1
            half_x = bb.extent.x * pad
            half_y = bb.extent.y * pad
            
            min_x = center.x - half_x
            max_x = center.x + half_x
            min_y = center.y - half_y
            max_y = center.y + half_y
            
            # Skip if ego is inside
            if ego_loc:
                if (min_x <= ego_loc.x <= max_x) and (min_y <= ego_loc.y <= max_y):
                    continue
            
            rects.append((min_x, max_x, min_y, max_y))
        
        return rects
    
    def _compute_occlusion_grid(self):
        """Compute 2D occlusion grid using ray-casting"""
        H = W = self.grid_cells
        grid = self.np.zeros((H, W), dtype=self.np.uint8)
        
        if self.ego_vehicle is None:
            return grid
        
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
                x_local = (ix + 0.5 - cx) * cell_size
                y_local = (cy - (iy + 0.5)) * cell_size
                
                dist = math.sqrt(x_local * x_local + y_local * y_local)
                
                if dist > self.max_range or dist < 1e-2:
                    continue
                
                dx_local = x_local / dist
                dy_local = y_local / dist
                
                dir_world_x = dx_local * right.x + dy_local * fwd.x
                dir_world_y = dx_local * right.y + dy_local * fwd.y
                
                occluded = False
                s = 1.0
                while s <= dist:
                    wx = ego_loc.x + s * dir_world_x
                    wy = ego_loc.y + s * dir_world_y
                    
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
    
    def _analyze_left_occlusion(self, grid):
        """
        Analyze occlusion on the LEFT side (for oncoming traffic during left turn).
        Returns ratio of occluded cells in left-forward quadrant.
        """
        H, W = grid.shape
        cx, cy = W // 2, H // 2
        cell_size = self.grid_world_size / self.grid_cells
        
        left_forward_cells = 0
        occluded_cells = 0
        
        for iy in range(H):
            for ix in range(W):
                x_local = (ix - cx) * cell_size
                y_local = (cy - iy) * cell_size
                
                # Left-forward quadrant: x < 0 (left) and y > 0 (forward)
                if x_local >= 0 or y_local <= 0:
                    continue
                
                distance = math.sqrt(x_local**2 + y_local**2)
                if distance < 3.0 or distance > self.max_range:
                    continue
                
                left_forward_cells += 1
                if grid[iy, ix] == 1:
                    occluded_cells += 1
        
        ratio = occluded_cells / max(left_forward_cells, 1)
        return ratio
    
    def _render_occlusion_grid(self, grid):
        """Render occlusion grid with left-side highlighting"""
        H, W = grid.shape
        img = self.np.zeros((self.frame_size_px, self.frame_size_px, 3), dtype=self.np.uint8)
        scale = self.frame_size_px / self.grid_cells
        
        center_px = self.frame_size_px // 2
        
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
        
        # Draw left-forward quadrant boundary (area we check for oncoming traffic)
        self.cv2.line(img, (center_px, center_px), (0, center_px), (100, 100, 0), 1)
        self.cv2.line(img, (center_px, center_px), (center_px, 0), (100, 100, 0), 1)
        
        # Draw ego
        ego_color = (0, 255, 0)  # Green
        if self.is_cautious_mode:
            ego_color = (0, 165, 255)  # Orange - caution
        self.cv2.circle(img, (center_px, center_px), 8, ego_color, -1)
        
        # Draw turn arrow if turning
        if self.is_turning:
            # Left turn arrow
            arrow_start = (center_px, center_px - 20)
            arrow_end = (center_px - 30, center_px - 40)
            self.cv2.arrowedLine(img, arrow_start, arrow_end, (255, 255, 0), 2)
        
        # Text overlay
        font = self.cv2.FONT_HERSHEY_SIMPLEX
        self.cv2.putText(img, f"Left Occ: {self.left_occlusion_ratio*100:.0f}%", (10, 25), font, 0.5, (255, 255, 255), 1)
        
        state = "CAUTIOUS" if self.is_cautious_mode else "NORMAL"
        color = (0, 165, 255) if self.is_cautious_mode else (0, 255, 0)
        self.cv2.putText(img, state, (10, 45), font, 0.5, color, 1)
        
        if self.is_turning:
            self.cv2.putText(img, "TURNING LEFT", (10, 65), font, 0.5, (255, 255, 0), 1)
        
        if self.grid_writer is not None:
            self.grid_writer.write(img)
            self.grid_frame_count += 1
    
    def update(self):
        """Occlusion-aware turn update"""
        if not self.ego_moving or not self.ego_vehicle:
            return
        
        # 1. Compute occlusion grid
        grid = self._compute_occlusion_grid()
        
        # 2. Analyze left-side occlusion (for oncoming traffic)
        self.left_occlusion_ratio = self._analyze_left_occlusion(grid)
        
        # 3. Determine if we should be cautious
        self.is_cautious_mode = self.left_occlusion_ratio > self.left_occlusion_threshold
        
        # 4. Render grid
        self._render_occlusion_grid(grid)
        
        current_location = self.ego_vehicle.get_location()
        current_transform = self.ego_vehicle.get_transform()
        
        # Check if we should start turning
        if not self.is_turning and self.turn_trigger_x:
            if current_location.x >= self.turn_trigger_x:
                self.is_turning = True
                if self.is_cautious_mode:
                    print(f"\n⚠️  Ego starting LEFT TURN (CAUTIOUS - {self.left_occlusion_ratio*100:.0f}% occluded)")
                else:
                    print(f"\n🔄 Ego starting LEFT TURN at x={current_location.x:.1f}")
        
        # Determine target speed based on occlusion and turn state
        if self.is_turning and self.is_cautious_mode:
            # Slow down during turn when view is blocked
            target_speed_ms = self.min_turn_speed_kmh / 3.6
        elif self.is_turning:
            # Normal turn speed
            target_speed_ms = self.safe_turn_speed_kmh / 3.6
        else:
            # Normal driving speed
            target_speed_ms = self.ego_speed_kmh / 3.6
        
        velocity = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Throttle/brake control
        if current_speed > target_speed_ms + 1.0:
            self.ego_control.throttle = 0.0
            self.ego_control.brake = 0.4
        elif current_speed < target_speed_ms:
            self.ego_control.throttle = 0.6
            self.ego_control.brake = 0.0
        else:
            self.ego_control.throttle = 0.3
            self.ego_control.brake = 0.0
        
        # Steering control (same as parent)
        if self.is_turning and self.turn_target and not self.turn_complete:
            dx = self.turn_target.x - current_location.x
            dy = self.turn_target.y - current_location.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 3.0:
                self.turn_complete = True
                print("✓ Turn complete")
                self.ego_control.steer = 0.0
            else:
                desired_yaw = math.degrees(math.atan2(dy, dx))
                current_yaw = current_transform.rotation.yaw
                
                yaw_diff = desired_yaw - current_yaw
                while yaw_diff > 180:
                    yaw_diff -= 360
                while yaw_diff < -180:
                    yaw_diff += 360
                
                steer = max(-1.0, min(1.0, yaw_diff / 45.0))
                self.ego_control.steer = steer
        else:
            self.ego_control.steer = 0.0
        
        self.ego_vehicle.apply_control(self.ego_control)
    
    def is_cautious(self):
        """Check if controller is in cautious mode due to occlusion"""
        return self.is_cautious_mode
    
    def destroy(self):
        """Release video writer"""
        if self.grid_writer is not None:
            self.grid_writer.release()
            print(f"[Occlusion Grid Video] Saved {self.grid_frame_count} frames to {self.grid_video_path}")


class Scenario4:
    """Scenario 4: Red-light runner hidden behind trucks during left turn"""
    
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.name = "Red-Light Runner Hidden Behind Trucks"
        self.description = "Ego turns left at intersection, view blocked by trucks, collides with hidden red-light runner"
        
        # Managers
        self.scenery = SceneryManager(world, blueprint_library)
        self.pedestrian_ctrl = PedestrianController(world, blueprint_library)
        self.vehicle_ctrl = TurningVehicleController(world, blueprint_library)
        
        # Configuration
        self.config = ScenarioConfig('scenario4')
        
        # Store ego data
        self.ego_vehicle = None
        self.ego_speed_kmh = 26
        self.turn_config = None
        
    def setup(self):
        """Setup the scenario"""
        print(f"\n{'='*60}")
        print(f"Setting up: {self.name}")
        print(f"Description: {self.description}")
        print(f"{'='*60}")
        
        # Load configurations
        vehicle_config = self.config.load_vehicle_config()
        pedestrian_config = self.config.load_pedestrian_config()
        
        # Spawn ego vehicle
        self.ego_vehicle, self.ego_speed_kmh = self.scenery.spawn_ego_vehicle(vehicle_config)
        
        # Get turn config for ego
        if 'ego_vehicle' in vehicle_config and 'turn' in vehicle_config['ego_vehicle']:
            self.turn_config = vehicle_config['ego_vehicle']['turn']
        
        # Spawn other moving vehicles (trucks + red-light runner)
        self.vehicle_ctrl.spawn_moving_vehicles(vehicle_config)
        
        # Spawn pedestrians (none in this scenario)
        self.pedestrian_ctrl.spawn_pedestrians(pedestrian_config)
        
        # Print summary
        print(f"\n{'='*60}")
        print("Scenario Setup Complete")
        print(f"{'='*60}")
        print(f"Ego vehicle: 1 (Speed: {self.ego_speed_kmh} km/h, turning LEFT)")
        print(f"Moving vehicles: {len(self.vehicle_ctrl.vehicles)}")
        print(f"  - truck_front: blocks ego's view (same lane)")
        print(f"  - truck_adjacent: blocks ego's view (adjacent lane)")
        print(f"  - red_light_runner: approaching from opposite direction")
        print(f"{'='*60}")
        print("\nScenario Logic:")
        print("  1. All vehicles start moving toward intersection")
        print("  2. Trucks block ego's view of oncoming traffic")
        print("  3. Ego initiates LEFT turn at intersection")
        print("  4. Red-light runner (hidden) approaches from opposite direction")
        print("  5. COLLISION when ego's turn path meets runner's straight path")
        print(f"{'='*60}\n")
        
        return {
            'ego_vehicle': self.ego_vehicle,
            'ego_speed_kmh': self.ego_speed_kmh,
            'turn_config': self.turn_config,
            'pedestrian_ctrl': self.pedestrian_ctrl,
            'vehicle_ctrl': self.vehicle_ctrl
        }
    
    def cleanup(self):
        """Cleanup all actors"""
        print("\nCleaning up scenario...")
        self.pedestrian_ctrl.destroy_all()
        self.vehicle_ctrl.destroy_all()
        self.scenery.destroy_all()
        print("Scenario cleanup complete!")


def main():
    """Run Scenario 4 directly for inspection"""
    print("Connecting to CARLA server...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    print("Loading Town 5...")
    world = client.load_world('Town05')
    blueprint_library = world.get_blueprint_library()
    
    scenario = Scenario4(world, blueprint_library)
    
    print("\nNote: Scenario defines the setup only.")
    print("Use 'python -m occl_simulator scenario4' to run with a controller.")
    print("\nSetting up scenario for inspection...")
    
    try:
        scenario_data = scenario.setup()
        input("\nPress Enter to cleanup and exit...")
    finally:
        scenario.cleanup()


if __name__ == '__main__':
    main()
