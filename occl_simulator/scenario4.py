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
            
            if current_speed < target_speed_ms:
                control.throttle = 0.7
                control.brake = 0.0
            else:
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


class EgoTurnController:
    """Controller for ego vehicle that needs to make a left turn"""
    
    def __init__(self, world):
        self.world = world
        self.ego_vehicle = None
        self.ego_speed_kmh = 26
        self.ego_control = carla.VehicleControl()
        self.ego_moving = False
        
        # Turn configuration
        self.turn_type = None
        self.turn_trigger_x = None
        self.turn_target = None
        self.is_turning = False
        self.turn_complete = False
        
    def set_ego_vehicle(self, vehicle, speed_kmh, turn_config=None):
        """Set the ego vehicle with optional turn configuration"""
        self.ego_vehicle = vehicle
        self.ego_speed_kmh = speed_kmh
        
        if turn_config:
            self.turn_type = turn_config.get('type', 'left')
            self.turn_trigger_x = turn_config.get('trigger_x', 15.0)
            start_loc = self.ego_vehicle.get_location()
            self._turn_decreasing_x = self.turn_trigger_x < start_loc.x
            target = turn_config.get('target_location', {})
            self.turn_target = carla.Location(
                x=target.get('x', 25.0),
                y=target.get('y', -20.0),
                z=target.get('z', 0.98)
            )
            direction = "decreasing" if self._turn_decreasing_x else "increasing"
            print(
                f"Ego vehicle will turn {self.turn_type} at x={self.turn_trigger_x} "
                f"along {direction} x"
            )
        
        print(f"Ego turn controller initialized at {speed_kmh} km/h")
    
    def start_movement(self):
        """Start ego movement"""
        self.ego_moving = True
        print("\nEgo vehicle starting to move...")
    
    def stop_movement(self):
        """Stop ego movement"""
        self.ego_moving = False
        self.ego_control.throttle = 0.0
        self.ego_control.brake = 1.0
        if self.ego_vehicle:
            self.ego_vehicle.apply_control(self.ego_control)
        print("\nEgo vehicle stopped")
    
    def update(self):
        """Update ego vehicle - go straight until turn trigger, then turn"""
        if not self.ego_moving or not self.ego_vehicle:
            return
        
        current_location = self.ego_vehicle.get_location()
        current_transform = self.ego_vehicle.get_transform()
        
        # Check if we should start turning
        # Check if we should start turning
        if not self.is_turning and self.turn_trigger_x is not None:
            if (
                (not getattr(self, "_turn_decreasing_x", False) and current_location.x >= self.turn_trigger_x)
                or
                (getattr(self, "_turn_decreasing_x", False) and current_location.x <= self.turn_trigger_x)
            ):
                self.is_turning = True
                print(f"\n🔄 Ego starting LEFT TURN at x={current_location.x:.1f}")

        
        # Calculate control
        target_speed_ms = self.ego_speed_kmh / 3.6
        velocity = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Throttle control
        if current_speed < target_speed_ms:
            self.ego_control.throttle = 0.6
            self.ego_control.brake = 0.0
        else:
            self.ego_control.throttle = 0.3
            self.ego_control.brake = 0.0
        
        # Steering control
        if self.is_turning and self.turn_target and not self.turn_complete:
            # Calculate direction to target
            dx = self.turn_target.x - current_location.x
            dy = self.turn_target.y - current_location.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 3.0:
                self.turn_complete = True
                print("✓ Turn complete")
                self.ego_control.steer = 0.0
            else:
                # Calculate desired yaw
                desired_yaw = math.degrees(math.atan2(dy, dx))
                current_yaw = current_transform.rotation.yaw
                
                # Calculate yaw difference
                yaw_diff = desired_yaw - current_yaw
                
                # Normalize to [-180, 180]
                while yaw_diff > 180:
                    yaw_diff -= 360
                while yaw_diff < -180:
                    yaw_diff += 360
                
                # Convert to steering (-1 to 1)
                # Negative yaw_diff means turn left (negative steer in CARLA)
                steer = max(-1.0, min(1.0, yaw_diff / 45.0))
                self.ego_control.steer = steer
        else:
            self.ego_control.steer = 0.0
        
        self.ego_vehicle.apply_control(self.ego_control)
    
    def get_location(self):
        """Get current ego location"""
        if self.ego_vehicle:
            return self.ego_vehicle.get_location()
        return None
    
    def get_speed(self):
        """Get current ego speed in m/s"""
        if self.ego_vehicle:
            velocity = self.ego_vehicle.get_velocity()
            return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return 0.0
    
    def is_turning_now(self):
        """Check if ego is currently turning"""
        return self.is_turning and not self.turn_complete


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
