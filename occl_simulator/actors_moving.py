"""
Moving Vehicle Controller Module
Handles spawning and control of moving vehicles (trucks/vans) that can brake
"""

import carla
import math


class MovingVehicleController:
    """Controls moving vehicles in adjacent lanes that can trigger braking"""
    
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicles = []  # List of dicts with vehicle actor and config
        self.pedestrian_locations = []  # Updated externally for proximity checks
        
    def spawn_moving_vehicles(self, vehicle_config):
        """Spawn moving vehicles from configuration"""
        if 'moving_vehicles' not in vehicle_config:
            print("\nNo moving vehicles found in config.")
            return
        
        print("\nSpawning moving vehicles...")
        
        for i, veh_config in enumerate(vehicle_config['moving_vehicles']):
            # Get vehicle type
            veh_type = veh_config.get('type', 'vehicle.carlamotors.firetruck')
            
            try:
                veh_bp = self.blueprint_library.find(veh_type)
            except:
                print(f"Warning: Vehicle type '{veh_type}' not found, using default")
                veh_bp = self.blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
            
            # Create spawn point
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
                
                # Store vehicle with its config
                vehicle_data = {
                    'actor': vehicle,
                    'id': veh_config.get('id', f'vehicle_{i}'),
                    'speed_kmh': veh_config.get('speed_kmh', 26),
                    'brake_trigger': veh_config.get('brake_trigger', 'none'),
                    'brake_distance': veh_config.get('brake_distance', 15.0),
                    'brake_decel': veh_config.get('brake_decel', 8.0),
                    'follow_distance': veh_config.get('follow_distance', 8.0),
                    'is_braking': False,
                    'is_moving': False,
                    'control': carla.VehicleControl()
                }
                self.vehicles.append(vehicle_data)
                
                veh_id = vehicle_data['id']
                print(f"Moving vehicle '{veh_id}' spawned at ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})")
                
            except Exception as e:
                print(f"Failed to spawn moving vehicle {i+1}: {e}")
    
    def start_movement(self):
        """Start all moving vehicles"""
        for veh_data in self.vehicles:
            veh_data['is_moving'] = True
        print(f"\n{len(self.vehicles)} moving vehicle(s) started")
    
    def update_pedestrian_locations(self, locations):
        """Update pedestrian locations for proximity checks"""
        self.pedestrian_locations = locations
    
    def get_front_vehicle(self):
        """Get the front vehicle (for follow trigger)"""
        for veh_data in self.vehicles:
            if veh_data['brake_trigger'] == 'pedestrian_proximity':
                return veh_data
        return None
    
    def is_any_vehicle_braking(self):
        """Check if any moving vehicle is braking"""
        return any(veh['is_braking'] for veh in self.vehicles)
    
    def get_braking_vehicle_location(self):
        """Get location of braking vehicle"""
        for veh_data in self.vehicles:
            if veh_data['is_braking']:
                return veh_data['actor'].get_location()
        return None
    
    def update(self):
        """Update all moving vehicles each frame"""
        front_vehicle = self.get_front_vehicle()
        
        for veh_data in self.vehicles:
            if not veh_data['is_moving']:
                continue
            
            vehicle = veh_data['actor']
            control = veh_data['control']
            
            # Check brake triggers
            should_brake = False
            
            if veh_data['brake_trigger'] == 'pedestrian_proximity':
                # Check distance to pedestrians
                veh_location = vehicle.get_location()
                for ped_loc in self.pedestrian_locations:
                    distance = veh_location.distance(ped_loc)
                    if distance < veh_data['brake_distance']:
                        should_brake = True
                        if not veh_data['is_braking']:
                            print(f"\n⚠ {veh_data['id']}: Pedestrian detected at {distance:.1f}m - HARD BRAKE!")
                        break
            
            elif veh_data['brake_trigger'] == 'follow_front':
                # Follow the front vehicle's braking
                if front_vehicle and front_vehicle['is_braking']:
                    # Check distance to front vehicle
                    veh_location = vehicle.get_location()
                    front_location = front_vehicle['actor'].get_location()
                    distance = veh_location.distance(front_location)
                    
                    if distance < veh_data['follow_distance'] + 5:
                        should_brake = True
                        if not veh_data['is_braking']:
                            print(f"\n⚠ {veh_data['id']}: Front vehicle braking - FOLLOWING BRAKE!")
            
            # Apply control
            if should_brake:
                veh_data['is_braking'] = True
                control.throttle = 0.0
                control.brake = 1.0
            else:
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
            
            control.steer = 0.0
            vehicle.apply_control(control)
    
    def get_vehicle_speeds(self):
        """Get current speeds of all vehicles"""
        speeds = {}
        for veh_data in self.vehicles:
            velocity = veh_data['actor'].get_velocity()
            speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speeds[veh_data['id']] = speed_ms * 3.6  # km/h
        return speeds
    
    def destroy_all(self):
        """Destroy all moving vehicles"""
        print("Destroying moving vehicles...")
        for veh_data in self.vehicles:
            if veh_data['actor'].is_alive:
                veh_data['actor'].destroy()
