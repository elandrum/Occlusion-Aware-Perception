"""
Vehicle Controller Module
Handles ego vehicle control logic and behaviors
"""

import carla
import math


class VehicleController:
    def __init__(self, world):
        self.world = world
        self.ego_vehicle = None
        self.ego_speed_kmh = 30
        self.ego_control = carla.VehicleControl()
        self.ego_moving = False
        
    def set_ego_vehicle(self, vehicle, speed_kmh=30):
        """Set the ego vehicle to control"""
        self.ego_vehicle = vehicle
        self.ego_speed_kmh = speed_kmh
        print(f"Ego vehicle control initialized at {speed_kmh} km/h")
        
    def start_movement(self):
        """Start the ego vehicle movement"""
        self.ego_moving = True
        print("\nEgo vehicle starting to move...")
    
    def stop_movement(self):
        """Stop the ego vehicle movement"""
        self.ego_moving = False
        self.ego_control.throttle = 0.0
        self.ego_control.brake = 1.0
        if self.ego_vehicle:
            self.ego_vehicle.apply_control(self.ego_control)
        print("\nEgo vehicle stopped")
    
    def update(self):
        """Update ego vehicle control each frame"""
        if not self.ego_moving or not self.ego_vehicle:
            return
        
        # Convert km/h to m/s
        target_speed_ms = self.ego_speed_kmh / 3.6
        
        # Get current velocity
        velocity = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Simple throttle control to maintain speed
        if current_speed < target_speed_ms:
            self.ego_control.throttle = 0.7
            self.ego_control.brake = 0.0
        else:
            self.ego_control.throttle = 0.3
            self.ego_control.brake = 0.0
        
        self.ego_control.steer = 0.0  # Keep straight
        self.ego_vehicle.apply_control(self.ego_control)
    
    def check_pedestrian_proximity(self, pedestrian_positions, stop_distance=5.0):
        """Check if pedestrian is nearby and stop if needed"""
        if not self.ego_vehicle or not self.ego_moving:
            return False
        
        ego_location = self.ego_vehicle.get_location()
        
        for ped_location in pedestrian_positions:
            distance = ego_location.distance(ped_location)
            if distance < stop_distance:
                self.stop_movement()
                print(f"Pedestrian detected at {distance:.1f}m - Emergency stop!")
                return True
        
        return False
    
    def get_location(self):
        """Get current ego vehicle location"""
        if self.ego_vehicle:
            return self.ego_vehicle.get_location()
        return None
    
    def get_speed(self):
        """Get current ego vehicle speed in m/s"""
        if self.ego_vehicle:
            velocity = self.ego_vehicle.get_velocity()
            return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return 0.0

