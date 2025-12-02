#!/usr/bin/env python

"""
Modular Turn Controller for Ego Vehicle
Can be used by any scenario that requires the ego vehicle to make a turn.
Supports left and right turns with configurable trigger points and targets.
"""

import math
import carla


class EgoTurnController:
    """
    Modular controller for ego vehicle turns.
    
    Can be used standalone (controls throttle/brake/steer) or with external
    throttle/brake control (only provides steering via get_steering()).
    
    Configuration via turn_config dict:
    {
        "type": "left" or "right",
        "trigger_x": float,  # X coordinate to start turn
        "target_location": {"x": float, "y": float, "z": float}
    }
    """
    
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
        self._computed_steer = 0.0  # Computed steering for external control
        
    def set_ego_vehicle(self, vehicle, speed_kmh, turn_config=None):
        """Set the ego vehicle with optional turn configuration"""
        self.ego_vehicle = vehicle
        self.ego_speed_kmh = speed_kmh
        
        if turn_config:
            self.turn_type = turn_config.get('type', 'left')
            self.turn_trigger_x = turn_config.get('trigger_x', 15.0)
            target = turn_config.get('target_location', {})
            self.turn_target = carla.Location(
                x=target.get('x', 25.0),
                y=target.get('y', -20.0),
                z=target.get('z', 0.98)
            )
            print(f"Ego vehicle will turn {self.turn_type} at x={self.turn_trigger_x}")
        
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
    
    def _check_turn_trigger(self):
        """
        Check if the turn should be triggered based on current position and direction.
        Handles both +X and -X travel directions automatically.
        
        Returns:
            bool: True if turn should start now
        """
        if self.is_turning or not self.turn_trigger_x or not self.ego_vehicle:
            return False
        
        current_location = self.ego_vehicle.get_location()
        current_transform = self.ego_vehicle.get_transform()
        
        # Determine direction of travel based on yaw
        # yaw ~= 0 means moving in +X direction, yaw ~= 180 means moving in -X direction
        current_yaw = current_transform.rotation.yaw
        moving_negative_x = abs(current_yaw) > 90  # yaw between 90-270 means -X direction
        
        if moving_negative_x:
            # Moving in -X direction: trigger when x <= trigger_x
            return current_location.x <= self.turn_trigger_x
        else:
            # Moving in +X direction: trigger when x >= trigger_x
            return current_location.x >= self.turn_trigger_x
    
    def _compute_steering(self):
        """
        Compute steering value to reach the turn target.
        
        Returns:
            float: Steering value between -1.0 and 1.0
        """
        if not self.ego_vehicle or not self.turn_target or self.turn_complete:
            return 0.0
        
        current_location = self.ego_vehicle.get_location()
        current_transform = self.ego_vehicle.get_transform()
        
        dx = self.turn_target.x - current_location.x
        dy = self.turn_target.y - current_location.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 3.0:
            self.turn_complete = True
            print("✓ Turn complete")
            return 0.0
        
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
        steer = max(-1.0, min(1.0, yaw_diff / 45.0))
        return steer
    
    def update(self):
        """Update ego vehicle - go straight until turn trigger, then turn"""
        if not self.ego_moving or not self.ego_vehicle:
            return
        
        # Check if we should start turning
        if not self.is_turning and self._check_turn_trigger():
            self.is_turning = True
            current_location = self.ego_vehicle.get_location()
            print(f"\n🔄 Ego starting {self.turn_type.upper()} TURN at x={current_location.x:.1f}")
        
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
        if self.is_turning and not self.turn_complete:
            self.ego_control.steer = self._compute_steering()
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
    
    def get_steering(self):
        """Get current steering value for external control"""
        return self._computed_steer
    
    def update_steering_only(self):
        """
        Compute steering without applying control.
        Use this when another controller handles throttle/brake.
        Call get_steering() after this to get the computed value.
        """
        if not self.ego_vehicle:
            self._computed_steer = 0.0
            return
        
        # Check if we should start turning
        if not self.is_turning and self._check_turn_trigger():
            self.is_turning = True
            current_location = self.ego_vehicle.get_location()
            print(f"\n🔄 Ego starting {self.turn_type.upper()} TURN at x={current_location.x:.1f}")
        
        # Compute steering
        if self.is_turning and not self.turn_complete:
            self._computed_steer = self._compute_steering()
        else:
            self._computed_steer = 0.0
    
    def is_turning_now(self):
        """Check if ego is currently turning"""
        return self.is_turning and not self.turn_complete
    
    def reset(self):
        """Reset turn state for reuse"""
        self.is_turning = False
        self.turn_complete = False
        self._computed_steer = 0.0
