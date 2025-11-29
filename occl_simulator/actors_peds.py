"""
Pedestrian Controller Module
Handles pedestrian spawning and movement logic
"""

import carla
import math
import random


class PedestrianController:
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.pedestrians = []
        self.ped_targets = []
        
    def spawn_pedestrians(self, pedestrian_config):
        """Spawn pedestrians from configuration"""
        if not pedestrian_config or 'pedestrians' not in pedestrian_config:
            print("\nNo pedestrian data found, skipping pedestrian spawn.")
            return
        
        print("\nSpawning pedestrians from JSON positions...")
        pedestrian_blueprints = self.blueprint_library.filter('walker.pedestrian.*')
        
        # Spawn pedestrians
        for i, ped_pos in enumerate(pedestrian_config['pedestrians']):
            ped_bp = random.choice(pedestrian_blueprints)
            
            # Create spawn point
            ped_location = carla.Location(
                x=ped_pos['location']['x'],
                y=ped_pos['location']['y'],
                z=ped_pos['location']['z']
            )
            ped_rotation = carla.Rotation(
                pitch=ped_pos['rotation']['pitch'],
                yaw=ped_pos['rotation']['yaw'],
                roll=ped_pos['rotation']['roll']
            )
            spawn_point = carla.Transform(ped_location, ped_rotation)
            
            try:
                pedestrian = self.world.spawn_actor(ped_bp, spawn_point)
                self.pedestrians.append(pedestrian)
                print(f"Pedestrian {i+1} spawned at ({ped_location.x:.1f}, {ped_location.y:.1f}, {ped_location.z:.1f})")
            except Exception as e:
                print(f"Failed to spawn pedestrian {i+1}: {e}")
    
    def setup_movement(self, pedestrian_config):
        """Setup pedestrian movement targets"""
        if not self.pedestrians:
            return
        
        print("\nSetting up pedestrian manual movement...")
        
        for i, pedestrian in enumerate(self.pedestrians):
            if pedestrian_config and 'pedestrians' in pedestrian_config and i < len(pedestrian_config['pedestrians']):
                ped_info = pedestrian_config['pedestrians'][i]
                if 'target_location' in ped_info:
                    target = carla.Location(
                        x=ped_info['target_location']['x'],
                        y=ped_info['target_location']['y'],
                        z=ped_info['target_location']['z']
                    )
                    self.ped_targets.append({
                        'location': target,
                        'speed': ped_info.get('speed_ms', 1.4)
                    })
                    speed = ped_info.get('speed_ms', 1.4)
                    print(f"Pedestrian {i+1} will move to ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) at {speed} m/s")
                else:
                    self.ped_targets.append(None)
            else:
                self.ped_targets.append(None)
        
        self.world.tick()
        print(f"Pedestrian movement ready for {len(self.pedestrians)} pedestrian(s)")
    
    def update_movement(self):
        """Update pedestrian positions each frame"""
        if not self.pedestrians or not self.ped_targets:
            return
        
        for i, pedestrian in enumerate(self.pedestrians):
            if i < len(self.ped_targets) and self.ped_targets[i] is not None:
                try:
                    current_location = pedestrian.get_location()
                    target_info = self.ped_targets[i]
                    target = target_info['location']
                    ped_speed = target_info['speed']
                    
                    # Calculate distance to target
                    dx = target.x - current_location.x
                    dy = target.y - current_location.y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Only move if not at target (within 0.5 meters)
                    if distance > 0.5:
                        # Normalize direction
                        direction_x = dx / distance
                        direction_y = dy / distance
                        
                        # Use speed from JSON (converted for 100ms tick)
                        speed = ped_speed * 0.1
                        
                        # Calculate new location
                        new_location = carla.Location(
                            x=current_location.x + direction_x * speed,
                            y=current_location.y + direction_y * speed,
                            z=current_location.z
                        )
                        
                        # Calculate rotation to face movement direction
                        yaw = math.degrees(math.atan2(direction_y, direction_x))
                        
                        # Set new transform
                        new_transform = carla.Transform(
                            new_location,
                            carla.Rotation(pitch=0, yaw=yaw, roll=0)
                        )
                        pedestrian.set_transform(new_transform)
                except Exception as e:
                    print(f"Error moving pedestrian {i+1}: {e}")
    
    def destroy_all(self):
        """Destroy all spawned pedestrians"""
        print("Destroying pedestrians...")
        for pedestrian in self.pedestrians:
            if pedestrian.is_alive:
                pedestrian.destroy()
