#!/usr/bin/env python

"""
Scenario 1: Pedestrian Crossing Between Two Trucks
Setup: Pedestrian crosses street between two stationary trucks
Logic: Vehicle approaches, pedestrian walks across
"""

import carla

from .config import ScenarioConfig
from .actors_static import SceneryManager
from .actors_peds import PedestrianController


class Scenario1:
    """Scenario 1: Pedestrian crossing between two trucks"""
    
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.name = "Pedestrian Crossing Between Two Trucks"
        self.description = "Pedestrian crosses street between parked trucks"
        
        # Scenery manager handles spawning
        self.scenery = SceneryManager(world, blueprint_library)
        self.pedestrian_ctrl = PedestrianController(world, blueprint_library)
        
        # Configuration files for this scenario
        self.config = ScenarioConfig('scenario1')
        
        # Store spawned actors
        self.ego_vehicle = None
        self.ego_speed_kmh = 30
        
    def setup(self):
        """Setup the scenario - spawn all actors"""
        print(f"\n{'='*60}")
        print(f"Setting up: {self.name}")
        print(f"Description: {self.description}")
        print(f"{'='*60}")
        
        # Load configurations
        vehicle_config = self.config.load_vehicle_config()
        pedestrian_config = self.config.load_pedestrian_config()
        
        # Spawn scenery elements
        self.ego_vehicle, self.ego_speed_kmh = self.scenery.spawn_ego_vehicle(vehicle_config)
        self.scenery.spawn_trucks(vehicle_config)
        
        # Spawn pedestrians
        self.pedestrian_ctrl.spawn_pedestrians(pedestrian_config)
        self.pedestrian_ctrl.setup_movement(pedestrian_config)
        
        # Print summary
        print(f"\n{'='*60}")
        print("Scenario Setup Complete")
        print(f"{'='*60}")
        print(f"Ego vehicle: 1 (Speed: {self.ego_speed_kmh} km/h)")
        print(f"Trucks: {self.scenery.get_actor_count() - 1}")
        print(f"Pedestrians: {len(self.pedestrian_ctrl.pedestrians)}")
        print(f"{'='*60}\n")
        
        return {
            'ego_vehicle': self.ego_vehicle,
            'ego_speed_kmh': self.ego_speed_kmh,
            'pedestrian_ctrl': self.pedestrian_ctrl
        }
    
    def cleanup(self):
        """Cleanup all spawned actors"""
        print("\nCleaning up scenario...")
        self.pedestrian_ctrl.destroy_all()
        self.scenery.destroy_all()
        print("Scenario cleanup complete!")


def main():
    """Run Scenario 1 directly"""
    # Connect to CARLA
    print("Connecting to CARLA server...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # Load Town 5
    print("Loading Town 5...")
    world = client.load_world('Town05')
    blueprint_library = world.get_blueprint_library()
    
    # Create scenario
    scenario = Scenario1(world, blueprint_library)
    
    # Setup requires a controller to run
    print("\nNote: Scenario defines the setup only.")
    print("Use 'python main.py scenario1' to run with a controller.")
    print("\nSetting up scenario for inspection...")
    
    try:
        scenario_data = scenario.setup()
        input("\nPress Enter to cleanup and exit...")
    finally:
        scenario.cleanup()


if __name__ == '__main__':
    main()
