#!/usr/bin/env python

"""
Scenario 2: Neighbouring-Lane Moving-Vehicle Occlusion
Setup: Two trucks moving in adjacent lane, pedestrian crosses in front
Logic: Front truck brakes for pedestrian, ego cannot see pedestrian (occluded by trucks)
       Occlusion-aware ego should detect truck braking and slow down preemptively
"""

import carla

from .config import ScenarioConfig
from .actors_static import SceneryManager
from .actors_peds import PedestrianController
from .actors_moving import MovingVehicleController


class Scenario2:
    """Scenario 2: Neighbouring-lane moving-vehicle occlusion"""
    
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.name = "Neighbouring-Lane Moving-Vehicle Occlusion"
        self.description = "Two trucks in adjacent lane, front truck brakes for hidden pedestrian"
        
        # Managers
        self.scenery = SceneryManager(world, blueprint_library)
        self.pedestrian_ctrl = PedestrianController(world, blueprint_library)
        self.moving_vehicle_ctrl = MovingVehicleController(world, blueprint_library)
        
        # Configuration files for this scenario
        self.config = ScenarioConfig('scenario2')
        
        # Store spawned actors
        self.ego_vehicle = None
        self.ego_speed_kmh = 26
        
    def setup(self):
        """Setup the scenario - spawn all actors"""
        print(f"\n{'='*60}")
        print(f"Setting up: {self.name}")
        print(f"Description: {self.description}")
        print(f"{'='*60}")
        
        # Load configurations
        vehicle_config = self.config.load_vehicle_config()
        pedestrian_config = self.config.load_pedestrian_config()
        
        # Spawn ego vehicle (using scenery manager for consistency)
        self.ego_vehicle, self.ego_speed_kmh = self.scenery.spawn_ego_vehicle(vehicle_config)
        
        # Spawn moving vehicles (trucks in adjacent lane)
        self.moving_vehicle_ctrl.spawn_moving_vehicles(vehicle_config)
        
        # Spawn pedestrians
        self.pedestrian_ctrl.spawn_pedestrians(pedestrian_config)
        self.pedestrian_ctrl.setup_movement(pedestrian_config)
        
        # Print summary
        print(f"\n{'='*60}")
        print("Scenario Setup Complete")
        print(f"{'='*60}")
        print(f"Ego vehicle: 1 (Speed: {self.ego_speed_kmh} km/h)")
        print(f"Moving vehicles (adjacent lane): {len(self.moving_vehicle_ctrl.vehicles)}")
        print(f"Pedestrians: {len(self.pedestrian_ctrl.pedestrians)}")
        print(f"{'='*60}")
        print("\nScenario Logic:")
        print("  1. Ego and trucks start moving together")
        print("  2. Truck_behind runs parallel to Ego (side by side)")
        print("  3. Pedestrian crosses in front of Truck_front")
        print("  4. Truck_front brakes hard → Truck_behind follows")
        print("  5. Ego must detect adjacent braking and react")
        print(f"{'='*60}\n")
        
        return {
            'ego_vehicle': self.ego_vehicle,
            'ego_speed_kmh': self.ego_speed_kmh,
            'pedestrian_ctrl': self.pedestrian_ctrl,
            'moving_vehicle_ctrl': self.moving_vehicle_ctrl
        }
    
    def cleanup(self):
        """Cleanup all spawned actors"""
        print("\nCleaning up scenario...")
        self.pedestrian_ctrl.destroy_all()
        self.moving_vehicle_ctrl.destroy_all()
        self.scenery.destroy_all()
        print("Scenario cleanup complete!")


def main():
    """Run Scenario 2 directly for inspection"""
    print("Connecting to CARLA server...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    print("Loading Town 5...")
    world = client.load_world('Town05')
    blueprint_library = world.get_blueprint_library()
    
    scenario = Scenario2(world, blueprint_library)
    
    print("\nNote: Scenario defines the setup only.")
    print("Use 'python -m occl_simulator scenario2' to run with a controller.")
    print("\nSetting up scenario for inspection...")
    
    try:
        scenario_data = scenario.setup()
        input("\nPress Enter to cleanup and exit...")
    finally:
        scenario.cleanup()


if __name__ == '__main__':
    main()
