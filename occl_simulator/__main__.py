#!/usr/bin/env python

"""
CARLA Scenario Runner - Main Entry Point
Runs scenarios with specified controller logic
"""

import os
import sys
import time

import carla

from occl_controller.controller import VehicleController
from .camera import CameraManager
from .scenario1 import Scenario1
from .scenario2 import Scenario2
from .scenario3 import Scenario3
from .scenario4 import Scenario4, EgoTurnController


def print_usage():
    """Print usage information"""
    print("\nUsage:")
    print("  python -m occl_simulator <scenario_name> [controller_name]")
    print("\nAvailable scenarios:")
    print("  scenario1  - Pedestrian crossing between two trucks")
    print("  scenario2  - Neighbouring-lane moving-vehicle occlusion")
    print("  scenario4  - Red-light runner hidden behind trucks (left turn)")
    print("\nAvailable controllers:")
    print("  default    - Basic collision scenario (default)")
    print("\nExample:")
    print("  python -m occl_simulator scenario1")
    print("  python -m occl_simulator scenario1 default")
    print()


def run_default_controller(world, scenario_data, scenario):
    """Default controller: Simple collision scenario"""
    print("\nRunning with default controller...")
    print("Logic: Vehicle drives at constant speed, pedestrian crosses")

    # Initialize vehicle controller
    vehicle_ctrl = VehicleController(world)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
    )
    if "occluders" in scenario_data:
        vehicle_ctrl.set_occluders(scenario_data["occluders"])
        print(f"Using {len(scenario_data['occluders'])} explicit occluder(s)")
    else:
        print("Using default occluders: all non-ego vehicles")    
    
    pedestrian_ctrl = scenario_data["pedestrian_ctrl"]

    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0

    try:
        while True:
            current_time = time.time() - start_time

            # Start ego vehicle after delay
            if current_time > ego_start_delay and not vehicle_ctrl.ego_moving:
                vehicle_ctrl.start_movement()

            # Update controllers
            ped_locations = getattr(pedestrian_ctrl, "pedestrians", [])

            # Update ego controller with pedestrian positions (for grid video)
            vehicle_ctrl.update(ped_locations)

            # Update pedestrian movement
            pedestrian_ctrl.update_movement()

            # Tick world
            world.tick()
            time.sleep(0.05)  # 20 Hz tick
    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        camera_mgr.destroy()


def run_scenario2_controller(world, scenario_data, scenario):
    """Controller for Scenario 2: Moving vehicles with occlusion"""
    print("\nRunning Scenario 2 controller...")
    print("Logic: Ego and trucks move together, trucks brake for pedestrian")
    print("       Ego must detect adjacent vehicle braking as hazard cue")

    # Initialize ego vehicle controller
    vehicle_ctrl = VehicleController(world)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
    )

    pedestrian_ctrl = scenario_data["pedestrian_ctrl"]
    moving_vehicle_ctrl = scenario_data["moving_vehicle_ctrl"]

    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0
    
    # Track if we've logged the braking event
    braking_logged = False

    try:
        while True:
            current_time = time.time() - start_time

            # Start all vehicles after delay
            if current_time > ego_start_delay:
                if not vehicle_ctrl.ego_moving:
                    vehicle_ctrl.start_movement()
                    moving_vehicle_ctrl.start_movement()

            # Update pedestrian locations for proximity checks
            ped_locations = []
            for ped in pedestrian_ctrl.pedestrians:
                ped_locations.append(ped.get_location())
            moving_vehicle_ctrl.update_pedestrian_locations(ped_locations)

            # Update all controllers
            vehicle_ctrl.update()
            pedestrian_ctrl.update_movement()
            moving_vehicle_ctrl.update()
            
            # Log when trucks start braking (for debugging/visualization)
            if moving_vehicle_ctrl.is_any_vehicle_braking() and not braking_logged:
                print("\n" + "="*50)
                print("🚨 ADJACENT VEHICLES BRAKING!")
                print("   Occlusion-aware ego should react now")
                print("="*50)
                braking_logged = True

            # Tick world
            world.tick()
            time.sleep(0.05)  # 20 Hz tick

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        # Print final speeds
        print("\nFinal vehicle speeds:")
        speeds = moving_vehicle_ctrl.get_vehicle_speeds()
        for veh_id, speed in speeds.items():
            print(f"  {veh_id}: {speed:.1f} km/h")
        ego_speed = vehicle_ctrl.get_speed() * 3.6
        print(f"  ego: {ego_speed:.1f} km/h")
        camera_mgr.destroy()
        
        
def run_scenario3_controller(world, scenario_data, scenario):
    """Controller for Scenario 3: Ego turns left at T-junction while ped crosses"""
    print("\nRunning Scenario 3 controller...")
    print("Logic: Ego drives toward T-junction, then turns LEFT,")
    print("       while pedestrian crosses between parked trucks.")
    
    # Ego left-turn controller (same class used in Scenario 4)
    ego_ctrl = EgoTurnController(world)
    ego_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
        scenario_data.get("turn_config"),
    )
    
    pedestrian_ctrl = scenario_data["pedestrian_ctrl"]
    
    # Camera on ego
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])
    
    print("\nScenario running - Press Ctrl+C to stop...\n")
    
    start_time = time.time()
    ego_start_delay = 2.0
    
    try:
        while True:
            current_time = time.time() - start_time
            
            # Start ego after short delay
            if current_time > ego_start_delay and not ego_ctrl.ego_moving:
                ego_ctrl.start_movement()
            
            # Update controllers
            ego_ctrl.update()
            pedestrian_ctrl.update_movement()
            
            # Step the world
            world.tick()
            time.sleep(0.05)  # ~20 Hz
    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        ego_speed = ego_ctrl.get_speed() * 3.6
        print(f"\nFinal ego speed: {ego_speed:.1f} km/h")
        camera_mgr.destroy()


def run_scenario4_controller(world, scenario_data, scenario):
    """Controller for Scenario 4: Left turn with hidden red-light runner"""
    print("\nRunning Scenario 4 controller...")
    print("Logic: Ego turns left at intersection, trucks block view of oncoming traffic")
    print("       Red-light runner approaches hidden behind trucks")

    # Initialize ego turn controller
    ego_ctrl = EgoTurnController(world)
    ego_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
        scenario_data.get("turn_config")
    )

    vehicle_ctrl = scenario_data["vehicle_ctrl"]

    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0
    
    # Track events
    turn_logged = False
    collision_zone_logged = False

    try:
        while True:
            current_time = time.time() - start_time

            # Start all vehicles after delay
            if current_time > ego_start_delay:
                if not ego_ctrl.ego_moving:
                    ego_ctrl.start_movement()
                    vehicle_ctrl.start_movement()

            # Update controllers
            ego_ctrl.update()
            vehicle_ctrl.update()
            
            # Log when ego starts turning
            if ego_ctrl.is_turning_now() and not turn_logged:
                print("\n" + "="*50)
                print("🔄 EGO TURNING LEFT - View blocked by trucks!")
                print("   Red-light runner approaching (HIDDEN)")
                print("="*50)
                turn_logged = True
            
            # Check for potential collision zone
            ego_loc = ego_ctrl.get_location()
            runner = vehicle_ctrl.get_vehicle_by_id("red_light_runner")
            if ego_loc and runner and not collision_zone_logged:
                runner_loc = runner['actor'].get_location()
                distance = ego_loc.distance(runner_loc)
                if distance < 20.0:
                    print("\n" + "="*50)
                    print("⚠️  COLLISION IMMINENT!")
                    print(f"   Distance to red-light runner: {distance:.1f}m")
                    print("="*50)
                    collision_zone_logged = True

            # Tick world
            world.tick()
            time.sleep(0.05)  # 20 Hz tick

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        # Print final speeds
        print("\nFinal vehicle speeds:")
        speeds = vehicle_ctrl.get_vehicle_speeds()
        for veh_id, speed in speeds.items():
            print(f"  {veh_id}: {speed:.1f} km/h")
        ego_speed = ego_ctrl.get_speed() * 3.6
        print(f"  ego: {ego_speed:.1f} km/h")
        camera_mgr.destroy()


def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Error: No scenario specified!")
        print_usage()
        sys.exit(1)

    scenario_name = sys.argv[1]
    controller_name = sys.argv[2] if len(sys.argv) > 2 else "default"

    print("=" * 60)
    print("CARLA Scenario Runner")
    print("=" * 60)
    print(f"Scenario: {scenario_name}")
    print(f"Controller: {controller_name}")
    print("=" * 60)

    # Map scenario names to classes
    SCENARIOS = {
        "scenario1": Scenario1,
        "scenario2": Scenario2,
        "scenario3": Scenario3,
        "scenario4": Scenario4,
    }

    if scenario_name not in SCENARIOS:
        print(f"\nError: Scenario '{scenario_name}' not found!")
        print_usage()
        sys.exit(1)

    # Connect to CARLA server
    try:
        print("\nConnecting to CARLA server...")
        port = int(os.environ.get("CARLA_PORT", "2000"))
        client = carla.Client("localhost", port)
        client.set_timeout(10.0)

        print(f"Loading Town 5 on port {port} ...")
        world = client.load_world("Town05")
        blueprint_library = world.get_blueprint_library()

    except Exception as e:
        print(f"Error connecting to CARLA: {e}")
        print("Make sure CARLA simulator is running!")
        sys.exit(1)

    # Load and setup the scenario
    scenario_class = SCENARIOS[scenario_name]

    try:
        # Create scenario instance
        scenario = scenario_class(world, blueprint_library)

        # Setup the scenario (spawn actors)
        scenario_data = scenario.setup()

        # Run with the specified controller
        try:
            if controller_name == "default":
                # Use scenario-specific controller if available
                if scenario_name == "scenario2":
                    run_scenario2_controller(world, scenario_data, scenario)
                elif scenario_name == "scenario3":
                    run_scenario3_controller(world, scenario_data, scenario)
                elif scenario_name == "scenario4":
                    run_scenario4_controller(world, scenario_data, scenario)
                else:
                    run_default_controller(world, scenario_data, scenario)
            else:
                print(f"Error: Unknown controller '{controller_name}'")
                sys.exit(1)
        finally:
            scenario.cleanup()

    except Exception as e:
        print(f"\nError running scenario: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
