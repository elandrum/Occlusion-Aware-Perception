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
from .scenario1 import Scenario1  # scenario file lives in occl_simulator/scenario1.py


def print_usage():
    """Print usage information"""
    print("\nUsage:")
    print("  python -m occl_simulator <scenario_name> [controller_name]")
    print("\nAvailable scenarios:")
    print("  scenario1  - Pedestrian crossing between two trucks")
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
            vehicle_ctrl.update()
            pedestrian_ctrl.update_movement()

            # Tick world
            world.tick()
            time.sleep(0.05)  # 20 Hz tick

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
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
