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


def print_usage():
    """Print usage information"""
    print("\nUsage:")
    print("  python -m occl_simulator <scenario_name> [controller_name]")
    print("\nAvailable scenarios:")
    print("  scenario1  - Pedestrian crossing between two trucks")
    print("  scenario2  - Pedestrian crossing between parked cars on the left")
    print("\nAvailable controllers:")
    print("  default    - Naive cruise controller (alias for 'naive')")
    print("  naive      - Same as default (ignores occlusion info)")
    print("  occl       - Occlusion-aware controller (uses occlusion grid)")
    print("\nExamples:")
    print("  python -m occl_simulator scenario1")
    print("  python -m occl_simulator scenario1 default")
    print("  python -m occl_simulator scenario1 occl")
    print()


def run_default_controller(world, scenario_data, scenario, controller_name: str):
    """Run scenario with the chosen controller mode."""
    print("\nRunning controller...")
    print("Logic: Vehicle drives at constant speed, pedestrian crosses")

    # Map CLI name -> internal mode
    name_l = controller_name.lower()
    if name_l in ("default", "naive"):
        mode = "naive"
    elif name_l in ("occl", "occlusion", "occlusion_aware"):
        mode = "occl"
    else:
        print(f"[Warning] Unknown controller '{controller_name}', falling back to naive")
        mode = "naive"

    # Initialize vehicle controller
    vehicle_ctrl = VehicleController(world, mode=mode)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
    )

    # Optional explicit occluders (e.g., scenario1 trucks)
    if "occluders" in scenario_data:
        vehicle_ctrl.set_occluders(scenario_data["occluders"])
        print(f"Using {len(scenario_data['occluders'])} explicit occluder(s)")
    else:
        print("Using default occluders: all non-ego vehicles")

    pedestrian_ctrl = scenario_data["pedestrian_ctrl"]

    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])

    print(f"\nScenario running in '{mode}' mode - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0

    try:
        while True:
            current_time = time.time() - start_time

            # Start ego vehicle after delay
            if current_time > ego_start_delay and not vehicle_ctrl.ego_moving:
                vehicle_ctrl.start_movement()

            # Get pedestrian positions for visualization (optional)
            ped_locations = getattr(pedestrian_ctrl, "pedestrians", [])

            # Update ego controller (will also render occlusion grid each frame)
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
        vehicle_ctrl.destroy()


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
            run_default_controller(world, scenario_data, scenario, controller_name)
        finally:
            scenario.cleanup()

    except Exception as e:
        print(f"\nError running scenario: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
