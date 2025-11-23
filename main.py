#!/usr/bin/env python

"""
CARLA Scenario Runner - Main Entry Point
Runs scenarios with specified controller logic
"""

import carla
import sys
import time
import importlib

from vehicle_controller import VehicleController
from camera_manager import CameraManager


def print_usage():
    """Print usage information"""
    print("\nUsage: python main.py <scenario_name> [controller_name]")
    print("\nAvailable scenarios:")
    print("  scenario1  - Pedestrian crossing between two trucks")
    print("\nAvailable controllers:")
    print("  default    - Basic collision scenario (default)")
    print("\nExample:")
    print("  python main.py scenario1")
    print("  python main.py scenario1 default")
    print()


def run_default_controller(world, scenario_data, scenario):
    """Default controller: Simple collision scenario"""
    print("\nRunning with default controller...")
    print("Logic: Vehicle drives at constant speed, pedestrian crosses")
    
    # Initialize vehicle controller
    vehicle_ctrl = VehicleController(world)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data['ego_vehicle'],
        scenario_data['ego_speed_kmh']
    )
    
    pedestrian_ctrl = scenario_data['pedestrian_ctrl']
    
    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data['ego_vehicle'])
    
    print("\nScenario running - Press Ctrl+C to stop...\n")
    
    # Timing
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
            time.sleep(0.05)  # 20Hz tick rate for smoother simulation
            
    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        # Cleanup - video is already saved during recording
        camera_mgr.destroy()


def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Error: No scenario specified!")
        print_usage()
        sys.exit(1)
    
    scenario_name = sys.argv[1]
    controller_name = sys.argv[2] if len(sys.argv) > 2 else 'default'
    
    print("="*60)
    print("CARLA Scenario Runner")
    print("="*60)
    print(f"Scenario: {scenario_name}")
    print(f"Controller: {controller_name}")
    print("="*60)
    
    # Connect to CARLA server
    try:
        print("\nConnecting to CARLA server...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        
        # Load Town 5
        print("Loading Town 5...")
        world = client.load_world('Town05')
        blueprint_library = world.get_blueprint_library()
        
    except Exception as e:
        print(f"Error connecting to CARLA: {e}")
        print("Make sure CARLA simulator is running!")
        sys.exit(1)
    
    # Load and setup the scenario
    try:
        # Import the scenario module
        scenario_module = importlib.import_module(scenario_name)
        
        # Get the scenario class
        scenario_class_name = ''.join([word.capitalize() for word in scenario_name.split('_')])
        scenario_class = getattr(scenario_module, scenario_class_name)
        
        # Create scenario instance
        scenario = scenario_class(world, blueprint_library)
        
        # Setup the scenario (spawn actors)
        scenario_data = scenario.setup()
        
        # Run with the specified controller
        try:
            if controller_name == 'default':
                run_default_controller(world, scenario_data, scenario)
            else:
                print(f"Error: Unknown controller '{controller_name}'")
                sys.exit(1)
        finally:
            scenario.cleanup()
            
    except ModuleNotFoundError:
        print(f"\nError: Scenario '{scenario_name}' not found!")
        print(f"Make sure {scenario_name}.py exists in the project directory.")
        print_usage()
        sys.exit(1)
        
    except AttributeError as e:
        print(f"\nError: Could not find scenario class in {scenario_name}.py")
        print(f"Details: {e}")
        sys.exit(1)
        
    except Exception as e:
        print(f"\nError running scenario: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
