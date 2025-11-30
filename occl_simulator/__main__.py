#!/usr/bin/env python

"""
CARLA Scenario Runner - Main Entry Point
Runs scenarios with baseline or universal occlusion-aware controller
"""

import os
import sys
import time

import carla

from occl_controller.controller import VehicleController, OcclusionAwareController
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
    print("  default    - Baseline controller (no occlusion awareness)")
    print("  aware      - Universal occlusion-aware controller (works for ALL scenarios)")
    print("\nExamples:")
    print("  python -m occl_simulator scenario1          # Baseline - no reaction to occlusion")
    print("  python -m occl_simulator scenario1 aware    # Occlusion-aware - slows for hidden hazards")
    print("  python -m occl_simulator scenario2 aware    # Works automatically for any scenario")
    print()


# =============================================================================
# UNIVERSAL OCCLUSION-AWARE CONTROLLER
# Works for ALL scenarios - no scenario-specific logic needed!
# =============================================================================

def run_aware_controller(world, scenario_data, scenario):
    """
    Universal Occlusion-Aware Controller - works for ALL scenarios automatically.
    
    No scenario-specific logic needed. The controller:
    1. Analyzes 360° occlusion grid
    2. Monitors adjacent vehicle behavior (social cues)
    3. Uses YOLO + Depth camera for pedestrian detection
    4. Computes risk from multiple factors using ML-like model
    5. Adjusts speed based on risk level with temporal smoothing
    """
    print("\n" + "="*60)
    print("🧠 UNIVERSAL OCCLUSION-AWARE CONTROLLER")
    print("="*60)
    print("Mode: Automatic - adapts to any scenario")
    print("Features:")
    print("  • 360° occlusion grid analysis (60x60, 30m range)")
    print("  • Region-of-interest (ROI) risk weighting")
    print("  • Social cue detection (adjacent vehicle braking)")
    print("  • YOLO + Depth pedestrian detection (vision-based)")
    print("  • Temporal risk smoothing (20-frame memory)")
    print("  • PID-like smooth control")
    print("="*60)

    # Setup camera FIRST (needed for vision-based detection)
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])
    camera_mgr.update_metrics({'mode': 'OCCLUSION-AWARE'})

    # Initialize universal occlusion-aware controller with vision detection
    vehicle_ctrl = OcclusionAwareController(world, use_vision_detection=True)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
    )
    
    # Connect camera to controller for vision-based pedestrian detection
    vehicle_ctrl.set_camera_manager(camera_mgr)
    
    if "occluders" in scenario_data:
        vehicle_ctrl.set_occluders(scenario_data["occluders"])
        print(f"Using {len(scenario_data['occluders'])} explicit occluder(s)")
    else:
        print("Using default occluders: all non-ego vehicles")
    
    # Get optional controllers from scenario (works with any scenario structure)
    pedestrian_ctrl = scenario_data.get("pedestrian_ctrl")
    moving_vehicle_ctrl = scenario_data.get("moving_vehicle_ctrl")
    vehicle_ctrl_scenario = scenario_data.get("vehicle_ctrl")  # For scenario4 moving vehicles
    turn_config = scenario_data.get("turn_config")  # For ego turning
    
    # Setup ego turn controller if turn config is provided (scenario 4)
    ego_turn_ctrl = None
    if turn_config:
        from .scenario4 import EgoTurnController
        ego_turn_ctrl = EgoTurnController(world)
        ego_turn_ctrl.set_ego_vehicle(
            scenario_data["ego_vehicle"],
            scenario_data["ego_speed_kmh"],
            turn_config
        )
        print(f"Ego turn controller initialized (turn type: {turn_config.get('type', 'left')})")

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0
    
    # Track events for logging
    braking_logged = False

    try:
        while True:
            current_time = time.time() - start_time

            # Start vehicles after delay
            if current_time > ego_start_delay:
                if not vehicle_ctrl.ego_moving:
                    vehicle_ctrl.start_movement()
                    # Start any moving vehicles in the scenario
                    if moving_vehicle_ctrl:
                        moving_vehicle_ctrl.start_movement()
                    if vehicle_ctrl_scenario:
                        vehicle_ctrl_scenario.start_movement()

            # Update pedestrian locations for proximity checks (if applicable)
            if moving_vehicle_ctrl and pedestrian_ctrl:
                ped_locations = []
                for ped in pedestrian_ctrl.pedestrians:
                    ped_locations.append(ped.get_location())
                moving_vehicle_ctrl.update_pedestrian_locations(ped_locations)

            # Update all controllers
            
            # If ego turn controller exists, get steering from it
            if ego_turn_ctrl:
                ego_turn_ctrl.update_steering_only()
                steer = ego_turn_ctrl.get_steering()
                if steer is not None:
                    vehicle_ctrl.set_external_steering(steer)
            
            vehicle_ctrl.update()  # Universal occlusion-aware logic handles everything
            
            # Update camera HUD with metrics from controller
            metrics = vehicle_ctrl.get_metrics() if hasattr(vehicle_ctrl, 'get_metrics') else {}
            metrics['speed_kmh'] = vehicle_ctrl.get_speed() * 3.6
            camera_mgr.update_metrics(metrics)
            
            if pedestrian_ctrl:
                pedestrian_ctrl.update_movement()
            if moving_vehicle_ctrl:
                moving_vehicle_ctrl.update()
                # Log when adjacent vehicles start braking (useful for debugging)
                if moving_vehicle_ctrl.is_any_vehicle_braking() and not braking_logged:
                    print("\n" + "="*50)
                    print("🚨 ADJACENT VEHICLES BRAKING!")
                    print("   Controller detecting social cue...")
                    print("="*50)
                    braking_logged = True
            if vehicle_ctrl_scenario:
                # Update scenario's moving vehicles (trucks, red-light runner)
                vehicle_ctrl_scenario.update()

            # Tick world
            world.tick()
            time.sleep(0.05)  # 20 Hz tick

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        # Print final speeds
        print("\nFinal vehicle speeds:")
        if moving_vehicle_ctrl:
            speeds = moving_vehicle_ctrl.get_vehicle_speeds()
            for veh_id, speed in speeds.items():
                print(f"  {veh_id}: {speed:.1f} km/h")
        if vehicle_ctrl_scenario:
            speeds = vehicle_ctrl_scenario.get_vehicle_speeds()
            for veh_id, speed in speeds.items():
                print(f"  {veh_id}: {speed:.1f} km/h")
        
        ego_speed = vehicle_ctrl.get_speed() * 3.6
        print(f"  ego: {ego_speed:.1f} km/h")
        
        vehicle_ctrl.destroy()
        camera_mgr.destroy()


# =============================================================================
# BASELINE (DEFAULT) CONTROLLERS
# =============================================================================

def run_default_controller(world, scenario_data, scenario):
    """
    Default baseline controller - no occlusion awareness.
    Ego vehicle drives at constant speed regardless of occlusion.
    Uses YOLO for detection visualization only (no action taken).
    """
    print("\n" + "="*60)
    print("🚗 BASELINE CONTROLLER (NO OCCLUSION AWARENESS)")
    print("="*60)
    print("Mode: Constant speed - ignores hidden hazards")
    print("YOLO: Detection visualization only (no action)")
    print("="*60)

    # Initialize baseline controller
    vehicle_ctrl = VehicleController(world)
    vehicle_ctrl.set_ego_vehicle(
        scenario_data["ego_vehicle"],
        scenario_data["ego_speed_kmh"],
    )

    # Get optional controllers
    pedestrian_ctrl = scenario_data.get("pedestrian_ctrl")
    moving_vehicle_ctrl = scenario_data.get("moving_vehicle_ctrl")

    # Setup camera
    camera_mgr = CameraManager(world)
    camera_mgr.setup_camera(scenario_data["ego_vehicle"])
    camera_mgr.update_metrics({'mode': 'BASELINE'})
    
    # Setup YOLO detectors for visualization only
    ped_detector = None
    veh_detector = None
    try:
        from occl_controller.pedestrian_detector import create_detector
        ped_detector = create_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5)
        ped_detector.set_camera_intrinsics(
            camera_mgr.get_camera_intrinsics()['width'],
            camera_mgr.get_camera_intrinsics()['height'],
            camera_mgr.get_camera_intrinsics()['fov']
        )
        print("✓ YOLO pedestrian detection enabled (visualization only)")
    except Exception as e:
        print(f"⚠ Pedestrian detection not available: {e}")
    
    try:
        from occl_controller.vehicle_detector import create_vehicle_detector
        veh_detector = create_vehicle_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5)
        veh_detector.set_camera_intrinsics(
            camera_mgr.get_camera_intrinsics()['width'],
            camera_mgr.get_camera_intrinsics()['height'],
            camera_mgr.get_camera_intrinsics()['fov']
        )
        print("✓ YOLO vehicle detection enabled (visualization only)")
    except Exception as e:
        print(f"⚠ Vehicle detection not available: {e}")

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0
    braking_logged = False

    try:
        while True:
            current_time = time.time() - start_time

            # Start vehicles after delay
            if current_time > ego_start_delay:
                if not vehicle_ctrl.ego_moving:
                    vehicle_ctrl.start_movement()
                    if moving_vehicle_ctrl:
                        moving_vehicle_ctrl.start_movement()

            # Update pedestrian locations for proximity checks
            if moving_vehicle_ctrl and pedestrian_ctrl:
                ped_locations = []
                for ped in pedestrian_ctrl.pedestrians:
                    ped_locations.append(ped.get_location())
                moving_vehicle_ctrl.update_pedestrian_locations(ped_locations)

            # Update all controllers
            vehicle_ctrl.update()
            
            # Run YOLO detection for visualization only (no action taken)
            rgb_frame = camera_mgr.get_raw_frame()
            depth_frame = camera_mgr.get_depth_frame()
            overlay = rgb_frame
            
            if rgb_frame is not None:
                # Detect and draw pedestrians
                if ped_detector:
                    ped_detections = ped_detector.detect(rgb_frame)
                    if ped_detections:
                        ped_positions = ped_detector.estimate_3d_positions(ped_detections, depth_frame)
                        ped_detector.get_pedestrians_in_path(ped_positions)  # Mark in_path for visualization
                        overlay = ped_detector.draw_detections(overlay, ped_positions)
                
                # Detect and draw vehicles
                if veh_detector:
                    veh_detections = veh_detector.detect(rgb_frame)
                    if veh_detections:
                        veh_positions = veh_detector.estimate_3d_positions(veh_detections, depth_frame)
                        overlay = veh_detector.draw_detections(overlay, veh_positions)
                
                # Set the overlay with detection boxes
                if overlay is not None:
                    camera_mgr.set_detection_overlay(overlay)
            
            # Update camera HUD with speed
            camera_mgr.update_metrics({'speed_kmh': vehicle_ctrl.get_speed() * 3.6})
            
            if pedestrian_ctrl:
                pedestrian_ctrl.update_movement()
            if moving_vehicle_ctrl:
                moving_vehicle_ctrl.update()
                if moving_vehicle_ctrl.is_any_vehicle_braking() and not braking_logged:
                    print("\n" + "="*50)
                    print("🚨 ADJACENT VEHICLES BRAKING!")
                    print("   (Baseline ego ignores this)")
                    print("="*50)
                    braking_logged = True

            # Tick world
            world.tick()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        print("\nFinal vehicle speeds:")
        if moving_vehicle_ctrl:
            speeds = moving_vehicle_ctrl.get_vehicle_speeds()
            for veh_id, speed in speeds.items():
                print(f"  {veh_id}: {speed:.1f} km/h")
        ego_speed = vehicle_ctrl.get_speed() * 3.6
        print(f"  ego: {ego_speed:.1f} km/h")
        vehicle_ctrl.destroy()
        camera_mgr.destroy()


def run_scenario4_default_controller(world, scenario_data, scenario):
    """
    Baseline controller for Scenario 4: Left turn with hidden red-light runner.
    Uses EgoTurnController for turn handling, but no occlusion awareness.
    Uses YOLO for detection visualization only (no action taken).
    """
    print("\n" + "="*60)
    print("🚗 SCENARIO 4 BASELINE (LEFT TURN - NO OCCLUSION AWARENESS)")
    print("="*60)
    print("Mode: Turns left without checking for hidden traffic")
    print("YOLO: Detection visualization only (no action)")
    print("="*60)

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
    camera_mgr.update_metrics({'mode': 'BASELINE (LEFT TURN)'})
    
    # Setup YOLO detectors for visualization only
    ped_detector = None
    veh_detector = None
    try:
        from occl_controller.pedestrian_detector import create_detector
        ped_detector = create_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5)
        ped_detector.set_camera_intrinsics(
            camera_mgr.get_camera_intrinsics()['width'],
            camera_mgr.get_camera_intrinsics()['height'],
            camera_mgr.get_camera_intrinsics()['fov']
        )
        print("✓ YOLO pedestrian detection enabled (visualization only)")
    except Exception as e:
        print(f"⚠ Pedestrian detection not available: {e}")
    
    try:
        from occl_controller.vehicle_detector import create_vehicle_detector
        veh_detector = create_vehicle_detector(use_yolo=True, model_name='yolov8n.pt', confidence=0.5)
        veh_detector.set_camera_intrinsics(
            camera_mgr.get_camera_intrinsics()['width'],
            camera_mgr.get_camera_intrinsics()['height'],
            camera_mgr.get_camera_intrinsics()['fov']
        )
        print("✓ YOLO vehicle detection enabled (visualization only)")
    except Exception as e:
        print(f"⚠ Vehicle detection not available: {e}")

    print("\nScenario running - Press Ctrl+C to stop...\n")

    start_time = time.time()
    ego_start_delay = 2.0
    turn_logged = False
    collision_zone_logged = False

    try:
        while True:
            current_time = time.time() - start_time

            if current_time > ego_start_delay:
                if not ego_ctrl.ego_moving:
                    ego_ctrl.start_movement()
                    vehicle_ctrl.start_movement()

            ego_ctrl.update()
            vehicle_ctrl.update()
            
            # Run YOLO detection for visualization only (no action taken)
            rgb_frame = camera_mgr.get_raw_frame()
            depth_frame = camera_mgr.get_depth_frame()
            overlay = rgb_frame
            
            if rgb_frame is not None:
                # Detect and draw pedestrians
                if ped_detector:
                    ped_detections = ped_detector.detect(rgb_frame)
                    if ped_detections:
                        ped_positions = ped_detector.estimate_3d_positions(ped_detections, depth_frame)
                        ped_detector.get_pedestrians_in_path(ped_positions)  # Mark in_path for visualization
                        overlay = ped_detector.draw_detections(overlay, ped_positions)
                
                # Detect and draw vehicles
                if veh_detector:
                    veh_detections = veh_detector.detect(rgb_frame)
                    if veh_detections:
                        veh_positions = veh_detector.estimate_3d_positions(veh_detections, depth_frame)
                        overlay = veh_detector.draw_detections(overlay, veh_positions)
                
                # Set the overlay with detection boxes
                if overlay is not None:
                    camera_mgr.set_detection_overlay(overlay)
            
            # Update camera HUD with speed
            camera_mgr.update_metrics({'speed_kmh': ego_ctrl.get_speed() * 3.6})
            
            if ego_ctrl.is_turning_now() and not turn_logged:
                print("\n" + "="*50)
                print("🔄 EGO TURNING LEFT - View blocked by trucks!")
                print("   Red-light runner approaching (HIDDEN)")
                print("="*50)
                turn_logged = True
            
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

            world.tick()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nScenario interrupted by user")
    finally:
        print("\nFinal vehicle speeds:")
        speeds = vehicle_ctrl.get_vehicle_speeds()
        for veh_id, speed in speeds.items():
            print(f"  {veh_id}: {speed:.1f} km/h")
        ego_speed = ego_ctrl.get_speed() * 3.6
        print(f"  ego: {ego_speed:.1f} km/h")
        camera_mgr.destroy()


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
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
        scenario = scenario_class(world, blueprint_library)
        scenario_data = scenario.setup()

        try:
            if controller_name == "default":
                # Baseline controller - no occlusion awareness
                if scenario_name == "scenario4":
                    # Scenario 4 needs special turn handling
                    run_scenario4_default_controller(world, scenario_data, scenario)
                else:
                    # Universal baseline for all other scenarios
                    run_default_controller(world, scenario_data, scenario)
                    
            elif controller_name == "aware":
                # Universal occlusion-aware controller - works for ALL scenarios!
                run_aware_controller(world, scenario_data, scenario)
                
            else:
                print(f"Error: Unknown controller '{controller_name}'")
                print("Available: default, aware")
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
