#!/usr/bin/env python

"""
CARLA Waypoint Capture Tool
Spawns a spectator camera that you can move around in CARLA
Press 'C' to capture the current position and rotation
Outputs waypoints to waypoints.json
"""

import carla
import json
import os
import time
from datetime import datetime

# Try to import pygame at module level
try:
    import pygame
    from pygame.locals import QUIT, KEYDOWN, K_q, K_e, K_t, K_p, K_s, K_r
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

class WaypointCapture:
    def __init__(self):
        self.waypoints = []
        self.ego_position = None
        self.trucks = []
        self.pedestrians = []
        
    def capture_spectator_position(self, spectator):
        """Capture current spectator camera position"""
        transform = spectator.get_transform()
        location = transform.location
        rotation = transform.rotation
        
        position_data = {
            "location": {
                "x": round(location.x, 2),
                "y": round(location.y, 2),
                "z": round(location.z, 2)
            },
            "rotation": {
                "pitch": round(rotation.pitch, 2),
                "yaw": round(rotation.yaw, 2),
                "roll": round(rotation.roll, 2)
            }
        }
        
        return position_data, location
    
    def save_to_json(self, filename='vehicles.json'):
        """Save captured waypoints to JSON file"""
        data = {}
        
        if self.ego_position:
            data['ego_vehicle'] = self.ego_position
        
        if self.trucks:
            data['trucks'] = self.trucks
        
        if self.pedestrians:
            data['pedestrians'] = self.pedestrians
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\n✓ Saved to {filename}")
    
    def print_summary(self):
        """Print current capture summary"""
        print("\n" + "="*50)
        print("CAPTURED WAYPOINTS SUMMARY")
        print("="*50)
        print(f"Ego vehicle: {'✓' if self.ego_position else '✗'}")
        print(f"Trucks: {len(self.trucks)}")
        print(f"Pedestrians: {len(self.pedestrians)}")
        print("="*50)

def main():
    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # Get world
    world = client.get_world()
    print(f"Connected to {world.get_map().name}")
    
    # Get spectator
    spectator = world.get_spectator()
    
    # Initialize capture system
    capture = WaypointCapture()
    
    print("\n" + "="*50)
    print("WAYPOINT CAPTURE MODE")
    print("="*50)
    print("Controls:")
    print("  • Move the spectator camera to desired positions")
    print("  • Press 'E' - Capture EGO vehicle position")
    print("  • Press 'T' - Capture TRUCK position")
    print("  • Press 'P' - Capture PEDESTRIAN position")
    print("  • Press 'S' - Save to vehicles.json")
    print("  • Press 'R' - Reset all captures")
    print("  • Press 'Q' - Quit")
    print("="*50)
    
    capture.print_summary()
    
    try:
        # Check if pygame is available for keyboard input
        if PYGAME_AVAILABLE:
            pygame.init()
            screen = pygame.display.set_mode((400, 200))
            pygame.display.set_caption('CARLA Waypoint Capture - Press E/T/P/S/R/Q')
            clock = pygame.time.Clock()
            
            print("\n✓ Pygame initialized - Use keyboard controls")
            print("  (Keep the pygame window in focus)")
            
            running = True
            while running:
                clock.tick(30)
                
                for event in pygame.event.get():
                    if event.type == QUIT:
                        running = False
                    
                    if event.type == KEYDOWN:
                        if event.key == K_q:
                            print("\nQuitting...")
                            running = False
                        
                        elif event.key == K_e:
                            # Capture ego position
                            pos_data, loc = capture.capture_spectator_position(spectator)
                            capture.ego_position = pos_data
                            print(f"\n✓ Captured EGO position at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                            capture.print_summary()
                        
                        elif event.key == K_t:
                            # Capture truck position
                            pos_data, loc = capture.capture_spectator_position(spectator)
                            capture.trucks.append(pos_data)
                            print(f"\n✓ Captured TRUCK #{len(capture.trucks)} at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                            capture.print_summary()
                        
                        elif event.key == K_p:
                            # Capture pedestrian position
                            pos_data, loc = capture.capture_spectator_position(spectator)
                            capture.pedestrians.append(pos_data)
                            print(f"\n✓ Captured PEDESTRIAN #{len(capture.pedestrians)} at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                            capture.print_summary()
                        
                        elif event.key == K_s:
                            # Save to JSON
                            if capture.ego_position or capture.trucks or capture.pedestrians:
                                capture.save_to_json('vehicles.json')
                                print("✓ File saved successfully!")
                            else:
                                print("\n⚠ No waypoints captured yet!")
                        
                        elif event.key == K_r:
                            # Reset
                            capture.ego_position = None
                            capture.trucks = []
                            capture.pedestrians = []
                            print("\n✓ All captures reset!")
                            capture.print_summary()
            
            pygame.quit()
            
        else:
            # Fallback: Manual input mode
            print("\n⚠ Pygame not available - Using manual input mode")
            print("Manual mode instructions:")
            print("  1. Move spectator in CARLA to desired position")
            print("  2. Come back to terminal and press Enter")
            print("  3. Choose option to capture")
            
            while True:
                input("\nPress Enter when spectator is in position (or Ctrl+C to quit)...")
                
                print("\nCapture as:")
                print("  1 - Ego vehicle")
                print("  2 - Truck")
                print("  3 - Pedestrian")
                print("  s - Save to vehicles.json")
                print("  r - Reset")
                print("  q - Quit")
                
                choice = input("Choice: ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    pos_data, loc = capture.capture_spectator_position(spectator)
                    capture.ego_position = pos_data
                    print(f"✓ Captured EGO at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                elif choice == '2':
                    pos_data, loc = capture.capture_spectator_position(spectator)
                    capture.trucks.append(pos_data)
                    print(f"✓ Captured TRUCK #{len(capture.trucks)} at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                elif choice == '3':
                    pos_data, loc = capture.capture_spectator_position(spectator)
                    capture.pedestrians.append(pos_data)
                    print(f"✓ Captured PEDESTRIAN #{len(capture.pedestrians)} at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                elif choice == 's':
                    if capture.ego_position or capture.trucks or capture.pedestrians:
                        capture.save_to_json('vehicles.json')
                    else:
                        print("⚠ No waypoints captured yet!")
                elif choice == 'r':
                    capture.ego_position = None
                    capture.trucks = []
                    capture.pedestrians = []
                    print("✓ Reset!")
                
                capture.print_summary()
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Save before exit if there's data
        if capture.ego_position or capture.trucks or capture.pedestrians:
            save = input("\nSave captured waypoints before exit? (y/n): ").strip().lower()
            if save == 'y':
                capture.save_to_json('vehicles.json')
        
        print("Done!")

if __name__ == '__main__':
    main()
