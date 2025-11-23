"""
Scenario Configuration Module
Handles loading and managing scenario configurations from JSON files
"""

import json
import os
import sys


class ScenarioConfig:
    def __init__(self, vehicle_json='vehicles.json', pedestrian_json='pedestrians.json'):
        self.vehicle_json = vehicle_json
        self.pedestrian_json = pedestrian_json
        self.vehicle_data = None
        self.pedestrian_data = None
        
    def load_vehicle_config(self):
        """Load vehicle positions from JSON file"""
        if not os.path.exists(self.vehicle_json):
            print(f"Error: {self.vehicle_json} not found!")
            sys.exit(1)
        
        with open(self.vehicle_json, 'r') as f:
            self.vehicle_data = json.load(f)
        
        print(f"Loaded vehicle configuration from {self.vehicle_json}")
        return self.vehicle_data
    
    def load_pedestrian_config(self):
        """Load pedestrian positions from JSON file"""
        if not os.path.exists(self.pedestrian_json):
            print(f"Warning: {self.pedestrian_json} not found! No pedestrians will be spawned.")
            return None
        
        with open(self.pedestrian_json, 'r') as f:
            self.pedestrian_data = json.load(f)
        
        print(f"Loaded pedestrian configuration from {self.pedestrian_json}")
        return self.pedestrian_data
    
    def get_vehicle_config(self):
        """Get vehicle configuration (load if not already loaded)"""
        if self.vehicle_data is None:
            self.load_vehicle_config()
        return self.vehicle_data
    
    def get_pedestrian_config(self):
        """Get pedestrian configuration (load if not already loaded)"""
        if self.pedestrian_data is None:
            self.load_pedestrian_config()
        return self.pedestrian_data
