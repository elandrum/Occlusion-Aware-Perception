import json
import os
import sys

# Repo root = parent of occl_simulator
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "test_data")


class ScenarioConfig:
    def __init__(self, scenario_name, vehicle_json=None, pedestrian_json=None):
        """
        scenario_name: short key like 'scenario1', 'truck_overtake', etc.

        Defaults:
          test_data/<scenario_name>_vehicles.json
          test_data/<scenario_name>_peds.json
        """
        self.scenario_name = scenario_name

        if vehicle_json is None:
            self.vehicle_json = os.path.join(
                DATA_DIR, f"{scenario_name}_vehicles.json"
            )
        else:
            self.vehicle_json = vehicle_json

        if pedestrian_json is None:
            self.pedestrian_json = os.path.join(
                DATA_DIR, f"{scenario_name}_peds.json"
            )
        else:
            self.pedestrian_json = pedestrian_json

        self.vehicle_data = None
        self.pedestrian_data = None

    def load_vehicle_config(self):
        if not os.path.exists(self.vehicle_json):
            print(f"Error: {self.vehicle_json} not found!")
            sys.exit(1)

        with open(self.vehicle_json, "r") as f:
            self.vehicle_data = json.load(f)

        print(f"Loaded vehicle configuration from {self.vehicle_json}")
        return self.vehicle_data

    def load_pedestrian_config(self):
        if not os.path.exists(self.pedestrian_json):
            print(
                f"Warning: {self.pedestrian_json} not found! No pedestrians will be spawned."
            )
            return None

        with open(self.pedestrian_json, "r") as f:
            self.pedestrian_data = json.load(f)

        print(f"Loaded pedestrian configuration from {self.pedestrian_json}")
        return self.pedestrian_data

    def get_vehicle_config(self):
        if self.vehicle_data is None:
            self.load_vehicle_config()
        return self.vehicle_data

    def get_pedestrian_config(self):
        if self.pedestrian_data is None:
            self.load_pedestrian_config()
        return self.pedestrian_data
