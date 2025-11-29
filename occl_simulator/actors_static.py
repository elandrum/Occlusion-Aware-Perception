"""
Scenery Module
Handles spawning and managing static/background actors (vehicles, trucks, props)
"""

import carla
import random


class SceneryManager:
    def __init__(self, world, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library

        # All actors we own (for cleanup)
        self.actors = []

        # Ego vehicle tracked separately
        self.ego_vehicle = None

        # Static occluders (trucks, parked cars, walls, etc.)
        self.static_actors = []

    # ----------------------------------------------------------------------
    # Ego vehicle
    # ----------------------------------------------------------------------
    def spawn_ego_vehicle(self, vehicle_config):
        """Spawn the ego vehicle from configuration"""
        print("\nSpawning ego vehicle...")
        ego_bp = self.blueprint_library.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("role_name", "ego")
        ego_bp.set_attribute("color", "0,0,255")  # Blue color

        ego_pos = vehicle_config["ego_vehicle"]
        ego_location = carla.Location(
            x=ego_pos["location"]["x"],
            y=ego_pos["location"]["y"],
            z=ego_pos["location"]["z"],
        )
        ego_rotation = carla.Rotation(
            pitch=ego_pos["rotation"]["pitch"],
            yaw=ego_pos["rotation"]["yaw"],
            roll=ego_pos["rotation"]["roll"],
        )
        ego_spawn_point = carla.Transform(ego_location, ego_rotation)

        ego_vehicle = self.world.spawn_actor(ego_bp, ego_spawn_point)
        ego_vehicle.set_simulate_physics(True)

        # Track ego + add to actors list for cleanup
        self.ego_vehicle = ego_vehicle
        self.actors.append(ego_vehicle)

        ego_speed_kmh = ego_pos.get("speed_kmh", 30)

        print(f"Ego vehicle spawned at {ego_location}")

        return ego_vehicle, ego_speed_kmh

    # ----------------------------------------------------------------------
    # Static trucks / vehicles (occluders)
    # ----------------------------------------------------------------------
    def spawn_trucks(self, vehicle_config):
        """Spawn trucks from configuration (stationary obstacles)"""
        if "trucks" not in vehicle_config:
            return

        print("\nSpawning trucks (scenery)...")

        for i, truck_pos in enumerate(vehicle_config["trucks"]):
            # Get truck type from config
            if "type" in truck_pos:
                truck_type = truck_pos["type"]
                try:
                    truck_bp = self.blueprint_library.find(truck_type)
                except Exception:
                    print(f"Warning: Truck type '{truck_type}' not found, using random truck")
                    truck_blueprints = [
                        bp
                        for bp in self.blueprint_library.filter("vehicle.*")
                        if "truck" in bp.id.lower() or "carlamotors" in bp.id.lower()
                    ]
                    truck_bp = random.choice(truck_blueprints)
            else:
                truck_blueprints = [
                    bp
                    for bp in self.blueprint_library.filter("vehicle.*")
                    if "truck" in bp.id.lower() or "carlamotors" in bp.id.lower()
                ]
                truck_bp = random.choice(truck_blueprints)

            truck_location = carla.Location(
                x=truck_pos["location"]["x"],
                y=truck_pos["location"]["y"],
                z=truck_pos["location"]["z"],
            )
            truck_rotation = carla.Rotation(
                pitch=truck_pos["rotation"]["pitch"],
                yaw=truck_pos["rotation"]["yaw"],
                roll=truck_pos["rotation"]["roll"],
            )
            spawn_point = carla.Transform(truck_location, truck_rotation)

            try:
                truck = self.world.spawn_actor(truck_bp, spawn_point)

                # Track as both "any actor" and specifically "static occluder"
                self.actors.append(truck)
                self.static_actors.append(truck)

                print(
                    f"Truck {i+1} spawned at "
                    f"({truck_location.x:.1f}, {truck_location.y:.1f}, {truck_location.z:.1f}): "
                    f"{truck_bp.id}"
                )
            except Exception as e:
                print(f"Failed to spawn truck {i+1}: {e}")

    def spawn_static_vehicles(self, config_list):
        """Spawn additional static vehicles for scenery (also treated as occluders)"""
        if not config_list:
            return

        print("\nSpawning static vehicles...")

        for i, vehicle_config in enumerate(config_list):
            vehicle_type = vehicle_config.get("type", "vehicle.audi.a2")

            try:
                vehicle_bp = self.blueprint_library.find(vehicle_type)
            except Exception:
                print(f"Warning: Vehicle type '{vehicle_type}' not found")
                continue

            location = carla.Location(
                x=vehicle_config["location"]["x"],
                y=vehicle_config["location"]["y"],
                z=vehicle_config["location"]["z"],
            )
            rotation = carla.Rotation(
                pitch=vehicle_config["rotation"]["pitch"],
                yaw=vehicle_config["rotation"]["yaw"],
                roll=vehicle_config["rotation"]["roll"],
            )
            spawn_point = carla.Transform(location, rotation)

            try:
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)

                # Static scenery + occluder
                self.actors.append(vehicle)
                self.static_actors.append(vehicle)

                print(f"Static vehicle {i+1} spawned: {vehicle_type}")
            except Exception as e:
                print(f"Failed to spawn static vehicle {i+1}: {e}")

    # ----------------------------------------------------------------------
    # API used by scenario / controller
    # ----------------------------------------------------------------------
    def get_actor_count(self):
        """Get count of spawned scenery actors (ego + static)"""
        return len(self.actors)

    def get_occluders(self):
        """Return list of actors to be treated as occluders for the grid/ray casting."""
        return list(self.static_actors)

    # ----------------------------------------------------------------------
    # Cleanup
    # ----------------------------------------------------------------------
    def destroy_all(self):
        """Destroy all spawned scenery actors"""
        print("Destroying scenery actors...")
        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception:
                # In case CARLA already cleaned something up
                pass
