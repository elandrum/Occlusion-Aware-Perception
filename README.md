# Occlusion-Aware-Perception

A modular CARLA simulation framework designed to test and evaluate autonomous vehicle perception systems in challenging occlusion scenarios. This project creates realistic urban environments where pedestrians and obstacles are strategically positioned to occlude critical objects from the ego vehicle's field of view, enabling researchers to study and improve perception algorithms under adverse visibility conditions.


## Project Structure

```
513_Proj/
├── main.py                    # Main entry point - runs scenarios
├── scenario1.py               # Scenario 1: Pedestrian crossing collision
├── scenario_config.py         # Configuration loader
├── vehicle_controller.py      # Ego vehicle control logic
├── pedestrian_controller.py   # Pedestrian spawning and movement
├── scenery_manager.py         # Static actors (trucks, props)
├── camera_manager.py          # Camera sensor and video recording
├── capture_waypoints.py       # Tool to capture positions
├── vehicles.json             # Vehicle configuration
├── pedestrians.json          # Pedestrian configuration
├── requirements.txt          # Python dependencies
└── output/                   # Captured frames and videos
```

## Files Overview

### Core Modules

- **`main.py`**: Main entry point that loads and runs scenarios
- **`scenario1.py`**: First scenario - pedestrian crossing collision
- **`scenario_config.py`**: Loads and manages JSON configuration files
- **`vehicle_controller.py`**: Controls ego vehicle behavior and logic
- **`pedestrian_controller.py`**: Handles pedestrian spawning and movement
- **`scenery_manager.py`**: Spawns and manages static scenery (trucks, vehicles)
- **`camera_manager.py`**: Manages front camera sensor, captures frames, compiles video

### Configuration Files

- **`vehicles.json`**: Defines ego vehicle, trucks positions, rotations, and speeds
- **`pedestrians.json`**: Defines pedestrian positions, targets, and speeds

### Tools

- **`capture_waypoints.py`**: Interactive tool to capture positions in CARLA

## Usage

### Running Scenarios

```bash
# Run scenario 1
python main.py scenario1

# General format
python main.py <scenario_name>
```

### Running Scenario Directly (Setup Only)

```bash
# Run scenario file directly - spawns actors but no movement logic
python scenario1.py
```

**Note:** Running a scenario file directly only sets up the scene (spawns all actors) without executing movement logic. Useful for inspecting actor placement and verifying positions. For full simulation with movement, use `python main.py scenario1`.

### Capturing New Positions

```bash
python capture_waypoints.py
```

Press keys to capture positions:
- **E** - Capture ego vehicle position
- **T** - Capture truck position
- **P** - Capture pedestrian position
- **S** - Save to JSON files


## Example Scenarios

### Scenario 1: Pedestrian Crossing Collision
- Ego vehicle drives at 30 km/h
- Pedestrian crosses street (not at crosswalk)
- Tests collision detection
- Located in: `scenario1.py`

## Configuration

### Vehicle Configuration (`vehicles.json`)

```json
{
  "ego_vehicle": {
    "location": {"x": -25.98, "y": 2.21, "z": 0.98},
    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
    "speed_kmh": 30
  },
  "trucks": [
    {
      "type": "vehicle.carlamotors.firetruck",
      "location": {"x": 2.32, "y": 6.5, "z": 2.05},
      "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
    }
  ]
}
```

### Pedestrian Configuration (`pedestrians.json`)

```json
{
  "pedestrians": [
    {
      "location": {"x": -3.8, "y": 10.54, "z": 1.03},
      "rotation": {"pitch": 0.0, "yaw": 270.0, "roll": 0.0},
      "target_location": {"x": -3.8, "y": -5.0, "z": 1.03},
      "speed_ms": 1.4
    }
  ]
}
```


## Video Recording

The camera automatically captures frames from the ego vehicle's front camera during simulation. When you stop the simulation (Ctrl+C), the frames are compiled into a video.

**Output:**
- Frames: `output/frames_<timestamp>/frame_XXXXXX.png`
- Video: `output/video_<timestamp>.mp4`
- Resolution: 1920x1080 @ 30fps

**Camera Position:**
- Mounted on front of ego vehicle
- Height: 1.0m above vehicle center
- Distance: 2.5m forward from vehicle center
- Field of View: 110 degrees

## Requirements

### Prerequisites

- **CARLA Simulator** (version 0.9.15 or compatible)
  - Download from [CARLA Releases](https://github.com/carla-simulator/carla/releases)
  - Make sure CARLA server is running before executing scenarios

#### Running CARLA with Docker

For GPU-enabled systems with NVIDIA Docker support:

```bash
# Pull CARLA Docker image
docker pull carlasim/carla:0.9.15

# Run CARLA server
docker run \
    --runtime=nvidia \
    --net=host \
    --user=$(id -u):$(id -g) \
    --env=DISPLAY=$DISPLAY \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    carlasim/carla:0.9.15 \
    bash -c "/home/carla/CarlaUE4.sh -nosound"
```

**Note:** Make sure your system has NVIDIA Docker runtime installed and X11 display is properly configured.

### Python Dependencies

- Python 3.9+

Install required packages:

```bash
pip install -r requirements.txt
```

**Core packages:**
- `numpy>=1.21.0` - Numerical operations
- `opencv-python>=4.5.0` - Video compilation from frames

**Optional packages:**
- `pygame>=2.0.0` - For interactive waypoint capture tool

**CARLA Python API:**
- The `carla` package must be installed from your CARLA distribution
- Typically found in `CARLA_ROOT/PythonAPI/carla/dist/carla-x.x.x-py3.x-xxx.egg`
- Add to PYTHONPATH or install using:
  ```bash
  # Example for CARLA 0.9.15
  easy_install CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
  ```
