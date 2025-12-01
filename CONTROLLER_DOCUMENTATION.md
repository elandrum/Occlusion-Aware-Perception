# Occlusion-Aware Perception for Autonomous Vehicles

A modular CARLA simulation framework for testing and evaluating autonomous vehicle perception and control in challenging **occlusion scenarios**. This project implements both a baseline controller (no occlusion awareness) and an advanced **universal occlusion-aware controller** that automatically adapts to various hazardous situations.

---

## Table of Contents

1. [Overview](#overview)
2. [Scenarios](#scenarios)
3. [Controller Architecture](#controller-architecture)
4. [Occlusion Grid System](#occlusion-grid-system)
5. [Risk Assessment Model](#risk-assessment-model)
6. [Speed Control Algorithm](#speed-control-algorithm)
7. [Vision-Based Detection](#vision-based-detection)
8. [Project Structure](#project-structure)
9. [Installation & Setup](#installation--setup)
10. [Usage](#usage)
11. [Configuration](#configuration)
12. [HPC Setup](#hpc-setup-discovery-cluster)

---

## Overview

In real-world driving, **occlusions** (blocked lines of sight) are a leading cause of accidents. A pedestrian stepping out from behind a parked truck, or a vehicle running a red light from an occluded intersection—these scenarios challenge even the most advanced autonomous driving systems.

This framework provides:

- **Multiple occlusion scenarios** for testing different hazard types
- **Baseline controller** that ignores occlusions (for comparison)
- **Occlusion-aware controller** with automatic risk assessment
- **Vision-based detection** using YOLOv8 + depth cameras
- **Real-time visualization** of occlusion grids and detection overlays

---

## Scenarios

### Scenario 1: Static Truck Occlusion
A pedestrian crosses between two stationary trucks. The ego vehicle approaches from behind, with its view of the pedestrian blocked by the truck.

**Key Challenge**: Detecting a pedestrian that suddenly appears from behind a large occluder.

### Scenario 2: Moving Vehicle Occlusion
Adjacent lane vehicles create dynamic occlusion as they move alongside the ego vehicle. A pedestrian crosses when occluded by the moving traffic.

**Key Challenge**: Tracking dynamic occlusions and detecting social cues (adjacent vehicle braking).

### Scenario 3: Parked Vehicle Gauntlet
Multiple parked vehicles line both sides of the road. Pedestrians cross at various points between parked cars.

**Key Challenge**: Managing multiple simultaneous occlusion sources with uncertain pedestrian emergence points.

### Scenario 4: Left Turn with Red-Light Runner
The ego vehicle makes a left turn at an intersection while a vehicle runs the red light from the occluded perpendicular road.

**Key Challenge**: Detecting oncoming traffic during a turn maneuver with limited visibility.

---

## Controller Architecture

### Baseline Controller (`VehicleController`)

The baseline controller operates without occlusion awareness:

```
┌─────────────────────────────────────────┐
│           Baseline Controller            │
├─────────────────────────────────────────┤
│  • Constant target speed                 │
│  • No occlusion analysis                 │
│  • YOLO detection (visualization only)   │
│  • No risk-based speed modulation        │
└─────────────────────────────────────────┘
```

**Purpose**: Provides a comparison baseline to demonstrate the value of occlusion awareness.

### Occlusion-Aware Controller (`OcclusionAwareController`)

The universal occlusion-aware controller automatically handles all scenarios:

```
┌─────────────────────────────────────────────────────────────┐
│              Occlusion-Aware Controller                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐    │
│  │  Occlusion  │   │   Social    │   │ Vision-Based    │    │
│  │    Grid     │   │    Cues     │   │   Detection     │    │
│  │  Analysis   │   │  Monitoring │   │  (YOLO+Depth)   │    │
│  └──────┬──────┘   └──────┬──────┘   └────────┬────────┘    │
│         │                 │                    │             │
│         └─────────────────┼────────────────────┘             │
│                           │                                  │
│                    ┌──────▼──────┐                           │
│                    │    Risk     │                           │
│                    │   Fusion    │                           │
│                    └──────┬──────┘                           │
│                           │                                  │
│                    ┌──────▼──────┐                           │
│                    │   Speed     │                           │
│                    │  Control    │                           │
│                    └─────────────┘                           │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**Key Features:**
1. **360° occlusion analysis** - Forward, sides, and peripheral monitoring
2. **Time-to-collision (TTC) estimation** - For occluded regions
3. **Adjacent vehicle behavior monitoring** - Social cues from nearby traffic
4. **Vision-based pedestrian/vehicle detection** - YOLO + Depth Camera
5. **Probabilistic risk model** - Combining multiple factors
6. **Smooth speed control** - With comfort constraints

---

## Occlusion Grid System

### How It Works

The controller builds a **360° bird's-eye-view occlusion grid** centered on the ego vehicle:

```
          Forward (+Y)
              ▲
              │
     ┌────────┼────────┐
     │   F_L  │  F_R   │   F = Forward region
     │ ░░░░░░░│        │   F_L = Forward-Left
     ├───────[E]───────┤   F_R = Forward-Right
     │   S_L  │  S_R   │   S_L = Side-Left
     │        │        │   S_R = Side-Right
     └────────┴────────┘   [E] = Ego vehicle
              │            ░░░ = Occluded area
         (ignored rear)
```

### Grid Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `grid_world_size` | 30.0 m | Total coverage area (30m × 30m) |
| `grid_cells` | 60 × 60 | Resolution (0.5m per cell) |
| `max_range` | 15.0 m | Maximum detection range from ego |
| `ray_step` | 0.5 m | Ray-casting sample interval |

### Ray-Casting Algorithm

For each grid cell, the system casts a ray from the ego vehicle:

```python
# Pseudo-code for occlusion detection
for each cell (x, y) in grid:
    ray = line_from(ego_position, cell_center)
    
    for occluder in all_vehicles:
        if ray_intersects(ray, occluder.bounding_box):
            grid[x, y] = OCCLUDED  # Mark as occluded (RED)
            break
    else:
        grid[x, y] = VISIBLE  # No obstruction (DARK GRAY)
```

### Region of Interest (ROI) Definitions

The grid is divided into **5 regions** with different risk weights:

| Region | Angle Range | Risk Weight | Purpose |
|--------|-------------|-------------|---------|
| `forward` | -30° to +30° | **1.0** | Direct path ahead (highest priority) |
| `forward_left` | -70° to -30° | 0.8 | Left turn path |
| `forward_right` | +30° to +70° | 0.8 | Right turn path |
| `side_left` | -110° to -70° | 0.5 | Adjacent lane left |
| `side_right` | +70° to +110° | 0.5 | Adjacent lane right |

### Grid Visualization

The occlusion grid video (`output/grid_*.mp4`) shows:
- **Red cells**: Occluded areas (blocked by vehicles)
- **Dark gray cells**: Visible/clear areas
- **Green dot**: Ego vehicle position (center)
- **Blue squares**: Detected pedestrians
- **Cyan boxes**: Detected vehicles
- **Risk level**: Displayed as text overlay

---

## Risk Assessment Model

### Multi-Factor Risk Fusion

The controller fuses multiple independent risk sources:

```python
Combined_Risk = max(
    Occlusion_Risk,
    Social_Cue_Risk × 0.6,
    Pedestrian_Risk,
    Vehicle_Risk
)
```

### 1. Occlusion Risk Calculation

For each region, the occlusion risk considers both coverage and proximity:

```python
# Distance-weighted importance (closer = more important)
importance[cell] = 1.0 - (distance[cell] / max_range)

# Weighted occlusion ratio
weighted_occlusion = sum(importance[occluded_cells]) / sum(importance[all_cells])

# Distance-based risk (closer occlusion = higher danger)
if min_occluded_distance < 10m:
    distance_risk = 1.0 - (min_distance / 10.0)
else:
    distance_risk = 0.0

# Combined region risk
region_risk = 0.6 × weighted_occlusion + 0.4 × distance_risk

# Total occlusion risk (weighted by region importance)
occlusion_risk = Σ(region_risk × region_weight) / Σ(region_weight)
```

### 2. Social Cue Risk

The controller monitors adjacent vehicles for behavioral cues that indicate hidden hazards:

| Behavior | Detection Method | Risk Contribution |
|----------|------------------|-------------------|
| **Hard braking** | Deceleration > 3.0 m/s² | +0.4 |
| **Stopped ahead** | Speed < 1.0 m/s, distance < 15m | +0.3 |
| **Fast approach** | Closing speed > 10 m/s | +0.2 |

```python
# Social cue risk is weighted at 60%
social_risk = detected_behaviors × social_cue_weight  # weight = 0.6
```

**Why this matters**: If a truck in the adjacent lane suddenly brakes hard, there's likely a hazard (e.g., pedestrian) that the ego vehicle cannot see yet.

### 3. Pedestrian Risk (Vision-Based)

YOLOv8 + Depth camera detects visible pedestrians:

```python
if pedestrian_in_path AND distance < 15m:
    ped_risk = 1.0  # EMERGENCY STOP
    
elif distance < 25m:
    # Proportional risk based on proximity
    proximity = 1.0 - (distance - 15) / (25 - 15)
    ped_risk = proximity × 0.8
    
else:
    ped_risk = 0.0
```

| Distance | Action |
|----------|--------|
| < 15m (`stop_distance`) | **Emergency stop** |
| 15-25m (`caution_distance`) | Proportional slowdown |
| ≥ 25m | Normal operation |

### 4. Vehicle Risk (Oncoming Traffic)

Enabled only for left-turn scenarios (Scenario 4):

| Distance | Action |
|----------|--------|
| < 20m | Emergency stop |
| 20-35m | Proportional slowdown |

### Risk Thresholds & Response

| Risk Level | Threshold | Controller Response |
|------------|-----------|---------------------|
| **Normal** | < 0.3 | Full target speed |
| **Caution** | 0.3 - 0.6 | Gradual speed reduction |
| **Danger** | 0.6 - 0.85 | Significant braking |
| **Emergency** | ≥ 0.85 | Full emergency stop |

### Temporal Smoothing

To prevent jerky behavior from sensor noise:

```python
risk_history.append(current_risk)
if len(risk_history) > 20:  # ~1 second at 20Hz
    risk_history.pop(0)

# Use MAXIMUM of recent risks for safety-first approach
smoothed_risk = max(risk_history)
```

---

## Speed Control Algorithm

### Target Speed Calculation

The safe target speed is calculated using physics-based stopping distance:

```python
# Risk-adjusted deceleration rate
# Higher risk → assume harder braking will be needed
a_adjusted = a_comfort + (a_max - a_comfort) × risk_level

# where:
#   a_comfort = 2.5 m/s²  (comfortable braking)
#   a_max = 6.0 m/s²      (emergency braking)

# Safe speed from stopping distance formula: v = √(2ad)
effective_distance = max(0, min_forward_distance - 3.0)  # 3m safety margin
v_safe = √(2 × a_adjusted × effective_distance)

# Risk-based speed reduction factor
# At max risk (1.0) → reduce to 30% of target speed
risk_factor = 1.0 - (risk_level × 0.7)

# Final target speed
v_target = min(v_safe, target_speed × risk_factor)

# Clamp to valid range
final_speed = clamp(v_target, min=1.5 m/s, max=target_speed)
```

### Speed Control Flow Diagram

```
                    ┌───────────────┐
                    │  Risk Level   │
                    └───────┬───────┘
                            │
            ┌───────────────┼───────────────┐
            │               │               │
            ▼               ▼               ▼
     ┌──────────┐    ┌──────────┐    ┌──────────┐
     │ Emergency│    │  Danger  │    │  Normal  │
     │ risk≥0.85│    │ 0.3-0.85 │    │ risk<0.3 │
     └────┬─────┘    └────┬─────┘    └────┬─────┘
          │               │               │
          ▼               ▼               ▼
    ┌───────────┐  ┌────────────┐  ┌────────────┐
    │ FULL STOP │  │ Proportional│  │ Full Speed │
    │ brake=1.0 │  │  Slowdown   │  │    or      │
    │throttle=0 │  │             │  │ maintain   │
    └───────────┘  └────────────┘  └────────────┘
```

### PID-Like Smooth Control

The controller uses proportional control for smooth speed transitions:

```python
speed_error = target_speed - current_speed

if speed_error > 0.5:          # Need to speed up
    throttle = min(0.8, 0.3 + speed_error × 0.2)
    brake = 0.0
    
elif speed_error < -0.5:       # Need to slow down
    throttle = 0.0
    brake = min(0.9, |speed_error| × 0.3)
    
else:                          # Maintain speed (within ±0.5 m/s)
    throttle = 0.2
    brake = 0.0
```

---

## Vision-Based Detection

### YOLOv8 + Depth Camera Pipeline

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  RGB Camera │    │Depth Camera │    │   YOLOv8    │
│   (960×540) │    │  (960×540)  │    │   Model     │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                  │
       ▼                  │                  │
┌──────────────┐          │                  │
│ 2D Detection │◄─────────┼──────────────────┘
│  Bounding    │          │
│    Boxes     │          │
└──────┬───────┘          │
       │                  │
       ▼                  ▼
┌────────────────────────────┐
│   3D Position Estimation   │
│   (Pinhole Camera Model)   │
│                            │
│   z = depth(cx, cy)        │
│   x = (cx - px) × z / fx   │
│   y = (cy - py) × z / fy   │
└──────────────┬─────────────┘
               │
               ▼
┌────────────────────────────┐
│   Path Collision Check     │
│   • Lane width: 3.5m       │
│   • Check if in ego path   │
└────────────────────────────┘
```

### Detected Object Classes

**Pedestrians** (COCO class 0):
- Model: YOLOv8n (fastest)
- Confidence threshold: 50%
- Drawn as: Blue bounding boxes in video, blue squares in grid

**Vehicles** (COCO classes 2, 3, 5, 7):
- Cars (2), Motorcycles (3), Buses (5), Trucks (7)
- Confidence threshold: 50%
- Drawn as: Cyan/Red bounding boxes depending on threat level

### 3D Position Estimation

Using the pinhole camera model:

```python
# Camera intrinsics calculated from CARLA settings
fov_rad = radians(110)  # 110° horizontal FOV
fx = image_width / (2 × tan(fov_rad / 2))
fy = fx  # Square pixels assumed
cx, cy = image_width / 2, image_height / 2  # Principal point

# Back-project 2D detection to 3D
detection_center = (bbox_x1 + bbox_x2) / 2, (bbox_y1 + bbox_y2) / 2
z = depth_frame[detection_center]  # Forward distance (meters)
x = (det_cx - cx) × z / fx         # Lateral offset (+ = right)
y = (det_cy - cy) × z / fy         # Vertical offset (+ = down)
```

### Path Collision Detection

```python
# Check if detected object is in ego's driving path
lane_width = 3.5  # meters
half_lane = lane_width / 2

for detection in detections_3d:
    if detection.z > 0:  # Object is ahead (not behind)
        if abs(detection.x) < half_lane:  # Within lane width
            detection.in_path = True
            
            if detection.z < stop_distance:
                EMERGENCY_STOP = True
```

---

## Project Structure

```
Occlusion-Aware-Perception/
├── occl_controller/
│   ├── controller.py           # VehicleController & OcclusionAwareController
│   ├── pedestrian_detector.py  # YOLO pedestrian detection + 3D estimation
│   ├── vehicle_detector.py     # YOLO vehicle detection + 3D estimation
│   └── __init__.py
├── occl_simulator/
│   ├── __main__.py             # Main entry point & scenario runners
│   ├── scenario1.py            # Static truck occlusion
│   ├── scenario2.py            # Moving vehicle occlusion
│   ├── scenario3.py            # Parked vehicle gauntlet
│   ├── scenario4.py            # Left turn with red-light runner
│   ├── turn_controller.py      # Modular ego turn handling
│   ├── config.py               # Scenario configuration loader
│   ├── actors_static.py        # Static actor spawning
│   ├── actors_moving.py        # Moving vehicle controller
│   ├── actors_peds.py          # Pedestrian spawning & movement
│   ├── camera.py               # Camera sensor & video recording
│   └── capture_waypoints.py    # Position capture utility
├── test_data/
│   ├── scenario*_vehicles.json # Vehicle spawn configurations
│   └── scenario*_peds.json     # Pedestrian spawn configurations
├── output/                     # Generated videos and frames
│   ├── video_*.mp4             # Camera feed with HUD
│   ├── grid_*.mp4              # Occlusion grid visualization
│   └── frames_*/               # Individual PNG frames
├── requirements.txt
└── README.md
```

---

## Installation & Setup

### Prerequisites

- Python 3.8+
- CARLA Simulator 0.9.15
- NVIDIA GPU (recommended for YOLO inference)

### Install Dependencies

```bash
pip install -r requirements.txt
```

**Required packages:**
- `carla==0.9.15`
- `numpy`
- `opencv-python`
- `ultralytics` (YOLOv8)

### Start CARLA Server

```bash
# Linux (with display)
./CarlaUE4.sh -nosound -vulkan

# Headless (HPC/server)
./CarlaUE4.sh -nosound -vulkan -RenderOffScreen
```

---

## Usage

### Run a Scenario

```bash
# Baseline controller (no occlusion awareness)
python -m occl_simulator scenario1 default
python -m occl_simulator scenario2 default
python -m occl_simulator scenario3 default
python -m occl_simulator scenario4 default

# Occlusion-aware controller
python -m occl_simulator scenario1 aware
python -m occl_simulator scenario2 aware
python -m occl_simulator scenario3 aware
python -m occl_simulator scenario4 aware
```

### Controller Comparison

| Controller | Command Flag | Behavior |
|------------|--------------|----------|
| `default` | `python -m occl_simulator scenario1` | Baseline - constant speed, no risk response |
| `aware` | `python -m occl_simulator scenario1 aware` | Occlusion-aware - dynamic speed based on risk |

### Output Files

After running, outputs are saved to `output/`:

| File | Description |
|------|-------------|
| `video_YYYYMMDD_HHMMSS.mp4` | Camera feed with HUD overlay & YOLO boxes |
| `grid_YYYYMMDD_HHMMSS.mp4` | Bird's-eye occlusion grid visualization |
| `frames_YYYYMMDD_HHMMSS/` | Individual PNG frames |

---

## Configuration

### Controller Parameters

Edit in `occl_controller/controller.py` (`OcclusionAwareController.__init__`):

```python
# Deceleration limits
self.a_comfort = 2.5      # Comfortable deceleration (m/s²)
self.a_max = 6.0          # Maximum emergency deceleration (m/s²)
self.min_speed_ms = 1.5   # Minimum creep speed (m/s)

# Risk thresholds
self.risk_caution = 0.3   # Start reducing speed
self.risk_danger = 0.6    # Significant braking
self.risk_emergency = 0.85 # Full emergency stop

# Pedestrian detection thresholds
self.ped_stop_distance = 15.0     # Emergency stop distance (meters)
self.ped_caution_distance = 25.0  # Start slowing distance (meters)

# Grid settings
self.grid_world_size = 30.0  # Coverage area (meters)
self.grid_cells = 60         # Grid resolution (60×60)

# Region risk weights
self.risk_weights = {
    'forward': 1.0,        # Straight ahead (highest)
    'forward_left': 0.8,   # Left turn path
    'forward_right': 0.8,  # Right turn path
    'side_left': 0.5,      # Adjacent left
    'side_right': 0.5,     # Adjacent right
}
```

### Scenario Configuration Files

**Vehicle Configuration** (`test_data/scenario*_vehicles.json`):
```json
{
  "ego": {
    "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.3, "yaw": 0.0},
    "speed_kmh": 30
  },
  "occluders": [
    {
      "id": "truck_1",
      "blueprint": "vehicle.carlamotors.firetruck",
      "spawn_point": {"x": 10.0, "y": 5.0, "z": 0.3, "yaw": 90.0}
    }
  ]
}
```

**Pedestrian Configuration** (`test_data/scenario*_peds.json`):
```json
{
  "pedestrians": [
    {
      "id": "ped_1",
      "spawn_point": {"x": 15.0, "y": -3.0, "z": 0.9},
      "target": {"x": 15.0, "y": 3.0, "z": 0.9},
      "speed": 1.4
    }
  ]
}
```

---

## HPC Setup (Discovery Cluster)

### 1. Environment Setup

```bash
module purge
module load conda
conda create -n carla python=3.8
conda activate carla
pip install carla==0.9.15 numpy opencv-python ultralytics
```

### 2. Acquire GPU Node

```bash
salloc --time=2:00:00 --cpus-per-task=8 --mem=32GB \
       --account=YOUR_ACCOUNT --partition=gpu --gres=gpu:1
nvidia-smi  # Verify GPU access
```

### 3. Launch CARLA via Apptainer

```bash
module load apptainer
singularity exec --nv /path/to/carla-0.9.15.sif bash

# Inside container
cd /home/carla/
./CarlaUE4.sh -nosound -vulkan -RenderOffScreen &
```

### 4. Run Simulation

```bash
# In a new terminal (same node)
conda activate carla
cd /path/to/Occlusion-Aware-Perception
python -m occl_simulator scenario1 aware
```

---

## Algorithm Summary

### Main Update Loop (20 Hz)

```
┌─────────────────────────────────────────────────────────────┐
│                    UPDATE LOOP (50ms)                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Compute Occlusion Grid (ray-casting)                     │
│         ↓                                                    │
│  2. Extract Risk Features (per-region analysis)              │
│         ↓                                                    │
│  3. Monitor Adjacent Vehicles (social cues)                  │
│         ↓                                                    │
│  4. Detect Pedestrians (YOLO + Depth → 3D positions)         │
│         ↓                                                    │
│  5. Detect Vehicles (YOLO + Depth → oncoming traffic)        │
│         ↓                                                    │
│  6. Fuse All Risks → Combined Risk Score                     │
│         ↓                                                    │
│  7. Apply Temporal Smoothing (20-frame history)              │
│         ↓                                                    │
│  8. Calculate Target Speed (physics-based)                   │
│         ↓                                                    │
│  9. Apply Vehicle Control (throttle/brake/steer)             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## License

This project is for research and educational purposes.

---

## Citation

If you use this framework in your research, please cite:

```bibtex
@software{occlusion_aware_perception,
  title = {Occlusion-Aware Perception for Autonomous Vehicles},
  year = {2025},
  url = {https://github.com/elandrum/Occlusion-Aware-Perception}
}
```
