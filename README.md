# Occlusion-Aware-Perception

A modular CARLA simulation framework designed to test and evaluate autonomous vehicle perception and control in challenging occlusion scenarios. The current baseline scenario places a pedestrian crossing between two parked trucks, creating a realistic occluded-crosswalk situation for the ego vehicle.

---

## Project Structure

```markdown

Occlusion-Aware-Perception/
├── occl_controller/
│   ├── controller.py           # Ego vehicle control logic (ACC / occlusion-aware logic)
│   └── __init__.py
├── occl_simulator/
│   ├── __main__.py             # Main entry point - runs scenarios
│   ├── scenario1.py            # Scenario 1: Pedestrian crossing between two trucks
│   ├── config.py               # Scenario configuration loader
│   ├── actors_static.py        # Static actors (trucks, props)
│   ├── actors_peds.py          # Pedestrian spawning and movement
│   ├── camera.py               # Camera sensor and video recording
│   └── capture_waypoints.py    # Tool to capture positions in CARLA
├── test_data/
│   ├── scenario1_vehicles.json # Vehicle configuration for scenario 1
│   └── scenario1_peds.json     # Pedestrian configuration for scenario 1
├── tests/
│   └── __init__.py
├── requirements.txt
└── README.md
```
---

# HPC SETUP (DISCOVERY)

## 1. Python / Conda Setup

```
module purge
module load conda
conda init bash
conda config --set auto_activate_base false
source ~/.bashrc

conda activate base
conda create -n carla python=3.8 conda -c conda-forge # only if setting up from scratch
~/.conda/envs/carla/condabin/conda init bash

conda activate carla
python --version
which python
```
---

## 2. Acquire a GPU Node
```
salloc --time=2:00:00 --cpus-per-task=8 --mem=32GB --account=jdeshmuk_786 --partition=gpu --gres=gpu:1
nvidia-smi
```
---

## 3. Load Apptainer + Conda
```
module load conda
module load apptainer
source ~/.bash_profile
conda activate carla
```
---

## 4. Launch CARLA via Apptainer
```
singularity exec --nv /project/jdeshmuk_786/carla-0.9.15_4.11.sif bash
```
Inside container:
```
source ~/.bashrc
conda activate carla
pip install -U carla==0.9.15 # only if setting up from scratch
```
---

## 5. Verify CARLA Works
```
cd /home/carla/
bash ./CarlaUE4.sh -nosound -vulkan -RenderOffScreen &

cd PythonAPI/util
python3 test_connection.py
```
Expected:
```
CARLA 0.9.15 connected at 127.0.0.1:2000.
```
---

# Running the Occlusion Simulator

Inside same container after CARLA is running:
```
cd ~/Occlusion-Aware-Perception
PYTHONPATH=. python3 -m occl_simulator scenario1 naive   # baseline ACC-like controller
PYTHONPATH=. python3 -m occl_simulator scenario1 occl    # occlusion + pedestrian-aware controller
```
This:
- Connects to CARLA
- Loads Town05
- Spawns ego vehicle + trucks + pedestrian
- Runs default controller
- Records video

Stop with Ctrl+C.

---

# Configuration Files

## scenario1_vehicles.json

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

## scenario1_peds.json

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

---

# Camera / Video Output

Frames → output/frames_<timestamp>/  
Video → output/video_<timestamp>.mp4  
FOV ~110°, 1080p, mounted front of vehicle.

# Occlusion Grid & Occluders

The controller builds a 2D ego–centric occlusion grid every tick and writes it out as a separate video (output/grid_<timestamp>.mp4). Each cell in this grid is marked as either “free” or “occluded” based on simple ray–casting from the ego vehicle to that cell.

Which actors count as occluders?

By default, if a scenario does not specify anything, the controller treats all vehicles except the ego as occluders. Internally it walks world.get_actors().filter("vehicle.*"), skips the ego, and uses their CARLA bounding boxes to block rays.

A scenario can optionally override this by returning an occluders list in its scenario_data dict (for example, a list of specific truck actors). In that case, only the actors in this list are used as occluders.

Actors are only used as occluders if they have a bounding_box attribute (typically vehicles, trucks, large static props, etc.).

For most new scenarios you don’t need to wire anything special: just spawn your vehicles, and the controller will automatically treat them as occluders. Only if you want fine–grained control (e.g., “only these two trucks should occlude, parked cars should not”) do you need to pass an explicit occluders list from your scenario. In scenario1.py, that line is commented out. See immplementation for usage details.
---

## Pedestrians (Visualization + Behavior)

Pedestrians for a scenario are spawned and scripted by
`occl_simulator/actors_peds.py` based on `test_data/scenario*_peds.json`.

Each simulation tick, the controller:

1. **Projects pedestrians into the ego frame**  
   - Uses the ego’s forward and right vectors to express each pedestrian
     in ego-centric coordinates `(x_local, y_local)`, where:
       - `x_local` = right
       - `y_local` = forward
   - Filters to pedestrians that are:
       - In front of the ego (`y_local > 0`)
       - Within a configurable “lane width” band (≈ ±2.5 m)
       - Within the grid range (≈ 20 m × 20 m around the car)

2. **Renders pedestrians into the occlusion grid video**  
   - The occlusion grid is a 40×40 top-down grid centered on the ego:
       - Free cells → dark gray
       - Occluded cells → red
       - Ego → green dot at the center
       - Pedestrians → blue dots in the cells corresponding to their projected
         positions
   - The grid video is written to `output/grid_<timestamp>.mp4`.

3. **Uses pedestrians in the occlusion-aware controller (`occl` mode)**  
   - Derives simple hazard metrics from the ego-centric positions:
       - Whether any pedestrian is ahead in the lane
       - Whether any pedestrian is within a “near” zone (~< 18 m)
       - Whether any pedestrian is within a “stop” zone (~< 10 m)
   - The controller then:
       - Caps the target speed when a pedestrian is ahead (the closer the ped,
         the lower the cap; very close ⇒ creep speed).
       - Triggers hard braking when a pedestrian is in the stop zone
         directly in the lane.
       - Once the pedestrian leaves the hazard zone, the controller gradually
         returns to the occlusion-based target speed and the ego continues
         driving.

These thresholds (lane width / near distance / stop distance) are defined as
constants in `VehicleController._compute_pedestrian_hazards` and can be tuned
per-project.

# Adding New Scenarios

1. Add:
```
test_data/my_scenario_vehicles.json  
test_data/my_scenario_peds.json
```
2. Add file:
```
occl_simulator/my_scenario.py
```
3. Register inside occl_simulator/__main__.py

Run:
```
PYTHONPATH=. python3 -m occl_simulator my_scenario
```
---



# Requirements

- CARLA 0.9.15
- Python 3.8
- GPU node on HPC
- Apptainer container
- Python dependencies in requirements.txt
- CARLA egg importable: python3 -c "import carla"

---
