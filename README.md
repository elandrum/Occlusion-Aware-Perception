# Occlusion-Aware-Perception

A modular CARLA simulation framework designed to test and evaluate autonomous vehicle perception and control in challenging occlusion scenarios. The current baseline scenario places a pedestrian crossing between two parked trucks, creating a realistic occluded-crosswalk situation for the ego vehicle.

---

## Project Structure

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

---

# HPC SETUP (DISCOVERY)

## 1. Python / Conda Setup

module purge
module load conda
conda init bash
conda config --set auto_activate_base false
source ~/.bashrc

conda activate base
conda create -n carla python=3.8 conda -c conda-forge
~/.conda/envs/carla/condabin/conda init bash

conda activate carla
python --version
which python

---

## 2. Acquire a GPU Node

salloc --time=2:00:00 --cpus-per-task=8 --mem=32GB --account=jdeshmuk_786 --partition=gpu --gres=gpu:1
nvidia-smi

---

## 3. Load Apptainer + Conda

module load conda
module load apptainer
source ~/.bash_profile
conda activate carla

---

## 4. Launch CARLA via Apptainer

singularity exec --nv /project/jdeshmuk_786/carla-0.9.15_4.11.sif bash

Inside container:

source ~/.bashrc
conda activate carla
pip install -U carla==0.9.15

---

## 5. Verify CARLA Works

cd /home/carla/
bash ./CarlaUE4.sh -nosound -vulkan -RenderOffScreen &

cd PythonAPI/util
python3 test_connection.py

Expected:
CARLA 0.9.15 connected at 127.0.0.1:2000.

---

# Running the Occlusion Simulator

Inside same container after CARLA is running:

cd ~/Occlusion-Aware-Perception
PYTHONPATH=. python3 -m occl_simulator scenario1

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

---

# Adding New Scenarios

1. Add:

test_data/my_scenario_vehicles.json  
test_data/my_scenario_peds.json

2. Add file:

occl_simulator/my_scenario.py

3. Register inside occl_simulator/__main__.py

Run:

PYTHONPATH=. python3 -m occl_simulator my_scenario

---

# Requirements

- CARLA 0.9.15
- Python 3.8
- GPU node on HPC
- Apptainer container
- Python dependencies in requirements.txt
- CARLA egg importable: python3 -c "import carla"

---
