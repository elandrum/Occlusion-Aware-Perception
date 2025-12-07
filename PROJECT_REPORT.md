# Occlusion-Aware Perception for Autonomous Vehicles: A Proactive Safety Approach

---

## Abstract

This project presents an occlusion-aware perception and control system for autonomous vehicles that fundamentally shifts the safety paradigm from reactive to proactive. Traditional autonomous driving systems respond only to detected hazards, creating dangerous blind spots where pedestrians or obstacles hidden behind parked vehicles, trucks, or other obstructions can emerge suddenly without adequate stopping distance. Our system addresses this critical limitation by treating occlusion—areas where the vehicle cannot see—as a first-class risk signal rather than simply unknown space. We implement a multi-layered architecture that combines (1) a real-time occlusion grid system using ray-casting to map visible versus hidden regions around the vehicle, (2) social cue monitoring that interprets nearby vehicle behavior such as sudden braking as indicators of unseen hazards, and (3) vision-based pedestrian and vehicle detection using YOLOv8 with depth estimation for 3D localization. These three perception modalities are fused using a maximum-risk strategy that ensures the most conservative appropriate response, which then drives a physics-based speed controller that calculates safe velocities based on stopping distance requirements. We validate the system in CARLA simulator across three challenging scenarios involving hidden pedestrians emerging from behind static trucks, moving vehicle occlusions with social cue opportunities, and extended corridors of parked vehicles. Comparative analysis against a baseline controller without occlusion awareness demonstrates that our proactive approach consistently maintains safe stopping distances while the baseline frequently encounters near-collision situations. The system represents a step toward human-like cautious driving behavior, where uncertainty about what lies ahead naturally leads to reduced speed and increased vigilance.

---

## 1. Introduction

### 1.1 Motivation

Every year, thousands of pedestrian fatalities occur in urban environments, with a significant proportion involving scenarios where the pedestrian was not visible to the driver until moments before impact. Parked vehicles, delivery trucks, buses at stops, and street furniture create visual barriers that obscure pedestrians until they step into the roadway. Human drivers navigate these situations by exercising caution—slowing down when approaching areas of limited visibility, watching for social cues from other drivers, and anticipating that someone might emerge from a blind spot. This intuitive human behavior of "slowing down when you can't see well" is notably absent from most autonomous vehicle systems.

Current autonomous vehicle perception systems have achieved remarkable accuracy in detecting visible objects. LiDAR sensors can map the environment with centimeter-level precision, cameras combined with deep neural networks can recognize pedestrians, vehicles, cyclists, and countless other object categories, and sensor fusion algorithms combine these modalities into comprehensive environmental models. However, these sophisticated systems share a fundamental limitation: they can only perceive what is directly visible. A state-of-the-art perception stack will accurately detect a pedestrian standing on the sidewalk but provides no information—and crucially, no warning—about a pedestrian hidden behind a parked truck who might step into the street at any moment.

This asymmetry between perception capability and safety requirement creates a dangerous gap. Autonomous vehicles equipped with only reactive safety systems—those that respond to detected hazards—may perform flawlessly in open environments with good visibility but face near-impossible stopping challenges when hazards emerge from occlusion at close range. The physics are unforgiving: at urban speeds, stopping distances often exceed the distance at which a pedestrian emerging from behind a vehicle becomes visible.

Our motivation is to bridge this gap by developing a perception and control system that explicitly reasons about occlusion. We aim to create an autonomous vehicle controller that behaves like a cautious human driver: one that recognizes when visibility is limited, adjusts speed accordingly, monitors the behavior of other road users for indirect signals of danger, and maintains safety margins that account for the possibility of hidden hazards. The core insight is simple yet powerful: **when you cannot see clearly, slow down.**

### 1.2 Problem Statement

The central problem this project addresses is the **hidden hazard problem** in autonomous vehicle perception and control. Specifically:

**Primary Problem**: How can an autonomous vehicle maintain safe operation when potential hazards (pedestrians, cyclists, obstacles) are occluded by other objects (parked vehicles, trucks, buildings) and may emerge suddenly into the vehicle's path?

This problem has several dimensions:

1. **Occlusion Detection**: The system must identify which regions around the vehicle are occluded (not visible) versus clear (visible). This requires analyzing the 3D environment and determining line-of-sight relationships between the ego vehicle and surrounding space.

2. **Risk Quantification**: Not all occlusions are equally dangerous. An occluded region directly ahead at close range poses far greater risk than one to the side or at distance. The system must quantify the risk associated with each occluded region based on its spatial relationship to the vehicle's trajectory.

3. **Indirect Hazard Sensing**: Beyond direct perception of occlusion, the system should extract information from indirect sources. The behavior of other vehicles—particularly sudden braking or stopping—often indicates hazards that the ego vehicle cannot yet see.

4. **Speed Modulation**: Given an assessed risk level, the system must determine an appropriate vehicle speed that balances safety (ability to stop if a hazard emerges) against practicality (making reasonable progress toward the destination).

5. **Graceful Degradation**: The system must handle varying levels of occlusion gracefully, from single occluding objects to extensive corridors of parked vehicles, without either over-reacting (crawling everywhere) or under-reacting (ignoring significant occlusion).

**Constraints**:
- The system must operate in real-time (≥20 Hz control loop)
- Speed adjustments must be smooth enough for passenger comfort
- The system must work with standard vehicle sensor configurations (cameras, potentially LiDAR)
- The approach must be validated in simulation before any real-world deployment

**Success Criteria**:
- Maintain safe stopping distance to occluded regions at all times
- Reduce speed proactively before hazards emerge, not reactively after detection
- Respond appropriately to social cues from nearby vehicles
- Demonstrate measurably improved safety margins compared to occlusion-unaware baseline

---

## 2. Background/Preliminaries and Related Work

### 2.1 Technical Background

#### 2.1.1 Occlusion in Perception Systems

Occlusion occurs when one object blocks the line of sight between an observer and another object or region of space. In the context of autonomous vehicles, occlusion is ubiquitous in urban environments: parked vehicles line streets, delivery trucks stop in lanes, pedestrians walk behind obstacles, and intersecting streets create visibility limitations.

From a geometric perspective, occlusion can be understood through ray-casting. If we trace a ray from the ego vehicle's sensor (camera, LiDAR) toward a point in space, the point is visible if the ray reaches it unobstructed, and occluded if the ray intersects an obstacle before reaching the point. The shadow region behind any obstacle—where rays are blocked—constitutes the occluded space.

Occlusion creates several challenges for perception:
- **Missing Data**: Sensors cannot provide information about occluded regions, creating gaps in environmental understanding
- **Sudden Appearance**: Objects in occluded regions can "suddenly appear" from the perception system's perspective when they move into visible space
- **Phantom Objects**: Previous detections may become occluded, requiring the system to maintain estimates of where objects might be even when not currently visible

#### 2.1.2 Stopping Distance Physics

The physics of vehicle stopping is fundamental to understanding safety requirements. When a vehicle traveling at velocity v applies braking at deceleration a, the stopping distance d is given by:

$$d = \frac{v^2}{2a}$$

Rearranging, the maximum safe velocity for a given stopping distance is:

$$v_{safe} = \sqrt{2 \cdot a \cdot d}$$

For typical urban driving scenarios:
- At 30 km/h (8.3 m/s) with comfortable braking (2.5 m/s²), stopping distance is approximately 14 meters
- At 50 km/h (13.9 m/s) with comfortable braking, stopping distance is approximately 39 meters
- With emergency braking (6 m/s²), these distances reduce by roughly 60%

The critical implication: if a pedestrian can emerge from behind a parked vehicle at 3 meters distance, a vehicle traveling at 30 km/h cannot stop in time even with emergency braking. This physics-based reality motivates the need for proactive speed reduction near occluded regions.

#### 2.1.3 Social Cues in Driving

Human drivers continuously extract information from the behavior of other road users. A car ahead that brakes suddenly causes following drivers to brake even before seeing the reason. A pedestrian at the curb looking at their phone suggests lower crossing probability than one actively looking at traffic. These social cues provide indirect information about the environment that supplements direct perception.

Research in social robotics and human-robot interaction has shown that robots that respond to social cues are perceived as more intelligent and safer by human observers. In the driving context, responding to social cues can provide earlier warning of hazards than waiting for direct perception.

Key social cues in driving include:
- **Sudden braking by nearby vehicles**: Often indicates an obstacle or pedestrian ahead
- **Stopped vehicles in unexpected locations**: May be yielding to pedestrians or waiting for obstacles
- **Vehicle position changes**: Swerving may indicate obstacle avoidance
- **Speed mismatches**: A slow vehicle in fast traffic may have visibility of something others don't

#### 2.1.4 Object Detection with Deep Learning

Modern object detection uses convolutional neural networks (CNNs) trained on large datasets to identify and localize objects in images. The YOLO (You Only Look Once) family of detectors has become particularly popular for real-time applications due to their speed and accuracy balance.

YOLOv8, the detector used in this project, processes an image in a single forward pass and outputs bounding boxes with class labels and confidence scores. Key characteristics:
- **Real-time performance**: Inference at 30+ FPS on modern GPUs
- **Multi-class detection**: Simultaneous detection of pedestrians, vehicles, cyclists, etc.
- **Bounding box output**: 2D pixel coordinates of detected objects
- **Confidence scores**: Probability estimate for each detection

To obtain 3D positions from 2D detections, depth information must be incorporated. Depth cameras (RGB-D), stereo vision, or LiDAR can provide per-pixel or per-point distance measurements that, combined with 2D detection, yield 3D object localization.

### 2.2 Prior Approaches

#### 2.2.1 Reactive Safety Systems

The most common approach to pedestrian safety in current autonomous vehicles is reactive: detect the hazard, then respond. Systems like Automatic Emergency Braking (AEB) monitor for pedestrians and vehicles in the path and trigger braking when a collision is imminent.

**Limitations**:
- No response until hazard is visible
- Reaction time + braking distance may exceed available distance
- Uncomfortable for passengers due to emergency braking
- Cannot handle occluded hazards by design

#### 2.2.2 Occupancy Grid Approaches

Some research has explored occupancy grids that distinguish between free space, occupied space, and unknown (potentially occluded) space. These approaches maintain probabilistic estimates of occupancy and can reason about uncertainty.

**Limitations**:
- Unknown space is often treated uniformly, without distinguishing high-risk from low-risk unknown regions
- Typically used for path planning rather than speed control
- Computational complexity for high-resolution grids

#### 2.2.3 Visibility-Aware Path Planning

Research in robotics has explored path planning algorithms that consider sensor visibility. These approaches generate paths that maximize visibility of the environment or avoid regions where visibility is limited.

**Limitations**:
- Focus on path selection rather than speed modulation
- May be computationally expensive for real-time use
- Limited consideration of dynamic occlusions from moving vehicles

#### 2.2.4 Predictive Models for Occluded Pedestrians

Some approaches attempt to predict pedestrian behavior even when occluded, using context about where pedestrians are likely to emerge (crosswalks, bus stops, school zones).

**Limitations**:
- Relies on prior knowledge of pedestrian-likely locations
- Does not address novel occlusion scenarios
- Prediction uncertainty is high for occluded objects

### 2.3 Gap in Existing Approaches

The existing literature reveals a gap: while reactive safety systems are well-developed and occupancy grids can identify unknown regions, few systems treat occlusion as a direct input to speed control. The insight that "occlusion itself is a risk signal" is underexploited.

Additionally, the use of social cues from nearby vehicle behavior to supplement direct perception is rarely integrated into autonomous driving systems, despite being a crucial component of human driving behavior.

Our approach addresses these gaps by:
1. Explicitly computing occlusion as a spatial risk map
2. Fusing occlusion risk with social cue risk and direct perception
3. Using the combined risk to drive physics-based speed control
4. Operating in real-time on standard vehicle hardware

---

## 3. Solution Overview

### 3.1 System Architecture

Our occlusion-aware perception and control system employs a three-layer architecture that separates perception, decision-making, and action:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PERCEPTION LAYER (Inputs)                     │
├─────────────────┬─────────────────┬─────────────────────────────┤
│                 │                 │                             │
│  ┌───────────┐  │  ┌───────────┐  │  ┌─────────────────────┐    │
│  │ Occlusion │  │  │  Social   │  │  │   Vision-Based      │    │
│  │   Grid    │  │  │   Cue     │  │  │   Detection         │    │
│  │  System   │  │  │ Monitoring│  │  │  (YOLO + Depth)     │    │
│  └─────┬─────┘  │  └─────┬─────┘  │  └──────────┬──────────┘    │
│        │        │        │        │             │               │
└────────┼────────┴────────┼────────┴─────────────┼───────────────┘
         │                 │                      │
         └─────────────────┼──────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                   DECISION LAYER (Processing)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│                    ┌────────────────────┐                        │
│                    │  Risk Assessment   │                        │
│                    │      Model         │                        │
│                    │                    │                        │
│                    │ • Max Fusion       │                        │
│                    │ • Temporal Memory  │                        │
│                    │ • Risk: 0.0 - 1.0  │                        │
│                    └─────────┬──────────┘                        │
│                              │                                   │
└──────────────────────────────┼───────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ACTION LAYER (Output)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│                  ┌─────────────────────┐                         │
│                  │  Physics-Based      │                         │
│                  │  Speed Control      │                         │
│                  │                     │                         │
│                  │ • v = √(2·a·d)      │                         │
│                  │ • PID Control       │                         │
│                  │ • Throttle/Brake    │                         │
│                  └─────────────────────┘                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### Perception Layer

The perception layer consists of three parallel modules that gather information about the environment:

**Occlusion Grid System**: Constructs a 30×30 meter grid centered on the vehicle, using ray-casting to determine which cells are visible and which are occluded by obstacles. The grid is divided into five regions (forward, forward-left, forward-right, side-left, side-right), each weighted by its relevance to vehicle safety.

**Social Cue Monitoring**: Tracks all vehicles within a 30-meter radius and monitors their behavior for signals that might indicate unseen hazards. Key behaviors include hard braking (deceleration > 3.0 m/s²), stopped vehicles ahead, and rapid approach.

**Vision-Based Detection**: Uses YOLOv8 neural network on RGB camera images to detect pedestrians and vehicles, combined with depth camera data to estimate 3D positions. Objects in the vehicle's path trigger appropriate risk responses.

#### Decision Layer

The decision layer processes perceptual inputs into a unified risk assessment:

**Risk Fusion**: Each perception module produces a risk score (0.0 to 1.0). These scores are combined using a maximum strategy—the highest individual risk becomes the overall risk. This conservative approach ensures that if any module detects danger, the vehicle responds appropriately.

**Temporal Memory**: The system maintains a 20-frame history of risk assessments and uses the maximum over this window. This provides temporal smoothing that prevents jerky responses to momentary sensor noise while ensuring that briefly-visible hazards are not immediately forgotten.

#### Action Layer

The action layer translates risk into vehicle control:

**Physics-Based Speed Control**: Using the formula v = √(2·a·d), the system calculates the maximum safe speed given the current risk level (which influences assumed deceleration capability) and distance to occluded regions. The target speed is the minimum of this physics-based limit and a risk-proportional speed reduction.

**PID Control**: A proportional-integral-derivative controller smoothly adjusts throttle and brake to achieve the target speed, ensuring comfortable speed transitions.

### 3.2 Assumptions and Constraints

#### Assumptions

1. **Flat Ground Plane**: The occlusion analysis assumes a flat ground plane for simplicity. Hills, ramps, and uneven terrain could affect occlusion geometry.

2. **Forward-Facing Primary Risk**: The system weights forward occlusion most heavily, assuming the vehicle is traveling forward. Reversing or complex maneuvers are not explicitly handled.

#### Constraints

1. **Real-Time Operation**: All processing must complete within the 50 ms control loop period (20 Hz).

2. **Computational Resources**: The system must operate on hardware typical of autonomous vehicle platforms (modern GPU for YOLO inference, CPU for grid calculations).

3. **Sensor Configuration**: The implementation assumes front-facing RGB and depth cameras. Adaptation to other sensor configurations (360° cameras, LiDAR-only) would require modifications.

4. **Simulation Environment**: Validation is performed in CARLA simulator. Real-world deployment would require additional considerations for sensor noise, weather, lighting, and calibration.

5. **Speed Range**: The system is designed for urban speeds (0-50 km/h). Highway speeds would require different parameters and potentially different approaches.

---

## 4. Methods/Algorithms

### 4.1 Occlusion Grid Construction

The occlusion grid algorithm constructs a spatial map of visibility around the ego vehicle:

**Algorithm: Ray-Cast Occlusion Grid**

```
Input: Ego vehicle position, list of obstacle bounding boxes
Output: 60×60 grid with occlusion states

1. Initialize grid centered on ego vehicle (30m × 30m, 0.5m cells)
2. For each cell (i, j) in grid:
   a. Calculate world position of cell center
   b. Calculate direction vector from ego to cell
   c. For each sample point along ray (every 0.5m):
      - Check if sample point is inside any obstacle bounding box
      - If inside obstacle: mark cell as OCCLUDED, break
   d. If no obstacles encountered: mark cell as VISIBLE
3. Return completed grid
```

**Computational Optimization**: Rather than casting rays to every cell independently, the algorithm can exploit coherence—nearby cells often have similar occlusion status. Hierarchical approaches or early termination when rays exit the grid boundary improve performance.

### 4.2 Region-Based Risk Calculation

The grid is partitioned into regions with different risk weights:

**Algorithm: Region Risk Assessment**

```
Input: Occlusion grid, region definitions, risk weights
Output: Per-region risk scores, overall occlusion risk

1. For each region R (forward, forward-left, etc.):
   a. Count occluded cells in region: N_occluded
   b. Count total cells in region: N_total
   c. Calculate coverage: coverage = N_occluded / N_total
   
   d. Find minimum distance to occluded cell: d_min
   e. Calculate proximity factor: proximity = 1 - (d_min / max_range)
   
   f. Combine: region_risk = weight_R × (0.6 × coverage + 0.4 × proximity)

2. Aggregate: overall_risk = weighted_average(region_risks)
3. Return overall_risk
```

**Innovation**: Traditional occupancy grids treat unknown space uniformly. Our region-based approach recognizes that occlusion directly ahead is more dangerous than occlusion to the side, and close occlusion is more dangerous than distant occlusion.

### 4.3 Social Cue Detection

The social cue algorithm monitors nearby vehicle behavior:

**Algorithm: Social Cue Risk Assessment**

```
Input: List of tracked vehicles with position/velocity history
Output: Social cue risk score

1. Initialize social_risk = 0
2. For each vehicle V within 30m:
   a. Calculate current deceleration from velocity history
   
   b. If deceleration > 3.0 m/s² (hard braking):
      social_risk = max(social_risk, 0.4)
      
   c. If V is ahead AND speed < 1.0 m/s AND distance < 15m:
      social_risk = max(social_risk, 0.3)
      
   d. If relative_approach_speed > 10 m/s:
      social_risk = max(social_risk, 0.2)

3. Return social_risk × 0.6  (weighted for uncertainty)
```

**Innovation**: Few autonomous driving systems explicitly use nearby vehicle behavior as a risk signal. Our approach codifies the human intuition that "when nearby cars brake hard, something is happening."

### 4.4 Risk Fusion Strategy

Multiple risk sources are combined using maximum selection:

**Algorithm: Risk Fusion**

```
Input: occlusion_risk, social_risk, pedestrian_risk, vehicle_risk
Output: Fused risk score

1. Collect all risk scores: risks = [occ, social, ped, veh]
2. fused_risk = max(risks)
3. Store in temporal memory buffer (last 20 frames)
4. smoothed_risk = max(temporal_memory_buffer)
5. Return smoothed_risk
```

**Rationale**: Maximum fusion is deliberately conservative. If any perception module indicates danger, the vehicle responds. This prevents situations where averaging might dilute a critical warning.

### 4.5 Physics-Based Speed Control

The speed controller calculates safe velocity based on stopping distance physics:

**Algorithm: Safe Speed Calculation**

```
Input: risk_level, distance_to_occlusion
Output: Target speed

1. Interpolate deceleration based on risk:
   a_comfortable = 2.5 m/s²
   a_emergency = 6.0 m/s²
   a_assumed = a_comfortable + risk × (a_emergency - a_comfortable)

2. Apply safety margin:
   effective_distance = distance_to_occlusion - 3.0m

3. Calculate stopping-distance-based limit:
   v_physics = √(2 × a_assumed × effective_distance)

4. Calculate risk-based speed factor:
   speed_factor = 1.0 - 0.7 × risk  (min 30% speed at max risk)

5. Apply to target speed:
   v_risk = base_target_speed × speed_factor

6. Select minimum:
   v_target = min(v_physics, v_risk)

7. Apply bounds:
   v_target = clamp(v_target, 1.5 m/s, max_speed)

8. Return v_target
```

**Innovation**: The interpolation between comfortable and emergency deceleration based on risk level is novel. It reflects the intuition that in high-risk situations, we should assume we might need to brake hard.

### 4.6 What's New in Our Approach

Our system introduces several innovations compared to prior work:

1. **Occlusion as First-Class Risk Signal**: Rather than treating occlusion as merely "unknown space," we quantify it as a direct contributor to risk that drives speed control.

2. **Region-Weighted Occlusion**: The five-region decomposition with different weights reflects the reality that forward occlusion is more dangerous than side occlusion.

3. **Social Cue Integration**: Explicit monitoring of nearby vehicle behavior for braking events and stopped vehicles, integrated into the risk model.

4. **Multi-Modal Risk Fusion**: Combining occlusion, social cues, and direct detection through maximum-risk fusion provides redundant safety coverage.

5. **Physics-Based Speed Adaptation**: The speed controller is grounded in stopping distance physics, not arbitrary speed reductions.

6. **Temporal Risk Memory**: The 20-frame maximum memory ensures that briefly-visible hazards are not immediately forgotten.

---

## 5. Implementation

### 5.1 Software Framework

#### 5.1.1 Simulation Environment

**CARLA Simulator (Version 0.9.15)**

CARLA is an open-source simulator for autonomous driving research built on Unreal Engine. It provides:

- High-fidelity 3D urban environments with realistic rendering
- Accurate vehicle dynamics and physics simulation
- Configurable sensors including RGB cameras, depth cameras, LiDAR, and semantic segmentation
- Traffic manager for controlling NPC vehicles and pedestrians
- Python API for scenario scripting and vehicle control
- Town05 map used for our scenarios (urban environment with parked vehicles)

The simulation runs at 20 Hz synchronous mode, ensuring deterministic behavior for repeatable experiments.

#### 5.1.2 Perception Stack

**YOLOv8 (Ultralytics)**

- Model: yolov8n.pt (nano version for real-time inference)
- Input: 960×540 RGB images
- Output: Bounding boxes, class labels, confidence scores
- Classes of interest: person, car, truck, bus, motorcycle
- Inference: ~15ms per frame on NVIDIA GPU

**Depth Processing**

- CARLA depth camera provides per-pixel distance in meters
- Depth lookup at detection bounding box centers
- Median filtering over 5×5 region for robustness
- Pinhole camera model for 3D projection

#### 5.1.3 Control Stack

**Vehicle Control Interface**

- CARLA vehicle control accepts throttle [0,1], brake [0,1], steer [-1,1]
- Proportional speed control for smooth transitions
- Maximum throttle: 0.8 (for controlled acceleration)
- Maximum brake: 0.9 (reserving margin for emergency)

**PID Controller**

- Proportional gain: 0.5
- No integral or derivative terms (proportional sufficient for speed control)
- Deadband of ±0.5 m/s around target speed

### 5.2 Integration Details

#### 5.2.1 Sensor Synchronization

A key implementation challenge was synchronizing multiple sensor streams:

**Problem**: RGB and depth cameras have separate callbacks that may arrive at slightly different times, while vehicle state updates come from a third source.

**Solution**: We implemented a synchronized sensor manager that buffers incoming sensor data and ensures temporal alignment before processing. The manager maintains separate storage for RGB images, depth images, and their respective timestamps. When either sensor callback fires, the manager stores the new data along with the current system time.

The synchronization logic operates as follows:

1. **Buffer incoming data**: Each sensor callback stores its image data and records a timestamp indicating when the data arrived.

2. **Check temporal alignment**: When the control loop requests sensor data, the manager compares the RGB and depth timestamps. If the timestamps differ by less than 100 milliseconds, the data is considered synchronized and returned as a matched pair.

3. **Reject stale data**: If the timestamps differ by more than 100 milliseconds, the manager returns null values, signaling to the control loop that it should skip this frame rather than process misaligned data.

4. **Continuous update**: Both sensor streams update independently, and the manager always provides the most recent synchronized pair available.

This approach ensures that the perception pipeline never processes an RGB image paired with depth data from a different moment in time, which could cause objects to appear at incorrect distances or positions.

#### 5.2.2 Coordinate Frame Transformations

Multiple coordinate frames required careful management:

**World Frame**: CARLA's global coordinate system (X-East, Y-North, Z-Up)

**Ego Vehicle Frame**: Origin at vehicle center, X-forward, Y-left, Z-up

**Camera Frame**: Origin at camera sensor, following camera conventions

**Grid Frame**: Origin at grid corner, indices [i,j] corresponding to spatial positions

We implemented transformation utilities to convert between these coordinate frames, which are essential for placing detected objects on the occlusion grid and interpreting grid-based risk in terms of real-world distances.

**World-to-Ego Transformation**

Converting from world coordinates to the ego vehicle frame requires two steps. First, we translate the world point by subtracting the ego vehicle's world position, yielding a vector from the vehicle to the point. Second, we rotate this vector by the negative of the vehicle's heading angle (yaw) to align the result with the ego frame where X points forward and Y points left. The rotation uses standard 2D rotation matrix operations with sine and cosine of the heading angle.

**Ego-to-Grid Transformation**

Converting from the ego frame to grid indices maps continuous spatial coordinates to discrete cell positions. Given the grid's total extent (30 meters in each direction from the vehicle, divided into 0.5-meter cells), we calculate the grid indices as follows:
- The row index corresponds to the Y-coordinate (lateral position), with the mapping inverted so that positive Y (left of vehicle) maps to lower row indices
- The column index corresponds to the X-coordinate (forward position), offset by half the grid extent so that the vehicle sits at the grid center

Both transformations use simple arithmetic operations and execute in constant time, adding negligible overhead to the perception loop.

#### 5.2.3 Real-Time Performance Optimization

Achieving real-time performance was a critical requirement for the system. The control loop must complete within 50 milliseconds (20 Hz) to maintain responsive vehicle control—any slower and the vehicle's reactions would lag dangerously behind environmental changes. Several careful optimizations were necessary across all system components to meet this timing budget.

**Occlusion Grid Calculation Optimization**

The occlusion grid presents the greatest computational challenge: naively, it requires casting rays to 3,600 grid cells and checking each ray against all obstacles in the environment. To achieve acceptable performance, we implemented several key optimizations:

- **Pre-computed ray directions**: Since the grid geometry relative to the ego vehicle never changes, we compute all 3,600 ray direction vectors once at system startup and reuse them every frame. This eliminates redundant trigonometric calculations that would otherwise dominate the computation.

- **Vectorized distance calculations**: Rather than using Python loops to iterate over cells, we leverage NumPy's vectorized operations to compute distances and angles for all cells simultaneously. This exploits CPU SIMD instructions and can process thousands of cells in the time a Python loop would handle dozens.

- **Early ray termination**: When a ray exits the grid boundary or reaches maximum range, we immediately stop processing that ray. This prevents wasted computation on rays that have already determined their cells' visibility status.

- **Obstacle bounding box caching**: At the start of each frame, we query all nearby vehicle positions once and cache their bounding boxes. Subsequent ray-obstacle intersection tests use this cache rather than repeatedly querying the simulation.

**YOLO Neural Network Inference Optimization**

Deep neural network inference can easily consume hundreds of milliseconds if not carefully managed. Our YOLO implementation achieves consistent 15ms inference through several design choices:

- **Model selection**: We use YOLOv8-nano (yolov8n.pt), the smallest and fastest variant in the YOLOv8 family. While larger models offer marginally better accuracy, the nano model provides sufficient detection quality for our pedestrian and vehicle detection needs while running 3-4x faster.

- **GPU acceleration**: All neural network inference runs on the GPU using CUDA. The model weights remain in GPU memory throughout execution, eliminating costly CPU-GPU data transfers for the model itself. Only the input images and output detections cross the GPU-CPU boundary.

- **Single-frame batching**: We process exactly one frame per inference call rather than attempting to batch multiple frames. In a real-time control system, we need results for the current frame immediately—batching would introduce latency without benefit.

- **Balanced input resolution**: The 960×540 input resolution represents a careful tradeoff. Higher resolution would improve detection of distant or small objects but increase inference time quadratically. Lower resolution would be faster but might miss critical pedestrian detections. Our chosen resolution reliably detects pedestrians at distances up to 30 meters while maintaining real-time performance.

**Risk Calculation Optimization**

The risk fusion and calculation module is relatively lightweight compared to perception, but we still applied optimizations to minimize its contribution to frame time:

- **Incremental updates**: Where possible, we update risk values incrementally rather than recomputing from scratch. For example, the temporal memory buffer uses a circular array that requires only one insertion and one maximum-finding operation per frame.

- **NumPy array operations**: All grid-based risk calculations use NumPy arrays, enabling efficient element-wise operations and reductions without Python interpreter overhead.

- **Loop elimination**: We systematically replaced Python for-loops over grid cells with NumPy broadcasting and vectorized operations. A single NumPy expression can compute the risk contribution of all 3,600 cells faster than a loop could process a few dozen.

**Timing Profile Analysis**

Through extensive profiling, we characterized the typical frame timing breakdown:

| Component | Time | Percentage |
|-----------|------|------------|
| Sensor acquisition | 5 ms | 15% |
| YOLO inference | 15 ms | 45% |
| Occlusion grid | 10 ms | 30% |
| Risk calculation | 2 ms | 6% |
| Control output | 1 ms | 3% |
| **Total** | **~33 ms** | **100%** |

The total frame time of approximately 33 milliseconds is well under our 50 millisecond budget, providing a comfortable margin for occasional spikes due to system load or complex scenes. This headroom is important for robust real-world operation where timing must be reliable, not just average-case acceptable.

#### 5.2.4 Visualization System

Effective visualization proved essential for both system development and demonstration. During development, visualization allowed us to verify that each component was functioning correctly—seeing the occlusion grid update in real-time made it immediately obvious when ray-casting was misconfigured or when vehicle bounding boxes were incorrectly positioned. For demonstration purposes, visualization communicates the system's reasoning to observers who would otherwise see only the vehicle's external behavior without understanding why it makes particular decisions.

**Camera Feed Overlay**

The primary visualization overlays detection results directly on the RGB camera feed, showing exactly what the perception system sees and how it interprets the scene:

- **Object bounding boxes**: Each detected object receives a colored rectangle indicating its class and threat level. Pedestrians in the vehicle's path appear with bright blue boxes, immediately drawing attention to the most critical detections. Vehicles receive cyan boxes when they pose no immediate threat, transitioning to orange and red as they approach the ego vehicle's path or exhibit concerning behavior.

- **Distance annotations**: Text labels above each bounding box display the estimated distance in meters, calculated from the depth camera data. This annotation helps verify that 3D position estimation is working correctly and provides intuitive understanding of the scene geometry.

- **Risk level indicator**: A colored bar or numerical display shows the current overall risk level, allowing observers to correlate perceived danger with the system's internal state.

**Occlusion Grid Display**

A bird's-eye-view visualization of the occlusion grid provides insight into the spatial reasoning that distinguishes this system from conventional perception:

- **Visibility mapping**: Each grid cell is colored according to its occlusion status—dark gray indicates visible areas where the vehicle has clear line-of-sight, while red indicates occluded areas hidden behind obstacles. Watching this display as the vehicle moves reveals how occlusion patterns shift dynamically with the environment.

- **Region boundaries**: Thin lines delineate the five risk regions (forward, forward-left, forward-right, side-left, side-right), making it clear which areas contribute most strongly to the risk assessment. The forward region, with its highest weight, is visually distinguished.

- **Object positions**: Detected pedestrians and vehicles are marked on the grid at their estimated positions, shown as colored squares or circles. This overlay allows verification that 3D position estimation correctly places objects in the spatial representation.

- **Ego vehicle indicator**: The ego vehicle's position and heading appear at the grid center, typically as a small arrow or vehicle icon. This reference point anchors the visualization and makes vehicle-relative spatial relationships intuitive.

**Heads-Up Display (HUD)**

Numerical and status information appears in a dashboard-style HUD overlay that summarizes system state at a glance:

- **Speed information**: Both current speed and target speed appear prominently, typically in large font. The difference between these values indicates whether the system is accelerating, braking, or maintaining speed.

- **Risk metrics**: The overall risk level appears as both a percentage (0-100%) and a color-coded bar. When risk is low, the indicator appears green; moderate risk shows yellow; high risk displays orange or red. This immediate visual feedback communicates system state even to casual observers.

- **Risk source breakdown**: A detailed breakdown shows the contribution from each risk source—occlusion, social cues, pedestrian detection, and vehicle detection. This decomposition is invaluable for debugging and for understanding which factors drive particular behaviors.

- **Controller mode indicator**: A clear label indicates whether the baseline or occlusion-aware controller is active. This distinction is essential during comparative testing to ensure observers know which system variant they are watching.

---
