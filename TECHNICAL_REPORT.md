# Occlusion-Aware Perception System for Autonomous Vehicles

## Technical Report

This document provides a comprehensive explanation of the occlusion-aware perception and control system designed for autonomous vehicles operating in challenging visibility conditions.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Problem Statement](#2-problem-statement)
3. [Test Scenarios](#3-test-scenarios)
4. [System Architecture](#4-system-architecture)
5. [Occlusion Grid System](#5-occlusion-grid-system)
6. [Risk Assessment Model](#6-risk-assessment-model)
7. [Speed Control Strategy](#7-speed-control-strategy)
8. [Vision-Based Detection System](#8-vision-based-detection-system)
9. [Controller Comparison](#9-controller-comparison)
10. [System Parameters](#10-system-parameters)

---

## 1. Introduction

Occlusion—the blocking of an observer's line of sight by intervening objects—represents one of the most challenging problems in autonomous vehicle perception. When a pedestrian is hidden behind a parked truck, or when a vehicle approaches from an occluded intersection, traditional perception systems that rely solely on direct observation fail to anticipate potential hazards.

This system addresses the occlusion problem through a multi-layered approach that combines:

- **Spatial awareness** through an occlusion grid that tracks which areas around the vehicle are visible versus hidden
- **Behavioral awareness** through monitoring of nearby vehicles for social cues that might indicate unseen hazards
- **Direct perception** through vision-based detection of pedestrians and vehicles using deep learning
- **Predictive caution** through risk-based speed modulation that slows the vehicle when approaching potentially hazardous occluded regions

The fundamental philosophy is simple: when you cannot see clearly, slow down. The system quantifies "how much you cannot see" and translates that into appropriate speed reductions.

---

## 2. Problem Statement

### The Hidden Hazard Problem

Consider a common urban scenario: a pedestrian decides to cross the street between two parked trucks. From the pedestrian's perspective, they can see the oncoming traffic. However, from the approaching vehicle's perspective, the pedestrian is completely invisible until the moment they step into the roadway.

Traditional autonomous driving systems face a critical limitation in these situations. They can only react to what they can perceive. By the time a hidden pedestrian becomes visible, the vehicle may not have sufficient stopping distance, especially if traveling at normal urban speeds.

### Why Existing Solutions Fall Short

Standard perception systems excel at detecting visible objects but provide no information about what might be hidden. A LiDAR system will accurately map the visible environment but cannot tell the vehicle that a dangerous blind spot exists behind that parked truck.

Existing safety systems like Automatic Emergency Braking (AEB) activate only when a hazard is directly detected. This reactive approach may be too late in occlusion scenarios where hazards emerge suddenly from hidden areas.

### The Proactive Approach

Our system takes a fundamentally different approach: rather than waiting to detect a hazard, we identify areas where hazards could potentially hide and adjust the vehicle's behavior accordingly. This means:

- Identifying occluded regions before any hazard appears
- Calculating the risk posed by each occluded region based on its location and size
- Reducing vehicle speed proportionally to the identified risk
- Monitoring nearby vehicles for behavioral cues that might indicate hidden hazards

---

## 3. Test Scenarios

The system is validated across three distinct occlusion scenarios, each presenting unique challenges.

### Scenario 1: Static Truck Occlusion

**Setup**: Two large trucks are parked along the roadside. A pedestrian waits between the trucks, invisible to approaching traffic. The ego vehicle approaches at a moderate speed.

**The Challenge**: The pedestrian is completely hidden by the trucks' large profiles. The occlusion is static and predictable, but the timing of pedestrian emergence is unknown.

**What the System Must Do**: Recognize that the area between and behind the trucks represents a high-risk occluded zone. Calculate the appropriate approach speed that would allow the vehicle to stop safely if a pedestrian suddenly appeared from behind either truck.

**Key Learning**: Even stationary obstacles can create dangerous blind spots that require proactive speed management.

### Scenario 2: Moving Vehicle Occlusion

**Setup**: Two trucks travel in the adjacent lane, slightly ahead of the ego vehicle. A pedestrian crosses in front of the trucks. The front truck brakes hard for the pedestrian.

**The Challenge**: The occlusion source is moving, making the occluded region dynamic. The ego vehicle cannot see the pedestrian but might observe the truck's sudden braking behavior.

**What the System Must Do**: 
1. Track the moving occlusion created by the adjacent trucks
2. Detect the social cue when the truck brakes suddenly
3. Interpret sudden braking by a nearby vehicle as a strong indicator of a potential hazard
4. React to the social cue even before the pedestrian becomes visible

**Key Learning**: Observing the behavior of other vehicles provides crucial information about hazards you cannot see directly. When a nearby vehicle brakes hard, there is usually a reason—and that reason might affect you too.

### Scenario 3: Parked Vehicle Gauntlet

**Setup**: Multiple vehicles are parked along both sides of the road, creating a "gauntlet" of potential occlusion sources. Pedestrians may cross at various points between the parked vehicles.

**The Challenge**: Unlike scenarios with one or two occluders, this scenario presents continuous occlusion along the entire route. The vehicle cannot simply slow down for one occluded area—it must navigate an extended zone of reduced visibility.

**What the System Must Do**: Maintain an appropriately reduced speed throughout the entire stretch of parked vehicles. Continuously scan for pedestrians emerging from between any of the parked cars. Balance safety (slow enough to stop) against practicality (not crawling at walking pace for extended distances).

**Key Learning**: Real urban environments often present extended occlusion zones. The system must find a sustainable balance between safety and mobility in these conditions.

---

## 4. System Architecture

### Overview

The system consists of two distinct controllers that can be selected for comparison purposes:

**Baseline Controller**: Represents a traditional autonomous driving system without occlusion awareness. This controller maintains a constant target speed regardless of occlusion conditions. It uses vision-based detection (YOLO) to identify pedestrians and vehicles, but only for visualization purposes—it does not modify its behavior based on potential hidden hazards. The baseline controller serves as a reference point to demonstrate what happens when occlusion is ignored.

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

**Occlusion-Aware Controller**: Implements the full occlusion-aware perception and control system. This controller actively analyzes the environment for occluded regions, monitors nearby vehicle behavior for social cues, performs vision-based detection of pedestrians and vehicles, and dynamically adjusts vehicle speed based on the combined risk assessment.

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

### Information Flow

The occlusion-aware controller processes information through several parallel pathways that ultimately converge into a unified risk assessment:

**Occlusion Analysis Pathway**: Every 50 milliseconds, the system constructs a bird's-eye-view map of the area around the vehicle, identifying which regions are visible and which are blocked by obstacles. This map is divided into regions of interest, and each region's occlusion level is converted into a risk score.

**Social Cue Pathway**: The system continuously tracks all vehicles within a 30-meter radius. It monitors their speeds, accelerations, and positions. Sudden braking by a nearby vehicle triggers an elevated risk assessment, as this often indicates a hazard that the ego vehicle may not yet see.

**Vision Detection Pathway**: RGB and depth cameras feed into a neural network (YOLOv8) that detects pedestrians and vehicles in real-time. Detected objects are projected into 3D space using depth information, allowing the system to determine their distance and whether they lie in the vehicle's path.

**Risk Fusion**: All three pathways contribute risk scores that are combined using a maximum-of-risks approach. This conservative strategy ensures that if any pathway identifies significant danger, the vehicle responds appropriately, even if other pathways indicate safety.

**Speed Control**: The fused risk score is translated into a target speed using physics-based calculations that ensure the vehicle can stop safely given the identified risk level and distance to the nearest occlusion.

---

## 5. Occlusion Grid System

### Conceptual Foundation

Imagine looking down at the ego vehicle from directly above. The system creates a square grid centered on the vehicle, covering an area of 30 meters by 30 meters (15 meters in each direction from the vehicle). This grid is divided into 3,600 cells (60 rows by 60 columns), with each cell representing a square area of approximately 0.5 meters on each side.

For each cell, the system determines whether a straight line from the ego vehicle to that cell's center would be blocked by any obstacle (primarily other vehicles). If blocked, the cell is marked as "occluded" and colored red in visualization. If clear, the cell is marked as "visible" and colored dark gray.

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

### The Ray-Casting Process

To determine occlusion, the system performs a process called ray-casting. Imagine shining a flashlight from the ego vehicle toward each cell in the grid. If the light reaches the cell unobstructed, the cell is visible. If a vehicle blocks the light path, everything behind that vehicle (from the ego's perspective) is occluded.

The system samples points along each ray at regular intervals (every 0.5 meters). At each sample point, it checks whether the point falls within the bounding box of any known obstacle. The first obstacle encountered along any ray blocks visibility to all cells beyond it on that ray.

### Regions of Interest

Not all directions around the vehicle carry equal importance. A pedestrian stepping out directly in front of the vehicle poses a more immediate threat than one appearing to the side. The system reflects this reality by dividing the grid into five regions, each with its own risk weight:

**Forward Region** (weight: 1.0): This covers the area directly ahead of the vehicle, within 30 degrees to either side of the vehicle's heading. Occlusion in this region receives the highest risk weight because hazards here have the shortest reaction time and are most likely to result in collision.

**Forward-Left and Forward-Right Regions** (weight: 0.8 each): These regions extend from 30 to 70 degrees on either side of the vehicle's heading. They are particularly relevant during turn maneuvers when the vehicle's path curves into these areas.

**Side-Left and Side-Right Regions** (weight: 0.5 each): Covering 70 to 110 degrees on either side, these regions monitor the adjacent lanes. While less immediately critical than the forward region, side occlusions matter because objects can quickly move from the side into the vehicle's path.

**Rear Region** (ignored): The area behind the vehicle is not monitored for occlusion risk, as hazards approaching from behind are primarily the responsibility of those following vehicles.

### Converting Occlusion to Risk

For each region, the system calculates a risk score based on two factors:

**Coverage Factor**: What percentage of the region is occluded? A region that is 80% occluded is more dangerous than one that is 20% occluded because there are more places where hazards could be hiding.

**Proximity Factor**: How close is the nearest occluded cell? An occluded area 3 meters ahead is far more dangerous than one 12 meters ahead because there is less time and distance to react.

These two factors are combined to produce a region risk score. Cells closer to the vehicle contribute more heavily to this score because nearby occlusions are more dangerous than distant ones. The mathematical weighting gives closer cells higher importance—a cell 2 meters away contributes much more to the risk score than a cell 10 meters away.

The overall occlusion risk is then computed by combining all region scores, weighted by their importance factors (forward region contributing more than side regions).

---

## 6. Risk Assessment Model

### The Multi-Source Approach

Rather than relying on a single indicator of danger, the system combines information from multiple independent sources. This redundancy provides robustness—even if one source fails to detect a hazard, another source may catch it.

The four risk sources are:

1. **Occlusion Risk**: Derived from the occlusion grid analysis described above
2. **Social Cue Risk**: Derived from observing nearby vehicle behavior
3. **Pedestrian Risk**: Derived from direct vision-based detection of pedestrians
4. **Vehicle Risk**: Derived from direct vision-based detection of other vehicles

### Social Cue Risk Explained

Humans are remarkably good at inferring information from the behavior of others. When you see a car ahead suddenly brake, you instinctively prepare to brake yourself, even before you see why they stopped. This system implements a computational version of this social intelligence.

The system continuously tracks all vehicles within 30 meters of the ego vehicle. For each tracked vehicle, it maintains a history of recent speeds and positions. Several behaviors trigger elevated risk assessments:

**Hard Braking Detection**: When a nearby vehicle decelerates faster than 3.0 meters per second squared, it indicates an urgent situation. The system interprets this as a strong signal that something unexpected has occurred ahead—possibly a pedestrian, an obstacle, or another hazard. This behavior contributes a risk increase of 0.4 (on a 0-1 scale).

**Stopped Vehicle Ahead**: When a vehicle is nearly stationary (moving slower than 1 meter per second) and positioned within 15 meters directly ahead, it raises concern. A stopped vehicle might be yielding to a pedestrian or waiting for an obstruction that the ego vehicle cannot yet see. This contributes a risk increase of 0.3.

**Rapid Approach**: When another vehicle is closing in quickly (relative speed greater than 10 meters per second), it may indicate an emergency situation or aggressive driving that could create hazards. This contributes a risk increase of 0.2.

The social cue risk is weighted at 60% of its raw value before being combined with other risks. This weighting reflects the inherently inferential nature of social cues—they suggest danger but do not confirm it directly.

### Pedestrian Risk Explained

When the vision system detects a pedestrian, the risk calculation depends primarily on two factors: whether the pedestrian is in the vehicle's path and how far away they are.

**Path Determination**: The system projects the detected pedestrian's position into the vehicle's reference frame and checks whether they fall within the lane width (approximately 3.5 meters centered on the vehicle's trajectory). Pedestrians outside this zone are noted but do not trigger path-based risk.

**Distance-Based Risk Scaling**: For pedestrians in the vehicle's path, the risk follows a proximity-based model:

Within 15 meters, the system assigns maximum pedestrian risk (1.0), indicating an emergency situation requiring immediate stopping. This distance threshold is based on the physics of braking: at typical urban speeds, 15 meters approaches the minimum stopping distance even with hard braking.

Between 15 and 25 meters, the risk scales proportionally. A pedestrian at 20 meters (halfway through this range) would generate a risk of approximately 0.4. This graduated response allows the system to begin slowing before an emergency develops.

Beyond 25 meters, pedestrians in the path generate minimal risk. This distance provides adequate stopping margin at normal urban speeds, and the pedestrian may move out of the path before the vehicle arrives.

### Risk Fusion Strategy

With four different risk sources, the system must combine them into a single actionable risk score. Several fusion strategies are possible:

- **Averaging** would produce a moderate response even when one source indicates extreme danger
- **Summation** could produce unrealistically high values when multiple sources contribute
- **Maximum selection** ensures that the highest individual risk dominates the response

This system uses maximum selection as its fusion strategy. This conservative approach means that if any single source indicates high danger, the vehicle responds with full caution, regardless of what other sources indicate. This reflects a safety-first philosophy: it is better to slow down unnecessarily than to miss a genuine threat.

### Temporal Smoothing

Sensor readings can fluctuate rapidly due to noise, temporary occlusions of sensors, or momentary detection failures. Without smoothing, these fluctuations would cause jerky, uncomfortable vehicle behavior with rapid alternations between acceleration and braking.

The system maintains a memory of the 20 most recent risk assessments (covering approximately one second at the 20 Hz update rate). Instead of using only the instantaneous risk, the system uses the maximum risk observed over this memory window.

This approach has an important safety implication: risk can increase instantaneously (responding immediately to new threats) but decreases gradually (ensuring that briefly-obscured threats are not forgotten). If a pedestrian is detected, lost from view momentarily, then redetected, the system maintains appropriate caution throughout rather than relaxing and re-engaging repeatedly.

---

## 7. Speed Control Strategy

### The Fundamental Question

Given a risk level and information about the environment, how fast should the vehicle travel? This question sits at the heart of the control system and requires balancing multiple considerations:

- **Safety**: The vehicle must be able to stop before hitting any hazard that might emerge from an occluded region
- **Comfort**: Passengers should not experience harsh, uncomfortable braking
- **Practicality**: The vehicle should make reasonable progress toward its destination, not crawl everywhere

### Physics-Based Speed Calculation

The system's approach to speed calculation is grounded in basic physics. When a vehicle brakes, it decelerates at some rate (measured in meters per second squared). The stopping distance depends on the initial speed and the deceleration rate according to the formula: stopping distance equals speed squared divided by twice the deceleration rate.

Rearranging this formula gives us the maximum safe speed for a given stopping distance: safe speed equals the square root of twice the deceleration rate times the available distance.

But what deceleration rate should we assume? This is where risk level enters the calculation.

### Risk-Adjusted Deceleration

The system defines two deceleration benchmarks:

**Comfortable Deceleration** (2.5 m/s²): This is the braking intensity that passengers find comfortable for routine slowdowns. It corresponds to gentle braking that does not cause passengers to lurch forward.

**Emergency Deceleration** (6.0 m/s²): This is near the maximum braking capability of most vehicles on dry pavement. It is uncomfortable but necessary in genuine emergencies.

The assumed deceleration rate interpolates between these two extremes based on the current risk level:

At low risk (near 0), the system assumes it would brake comfortably if needed, allowing higher speeds.

At high risk (near 1), the system assumes emergency braking might be necessary, requiring lower speeds to maintain safe stopping distances.

This interpolation reflects a practical reality: if the risk is high, we must assume the worst-case scenario might materialize, requiring hard braking. If the risk is low, we can assume gentler braking will suffice.

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

### Calculating Target Speed

The target speed calculation proceeds in several steps:

**Step 1 - Determine Available Distance**: The system identifies the distance to the nearest significant occluded region in the forward direction. This is the distance within which a hazard might appear. A safety margin of 3 meters is subtracted from this distance to provide additional buffer.

**Step 2 - Calculate Stopping-Distance-Based Limit**: Using the physics formula and the risk-adjusted deceleration rate, the system calculates the maximum speed that would allow stopping within the available distance.

**Step 3 - Apply Risk-Based Speed Reduction**: Independent of stopping distance, the system applies a speed reduction factor based on risk level. At zero risk, no reduction is applied. At maximum risk, speed is reduced to 30% of the target. This factor provides additional conservatism beyond the physics-based calculation.

**Step 4 - Select the Lower Value**: The final target speed is the lower of the stopping-distance-based limit and the risk-adjusted target speed. This ensures that both physical constraints and risk-based caution are respected.

**Step 5 - Apply Bounds**: The target speed is constrained to be at least 1.5 meters per second (allowing creeping progress even in high-risk situations) and at most the scenario's designated target speed.

### Smooth Control Application

With a target speed determined, the system must control the vehicle's throttle and brake to achieve that speed. Rather than using abrupt on/off control, the system uses proportional control that scales the control input based on the speed error.

When the current speed is more than 0.5 meters per second below the target, the system applies throttle proportionally to the speed deficit. Larger deficits produce more throttle, up to a maximum of 80%.

When the current speed is more than 0.5 meters per second above the target, the system applies braking proportionally to the excess speed. Larger excesses produce more braking, up to a maximum of 90%.

When the current speed is within 0.5 meters per second of the target, the system applies minimal throttle to maintain speed against friction and rolling resistance.

This proportional approach produces smooth, gradual speed changes that are comfortable for passengers while still responding appropriately to risk conditions.

---

## 8. Vision-Based Detection System

### Overview

While the occlusion grid identifies where hazards might hide, the vision system detects hazards that are actually visible. These two systems complement each other: the occlusion grid provides proactive caution for unseen threats, while vision detection provides reactive response to visible threats.

### Camera Configuration

The system uses two cameras mounted on the front of the vehicle:

**RGB Camera**: Captures color images at 960 by 540 pixel resolution with a 110-degree horizontal field of view. This wide field of view allows detection of pedestrians and vehicles approaching from the sides, not just directly ahead.

**Depth Camera**: Captures distance information at the same resolution and field of view as the RGB camera. Each pixel in the depth image contains the distance from the camera to the object at that pixel location, measured in meters.

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

### Object Detection with YOLOv8

The RGB images are processed by YOLOv8, a state-of-the-art neural network for object detection. This network has been trained on millions of images to recognize common objects including people, cars, trucks, buses, and motorcycles.

For each frame, YOLOv8 outputs a list of detected objects, each described by:
- A bounding box indicating the object's location in the image
- A class label indicating what type of object was detected
- A confidence score indicating how certain the network is about the detection

The system filters detections to only consider those with confidence scores above 50%, reducing false positives while maintaining good detection of genuine objects.

### From 2D Detection to 3D Position

A bounding box in the image tells us where an object appears on the camera sensor, but not how far away it is. The depth camera fills this gap.

For each detected object, the system samples the depth image at the center of the detection bounding box. To improve robustness, it actually samples a small region around the center and takes the median depth value, reducing the impact of any single noisy measurement.

With the depth (distance from camera) known, the system can calculate the object's 3D position relative to the vehicle using the pinhole camera model. This model uses the camera's known field of view to convert the object's position in the image (measured in pixels from center) into angular offsets, which combined with depth give the full 3D position.

The result is a position in vehicle-relative coordinates:
- Forward distance: how far ahead of the vehicle the object is
- Lateral offset: how far to the left or right of the vehicle's centerline the object is

### Path Collision Assessment

With 3D positions computed, the system can determine whether each detected object poses a collision risk.

An object is considered "in path" if:
1. It is ahead of the vehicle (positive forward distance)
2. Its lateral offset is within half the lane width (approximately 1.75 meters) of the vehicle's centerline

Objects that satisfy both criteria could potentially collide with the vehicle if both continue on their current trajectories.

For pedestrians in the path, the system calculates risk based on distance as described in the Risk Assessment section. If any pedestrian is within the emergency stop distance (15 meters), the system triggers an immediate emergency stop override, applying maximum braking regardless of other factors.

### Visualization

Detected objects are visualized in two ways:

**Camera Feed Overlay**: Bounding boxes are drawn on the camera images with color coding. Pedestrians in the path appear with bright blue boxes. Vehicles appear with cyan boxes for those not in the path and orange or red boxes for those that pose a threat.

**Grid Visualization**: Detected pedestrians are also shown on the occlusion grid as blue squares at their estimated positions. This allows visualization of both the occlusion analysis and the vision detections in a single unified view.

---

## 9. Controller Comparison

### Baseline Controller Behavior

The baseline controller represents how a traditional autonomous vehicle might behave without occlusion awareness. Its behavior is straightforward:

It maintains a constant target speed throughout the scenario, regardless of occlusion conditions. The vehicle accelerates to reach this speed and maintains it until the scenario ends or the vehicle is manually stopped.

Vision detection is active but used only for visualization. The baseline controller draws bounding boxes around detected pedestrians and vehicles but does not modify its behavior based on these detections. This allows the visualization to show what the system can see without affecting the comparison.

The only exception is if a pedestrian or vehicle is detected extremely close (within emergency braking distance), the baseline will brake. However, in occlusion scenarios, this reaction often comes too late because the hazard was hidden until the last moment.

### Occlusion-Aware Controller Behavior

The occlusion-aware controller exhibits fundamentally different behavior:

When approaching occluded regions, it proactively reduces speed before any hazard is visible. This preemptive slowing provides the reaction time and stopping distance needed if a hazard emerges.

When adjacent vehicles brake suddenly, the controller responds by slowing down even if it cannot see the reason for their braking. This social cue response often provides earlier warning than direct perception.

When pedestrians are detected, the controller smoothly reduces speed based on distance, with emergency stopping reserved for truly close encounters.

The result is typically a lower average speed but a dramatically higher safety margin. In scenarios where the baseline controller would have insufficient stopping distance when a hazard appears, the occlusion-aware controller has already slowed down and can stop safely.

### Quantitative Differences

In typical test runs:

**Scenario 1 (Static Trucks)**: The baseline controller approaches the trucks at full speed and must brake hard when the pedestrian suddenly appears. The occlusion-aware controller has already reduced speed when passing occluded regions, allowing comfortable stopping.

**Scenario 2 (Moving Trucks)**: The baseline controller does not react to the adjacent truck's braking and must perform emergency braking when the pedestrian becomes visible. The occlusion-aware controller responds to the social cue and begins slowing before the pedestrian is visible.

**Scenario 3 (Parked Vehicle Gauntlet)**: The baseline controller maintains constant speed through the gauntlet, with only reactive braking for detected pedestrians. The occlusion-aware controller maintains a reduced speed throughout, providing consistent safety margins.

---

## 10. System Parameters

### Occlusion Grid Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Grid Size | 30 × 30 meters | Total area monitored around vehicle |
| Cell Count | 60 × 60 cells | Resolution of the grid |
| Cell Size | 0.5 × 0.5 meters | Size of each grid cell |
| Maximum Range | 15 meters | Distance from ego to grid edge |
| Ray Sample Interval | 0.5 meters | Spacing between samples along each ray |

### Risk Assessment Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Caution Threshold | 0.3 | Risk level at which speed reduction begins |
| Danger Threshold | 0.6 | Risk level indicating significant hazard |
| Emergency Threshold | 0.85 | Risk level triggering maximum response |
| Temporal Memory | 20 frames | Number of frames over which risk is smoothed |
| Social Cue Weight | 0.6 | Weighting applied to social cue risk |

### Speed Control Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Comfortable Deceleration | 2.5 m/s² | Braking rate for normal slowdowns |
| Emergency Deceleration | 6.0 m/s² | Maximum braking rate for emergencies |
| Minimum Speed | 1.5 m/s | Floor speed even in high-risk situations |
| Safety Margin | 3.0 meters | Buffer subtracted from stopping distance |

### Pedestrian Detection Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Stop Distance | 15 meters | Distance triggering emergency stop |
| Caution Distance | 25 meters | Distance at which slowing begins |
| Lane Width | 3.5 meters | Width used for path collision detection |
| Confidence Threshold | 50% | Minimum detection confidence to consider |

### Region Risk Weights

| Region | Weight | Coverage |
|--------|--------|----------|
| Forward | 1.0 | ±30° from heading |
| Forward-Left | 0.8 | -70° to -30° |
| Forward-Right | 0.8 | +30° to +70° |
| Side-Left | 0.5 | -110° to -70° |
| Side-Right | 0.5 | +70° to +110° |

---

## Conclusion

The occlusion-aware perception system represents a shift from reactive to proactive safety in autonomous driving. Rather than waiting for hazards to become visible, the system identifies where hazards might be hiding and adjusts vehicle behavior accordingly.

The multi-layered approach—combining spatial occlusion analysis, social cue monitoring, and vision-based detection—provides redundant safety coverage. Each layer can catch threats that others might miss, and the maximum-risk fusion strategy ensures that the most conservative appropriate response is always selected.

While this approach does result in lower average speeds compared to occlusion-unaware systems, the dramatic improvement in safety margins justifies this tradeoff. In safety-critical applications like urban autonomous driving, the ability to anticipate and prepare for potential hazards—rather than merely reacting to confirmed ones—can make the difference between a safe stop and a collision.
