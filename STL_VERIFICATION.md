# STL Verification for Occlusion-Aware Perception

---

## What is STL?

**Signal Temporal Logic (STL)** is a formal specification language for real-time systems. It allows us to write precise, mathematical requirements about how signals should behave over time.

### Why STL for Autonomous Vehicles?

| Challenge | How STL Helps |
|-----------|---------------|
| Complex timing requirements | Temporal operators with time bounds |
| Continuous signals (speed, distance) | Works directly with real-valued signals |
| Need quantitative safety margins | Robustness semantics (not just pass/fail) |
| Finding edge cases | Enables automated falsification |

---

## STL Syntax

STL formulas are built from **atomic predicates** and **temporal operators**:

### Atomic Predicates

Compare signals to thresholds:
- `v ≤ 5` — speed is at most 5 m/s
- `distance > 10` — distance is greater than 10 m
- `occlusion ≥ 0.5` — occlusion level is at least 50%

### Boolean Operators

| Operator | Symbol | Meaning |
|----------|--------|---------|
| AND | ∧ | Both must be true |
| OR | ∨ | At least one must be true |
| NOT | ¬ | Negation |
| IMPLIES | → | If A then B |

### Temporal Operators

| Operator | Symbol | Meaning |
|----------|--------|---------|
| **Globally** | G[a,b] | Must hold for ALL time in [a,b] |
| **Eventually** | F[a,b] | Must hold at SOME time in [a,b] |
| **Until** | U[a,b] | φ₁ holds until φ₂ becomes true |

**Reading STL Formulas**:
- `G[0,10](v ≤ 5)` — "Speed stays ≤ 5 m/s for the next 10 seconds"
- `F[0,3](v ≤ 0.5)` — "Speed drops to near-zero within 3 seconds"
- `G(danger → F[0,2](brake))` — "Whenever danger, brake within 2 seconds"

### Combining Operators

STL's power comes from **nesting** operators:

```
G(condition → F[0,τ](response))
│      │           │
│      │           └── Response must happen within τ seconds
│      └────────────── When this condition is true...
└──────────────────── At ALL times throughout the trace
```

**Example**: `G(StopPed → F[0,3](v ≤ 0.5))`

*"Globally, whenever a pedestrian is within stop distance, the vehicle must reach near-zero speed within 3 seconds."*

This captures:
- **Safety** — must respond to pedestrians
- **Timing** — response must be fast (within 3s)
- **Quantitative** — speed must drop below threshold

---

## How STL Verification Works

For autonomous vehicle verification, STL provides three capabilities:

### 1. Monitoring
Check if simulation traces satisfy or violate specifications.

### 2. Robustness
Quantify *how strongly* satisfied or violated (not just pass/fail).

### 3. Falsification
Systematically search for inputs that break specifications.

```
┌─────────────────────────────────────────────────────────────┐
│                  STL Verification Pipeline                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌───────────┐    ┌───────────┐    ┌───────────────────┐   │
│   │  CARLA    │───▶│  Signal   │───▶│  STL Monitor      │   │
│   │ Simulation│    │   Log     │    │  (evaluate specs) │   │
│   └───────────┘    └───────────┘    └─────────┬─────────┘   │
│                                               │              │
│                                               ▼              │
│                                        ┌─────────────┐       │
│                                        │ Robustness  │       │
│                                        │  ρ > 0 PASS │       │
│                                        │  ρ < 0 FAIL │       │
│                                        └─────────────┘       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Robustness: Quantitative Satisfaction

Unlike boolean logic (true/false), STL computes **robustness** (ρ) — a real number indicating *how much* a specification is satisfied or violated.

### Robustness Rules

| Formula | Robustness Computation |
|---------|------------------------|
| `x ≤ c` | ρ = c - x (positive if satisfied) |
| `x ≥ c` | ρ = x - c (positive if satisfied) |
| `φ₁ ∧ φ₂` | ρ = min(ρ₁, ρ₂) — weakest link |
| `φ₁ ∨ φ₂` | ρ = max(ρ₁, ρ₂) — strongest link |
| `G[a,b](φ)` | ρ = min over [a,b] — worst moment |
| `F[a,b](φ)` | ρ = max over [a,b] — best moment |

### Worked Example

**Specification**: `G[0,5](v ≤ 10)` — "Speed stays ≤ 10 m/s for 5 seconds"

**Trace**:
```
Time:    0    1    2    3    4    5
Speed:   8    9   11    9    7    6
         ▲    ▲    ▲    ▲    ▲    ▲
ρ(t):   +2   +1   -1   +1   +3   +4
```

At each time: ρ(t) = 10 - v(t)

**G (Globally)** takes the **minimum**: ρ = min(+2, +1, -1, +1, +3, +4) = **-1**

**Result**: Specification **VIOLATED** (ρ < 0). The vehicle exceeded 10 m/s by 1 m/s at t=2.

### Why Robustness Matters

| Scenario | Boolean Result | Robustness |
|----------|----------------|------------|
| Speed = 9.9 m/s (limit 10) | ✓ PASS | ρ = +0.1 (barely safe) |
| Speed = 5.0 m/s (limit 10) | ✓ PASS | ρ = +5.0 (very safe) |
| Speed = 10.1 m/s (limit 10) | ✗ FAIL | ρ = -0.1 (barely unsafe) |
| Speed = 15.0 m/s (limit 10) | ✗ FAIL | ρ = -5.0 (very unsafe) |

Robustness tells you **how close** you are to the boundary — essential for:
- Understanding safety margins
- Guiding falsification search (minimize ρ to find violations)
- Comparing controllers (higher ρ = safer)

---

## STL Specifications for Our Controller

### Signals

| Signal | Description |
|--------|-------------|
| `d_ped(t)` | Distance to nearest pedestrian (m) |
| `v(t)` | Ego vehicle speed (m/s) |
| `r_occ(t)` | Occlusion risk level [0,1] |
| `a(t)` | Ego acceleration (m/s²) |
| `adj_brake(t)` | Adjacent vehicle braking hard (boolean) |
| `ped_in_path(t)` | Pedestrian detected in ego's path (boolean) |

---

### 1. Collision Avoidance (Hard Safety)

```
φ₁ = G(d_ped ≥ 0.5)
```

**Plain English**: *"Always maintain at least 0.5m distance from any pedestrian."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Always active |
| **Required Response** | Keep minimum 0.5m clearance from all pedestrians |
| **Time Constraint** | Continuous (no violations allowed) |
| **What it Prevents** | Physical collisions with pedestrians |

**Robustness Interpretation**:
- ρ = +3.0 → Maintained 3.5m distance (very safe)
- ρ = +0.5 → Maintained 1.0m distance (safe but close)
- ρ = -0.3 → Got within 0.2m of pedestrian (VIOLATION — near collision)

**Why This Matters**: This is the **hard safety constraint**. Any ρ < 0 means the vehicle came dangerously close to a pedestrian. Higher robustness = larger safety margin.

---

### 2. Proactive Braking — Occlusion Response ⭐ CORE NOVELTY

```
φ₂ = G(r_occ ≥ 0.5 → F[0,2](v ≤ 0.5 × v_target))
```

**Plain English**: *"Whenever occlusion risk exceeds 0.5, within 2 seconds speed should drop below half the target speed."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Occlusion risk ≥ 50% |
| **Required Response** | Reduce speed to ≤ 50% of target |
| **Time Constraint** | Within 2 seconds |
| **What it Prevents** | Driving fast through blind spots |

**Robustness Interpretation**:
- ρ > 0 → Successfully slowed down with margin
- ρ < 0 → Failed to slow down in time (VIOLATION)

**Why This Matters**: 
```
┌────────────────────────────────────────────────────────────┐
│  This is THE KEY DIFFERENTIATOR between controllers        │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  BASELINE CONTROLLER:                                      │
│  • High occlusion? Doesn't care, keeps driving fast        │
│  • VIOLATES φ₂                                             │
│                                                            │
│  AWARE CONTROLLER:                                         │
│  • High occlusion? Slows to half speed within 2 seconds    │
│  • SATISFIES φ₂                                            │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Test Question**: Does the system actually slow down when it detects occlusion?

---

### 3. Social Cue Response

```
φ₃ = G(adj_brake → F[0,1](a < 0))
```

**Plain English**: *"If an adjacent vehicle brakes hard, ego should start braking within 1 second."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Adjacent vehicle braking hard |
| **Required Response** | Begin braking (negative acceleration) |
| **Time Constraint** | Within 1 second |
| **What it Prevents** | Ignoring social cues that indicate danger |

**Robustness Interpretation**:
- ρ > 0 → Responded quickly to social cue
- ρ < 0 → Ignored adjacent vehicle's warning (VIOLATION)

**Why This Matters**: If another driver brakes suddenly, they likely see something you don't. A smart controller uses this social information.

---

### 4. Emergency Stop Capability

```
φ₄ = G(ped_in_path ∧ d_ped ≤ 15 → F[0,3](v ≤ 0.5))
```

**Plain English**: *"If a pedestrian is detected in path within 15m, come to a near-stop within 3 seconds."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Pedestrian in path AND within 15m |
| **Required Response** | Reduce speed to near-zero (≤ 0.5 m/s) |
| **Time Constraint** | Within 3 seconds |
| **What it Prevents** | Collisions with detected pedestrians |

**Robustness Interpretation**:
- ρ = +0.4 → Stopped with 0.4 m/s margin (good)
- ρ = -1.0 → Still going 1.5 m/s when should be stopped (VIOLATION)

**Why This Matters**: This validates the controller's ability to perform emergency stops when needed.

---

### 5. Comfort Constraint

```
φ₅ = G(¬emergency → a ≥ -3)
```

**Plain English**: *"Deceleration should stay comfortable (≥ -3 m/s²) except in emergencies."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Non-emergency situations |
| **Required Response** | Keep deceleration ≥ -3 m/s² |
| **Exception** | Emergency situations allow harder braking |
| **What it Prevents** | Uncomfortable, jerky driving |

**Robustness Interpretation**:
- ρ = +1.0 → Smooth braking at -2 m/s²
- ρ = -2.0 → Harsh braking at -5 m/s² without emergency (VIOLATION)

**Why This Matters**: Smooth driving = proactive safety. Constant harsh braking indicates reactive/panic behavior.

---

### 6. Liveness (Progress)

```
φ₆ = G(F[0,60](Δposition > 10))
```

**Plain English**: *"The vehicle should eventually make progress (move at least 10m every 60 seconds)."*

| Aspect | Description |
|--------|-------------|
| **Trigger** | Always active |
| **Required Response** | Make forward progress |
| **Time Constraint** | At least 10m every 60 seconds |
| **What it Prevents** | Getting stuck forever; overly conservative behavior |

**Robustness Interpretation**:
- ρ > 0 → Making good progress
- ρ < 0 → Stuck or crawling too slowly (VIOLATION)

**Why This Matters**: 
```
┌────────────────────────────────────────────────────────────┐
│  Without liveness, a controller could "cheat":             │
│                                                            │
│  • Just stop forever → trivially "safe" but useless        │
│  • Crawl at 0.1 m/s → no collisions but impractical        │
│                                                            │
│  Liveness ensures the controller is USEFUL, not just safe. │
└────────────────────────────────────────────────────────────┘
```

---

### Complete Controller Specification

```
φ_controller = φ₁ ∧ φ₂ ∧ φ₃ ∧ φ₄ ∧ φ₅ ∧ φ₆
```

**All specifications must be satisfied simultaneously:**

| # | Name | Requirement |
|---|------|-------------|
| φ₁ | Collision Avoidance | Always ≥ 0.5m from pedestrians |
| φ₂ | Occlusion Response | Slow to half speed within 2s when occluded |
| φ₃ | Social Cue Response | Brake within 1s if adjacent vehicle brakes |
| φ₄ | Emergency Stop | Stop within 3s if pedestrian in path ≤ 15m |
| φ₅ | Comfort | No harsh braking except emergencies |
| φ₆ | Liveness | Make progress (≥ 10m per minute) |

### Specification Priority

```
HIGHEST PRIORITY (Safety)
      │
      ▼
  ┌───────────┐
  │    φ₁    │  ← Collision avoidance (hard constraint)
  └─────┬─────┘
        ▼
  ┌───────────┐
  │    φ₄    │  ← Emergency stop capability
  └─────┬─────┘
        ▼
  ┌───────────┐
  │    φ₂    │  ← Occlusion response (proactive)
  └─────┬─────┘
        ▼
  ┌───────────┐
  │    φ₃    │  ← Social cue response
  └─────┬─────┘
        ▼
  ┌───────────┐
  │    φ₅    │  ← Comfort (soft constraint)
  └─────┬─────┘
        ▼
  ┌───────────┐
  │    φ₆    │  ← Liveness (progress)
  └───────────┘

LOWEST PRIORITY (Performance)
```

---

## Robustness Semantics

Robustness (ρ) measures *how strongly* a specification is satisfied or violated:

| Robustness Value | Interpretation |
|------------------|----------------|
| **ρ > 0** | Specification SATISFIED with margin |
| **ρ = 0** | Boundary case (just barely satisfied) |
| **ρ < 0** | Specification VIOLATED |

**Example**: For `v ≤ 5 m/s`:
- If v = 3 m/s → ρ = +2 (satisfied by 2 m/s margin)
- If v = 5 m/s → ρ = 0 (exactly at boundary)
- If v = 7 m/s → ρ = -2 (violated by 2 m/s)

Robustness provides quantitative safety margins, not just pass/fail.

---

## Falsification Approach

**Falsification** = searching for inputs that cause specification violations.

```
┌─────────────────────────────────────────────────────────────┐
│                    Falsification Loop                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌─────────────────┐                                        │
│   │ Sample scenario │  (ped_time, ped_speed, ego_speed...)   │
│   │   parameters    │                                        │
│   └────────┬────────┘                                        │
│            │                                                 │
│            ▼                                                 │
│   ┌─────────────────┐                                        │
│   │  Run simulation │                                        │
│   └────────┬────────┘                                        │
│            │                                                 │
│            ▼                                                 │
│   ┌─────────────────┐     ┌────────────────────┐             │
│   │  Evaluate STL   │────▶│ ρ < 0? → VIOLATION │             │
│   │  (compute ρ)    │     │ ρ ≥ 0? → try again │             │
│   └─────────────────┘     └────────────────────┘             │
│                                                              │
│   Goal: Minimize ρ (find worst-case scenarios)               │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Search Space

| Parameter | Range | Description |
|-----------|-------|-------------|
| `ped_start_time` | 0-10s | When pedestrian starts crossing |
| `ped_speed` | 0.5-3.0 m/s | Pedestrian walking speed |
| `ped_offset` | -3 to +3m | Lateral start position |
| `ego_start_speed` | 20-50 km/h | Initial ego vehicle speed |
| `occlusion_offset` | -2 to +2m | Position of occluding object |

### Optimization Algorithms

| Algorithm | Description |
|-----------|-------------|
| Random Search | Uniform sampling (baseline exploration) |
| CMA-ES | Covariance Matrix Adaptation (evolutionary) |
| Bayesian Optimization | Sample-efficient with surrogate model |
| Simulated Annealing | Probabilistic local search |

---

## Tools

| Tool | Language | Notes |
|------|----------|-------|
| **RTAMT** | Python | Recommended — easy CARLA integration |
| **Breach** | MATLAB | Built-in falsification |
| **S-TaLiRo** | MATLAB | Multiple optimization algorithms |

---

## Expected Results

| Specification | Baseline | Aware |
|---------------|----------|-------|
| φ_stop (collision) | May Violate | Satisfies |
| φ_occl (occlusion) | **VIOLATES** | **Satisfies** |
| φ_creep (social) | Satisfies (if detected) | Satisfies |
| φ_cruise (progress) | Satisfies | Satisfies |
| **Overall** | **FAILS** | **PASSES** |

The key differentiator is **φ_occl**: the baseline has no occlusion awareness and drives at full speed through blind spots.

---

## Summary

STL verification transforms "it seems to work" into **mathematical evidence of safety margins**.

- **Specifications** capture precise safety requirements
- **Robustness** quantifies safety margins
- **Falsification** finds edge cases
- **Comparison** proves aware controller > baseline controller

---
