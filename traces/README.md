# Trace Files for STL Verification

This directory contains JSON trace files for each scenario. These files define the scenario events declaratively, and the verification script generates the actual signal traces automatically.

## No CARLA Required!

You don't need to implement scenarios S5-S8 in CARLA. The JSON files here define the events, and `stl_verification.py` generates realistic traces from them.

## File Format

Each JSON file supports two formats:

### Format 1: Event-Based (Recommended)

Define high-level events and let the script generate the trace:

```json
{
  "description": "Scenario description",
  "controller": "baseline" or "aware",
  "scenario_config": {
    "duration": 25.0,
    "dt": 0.05,
    "v_target": 7.2,
    "initial_speed": 7.2
  },
  "events": [
    {
      "type": "occlusion",
      "time": 3.0,
      "duration": 9.0,
      "risk": 0.7
    },
    {
      "type": "pedestrian_reveal",
      "time": 10.0,
      "duration": 8.0,
      "initial_distance": 12.0,
      "ped_speed": 1.5
    }
  ]
}
```

### Format 2: Full Trace (For CARLA Data)

If you have actual CARLA simulation data, provide the full trace:

```json
{
  "trace": [
    {"time": 0.0, "v": 7.2, "d_ped": 50.0, "r_occ": 0.0, "a": 0.0, ...},
    {"time": 0.05, "v": 7.2, "d_ped": 49.8, "r_occ": 0.1, "a": 0.0, ...},
    ...
  ]
}
```

## Event Types

| Type | Description | Parameters |
|------|-------------|------------|
| `occlusion` | Occlusion zone active | `risk` (0-1) |
| `pedestrian_reveal` | Pedestrian suddenly appears | `initial_distance`, `ped_speed` |
| `pedestrian_crossing` | Pedestrian crosses road | `initial_distance`, `ped_speed` |
| `social_cue` | Adjacent vehicle brakes | - |
| `truck_departure` | Truck leaves, reveals area | `initial_occlusion` |

## Running Verification

### Option 1: Use JSON traces (no CARLA)
```bash
python stl_verification.py --trace-dir traces --no-synthetic
```

### Option 2: Use built-in synthetic generation
```bash
python stl_verification.py --synthetic
```

### Option 3: Specific scenarios only
```bash
python stl_verification.py --scenarios S1 S4 S5 --trace-dir traces
```

## Scenario Summary

| Scenario | STL Focus | Key Challenge |
|----------|-----------|---------------|
| S1 | φ₂ Occlusion | Parked car occludes pedestrian |
| S2 | φ₃ Social | Adjacent vehicle brakes before ped visible |
| S3 | φ₁ Collision | Sudden jaywalker, no warning |
| S4 | φ₂ Occlusion | Truck departure reveals pedestrian late |
| S5 | φ₂ Occlusion | Two-stage emergence, distraction first |
| S6 | φ₃ Social | False social cue (brake for other reason) |
| S7 | φ₂ Occlusion | Oncoming vehicle creates moving occlusion |
| S8 | φ₂ Occlusion | Following truck at high speed |

## Output

Running verification generates these files in `stl_logs/`:
- `signals_*.csv` - Raw signal traces
- `robustness_*.csv` - Robustness values over time
- `summary_*.csv` - Final results per scenario
- `comparison_*.csv` - Baseline vs Aware comparison
