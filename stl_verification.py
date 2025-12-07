"""
STL Verification for Occlusion-Aware Perception System
Uses RTAMT library for Signal Temporal Logic evaluation

This script:
1. Loads signal traces from simulation logs
2. Evaluates STL specifications using RTAMT
3. Computes robustness values at each timestep
4. Generates comprehensive CSV logs for graph creation

Author: Occlusion-Aware Perception Project
"""

import rtamt
import csv
import json
import os
import numpy as np
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Dict, Tuple, Optional
import argparse


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class STLConfig:
    """Configuration for STL verification."""
    sampling_period: float = 0.05  # 20 Hz
    time_unit: str = 's'
    
    # STL Specification thresholds
    min_ped_distance: float = 0.5      # φ₁: meters
    occlusion_threshold: float = 0.5    # φ₂: risk level
    speed_reduction_factor: float = 0.5 # φ₂: reduce to half
    occlusion_response_time: float = 2.0 # φ₂: seconds
    social_cue_response_time: float = 1.0 # φ₃: seconds
    emergency_distance: float = 15.0    # φ₄: meters
    emergency_stop_time: float = 3.0    # φ₄: seconds
    stop_speed: float = 0.5             # φ₄: m/s
    comfort_decel: float = -3.0         # φ₅: m/s²
    liveness_distance: float = 10.0     # φ₆: meters
    liveness_time: float = 60.0         # φ₆: seconds


# =============================================================================
# STL Specifications
# =============================================================================

STL_SPECIFICATIONS = {
    'phi1_collision': {
        'name': 'Collision Avoidance',
        'formula': 'always(d_ped >= 0.5)',
        'description': 'Always maintain at least 0.5m from any pedestrian',
        'priority': 1,
        'critical': True
    },
    'phi2_occlusion': {
        'name': 'Occlusion Response',
        'formula': 'always((r_occ >= 0.5) implies (eventually[0:40](v <= v_target * 0.5)))',
        'description': 'When occlusion risk ≥50%, reduce speed to half within 2s',
        'priority': 2,
        'critical': True
    },
    'phi3_social': {
        'name': 'Social Cue Response',
        'formula': 'always((adj_brake > 0.5) implies (eventually[0:20](a < 0)))',
        'description': 'If adjacent vehicle brakes, begin braking within 1s',
        'priority': 3,
        'critical': False
    },
    'phi4_emergency': {
        'name': 'Emergency Stop',
        'formula': 'always(((ped_in_path > 0.5) and (d_ped <= 15)) implies (eventually[0:60](v <= 0.5)))',
        'description': 'If pedestrian in path within 15m, stop within 3s',
        'priority': 4,
        'critical': True
    },
    'phi5_comfort': {
        'name': 'Comfort Constraint',
        'formula': 'always((not emergency) implies (a >= -3))',
        'description': 'Maintain comfortable braking except in emergencies',
        'priority': 5,
        'critical': False
    },
    'phi6_liveness': {
        'name': 'Liveness/Progress',
        'formula': 'always(eventually[0:1200](delta_pos > 10))',
        'description': 'Make at least 10m progress every 60s',
        'priority': 6,
        'critical': True
    }
}


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class SignalSample:
    """Single timestep of signal data."""
    time: float
    v: float              # Speed (m/s)
    d_ped: float          # Distance to nearest pedestrian (m)
    r_occ: float          # Occlusion risk [0,1]
    a: float              # Acceleration (m/s²)
    adj_brake: float      # Adjacent vehicle braking (0/1)
    ped_in_path: float    # Pedestrian in path (0/1)
    delta_pos: float      # Position change since start (m)
    v_target: float       # Target speed (m/s)
    emergency: float      # Emergency situation (0/1)
    x: float = 0.0        # X position
    y: float = 0.0        # Y position


@dataclass
class RobustnessResult:
    """Result of STL evaluation at a timestep."""
    time: float
    spec_name: str
    robustness: float
    satisfied: bool


@dataclass
class ScenarioResult:
    """Complete result for one scenario."""
    scenario_id: str
    controller_type: str
    final_robustness: Dict[str, float]
    min_robustness: Dict[str, float]
    max_robustness: Dict[str, float]
    all_satisfied: bool
    violations: List[str]
    timestep_robustness: Dict[str, List[Tuple[float, float]]]


# =============================================================================
# RTAMT Monitor
# =============================================================================

class STLMonitor:
    """RTAMT-based STL monitor for occlusion-aware controller."""
    
    def __init__(self, config: STLConfig = None):
        self.config = config or STLConfig()
        self.specs = {}
        self._setup_specs()
    
    def _setup_specs(self):
        """Initialize STL specifications."""
        # Store spec definitions - we'll create fresh monitors for each evaluation
        self.specs = {
            'phi1_collision': {
                'vars': [('d_ped', 'float')],
                'formula': 'always(d_ped >= 0.5)'
            },
            'phi2_occlusion': {
                'vars': [('r_occ', 'float'), ('v', 'float'), ('v_half', 'float')],
                'formula': 'always((r_occ >= 0.5) implies (eventually[0:40](v <= v_half)))'
            },
            'phi3_social': {
                'vars': [('adj_brake', 'float'), ('a', 'float')],
                'formula': 'always((adj_brake > 0.5) implies (eventually[0:20](a < 0)))'
            },
            'phi4_emergency': {
                'vars': [('ped_in_path', 'float'), ('d_ped', 'float'), ('v', 'float')],
                'formula': 'always(((ped_in_path > 0.5) and (d_ped <= 15)) implies (eventually[0:60](v <= 0.5)))'
            },
            'phi5_comfort': {
                'vars': [('emergency', 'float'), ('a', 'float')],
                'formula': 'always((emergency < 0.5) implies (a >= -3))'
            },
            'phi6_liveness': {
                'vars': [('delta_pos', 'float')],
                'formula': 'always(eventually[0:1200](delta_pos > 10))'
            }
        }
    
    def _create_monitor(self, spec_name: str):
        """Create a fresh RTAMT monitor for a specification."""
        spec_def = self.specs.get(spec_name)
        if not spec_def:
            return None
        
        try:
            monitor = rtamt.StlDiscreteTimeSpecification()
            for var_name, var_type in spec_def['vars']:
                monitor.declare_var(var_name, var_type)
            monitor.spec = spec_def['formula']
            monitor.parse()
            return monitor
        except Exception as e:
            print(f"Warning: Could not create monitor for {spec_name}: {e}")
            return None
    
    def _build_signal_lists(self, trace: List[SignalSample], spec_name: str) -> Dict[str, List]:
        """Build signal lists in RTAMT format: {'var': [(t, val), ...]}"""
        signals = {}
        
        if spec_name == 'phi1_collision':
            signals['d_ped'] = [(s.time, s.d_ped) for s in trace]
        elif spec_name == 'phi2_occlusion':
            signals['r_occ'] = [(s.time, s.r_occ) for s in trace]
            signals['v'] = [(s.time, s.v) for s in trace]
            signals['v_half'] = [(s.time, s.v_target * 0.5) for s in trace]
        elif spec_name == 'phi3_social':
            signals['adj_brake'] = [(s.time, s.adj_brake) for s in trace]
            signals['a'] = [(s.time, s.a) for s in trace]
        elif spec_name == 'phi4_emergency':
            signals['ped_in_path'] = [(s.time, s.ped_in_path) for s in trace]
            signals['d_ped'] = [(s.time, s.d_ped) for s in trace]
            signals['v'] = [(s.time, s.v) for s in trace]
        elif spec_name == 'phi5_comfort':
            signals['emergency'] = [(s.time, s.emergency) for s in trace]
            signals['a'] = [(s.time, s.a) for s in trace]
        elif spec_name == 'phi6_liveness':
            signals['delta_pos'] = [(s.time, s.delta_pos) for s in trace]
        
        return signals
    
    def evaluate_trace(self, trace: List[SignalSample]) -> Dict[str, List[RobustnessResult]]:
        """
        Evaluate all specifications on a signal trace using offline evaluation.
        
        Returns dict mapping spec_name -> list of RobustnessResult for each timestep.
        """
        results = {}
        
        for spec_name in self.specs.keys():
            spec_results = []
            
            try:
                # Create fresh monitor
                monitor = self._create_monitor(spec_name)
                if monitor is None:
                    continue
                
                # Build signal lists
                signals = self._build_signal_lists(trace, spec_name)
                
                # Use offline evaluation - evaluate entire trace at once
                robustness_trace = monitor.evaluate(*[signals[var] for var, _ in self.specs[spec_name]['vars']])
                
                # robustness_trace is list of (time, robustness) tuples
                for t, rho in robustness_trace:
                    spec_results.append(RobustnessResult(
                        time=t,
                        spec_name=spec_name,
                        robustness=rho,
                        satisfied=(rho >= 0)
                    ))
                    
            except Exception as e:
                print(f"Warning: Error evaluating {spec_name}: {e}")
                # Fall back to simple direct computation
                spec_results = self._fallback_evaluation(trace, spec_name)
            
            results[spec_name] = spec_results
        
        return results
    
    def _fallback_evaluation(self, trace: List[SignalSample], spec_name: str) -> List[RobustnessResult]:
        """Simple direct robustness computation without RTAMT."""
        results = []
        
        for sample in trace:
            if spec_name == 'phi1_collision':
                # ρ = d_ped - 0.5
                rho = sample.d_ped - 0.5
            elif spec_name == 'phi2_occlusion':
                # Simplified: if r_occ >= 0.5, check if v <= v_target/2
                if sample.r_occ >= 0.5:
                    rho = (sample.v_target * 0.5) - sample.v  # positive if speed is low enough
                else:
                    rho = 1.0  # antecedent false, so satisfied
            elif spec_name == 'phi3_social':
                # If adj_brake, check if a < 0
                if sample.adj_brake > 0.5:
                    rho = -sample.a  # positive if decelerating
                else:
                    rho = 1.0
            elif spec_name == 'phi4_emergency':
                # If ped_in_path and d_ped <= 15, eventually stop
                if sample.ped_in_path > 0.5 and sample.d_ped <= 15:
                    rho = 0.5 - sample.v  # positive if nearly stopped
                else:
                    rho = 1.0
            elif spec_name == 'phi5_comfort':
                # If not emergency, a >= -3
                if sample.emergency < 0.5:
                    rho = sample.a + 3  # positive if a >= -3
                else:
                    rho = 1.0
            elif spec_name == 'phi6_liveness':
                # delta_pos > 10
                rho = sample.delta_pos - 10
            else:
                rho = 0.0
            
            results.append(RobustnessResult(
                time=sample.time,
                spec_name=spec_name,
                robustness=rho,
                satisfied=(rho >= 0)
            ))
        
        return results
    
    def get_final_robustness(self, results: Dict[str, List[RobustnessResult]]) -> Dict[str, float]:
        """Get final robustness value for each specification."""
        final = {}
        for spec_name, spec_results in results.items():
            if spec_results:
                # Take minimum robustness (worst case) for global specs
                valid_values = [r.robustness for r in spec_results if not np.isnan(r.robustness)]
                if valid_values:
                    final[spec_name] = min(valid_values)
                else:
                    final[spec_name] = float('nan')
            else:
                final[spec_name] = float('nan')
        return final
                valid_values = [r.robustness for r in spec_results if not np.isnan(r.robustness)]
                if valid_values:
                    final[spec_name] = min(valid_values)
                else:
                    final[spec_name] = float('nan')
            else:
                final[spec_name] = float('nan')
        return final


# =============================================================================
# Logging
# =============================================================================

class VerificationLogger:
    """Generates comprehensive logs for graph creation."""
    
    def __init__(self, output_dir: str = 'stl_logs'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    def log_signal_trace(self, scenario_id: str, controller: str, trace: List[SignalSample]):
        """Log raw signal trace to CSV."""
        filename = f"{self.output_dir}/signals_{scenario_id}_{controller}_{self.timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time', 'v', 'd_ped', 'r_occ', 'a', 'adj_brake', 
                'ped_in_path', 'delta_pos', 'v_target', 'emergency', 'x', 'y'
            ])
            for sample in trace:
                writer.writerow([
                    sample.time, sample.v, sample.d_ped, sample.r_occ,
                    sample.a, sample.adj_brake, sample.ped_in_path,
                    sample.delta_pos, sample.v_target, sample.emergency,
                    sample.x, sample.y
                ])
        
        print(f"  → Signal trace saved: {filename}")
        return filename
    
    def log_robustness_trace(self, scenario_id: str, controller: str, 
                             results: Dict[str, List[RobustnessResult]]):
        """Log robustness values over time to CSV."""
        filename = f"{self.output_dir}/robustness_{scenario_id}_{controller}_{self.timestamp}.csv"
        
        # Get all unique timestamps
        all_times = set()
        for spec_results in results.values():
            for r in spec_results:
                all_times.add(r.time)
        all_times = sorted(all_times)
        
        # Build header
        spec_names = list(results.keys())
        header = ['time'] + [f'{name}_rho' for name in spec_names] + [f'{name}_sat' for name in spec_names]
        
        # Build rows
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            
            for t in all_times:
                row = [t]
                # Robustness values
                for spec_name in spec_names:
                    spec_results = results.get(spec_name, [])
                    rho = next((r.robustness for r in spec_results if r.time == t), float('nan'))
                    row.append(rho)
                # Satisfaction values
                for spec_name in spec_names:
                    spec_results = results.get(spec_name, [])
                    sat = next((1 if r.satisfied else 0 for r in spec_results if r.time == t), 0)
                    row.append(sat)
                writer.writerow(row)
        
        print(f"  → Robustness trace saved: {filename}")
        return filename
    
    def log_summary(self, all_results: List[ScenarioResult]):
        """Log summary statistics across all scenarios."""
        filename = f"{self.output_dir}/summary_{self.timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'scenario_id', 'controller', 'all_satisfied',
                'phi1_rho', 'phi2_rho', 'phi3_rho', 'phi4_rho', 'phi5_rho', 'phi6_rho',
                'violations'
            ])
            
            for result in all_results:
                writer.writerow([
                    result.scenario_id,
                    result.controller_type,
                    result.all_satisfied,
                    result.final_robustness.get('phi1_collision', float('nan')),
                    result.final_robustness.get('phi2_occlusion', float('nan')),
                    result.final_robustness.get('phi3_social', float('nan')),
                    result.final_robustness.get('phi4_emergency', float('nan')),
                    result.final_robustness.get('phi5_comfort', float('nan')),
                    result.final_robustness.get('phi6_liveness', float('nan')),
                    ';'.join(result.violations)
                ])
        
        print(f"  → Summary saved: {filename}")
        return filename
    
    def log_comparison(self, baseline_results: List[ScenarioResult], 
                       aware_results: List[ScenarioResult]):
        """Log baseline vs aware comparison."""
        filename = f"{self.output_dir}/comparison_{self.timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'scenario_id', 'spec_name',
                'baseline_rho', 'aware_rho', 'improvement',
                'baseline_sat', 'aware_sat'
            ])
            
            for b_result, a_result in zip(baseline_results, aware_results):
                if b_result.scenario_id != a_result.scenario_id:
                    continue
                    
                for spec_name in ['phi1_collision', 'phi2_occlusion', 'phi3_social', 
                                  'phi4_emergency', 'phi5_comfort', 'phi6_liveness']:
                    b_rho = b_result.final_robustness.get(spec_name, float('nan'))
                    a_rho = a_result.final_robustness.get(spec_name, float('nan'))
                    
                    improvement = a_rho - b_rho if not (np.isnan(a_rho) or np.isnan(b_rho)) else float('nan')
                    
                    writer.writerow([
                        b_result.scenario_id,
                        spec_name,
                        b_rho,
                        a_rho,
                        improvement,
                        spec_name not in b_result.violations,
                        spec_name not in a_result.violations
                    ])
        
        print(f"  → Comparison saved: {filename}")
        return filename


# =============================================================================
# Simulation Data Loader
# =============================================================================

def load_trace_from_csv(filepath: str) -> List[SignalSample]:
    """Load signal trace from CSV file."""
    trace = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            trace.append(SignalSample(
                time=float(row.get('time', 0)),
                v=float(row.get('v', row.get('speed', 0))),
                d_ped=float(row.get('d_ped', row.get('ped_distance', 100))),
                r_occ=float(row.get('r_occ', row.get('occlusion_risk', 0))),
                a=float(row.get('a', row.get('acceleration', 0))),
                adj_brake=float(row.get('adj_brake', row.get('adjacent_braking', 0))),
                ped_in_path=float(row.get('ped_in_path', row.get('pedestrian_in_path', 0))),
                delta_pos=float(row.get('delta_pos', row.get('position_change', 0))),
                v_target=float(row.get('v_target', row.get('target_speed', 7.2))),
                emergency=float(row.get('emergency', 0)),
                x=float(row.get('x', 0)),
                y=float(row.get('y', 0))
            ))
    return trace


def load_trace_from_json(filepath: str) -> List[SignalSample]:
    """
    Load signal trace from JSON file.
    
    JSON format options:
    1. Full trace: {"trace": [{"time": 0.0, "v": 7.2, ...}, ...]}
    2. Scenario config: {"scenario_config": {...}, "events": [...]}
       The events-based format auto-generates the trace.
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Option 1: Full trace provided
    if 'trace' in data:
        trace = []
        for row in data['trace']:
            trace.append(SignalSample(
                time=float(row.get('time', 0)),
                v=float(row.get('v', row.get('speed', 7.2))),
                d_ped=float(row.get('d_ped', row.get('ped_distance', 100))),
                r_occ=float(row.get('r_occ', row.get('occlusion_risk', 0))),
                a=float(row.get('a', row.get('acceleration', 0))),
                adj_brake=float(row.get('adj_brake', 0)),
                ped_in_path=float(row.get('ped_in_path', 0)),
                delta_pos=float(row.get('delta_pos', 0)),
                v_target=float(row.get('v_target', 7.2)),
                emergency=float(row.get('emergency', 0)),
                x=float(row.get('x', 0)),
                y=float(row.get('y', 0))
            ))
        return trace
    
    # Option 2: Event-based scenario config - generate trace from events
    if 'scenario_config' in data:
        return generate_trace_from_events(data)
    
    raise ValueError(f"Unknown JSON format in {filepath}")


def generate_trace_from_events(data: dict) -> List[SignalSample]:
    """
    Generate trace from event-based scenario configuration.
    
    This allows defining scenarios declaratively without running CARLA.
    """
    config = data.get('scenario_config', {})
    events = data.get('events', [])
    controller = data.get('controller', 'baseline')
    
    duration = config.get('duration', 30.0)
    dt = config.get('dt', 0.05)
    v_target = config.get('v_target', 7.2)
    initial_speed = config.get('initial_speed', v_target)
    
    is_aware = controller == 'aware'
    
    trace = []
    t = 0.0
    v = initial_speed
    x, y = 0.0, 0.0
    
    # Sort events by time
    events = sorted(events, key=lambda e: e.get('time', 0))
    
    while t < duration:
        # Default values
        r_occ = 0.0
        d_ped = 100.0
        ped_in_path = 0.0
        adj_brake = 0.0
        emergency = 0.0
        
        # Apply active events
        for event in events:
            event_start = event.get('time', 0)
            event_duration = event.get('duration', 10.0)
            event_end = event_start + event_duration
            
            if event_start <= t <= event_end:
                event_type = event.get('type', '')
                progress = (t - event_start) / event_duration if event_duration > 0 else 1.0
                
                if event_type == 'occlusion':
                    r_occ = max(r_occ, event.get('risk', 0.7))
                
                elif event_type == 'pedestrian_reveal':
                    initial_distance = event.get('initial_distance', 20.0)
                    ped_speed = event.get('ped_speed', 1.5)
                    d_ped = min(d_ped, initial_distance - (t - event_start) * (v + ped_speed) * 0.3)
                    d_ped = max(0.0, d_ped)
                    ped_in_path = 1.0 if d_ped < 15 else 0.0
                
                elif event_type == 'pedestrian_crossing':
                    initial_distance = event.get('initial_distance', 15.0)
                    ped_speed = event.get('ped_speed', 1.5)
                    lateral_pos = (t - event_start) * ped_speed
                    if lateral_pos < 4.0:  # Still in path
                        d_ped = min(d_ped, initial_distance - (t - event_start) * v * 0.2)
                        d_ped = max(0.5, d_ped)
                        ped_in_path = 1.0
                    else:
                        ped_in_path = 0.0
                
                elif event_type == 'social_cue':
                    adj_brake = 1.0
                
                elif event_type == 'truck_departure':
                    # Truck leaving creates reveal
                    reveal_progress = min(1.0, (t - event_start) / 2.0)
                    r_occ = max(0.0, event.get('initial_occlusion', 0.9) * (1 - reveal_progress))
        
        # Controller response
        if is_aware:
            # Proactive occlusion-aware response
            if r_occ > 0.5:
                target_v = v_target * 0.5
            elif adj_brake > 0.5:
                target_v = v_target * 0.6
            elif ped_in_path > 0.5:
                if d_ped < 5:
                    target_v = 0.0
                elif d_ped < 10:
                    target_v = min(v, 2.0)
                else:
                    target_v = min(v, 4.0)
            else:
                target_v = v_target
            
            # Smooth comfortable deceleration
            if v > target_v:
                a = max(-2.5, (target_v - v) / 1.0)
            else:
                a = min(1.5, (target_v - v) / 2.0)
        else:
            # Baseline reactive controller
            if ped_in_path > 0.5 and d_ped < 5:
                a = -6.0  # Emergency brake
                emergency = 1.0
            elif ped_in_path > 0.5 and d_ped < 10:
                a = -4.0
            elif ped_in_path > 0.5 and d_ped < 15:
                a = -2.5
            else:
                a = (v_target - v) / 2.0
        
        # Update kinematics
        v = max(0, v + a * dt)
        x += v * dt
        
        if ped_in_path > 0.5 and d_ped < 3:
            emergency = 1.0
        
        trace.append(SignalSample(
            time=round(t, 3),
            v=round(v, 3),
            d_ped=round(d_ped, 3),
            r_occ=round(r_occ, 3),
            a=round(a, 3),
            adj_brake=adj_brake,
            ped_in_path=ped_in_path,
            delta_pos=round(x, 3),
            v_target=v_target,
            emergency=emergency,
            x=round(x, 3),
            y=round(y, 3)
        ))
        
        t += dt
    
    return trace


def load_trace(scenario_id: str, controller_type: str, trace_dir: str) -> Optional[List[SignalSample]]:
    """
    Load trace from file (JSON or CSV) if available.
    
    Looks for files in order:
    1. {trace_dir}/{scenario_id}_{controller_type}.json
    2. {trace_dir}/{scenario_id}_{controller_type}.csv
    
    Returns None if no file found.
    """
    # Try JSON first
    json_path = os.path.join(trace_dir, f"{scenario_id}_{controller_type}.json")
    if os.path.exists(json_path):
        print(f"    Loading from: {json_path}")
        return load_trace_from_json(json_path)
    
    # Try CSV
    csv_path = os.path.join(trace_dir, f"{scenario_id}_{controller_type}.csv")
    if os.path.exists(csv_path):
        print(f"    Loading from: {csv_path}")
        return load_trace_from_csv(csv_path)
    
    return None


def generate_synthetic_trace(scenario_id: str, controller_type: str, 
                             duration: float = 30.0, dt: float = 0.05) -> List[SignalSample]:
    """
    Generate realistic synthetic trace for each scenario.
    
    Scenarios:
    - S1: Sidewalk Occlusion (pedestrian behind parked car)
    - S2: Social Cue Integration (adjacent vehicle brakes)
    - S3: Jaywalking Pedestrian (sudden appearance)
    - S4: Late Reveal Behind Departing Truck
    - S5: Two-Stage Pedestrian Emergence
    - S6: False Social Cue (adjacent brakes for unrelated reason)
    - S7: Oncoming Vehicle Narrow Gap
    - S8: High-Speed Shadowing Truck
    """
    trace = []
    t = 0.0
    x, y = 0.0, 0.0
    v_target = 7.2  # 26 km/h default
    v = v_target
    
    is_aware = controller_type == 'aware'
    
    # Scenario-specific parameters
    if scenario_id == 'S1':
        # Sidewalk Occlusion: parked car occludes pedestrian
        occlusion_start, occlusion_end = 3.0, 12.0
        ped_reveal_time = 10.0
        ped_initial_distance = 12.0
        social_cue_time = -1
        
    elif scenario_id == 'S2':
        # Social Cue: adjacent vehicle brakes, pedestrian follows
        occlusion_start, occlusion_end = 5.0, 15.0
        ped_reveal_time = 12.0
        ped_initial_distance = 15.0
        social_cue_time = 8.0
        
    elif scenario_id == 'S3':
        # Jaywalking: sudden pedestrian appearance, no occlusion warning
        occlusion_start, occlusion_end = -1, -1
        ped_reveal_time = 8.0
        ped_initial_distance = 10.0  # Very close!
        social_cue_time = -1
        
    elif scenario_id == 'S4':
        # Late Reveal Behind Departing Truck
        # Truck departs at t=5, reveals pedestrian at t=7
        occlusion_start, occlusion_end = 0.0, 7.0  # High occlusion until truck leaves
        ped_reveal_time = 7.0
        ped_initial_distance = 8.0  # Very close when revealed
        social_cue_time = -1
        v_target = 8.33  # 30 km/h
        
    elif scenario_id == 'S5':
        # Two-Stage Emergence: first ped distracts, second ped is the threat
        occlusion_start, occlusion_end = 2.0, 20.0
        ped_reveal_time = 15.0  # Second pedestrian
        ped_initial_distance = 6.0  # Very close
        social_cue_time = -1
        # First pedestrian appears at t=8, safely on sidewalk
        
    elif scenario_id == 'S6':
        # False Social Cue: adjacent brakes but no pedestrian threat
        occlusion_start, occlusion_end = 4.0, 18.0
        ped_reveal_time = 25.0  # Pedestrian appears late, not a threat
        ped_initial_distance = 20.0
        social_cue_time = 6.0  # Brake happens but unrelated
        
    elif scenario_id == 'S7':
        # Oncoming Vehicle Narrow Gap
        occlusion_start, occlusion_end = 5.0, 12.0
        ped_reveal_time = 10.0
        ped_initial_distance = 10.0
        social_cue_time = -1
        v_target = 5.56  # 20 km/h narrow street
        
    elif scenario_id == 'S8':
        # High-Speed Shadowing Truck
        occlusion_start, occlusion_end = 0.0, 25.0  # Continuous occlusion
        ped_reveal_time = 18.0
        ped_initial_distance = 12.0
        social_cue_time = -1
        v_target = 11.11  # 40 km/h
        
    else:
        # Default scenario
        occlusion_start, occlusion_end = 5.0, 15.0
        ped_reveal_time = 10.0
        ped_initial_distance = 15.0
        social_cue_time = -1
    
    v = v_target  # Start at target speed
    
    while t < duration:
        # Compute occlusion risk
        if occlusion_start <= t <= occlusion_end:
            if scenario_id == 'S4':
                # Truck departing - occlusion decreases after t=5
                if t < 5:
                    r_occ = 0.9
                else:
                    r_occ = max(0.0, 0.9 - (t - 5) * 0.45)  # Drops over 2s
            elif scenario_id == 'S8':
                # Shadowing truck - persistent high occlusion
                r_occ = 0.8 + 0.1 * np.sin(t * 0.3)
            else:
                r_occ = 0.6 + 0.2 * np.sin(t * 0.5)
        else:
            r_occ = 0.1
        
        # Compute pedestrian distance
        if t >= ped_reveal_time:
            ped_time = t - ped_reveal_time
            
            if scenario_id == 'S5' and 8.0 <= t < 12.0:
                # First pedestrian (distraction) - on sidewalk, not threat
                d_ped = 8.0  # Safe distance
                ped_in_path = 0.0
            else:
                # Main pedestrian threat
                ped_speed = 1.5  # m/s walking
                closing_rate = v + ped_speed * 0.3  # Relative approach
                d_ped = max(0.3, ped_initial_distance - ped_time * closing_rate * 0.3)
                ped_in_path = 1.0 if d_ped < 15 else 0.0
        else:
            d_ped = 50.0
            ped_in_path = 0.0
        
        # Social cue
        if social_cue_time > 0 and social_cue_time <= t <= social_cue_time + 2.5:
            adj_brake = 1.0
        else:
            adj_brake = 0.0
        
        # Controller response
        if is_aware:
            # Occlusion-aware proactive controller
            if r_occ > 0.5:
                target_v = v_target * 0.5  # Reduce speed in occlusion
            elif adj_brake > 0.5 and scenario_id != 'S6':
                # Respond to social cue (except in false cue scenario we're more cautious)
                target_v = v_target * 0.6
            elif adj_brake > 0.5 and scenario_id == 'S6':
                # In S6, aware controller still slows but less aggressively
                target_v = v_target * 0.8
            elif ped_in_path > 0.5:
                if d_ped < 5:
                    target_v = 0.0
                elif d_ped < 10:
                    target_v = min(v, 2.0)
                else:
                    target_v = min(v, 4.0)
            else:
                target_v = v_target
            
            # Smooth comfortable deceleration
            if v > target_v:
                a = max(-2.5, (target_v - v) / 1.0)
            else:
                a = min(1.5, (target_v - v) / 2.0)
        else:
            # Baseline reactive controller - only reacts to visible pedestrians
            if ped_in_path > 0.5 and d_ped < 5:
                a = -6.0  # Emergency brake
            elif ped_in_path > 0.5 and d_ped < 10:
                a = -4.5
            elif ped_in_path > 0.5 and d_ped < 15:
                a = -2.5
            elif adj_brake > 0.5:
                # Baseline may react to social cue but delayed
                a = -1.5
            else:
                a = (v_target - v) / 2.0
        
        # Update velocity
        v = max(0, v + a * dt)
        
        # Update position
        x += v * dt
        
        # Emergency flag
        emergency = 1.0 if (ped_in_path > 0.5 and d_ped < 3) else 0.0
        
        # Create sample
        trace.append(SignalSample(
            time=t,
            v=v,
            d_ped=d_ped,
            r_occ=r_occ,
            a=a,
            adj_brake=adj_brake,
            ped_in_path=ped_in_path,
            delta_pos=x,
            v_target=v_target,
            emergency=emergency,
            x=x,
            y=y
        ))
        
        t += dt
    
    return trace


# =============================================================================
# Main Verification Pipeline
# =============================================================================

def run_verification(scenarios: List[str] = None, 
                     use_synthetic: bool = True,
                     trace_dir: str = None,
                     output_dir: str = 'stl_logs'):
    """
    Run complete STL verification pipeline.
    
    Args:
        scenarios: List of scenario IDs to verify
        use_synthetic: If True, generate synthetic traces; otherwise load from files
        trace_dir: Directory containing trace JSON/CSV files
        output_dir: Directory for output logs
    
    Trace Loading Priority:
    1. If use_synthetic=False, tries to load from {trace_dir}/{scenario}_{controller}.json
    2. Falls back to {trace_dir}/{scenario}_{controller}.csv
    3. Falls back to synthetic generation if no file found
    """
    
    if scenarios is None:
        scenarios = ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8']
    
    if trace_dir is None:
        trace_dir = 'traces'
    
    monitor = STLMonitor()
    logger = VerificationLogger(output_dir)
    
    baseline_results = []
    aware_results = []
    
    print("=" * 60)
    print("STL Verification for Occlusion-Aware Perception")
    print("=" * 60)
    print(f"Scenarios: {', '.join(scenarios)}")
    print(f"Trace source: {'Synthetic' if use_synthetic else f'Files in {trace_dir}/'}")
    print(f"Output: {output_dir}/")
    print("=" * 60)
    
    for scenario_id in scenarios:
        print(f"\n[{scenario_id}] Running verification...")
        
        for controller_type in ['baseline', 'aware']:
            print(f"  Controller: {controller_type}")
            
            # Get trace
            trace = None
            if not use_synthetic:
                trace = load_trace(scenario_id, controller_type, trace_dir)
            
            if trace is None:
                if not use_synthetic:
                    print(f"    No trace file found, using synthetic generation")
                trace = generate_synthetic_trace(scenario_id, controller_type)
            
            # Log signal trace
            logger.log_signal_trace(scenario_id, controller_type, trace)
            
            # Evaluate STL specifications
            results = monitor.evaluate_trace(trace)
            
            # Log robustness trace
            logger.log_robustness_trace(scenario_id, controller_type, results)
            
            # Compute summary
            final_rho = monitor.get_final_robustness(results)
            
            violations = [name for name, rho in final_rho.items() 
                         if not np.isnan(rho) and rho < 0]
            
            scenario_result = ScenarioResult(
                scenario_id=scenario_id,
                controller_type=controller_type,
                final_robustness=final_rho,
                min_robustness={k: min(r.robustness for r in v) if v else float('nan') 
                               for k, v in results.items()},
                max_robustness={k: max(r.robustness for r in v) if v else float('nan') 
                               for k, v in results.items()},
                all_satisfied=len(violations) == 0,
                violations=violations,
                timestep_robustness={k: [(r.time, r.robustness) for r in v] 
                                    for k, v in results.items()}
            )
            
            if controller_type == 'baseline':
                baseline_results.append(scenario_result)
            else:
                aware_results.append(scenario_result)
            
            # Print summary
            status = "✓ ALL PASS" if scenario_result.all_satisfied else f"✗ VIOLATIONS: {violations}"
            print(f"    Status: {status}")
            for spec_name, rho in final_rho.items():
                print(f"      {spec_name}: ρ = {rho:+.2f}")
    
    # Log summaries
    print("\n" + "=" * 60)
    print("Generating summary logs...")
    logger.log_summary(baseline_results + aware_results)
    logger.log_comparison(baseline_results, aware_results)
    
    print("\n" + "=" * 60)
    print("Verification complete!")
    print(f"Logs saved to: {output_dir}/")
    
    return baseline_results, aware_results


# =============================================================================
# Graph Suggestions
# =============================================================================

GRAPH_IDEAS = """
================================================================================
SUGGESTED GRAPHS FOR STL VERIFICATION RESULTS
================================================================================

Based on the generated logs, you can create the following graphs:

1. ROBUSTNESS TIME SERIES (per scenario)
   - File: robustness_S*_*.csv
   - X-axis: Time (s)
   - Y-axis: Robustness value (ρ)
   - Lines: One per specification (φ₁-φ₆)
   - Compare: Baseline vs Aware overlaid
   - Highlight: ρ < 0 regions (violations)

2. ROBUSTNESS COMPARISON BAR CHART
   - File: comparison_*.csv
   - X-axis: Specification (φ₁-φ₆)
   - Y-axis: Final robustness (ρ)
   - Bars: Baseline vs Aware (grouped)
   - Across: All scenarios (faceted)

3. SPECIFICATION SATISFACTION HEATMAP
   - File: summary_*.csv
   - Rows: Scenarios (S1-S8)
   - Columns: Specifications (φ₁-φ₆)
   - Color: Green (ρ > 0), Red (ρ < 0), Yellow (marginal)
   - Split: Baseline vs Aware

4. SIGNAL TRACE PLOTS (per scenario)
   - File: signals_S*_*.csv
   - Multi-panel:
     a) Speed (v) over time
     b) Pedestrian distance (d_ped) over time
     c) Occlusion risk (r_occ) over time
     d) Acceleration (a) over time
   - Annotations: Key events (pedestrian appears, occlusion starts)

5. IMPROVEMENT WATERFALL CHART
   - File: comparison_*.csv
   - X-axis: Specification
   - Y-axis: Robustness improvement (aware - baseline)
   - Bars: Positive (green), Negative (red)

6. SAFETY MARGIN DISTRIBUTION
   - File: robustness_*.csv
   - Histogram of robustness values
   - Faceted by: Specification and Controller type
   - Vertical line: ρ = 0 boundary

7. PHASE DIAGRAM (d_ped vs v)
   - File: signals_*.csv
   - X-axis: Speed (v)
   - Y-axis: Pedestrian distance (d_ped)
   - Trajectory: Time evolution
   - Overlay: Safe stopping distance curve

8. OCCLUSION RESPONSE TIMING
   - File: signals_*.csv + robustness_*.csv
   - X-axis: Time since occlusion detected
   - Y-axis: Speed reduction achieved
   - Compare: Baseline (flat) vs Aware (responsive)

9. AGGREGATE RADAR/SPIDER CHART
   - File: summary_*.csv
   - Axes: φ₁-φ₆ (normalized robustness)
   - Polygons: Baseline (inner) vs Aware (outer)
   - Per scenario or aggregated

10. VIOLATION TIMELINE
    - File: robustness_*.csv
    - X-axis: Time
    - Y-axis: Scenarios (stacked)
    - Markers: When ρ < 0 for any spec
    - Color: By specification

================================================================================
PYTHON CODE FOR BASIC GRAPHS (using matplotlib/seaborn)
================================================================================
"""


def print_graph_ideas():
    """Print suggestions for graphs to create."""
    print(GRAPH_IDEAS)


# =============================================================================
# Entry Point
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='STL Verification for Occlusion-Aware Perception',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with built-in synthetic traces (default)
  python stl_verification.py

  # Run using JSON trace files from traces/ directory
  python stl_verification.py --trace-dir traces --no-synthetic

  # Run specific scenarios only
  python stl_verification.py --scenarios S1 S4 S5 S8

  # Print graph creation ideas
  python stl_verification.py --graph-ideas
        """
    )
    parser.add_argument('--scenarios', nargs='+', default=None,
                        help='Scenario IDs to verify (default: S1-S8)')
    parser.add_argument('--synthetic', action='store_true', dest='synthetic',
                        help='Use synthetic trace generation (default)')
    parser.add_argument('--no-synthetic', action='store_false', dest='synthetic',
                        help='Load traces from JSON/CSV files instead of generating')
    parser.set_defaults(synthetic=True)
    parser.add_argument('--trace-dir', type=str, default='traces',
                        help='Directory containing trace JSON/CSV files (default: traces/)')
    parser.add_argument('--output-dir', type=str, default='stl_logs',
                        help='Output directory for logs (default: stl_logs/)')
    parser.add_argument('--graph-ideas', action='store_true',
                        help='Print graph creation ideas and exit')
    
    args = parser.parse_args()
    
    if args.graph_ideas:
        print_graph_ideas()
    else:
        run_verification(
            scenarios=args.scenarios,
            use_synthetic=args.synthetic,
            trace_dir=args.trace_dir,
            output_dir=args.output_dir
        )
        print("\n")
        print_graph_ideas()
