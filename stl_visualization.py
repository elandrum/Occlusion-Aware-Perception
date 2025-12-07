"""
Visualization Script for STL Verification Results
Generates comprehensive graphs from verification logs

Usage:
    python stl_visualization.py --log-dir stl_logs

Author: Occlusion-Aware Perception Project
"""

import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import argparse
from typing import List, Dict, Tuple

# Set style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 11
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 12


# =============================================================================
# Color Schemes
# =============================================================================

COLORS = {
    'baseline': '#E74C3C',  # Red
    'aware': '#27AE60',     # Green (Occlusion-Aware)
    'phi1': '#3498DB',      # Blue
    'phi2': '#9B59B6',      # Purple
    'phi3': '#F39C12',      # Orange
    'phi4': '#1ABC9C',      # Teal
    'phi5': '#E91E63',      # Pink
    'phi6': '#00BCD4',      # Cyan
    'pass': '#27AE60',
    'fail': '#E74C3C',
    'marginal': '#F39C12'
}

SPEC_NAMES = {
    'phi1_collision': 'φ₁ Collision',
    'phi2_occlusion': 'φ₂ Occlusion',
    'phi3_social': 'φ₃ Social Cue',
    'phi4_emergency': 'φ₄ Emergency',
    'phi5_comfort': 'φ₅ Comfort',
    'phi6_liveness': 'φ₆ Liveness'
}


# =============================================================================
# Data Loading
# =============================================================================

def load_summary(log_dir: str) -> pd.DataFrame:
    """Load summary CSV."""
    files = glob.glob(f"{log_dir}/summary_*.csv")
    if not files:
        raise FileNotFoundError(f"No summary files found in {log_dir}")
    return pd.read_csv(sorted(files)[-1])  # Latest


def load_comparison(log_dir: str) -> pd.DataFrame:
    """Load comparison CSV."""
    files = glob.glob(f"{log_dir}/comparison_*.csv")
    if not files:
        raise FileNotFoundError(f"No comparison files found in {log_dir}")
    return pd.read_csv(sorted(files)[-1])


def load_robustness_traces(log_dir: str) -> Dict[str, pd.DataFrame]:
    """Load all robustness trace CSVs."""
    traces = {}
    files = glob.glob(f"{log_dir}/robustness_*.csv")
    for f in files:
        # Extract scenario_controller from filename
        basename = os.path.basename(f)
        parts = basename.replace('.csv', '').split('_')
        if len(parts) >= 3:
            key = f"{parts[1]}_{parts[2]}"
            traces[key] = pd.read_csv(f)
    return traces


def load_signal_traces(log_dir: str) -> Dict[str, pd.DataFrame]:
    """Load all signal trace CSVs."""
    traces = {}
    files = glob.glob(f"{log_dir}/signals_*.csv")
    for f in files:
        basename = os.path.basename(f)
        parts = basename.replace('.csv', '').split('_')
        if len(parts) >= 3:
            key = f"{parts[1]}_{parts[2]}"
            traces[key] = pd.read_csv(f)
    return traces


# =============================================================================
# Graph 1: Robustness Time Series
# =============================================================================

def plot_robustness_timeseries(robustness_traces: Dict[str, pd.DataFrame],
                                scenario: str,
                                output_dir: str):
    """Plot robustness over time for a specific scenario."""
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'Robustness Time Series - Scenario {scenario}', fontsize=16, fontweight='bold')
    
    baseline_key = f"{scenario}_baseline"
    aware_key = f"{scenario}_aware"
    
    spec_cols = ['phi1_collision_rho', 'phi2_occlusion_rho', 'phi3_social_rho',
                 'phi4_emergency_rho', 'phi5_comfort_rho', 'phi6_liveness_rho']
    spec_labels = ['φ₁ Collision', 'φ₂ Occlusion', 'φ₃ Social', 
                   'φ₄ Emergency', 'φ₅ Comfort', 'φ₆ Liveness']
    
    for idx, (col, label) in enumerate(zip(spec_cols, spec_labels)):
        ax = axes[idx // 3, idx % 3]
        
        # Plot baseline
        if baseline_key in robustness_traces:
            df = robustness_traces[baseline_key]
            if col in df.columns:
                ax.plot(df['time'], df[col], color=COLORS['baseline'], 
                       label='Baseline', linewidth=2, alpha=0.8)
        
        # Plot aware
        if aware_key in robustness_traces:
            df = robustness_traces[aware_key]
            if col in df.columns:
                ax.plot(df['time'], df[col], color=COLORS['aware'],
                       label='Occlusion-Aware', linewidth=2, alpha=0.8)
        
        # Add violation region
        ax.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5)
        ax.axhspan(-10, 0, alpha=0.1, color='red', label='Violation Zone')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Robustness (ρ)')
        ax.set_title(label)
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = f"{output_dir}/graph1_robustness_timeseries_{scenario}.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 2: Robustness Comparison Bar Chart
# =============================================================================

def plot_robustness_comparison(comparison_df: pd.DataFrame, output_dir: str):
    """Bar chart comparing baseline vs aware robustness."""
    
    scenarios = comparison_df['scenario_id'].unique()
    specs = comparison_df['spec_name'].unique()
    
    fig, axes = plt.subplots(2, 4, figsize=(18, 10))
    fig.suptitle('Robustness Comparison: Baseline vs Occlusion-Aware', 
                 fontsize=16, fontweight='bold')
    
    for idx, scenario in enumerate(scenarios[:8]):
        ax = axes[idx // 4, idx % 4]
        
        scenario_data = comparison_df[comparison_df['scenario_id'] == scenario]
        
        x = np.arange(len(specs))
        width = 0.35
        
        baseline_vals = scenario_data['baseline_rho'].values
        aware_vals = scenario_data['aware_rho'].values
        
        bars1 = ax.bar(x - width/2, baseline_vals, width, label='Baseline', 
                       color=COLORS['baseline'], alpha=0.8)
        bars2 = ax.bar(x + width/2, aware_vals, width, label='Occlusion-Aware',
                       color=COLORS['aware'], alpha=0.8)
        
        ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        ax.set_xlabel('Specification')
        ax.set_ylabel('Robustness (ρ)')
        ax.set_title(f'Scenario {scenario}')
        ax.set_xticks(x)
        ax.set_xticklabels(['φ₁', 'φ₂', 'φ₃', 'φ₄', 'φ₅', 'φ₆'], fontsize=10)
        
        if idx == 0:
            ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3, axis='y')
    
    # Hide empty subplots
    for idx in range(len(scenarios), 8):
        axes[idx // 4, idx % 4].set_visible(False)
    
    plt.tight_layout()
    output_path = f"{output_dir}/graph2_robustness_comparison.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 3: Specification Satisfaction Heatmap
# =============================================================================

def plot_satisfaction_heatmap(summary_df: pd.DataFrame, output_dir: str):
    """Heatmap showing pass/fail for each spec and scenario."""
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Specification Satisfaction Heatmap', fontsize=16, fontweight='bold')
    
    for idx, controller in enumerate(['baseline', 'aware']):
        ax = axes[idx]
        
        ctrl_data = summary_df[summary_df['controller'] == controller]
        
        # Build matrix
        scenarios = ctrl_data['scenario_id'].unique()
        specs = ['phi1_rho', 'phi2_rho', 'phi3_rho', 'phi4_rho', 'phi5_rho', 'phi6_rho']
        spec_labels = ['φ₁', 'φ₂', 'φ₃', 'φ₄', 'φ₅', 'φ₆']
        
        matrix = np.zeros((len(scenarios), len(specs)))
        for i, scenario in enumerate(scenarios):
            row = ctrl_data[ctrl_data['scenario_id'] == scenario].iloc[0]
            for j, spec in enumerate(specs):
                val = row[spec]
                if pd.isna(val):
                    matrix[i, j] = 0
                elif val > 1:
                    matrix[i, j] = 2  # Strong pass
                elif val >= 0:
                    matrix[i, j] = 1  # Pass
                elif val >= -1:
                    matrix[i, j] = -1  # Weak fail
                else:
                    matrix[i, j] = -2  # Strong fail
        
        # Custom colormap
        cmap = LinearSegmentedColormap.from_list('custom', 
            ['#E74C3C', '#F5B041', '#F7DC6F', '#82E0AA', '#27AE60'], N=5)
        
        im = ax.imshow(matrix, cmap=cmap, aspect='auto', vmin=-2, vmax=2)
        
        ax.set_xticks(np.arange(len(specs)))
        ax.set_yticks(np.arange(len(scenarios)))
        ax.set_xticklabels(spec_labels)
        ax.set_yticklabels(scenarios)
        
        ax.set_xlabel('Specification')
        ax.set_ylabel('Scenario')
        title = 'Baseline Controller' if controller == 'baseline' else 'Occlusion-Aware Controller'
        ax.set_title(title)
        
        # Add text annotations
        for i in range(len(scenarios)):
            for j in range(len(specs)):
                val = matrix[i, j]
                text = '✓' if val >= 0 else '✗'
                color = 'white' if abs(val) > 1 else 'black'
                ax.text(j, i, text, ha='center', va='center', color=color, fontsize=12)
    
    plt.colorbar(im, ax=axes, label='Robustness Level', shrink=0.8)
    plt.tight_layout()
    output_path = f"{output_dir}/graph3_satisfaction_heatmap.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 4: Signal Trace Plots
# =============================================================================

def plot_signal_traces(signal_traces: Dict[str, pd.DataFrame], 
                       scenario: str, output_dir: str):
    """Multi-panel signal trace plot for a scenario."""
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f'Signal Traces - Scenario {scenario}', fontsize=16, fontweight='bold')
    
    baseline_key = f"{scenario}_baseline"
    aware_key = f"{scenario}_aware"
    
    signals = [
        ('v', 'Speed (m/s)', 'Speed'),
        ('d_ped', 'Distance (m)', 'Pedestrian Distance'),
        ('r_occ', 'Risk [0-1]', 'Occlusion Risk'),
        ('a', 'Accel (m/s²)', 'Acceleration')
    ]
    
    for idx, (col, ylabel, title) in enumerate(signals):
        ax = axes[idx]
        
        if baseline_key in signal_traces:
            df = signal_traces[baseline_key]
            if col in df.columns:
                ax.plot(df['time'], df[col], color=COLORS['baseline'],
                       label='Baseline', linewidth=2, alpha=0.8)
        
        if aware_key in signal_traces:
            df = signal_traces[aware_key]
            if col in df.columns:
                ax.plot(df['time'], df[col], color=COLORS['aware'],
                       label='Occlusion-Aware', linewidth=2, alpha=0.8)
        
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Add reference lines
        if col == 'd_ped':
            ax.axhline(y=0.5, color='red', linestyle='--', alpha=0.5, label='Min Safe (0.5m)')
            ax.axhline(y=15, color='orange', linestyle='--', alpha=0.5, label='Emergency Zone')
        elif col == 'r_occ':
            ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='Response Threshold')
        elif col == 'a':
            ax.axhline(y=-3, color='orange', linestyle='--', alpha=0.5, label='Comfort Limit')
    
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    output_path = f"{output_dir}/graph4_signal_traces_{scenario}.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 5: Improvement Waterfall Chart
# =============================================================================

def plot_improvement_waterfall(comparison_df: pd.DataFrame, output_dir: str):
    """Waterfall chart showing robustness improvement per spec."""
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Aggregate improvements across scenarios
    improvements = comparison_df.groupby('spec_name')['improvement'].mean()
    
    spec_labels = [SPEC_NAMES.get(s, s) for s in improvements.index]
    values = improvements.values
    
    colors = [COLORS['pass'] if v >= 0 else COLORS['fail'] for v in values]
    
    bars = ax.bar(spec_labels, values, color=colors, alpha=0.8, edgecolor='black')
    
    ax.axhline(y=0, color='black', linewidth=1)
    ax.set_xlabel('Specification')
    ax.set_ylabel('Mean Robustness Improvement (Aware - Baseline)')
    ax.set_title('Robustness Improvement by Specification', fontsize=14, fontweight='bold')
    
    # Add value labels
    for bar, val in zip(bars, values):
        height = bar.get_height()
        ax.annotate(f'{val:+.2f}',
                   xy=(bar.get_x() + bar.get_width() / 2, height),
                   xytext=(0, 3 if height >= 0 else -15),
                   textcoords="offset points",
                   ha='center', va='bottom' if height >= 0 else 'top',
                   fontsize=11, fontweight='bold')
    
    ax.grid(True, alpha=0.3, axis='y')
    plt.xticks(rotation=15)
    plt.tight_layout()
    output_path = f"{output_dir}/graph5_improvement_waterfall.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 6: Safety Margin Distribution
# =============================================================================

def plot_safety_margin_distribution(robustness_traces: Dict[str, pd.DataFrame],
                                    output_dir: str):
    """Histogram of robustness values by controller type."""
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Safety Margin Distribution (Robustness Values)', 
                 fontsize=16, fontweight='bold')
    
    spec_cols = ['phi1_collision_rho', 'phi2_occlusion_rho', 'phi3_social_rho',
                 'phi4_emergency_rho', 'phi5_comfort_rho', 'phi6_liveness_rho']
    spec_labels = ['φ₁ Collision', 'φ₂ Occlusion', 'φ₃ Social', 
                   'φ₄ Emergency', 'φ₅ Comfort', 'φ₆ Liveness']
    
    for idx, (col, label) in enumerate(zip(spec_cols, spec_labels)):
        ax = axes[idx // 3, idx % 3]
        
        # Collect all values
        baseline_vals = []
        aware_vals = []
        
        for key, df in robustness_traces.items():
            if col in df.columns:
                vals = df[col].dropna().values
                if 'baseline' in key:
                    baseline_vals.extend(vals)
                else:
                    aware_vals.extend(vals)
        
        if baseline_vals:
            ax.hist(baseline_vals, bins=30, alpha=0.6, color=COLORS['baseline'],
                   label='Baseline', density=True)
        if aware_vals:
            ax.hist(aware_vals, bins=30, alpha=0.6, color=COLORS['aware'],
                   label='Occlusion-Aware', density=True)
        
        ax.axvline(x=0, color='black', linestyle='--', linewidth=2, label='ρ=0 (boundary)')
        ax.set_xlabel('Robustness (ρ)')
        ax.set_ylabel('Density')
        ax.set_title(label)
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = f"{output_dir}/graph6_safety_margin_dist.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 7: Phase Diagram (d_ped vs v)
# =============================================================================

def plot_phase_diagram(signal_traces: Dict[str, pd.DataFrame], 
                       scenario: str, output_dir: str):
    """Phase diagram showing trajectory in (v, d_ped) space."""
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    baseline_key = f"{scenario}_baseline"
    aware_key = f"{scenario}_aware"
    
    # Plot safe stopping distance curve
    v_range = np.linspace(0, 10, 100)
    # Stopping distance = v² / (2 * decel), assuming decel = 3 m/s²
    stopping_dist = v_range ** 2 / (2 * 3) + 0.5  # Add safety margin
    ax.plot(v_range, stopping_dist, 'k--', linewidth=2, label='Safe Stopping Distance')
    ax.fill_between(v_range, 0, stopping_dist, alpha=0.1, color='red', label='Danger Zone')
    
    # Plot trajectories
    if baseline_key in signal_traces:
        df = signal_traces[baseline_key]
        if 'v' in df.columns and 'd_ped' in df.columns:
            ax.plot(df['v'], df['d_ped'], color=COLORS['baseline'],
                   linewidth=2, alpha=0.8, label='Baseline Trajectory')
            ax.scatter(df['v'].iloc[0], df['d_ped'].iloc[0], 
                      color=COLORS['baseline'], s=100, marker='o', zorder=5)
            ax.scatter(df['v'].iloc[-1], df['d_ped'].iloc[-1],
                      color=COLORS['baseline'], s=100, marker='s', zorder=5)
    
    if aware_key in signal_traces:
        df = signal_traces[aware_key]
        if 'v' in df.columns and 'd_ped' in df.columns:
            ax.plot(df['v'], df['d_ped'], color=COLORS['aware'],
                   linewidth=2, alpha=0.8, label='Occlusion-Aware Trajectory')
            ax.scatter(df['v'].iloc[0], df['d_ped'].iloc[0],
                      color=COLORS['aware'], s=100, marker='o', zorder=5)
            ax.scatter(df['v'].iloc[-1], df['d_ped'].iloc[-1],
                      color=COLORS['aware'], s=100, marker='s', zorder=5)
    
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('Pedestrian Distance (m)')
    ax.set_title(f'Phase Diagram - Scenario {scenario}', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 25)
    
    plt.tight_layout()
    output_path = f"{output_dir}/graph7_phase_diagram_{scenario}.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Graph 8: Aggregate Radar Chart
# =============================================================================

def plot_radar_chart(summary_df: pd.DataFrame, output_dir: str):
    """Radar/spider chart comparing aggregate robustness."""
    
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(polar=True))
    
    specs = ['phi1_rho', 'phi2_rho', 'phi3_rho', 'phi4_rho', 'phi5_rho', 'phi6_rho']
    labels = ['φ₁ Collision', 'φ₂ Occlusion', 'φ₃ Social', 
              'φ₄ Emergency', 'φ₅ Comfort', 'φ₆ Liveness']
    
    # Compute mean robustness per controller
    baseline_data = summary_df[summary_df['controller'] == 'baseline']
    aware_data = summary_df[summary_df['controller'] == 'aware']
    
    baseline_means = [baseline_data[s].mean() for s in specs]
    aware_means = [aware_data[s].mean() for s in specs]
    
    # Normalize to [0, 1] for visualization
    all_vals = baseline_means + aware_means
    max_val = max(abs(v) for v in all_vals if not np.isnan(v))
    if max_val > 0:
        baseline_norm = [(v / max_val + 1) / 2 for v in baseline_means]
        aware_norm = [(v / max_val + 1) / 2 for v in aware_means]
    else:
        baseline_norm = [0.5] * len(specs)
        aware_norm = [0.5] * len(specs)
    
    # Setup angles
    angles = np.linspace(0, 2 * np.pi, len(specs), endpoint=False).tolist()
    baseline_norm += baseline_norm[:1]  # Close the polygon
    aware_norm += aware_norm[:1]
    angles += angles[:1]
    
    ax.plot(angles, baseline_norm, 'o-', linewidth=2, color=COLORS['baseline'],
           label='Baseline', markersize=8)
    ax.fill(angles, baseline_norm, alpha=0.25, color=COLORS['baseline'])
    
    ax.plot(angles, aware_norm, 'o-', linewidth=2, color=COLORS['aware'],
           label='Occlusion-Aware', markersize=8)
    ax.fill(angles, aware_norm, alpha=0.25, color=COLORS['aware'])
    
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels, size=11)
    ax.set_ylim(0, 1)
    ax.set_title('Aggregate Robustness Comparison', fontsize=14, fontweight='bold', pad=20)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    
    plt.tight_layout()
    output_path = f"{output_dir}/graph8_radar_chart.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {output_path}")


# =============================================================================
# Main
# =============================================================================

def generate_all_graphs(log_dir: str, output_dir: str = None):
    """Generate all visualization graphs."""
    
    if output_dir is None:
        output_dir = f"{log_dir}/graphs"
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 60)
    print("Generating STL Verification Graphs")
    print("=" * 60)
    
    # Load data
    print("\nLoading data...")
    summary_df = load_summary(log_dir)
    comparison_df = load_comparison(log_dir)
    robustness_traces = load_robustness_traces(log_dir)
    signal_traces = load_signal_traces(log_dir)
    
    scenarios = summary_df['scenario_id'].unique()
    
    print(f"  Found {len(scenarios)} scenarios")
    print(f"  Found {len(robustness_traces)} robustness traces")
    print(f"  Found {len(signal_traces)} signal traces")
    
    # Generate graphs
    print("\nGenerating graphs...")
    
    # Graph 1: Robustness time series (per scenario)
    print("\n[1/8] Robustness Time Series...")
    for scenario in scenarios:
        plot_robustness_timeseries(robustness_traces, scenario, output_dir)
    
    # Graph 2: Robustness comparison bar chart
    print("\n[2/8] Robustness Comparison...")
    plot_robustness_comparison(comparison_df, output_dir)
    
    # Graph 3: Satisfaction heatmap
    print("\n[3/8] Satisfaction Heatmap...")
    plot_satisfaction_heatmap(summary_df, output_dir)
    
    # Graph 4: Signal traces (per scenario)
    print("\n[4/8] Signal Traces...")
    for scenario in scenarios:
        plot_signal_traces(signal_traces, scenario, output_dir)
    
    # Graph 5: Improvement waterfall
    print("\n[5/8] Improvement Waterfall...")
    plot_improvement_waterfall(comparison_df, output_dir)
    
    # Graph 6: Safety margin distribution
    print("\n[6/8] Safety Margin Distribution...")
    plot_safety_margin_distribution(robustness_traces, output_dir)
    
    # Graph 7: Phase diagrams (per scenario)
    print("\n[7/8] Phase Diagrams...")
    for scenario in scenarios:
        plot_phase_diagram(signal_traces, scenario, output_dir)
    
    # Graph 8: Radar chart
    print("\n[8/8] Radar Chart...")
    plot_radar_chart(summary_df, output_dir)
    
    print("\n" + "=" * 60)
    print(f"All graphs saved to: {output_dir}/")
    print("=" * 60)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate STL Verification Graphs')
    parser.add_argument('--log-dir', type=str, default='stl_logs',
                       help='Directory containing verification logs')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for graphs (default: log_dir/graphs)')
    
    args = parser.parse_args()
    generate_all_graphs(args.log_dir, args.output_dir)
