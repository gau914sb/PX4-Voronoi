#!/usr/bin/env python3
"""
Static Data Visualization for Voronoi Pursuit-Evasion
Creates various plots for data analysis

Usage:
    python3 plot_static.py merged_all_drones_20260212_143022.csv
    python3 plot_static.py --auto
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def load_data(csv_file):
    """Load merged CSV data"""
    print(f"📂 Loading data from {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"   {len(df)} rows, {len(df['drone_id'].unique())} drones")
    print(f"   Duration: {df['timestamp'].max():.2f}s")
    return df


def plot_trajectories_2d(df, output_dir):
    """Plot 2D projections of trajectories"""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # Get drone info
    pursuer_ids = sorted([int(d) for d in df[df['role'] == 'pursuer']['drone_id'].unique()])
    evader_id = int(df[df['role'] == 'evader']['drone_id'].unique()[0])
    
    pursuer_colors = ['b', 'g', 'm', 'c']
    
    # XY projection
    ax = axes[0]
    for i, drone_id in enumerate(pursuer_ids):
        drone_data = df[df['drone_id'] == drone_id].sort_values('timestamp')
        ax.plot(drone_data['x'], drone_data['y'], 
                f'{pursuer_colors[i]}-', label=f'Pursuer {drone_id}', alpha=0.7)
        # Mark start
        ax.plot(drone_data['x'].iloc[0], drone_data['y'].iloc[0], 
                f'{pursuer_colors[i]}*', markersize=15)
    
    evader_data = df[df['drone_id'] == evader_id].sort_values('timestamp')
    ax.plot(evader_data['x'], evader_data['y'], 
            'r-', label=f'Evader {evader_id}', linewidth=2, alpha=0.8)
    ax.plot(evader_data['x'].iloc[0], evader_data['y'].iloc[0], 
            'r*', markersize=18)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Projection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # XZ projection
    ax = axes[1]
    for i, drone_id in enumerate(pursuer_ids):
        drone_data = df[df['drone_id'] == drone_id].sort_values('timestamp')
        ax.plot(drone_data['x'], drone_data['z'], 
                f'{pursuer_colors[i]}-', label=f'Pursuer {drone_id}', alpha=0.7)
        ax.plot(drone_data['x'].iloc[0], drone_data['z'].iloc[0], 
                f'{pursuer_colors[i]}*', markersize=15)
    
    evader_data = df[df['drone_id'] == evader_id].sort_values('timestamp')
    ax.plot(evader_data['x'], evader_data['z'], 
            'r-', label=f'Evader {evader_id}', linewidth=2, alpha=0.8)
    ax.plot(evader_data['x'].iloc[0], evader_data['z'].iloc[0], 
            'r*', markersize=18)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('XZ Projection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # YZ projection
    ax = axes[2]
    for i, drone_id in enumerate(pursuer_ids):
        drone_data = df[df['drone_id'] == drone_id].sort_values('timestamp')
        ax.plot(drone_data['y'], drone_data['z'], 
                f'{pursuer_colors[i]}-', label=f'Pursuer {drone_id}', alpha=0.7)
        ax.plot(drone_data['y'].iloc[0], drone_data['z'].iloc[0], 
                f'{pursuer_colors[i]}*', markersize=15)
    
    evader_data = df[df['drone_id'] == evader_id].sort_values('timestamp')
    ax.plot(evader_data['y'], evader_data['z'], 
            'r-', label=f'Evader {evader_id}', linewidth=2, alpha=0.8)
    ax.plot(evader_data['y'].iloc[0], evader_data['z'].iloc[0], 
            'r*', markersize=18)
    
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('YZ Projection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = output_dir / 'trajectories_2d.png'
    plt.savefig(output_file, dpi=150)
    print(f"   Saved: {output_file}")
    plt.close()


def plot_distances(df, output_dir):
    """Plot distances from each pursuer to evader over time"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    evader_id = int(df[df['role'] == 'evader']['drone_id'].unique()[0])
    pursuer_ids = sorted([int(d) for d in df[df['role'] == 'pursuer']['drone_id'].unique()])
    
    pursuer_colors = ['b', 'g', 'm', 'c']
    
    for i, pursuer_id in enumerate(pursuer_ids):
        distances = []
        timestamps = []
        
        for t in sorted(df['timestamp'].unique()):
            pursuer_data = df[(df['drone_id'] == pursuer_id) & (df['timestamp'] == t)]
            evader_data = df[(df['drone_id'] == evader_id) & (df['timestamp'] == t)]
            
            if not pursuer_data.empty and not evader_data.empty:
                p_pos = pursuer_data[['x', 'y', 'z']].values[0]
                e_pos = evader_data[['x', 'y', 'z']].values[0]
                distance = np.linalg.norm(p_pos - e_pos)
                distances.append(distance)
                timestamps.append(t)
        
        ax.plot(timestamps, distances, f'{pursuer_colors[i]}-', 
                label=f'Pursuer {pursuer_id}', linewidth=2)
    
    # Add capture radius line
    ax.axhline(y=0.4, color='r', linestyle='--', label='Capture radius', alpha=0.5)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Distance from Each Pursuer to Evader')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = output_dir / 'distances.png'
    plt.savefig(output_file, dpi=150)
    print(f"   Saved: {output_file}")
    plt.close()


def plot_velocities(df, output_dir):
    """Plot velocity magnitudes over time"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    pursuer_ids = sorted([int(d) for d in df[df['role'] == 'pursuer']['drone_id'].unique()])
    evader_id = int(df[df['role'] == 'evader']['drone_id'].unique()[0])
    
    pursuer_colors = ['b', 'g', 'm', 'c']
    
    for i, drone_id in enumerate(pursuer_ids):
        drone_data = df[df['drone_id'] == drone_id].sort_values('timestamp')
        vel_mag = np.sqrt(drone_data['vx']**2 + drone_data['vy']**2 + drone_data['vz']**2)
        ax.plot(drone_data['timestamp'], vel_mag, 
                f'{pursuer_colors[i]}-', label=f'Pursuer {drone_id}', alpha=0.7)
    
    evader_data = df[df['drone_id'] == evader_id].sort_values('timestamp')
    vel_mag = np.sqrt(evader_data['vx']**2 + evader_data['vy']**2 + evader_data['vz']**2)
    ax.plot(evader_data['timestamp'], vel_mag, 
            'r-', label=f'Evader {evader_id}', linewidth=2, alpha=0.8)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity Magnitude (m/s)')
    ax.set_title('Velocity Magnitudes Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = output_dir / 'velocities.png'
    plt.savefig(output_file, dpi=150)
    print(f"   Saved: {output_file}")
    plt.close()


def plot_positions_vs_time(df, output_dir):
    """Plot x, y, z positions over time"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    pursuer_ids = sorted([int(d) for d in df[df['role'] == 'pursuer']['drone_id'].unique()])
    evader_id = int(df[df['role'] == 'evader']['drone_id'].unique()[0])
    
    pursuer_colors = ['b', 'g', 'm', 'c']
    coords = ['x', 'y', 'z']
    labels = ['X Position', 'Y Position', 'Z Position']
    
    for coord_idx, (coord, label) in enumerate(zip(coords, labels)):
        ax = axes[coord_idx]
        
        for i, drone_id in enumerate(pursuer_ids):
            drone_data = df[df['drone_id'] == drone_id].sort_values('timestamp')
            ax.plot(drone_data['timestamp'], drone_data[coord], 
                    f'{pursuer_colors[i]}-', label=f'Pursuer {drone_id}', alpha=0.7)
        
        evader_data = df[df['drone_id'] == evader_id].sort_values('timestamp')
        ax.plot(evader_data['timestamp'], evader_data[coord], 
                'r-', label=f'Evader {evader_id}', linewidth=2, alpha=0.8)
        
        ax.set_ylabel(f'{label} (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title('Position Components Over Time')
    
    plt.tight_layout()
    output_file = output_dir / 'positions_vs_time.png'
    plt.savefig(output_file, dpi=150)
    print(f"   Saved: {output_file}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Create static plots for analysis',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'input_file',
        nargs='?',
        help='Merged CSV file to visualize'
    )
    parser.add_argument(
        '--auto',
        action='store_true',
        help='Auto-detect latest merged CSV file'
    )
    parser.add_argument(
        '--data-dir',
        type=str,
        default='data',
        help='Data directory'
    )
    
    args = parser.parse_args()
    
    data_dir = Path(__file__).parent / args.data_dir
    
    # Determine input file
    if args.auto:
        merged_files = list(data_dir.glob('merged_all_drones_*.csv'))
        if not merged_files:
            print(f"❌ No merged CSV files found in {data_dir}")
            sys.exit(1)
        input_file = max(merged_files, key=lambda p: p.stat().st_mtime)
        print(f"🔍 Auto-detected: {input_file.name}")
    elif args.input_file:
        input_file = Path(args.input_file)
    else:
        print("❌ Error: Provide input file or use --auto")
        parser.print_help()
        sys.exit(1)
    
    if not input_file.exists():
        print(f"❌ Error: File not found: {input_file}")
        sys.exit(1)
    
    # Load data
    df = load_data(input_file)
    
    # Create plots
    print(f"\n📊 Generating plots...")
    plot_trajectories_2d(df, data_dir)
    plot_distances(df, data_dir)
    plot_velocities(df, data_dir)
    plot_positions_vs_time(df, data_dir)
    
    print(f"\n✅ All plots generated in {data_dir}/")


if __name__ == '__main__':
    main()
