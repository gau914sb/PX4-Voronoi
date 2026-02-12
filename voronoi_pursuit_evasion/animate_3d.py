#!/usr/bin/env python3
"""
3D Trajectory Animation for Voronoi Pursuit-Evasion
Creates animated 3D visualization of the pursuit-evasion game

Usage:
    python3 animate_3d.py merged_all_drones_20260212_143022.csv
    python3 animate_3d.py --auto  # Auto-detect latest merged file
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter
from pathlib import Path
import sys


def load_data(csv_file):
    """Load merged CSV data"""
    print(f"📂 Loading data from {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"   {len(df)} rows, {len(df['drone_id'].unique())} drones")
    return df


def create_animation(df, output_file='animation.gif', fps=20, skip_frames=2):
    """
    Create 3D animated trajectory
    
    Args:
        df: DataFrame with columns [timestamp, drone_id, role, x, y, z, ...]
        output_file: Output GIF filename
        fps: Frames per second
        skip_frames: Skip every N frames to speed up
    """
    print(f"\n🎬 Creating 3D animation...")
    
    # Extract unique timestamps
    timestamps = sorted(df['timestamp'].unique())
    
    # Setup figure
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get data ranges for axis limits
    x_range = [df['x'].min(), df['x'].max()]
    y_range = [df['y'].min(), df['y'].max()]
    z_range = [df['z'].min(), df['z'].max()]
    
    # Add margins
    x_margin = (x_range[1] - x_range[0]) * 0.1
    y_margin = (y_range[1] - y_range[0]) * 0.1
    z_margin = (z_range[1] - z_range[0]) * 0.1
    
    ax.set_xlim(x_range[0] - x_margin, x_range[1] + x_margin)
    ax.set_ylim(y_range[0] - y_margin, y_range[1] + y_margin)
    ax.set_zlim(z_range[0] - z_margin, z_range[1] + z_margin)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Voronoi Pursuit-Evasion 3D Trajectory')
    
    # Colors for drones
    pursuer_colors = ['b', 'g', 'm','c']
    evader_color = 'r'
    
    # Get drone IDs
    pursuer_ids = sorted([int(d) for d in df[df['role'] == 'pursuer']['drone_id'].unique()])
    evader_id = int(df[df['role'] == 'evader']['drone_id'].unique()[0])
    
    # Initialize plot elements
    pursuer_points = []
    pursuer_trails = []
    
    for i, drone_id in enumerate(pursuer_ids):
        # Current position marker
        point, = ax.plot([], [], [], f'{pursuer_colors[i]}o', 
                          markersize=8, label=f'Pursuer {drone_id}')
        pursuer_points.append(point)
        
        # Trail (dotted line)
        trail, = ax.plot([], [], [], f'{pursuer_colors[i]}--', 
                         alpha=0.5, linewidth=1.5)
        pursuer_trails.append(trail)
    
    # Evader
    evader_point, = ax.plot([], [], [], f'{evader_color}o', 
                            markersize=10, label=f'Evader {evader_id}')
    evader_trail, = ax.plot([], [], [], f'{evader_color}--', 
                            alpha=0.6, linewidth=2)
    
    # Initial positions (stars)
    initial_markers = []
    
    ax.legend(loc='upper right')
    
    # Animation function
    def animate(frame_idx):
        frame_idx = frame_idx * skip_frames
        if frame_idx >= len(timestamps):
            frame_idx = len(timestamps) - 1
        
        current_time = timestamps[frame_idx]
        frame_data = df[df['timestamp'] == current_time]
        
        # Update pursuers
        for i, drone_id in enumerate(pursuer_ids):
            drone_data = frame_data[frame_data['drone_id'] == drone_id]
            if not drone_data.empty:
                x, y, z = drone_data.iloc[0][['x', 'y', 'z']]
                pursuer_points[i].set_data([x], [y])
                pursuer_points[i].set_3d_properties([z])
                
                # Update trail (all positions up to current)
                trail_data = df[(df['drone_id'] == drone_id) & (df['timestamp'] <= current_time)]
                if len(trail_data) > 1:
                    pursuer_trails[i].set_data(trail_data['x'], trail_data['y'])
                    pursuer_trails[i].set_3d_properties(trail_data['z'])
        
        # Update evader
        evader_data = frame_data[frame_data['drone_id'] == evader_id]
        if not evader_data.empty:
            x, y, z = evader_data.iloc[0][['x', 'y', 'z']]
            evader_point.set_data([x], [y])
            evader_point.set_3d_properties([z])
            
            # Update trail
            trail_data = df[(df['drone_id'] == evader_id) & (df['timestamp'] <= current_time)]
            if len(trail_data) > 1:
                evader_trail.set_data(trail_data['x'], trail_data['y'])
                evader_trail.set_3d_properties(trail_data['z'])
        
        # Show initial positions on first frame
        if frame_idx == 0:
            for i, drone_id in enumerate(pursuer_ids):
                initial = df[df['drone_id'] == drone_id].iloc[0]
                ax.plot([initial['x']], [initial['y']], [initial['z']], 
                        f'{pursuer_colors[i]}*', markersize=15, alpha=0.5)
            
            initial = df[df['drone_id'] == evader_id].iloc[0]
            ax.plot([initial['x']], [initial['y']], [initial['z']], 
                    f'{evader_color}*', markersize=18, alpha=0.5)
        
        # Update title with time
        ax.set_title(f'Voronoi Pursuit-Evasion 3D Trajectory (t = {current_time:.2f}s)')
        
        return pursuer_points + pursuer_trails + [evader_point, evader_trail]
    
    # Calculate number of frames
    num_frames = len(timestamps) // skip_frames
    print(f"   Total frames: {num_frames}")
    print(f"   Duration: {timestamps[-1]:.2f}s")
    
    # Create animation
    anim = FuncAnimation(fig, animate, frames=num_frames, 
                        interval=1000//fps, blit=False, repeat=True)
    
    # Save as GIF
    print(f"   Saving GIF...")
    writer = PillowWriter(fps=fps)
    anim.save(output_file, writer=writer)
    
    print(f"✅ Animation saved to {output_file}")
    
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Create 3D trajectory animation',
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
    parser.add_argument(
        '--output',
        '-o',
        type=str,
        help='Output GIF filename (default: animation_TIMESTAMP.gif)'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=20,
        help='Frames per second'
    )
    parser.add_argument(
        '--skip',
        type=int,
        default=2,
        help='Skip every N frames to speed up animation'
    )
    
    args = parser.parse_args()
    
    data_dir = Path(__file__).parent / args.data_dir
    
    # Determine input file
    if args.auto:
        # Find latest merged CSV
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
    
    # Determine output file
    if args.output:
        output_file = args.output
    else:
        timestamp = input_file.stem.replace('merged_all_drones_', '')
        output_file = data_dir / f'animation_{timestamp}.gif'
    
    # Create animation
    create_animation(df, output_file, args.fps, args.skip)


if __name__ == '__main__':
    main()
