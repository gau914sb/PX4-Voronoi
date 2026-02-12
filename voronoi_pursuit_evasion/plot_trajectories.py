#!/usr/bin/env python3
"""
Comprehensive Trajectory Plotting Script
Creates all trajectory visualizations and animations for drone data

Plots:
- XY, YZ, XZ plane trajectories
- X vs time, Y vs time, Z vs time
- 3D XYZ trajectories
- Animated 3D visualization

Usage:
    python3 plot_trajectories.py data/sim_TIMESTAMP/
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from pathlib import Path


def load_drone_data(sim_dir):
    """Load all drone CSV files from simulation directory"""
    sim_path = Path(sim_dir)
    csv_files = sorted(sim_path.glob('drone_*.csv'))
    
    if len(csv_files) == 0:
        print(f"❌ No CSV files found in {sim_dir}")
        return None, None
    
    print(f"📁 Loading {len(csv_files)} drone files...")
    
    drones = {}
    for csv_file in csv_files:
        data = pd.read_csv(csv_file)
        
        # Extract drone ID and role from filename
        parts = csv_file.stem.split('_')
        drone_id = int(parts[1])
        role = parts[2]
        
        drones[drone_id] = {
            'role': role,
            'data': data,
            'file': csv_file
        }
        print(f"   Drone {drone_id} ({role}): {len(data)} samples")
    
    return drones, sim_path


def plot_plane_trajectories(drones, output_dir):
    """Plot XY, YZ, and XZ plane trajectories"""
    print("\n📊 Plotting plane trajectories...")
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    planes = [
        ('XY', 'pos_x', 'pos_y', 'X (North) [m]', 'Y (East) [m]', axes[0]),
        ('YZ', 'pos_y', 'pos_z', 'Y (East) [m]', 'Z (Down) [m]', axes[1]),
        ('XZ', 'pos_x', 'pos_z', 'X (North) [m]', 'Z (Down) [m]', axes[2])
    ]
    
    for plane_name, x_col, y_col, x_label, y_label, ax in planes:
        for drone_id, info in sorted(drones.items()):
            data = info['data']
            color = 'red' if info['role'] == 'evader' else 'blue'
            label = f"Drone {drone_id} ({info['role']})"
            
            # Plot trajectory
            ax.plot(data[x_col].values, data[y_col].values, color=color, linewidth=1.5, 
                   alpha=0.7, label=label)
            
            # Mark start and end
            ax.scatter(data[x_col].iloc[0], data[y_col].iloc[0], 
                      color=color, marker='o', s=100, edgecolors='black', linewidths=2)
            ax.scatter(data[x_col].iloc[-1], data[y_col].iloc[-1], 
                      color=color, marker='s', s=100, edgecolors='black', linewidths=2)
        
        ax.set_xlabel(x_label, fontsize=12)
        ax.set_ylabel(y_label, fontsize=12)
        ax.set_title(f'{plane_name} Plane Trajectories', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        ax.axis('equal')
    
    plt.tight_layout()
    output_file = output_dir / 'trajectories_planes.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: {output_file}")


def plot_time_series(drones, output_dir):
    """Plot X vs time, Y vs time, Z vs time"""
    print("\n📊 Plotting time series...")
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    coords = [
        ('X (North)', 'pos_x', 'm', axes[0]),
        ('Y (East)', 'pos_y', 'm', axes[1]),
        ('Z (Down)', 'pos_z', 'm', axes[2])
    ]
    
    for coord_name, col, unit, ax in coords:
        for drone_id, info in sorted(drones.items()):
            data = info['data']
            color = 'red' if info['role'] == 'evader' else 'blue'
            label = f"Drone {drone_id} ({info['role']})"
            
            ax.plot(data['time'].values, data[col].values, color=color, linewidth=2, 
                   alpha=0.7, label=label)
        
        ax.set_xlabel('Time [s]', fontsize=12)
        ax.set_ylabel(f'{coord_name} [{unit}]', fontsize=12)
        ax.set_title(f'{coord_name} vs Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    output_file = output_dir / 'trajectories_time_series.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: {output_file}")


def plot_3d_trajectories(drones, output_dir):
    """Plot 3D XYZ trajectories"""
    print("\n📊 Plotting 3D trajectories...")
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    for drone_id, info in sorted(drones.items()):
        data = info['data']
        color = 'red' if info['role'] == 'evader' else 'blue'
        label = f"Drone {drone_id} ({info['role']})"
        
        # Plot trajectory
        ax.plot(data['pos_x'].values, data['pos_y'].values, -data['pos_z'].values, 
               color=color, linewidth=2, alpha=0.7, label=label)
        
        # Mark start (circle) and end (square)
        ax.scatter(data['pos_x'].iloc[0], data['pos_y'].iloc[0], -data['pos_z'].iloc[0],
                  color=color, marker='o', s=150, edgecolors='black', linewidths=2)
        ax.scatter(data['pos_x'].iloc[-1], data['pos_y'].iloc[-1], -data['pos_z'].iloc[-1],
                  color=color, marker='s', s=150, edgecolors='black', linewidths=2)
    
    ax.set_xlabel('X (North) [m]', fontsize=12)
    ax.set_ylabel('Y (East) [m]', fontsize=12)
    ax.set_zlabel('Z (Altitude) [m]', fontsize=12)
    ax.set_title('3D Trajectories (XYZ Space)', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    
    # Set equal aspect ratio
    all_data = pd.concat([info['data'] for info in drones.values()])
    ax.set_xlim([all_data['pos_x'].min() - 2, all_data['pos_x'].max() + 2])
    ax.set_ylim([all_data['pos_y'].min() - 2, all_data['pos_y'].max() + 2])
    ax.set_zlim([-all_data['pos_z'].max() - 2, -all_data['pos_z'].min() + 2])
    
    plt.tight_layout()
    output_file = output_dir / 'trajectories_3d.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: {output_file}")


def create_3d_animation(drones, output_dir):
    """Create animated 3D visualization"""
    print("\n🎬 Creating 3D animation...")
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Find max timesteps
    max_samples = max(len(info['data']) for info in drones.values())
    
    # Initialize plot elements - trajectories and current positions
    lines = {}
    points = {}
    
    for drone_id, info in drones.items():
        color = 'red' if info['role'] == 'evader' else 'blue'
        label = f"Drone {drone_id} ({info['role']})"
        
        lines[drone_id], = ax.plot([], [], [], color=color, linewidth=2, 
                                     alpha=0.7, label=label)
        points[drone_id], = ax.plot([], [], [], color=color, marker='o', 
                                      markersize=12, markeredgecolor='black', 
                                      markeredgewidth=2)
    
    # Set axis limits
    all_data = pd.concat([info['data'] for info in drones.values()])
    ax.set_xlim([all_data['pos_x'].min() - 2, all_data['pos_x'].max() + 2])
    ax.set_ylim([all_data['pos_y'].min() - 2, all_data['pos_y'].max() + 2])
    ax.set_zlim([-all_data['pos_z'].max() - 2, -all_data['pos_z'].min() + 2])
    
    ax.set_xlabel('X (North) [m]', fontsize=12)
    ax.set_ylabel('Y (East) [m]', fontsize=12)
    ax.set_zlabel('Z (Altitude) [m]', fontsize=12)
    ax.legend(loc='best', fontsize=10)
    
    def animate(frame):
        """Update animation frame"""
        current_time = None
        
        for drone_id, info in drones.items():
            data = info['data']
            if frame < len(data):
                # Update trajectory trail
                x = data['pos_x'].iloc[:frame+1].values
                y = data['pos_y'].iloc[:frame+1].values
                z = -data['pos_z'].iloc[:frame+1].values
                
                lines[drone_id].set_data(x, y)
                lines[drone_id].set_3d_properties(z)
                
                # Update current position marker
                points[drone_id].set_data([x[-1]], [y[-1]])
                points[drone_id].set_3d_properties([z[-1]])
                
                if current_time is None:
                    current_time = data['time'].iloc[frame]
        
        time_str = f"t = {current_time:.2f}s" if current_time else ""
        ax.set_title(f'3D Trajectory Animation - Frame {frame}/{max_samples}\n{time_str}', 
                    fontsize=14, fontweight='bold')
        
        return list(lines.values()) + list(points.values())
    
    # Create animation (25 fps)
    anim = animation.FuncAnimation(fig, animate, frames=max_samples, 
                                    interval=40, blit=False)
    
    # Save animation
    output_file = output_dir / 'trajectories_3d_animation.mp4'
    anim.save(str(output_file), writer='ffmpeg', fps=25, dpi=150)
    print(f"✅ Saved: {output_file}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Plot comprehensive trajectory visualizations')
    parser.add_argument('sim_dir', help='Simulation directory containing drone CSV files')
    parser.add_argument('--no-animation', action='store_true', help='Skip animation generation')
    args = parser.parse_args()
    
    print("="*60)
    print("  TRAJECTORY PLOTTING")
    print("="*60)
    
    # Load drone data
    result = load_drone_data(args.sim_dir)
    if result is None or result[0] is None:
        return 1
    
    drones, sim_path = result
    
    print(f"\n📂 Output directory: {sim_path}")
    
    # Generate all plots
    plot_plane_trajectories(drones, sim_path)
    plot_time_series(drones, sim_path)
    plot_3d_trajectories(drones, sim_path)
    
    # Create animation
    if not args.no_animation:
        try:
            create_3d_animation(drones, sim_path)
        except Exception as e:
            print(f"⚠️  Animation failed: {e}")
            print("   (ffmpeg may not be installed)")
    
    print("\n" + "="*60)
    print("  PLOTTING COMPLETE")
    print("="*60)
    print(f"\n📁 All plots saved to: {sim_path}")
    print("   - trajectories_planes.png (XY, YZ, XZ)")
    print("   - trajectories_time_series.png (X, Y, Z vs time)")
    print("   - trajectories_3d.png (3D XYZ)")
    if not args.no_animation:
        print("   - trajectories_3d_animation.mp4")
    
    return 0


if __name__ == '__main__':
    exit(main())
