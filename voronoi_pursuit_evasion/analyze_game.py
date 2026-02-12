#!/usr/bin/env python3
"""
Voronoi Game Analysis Script
Analyzes recorded CSV data from unified Voronoi pursuit-evasion game

Generates:
- Numerical statistics
- 2D/3D trajectory plots
- Distance over time plots
- Velocity analysis
- Animation

Usage:
    python3 analyze_game.py data/
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import matplotlib.animation as animation
from pathlib import Path
import sys
import argparse
from datetime import datetime
from scipy.spatial import Voronoi, ConvexHull


def load_game_data(data_dir):
    """Load all CSV files from a game run"""
    data_path = Path(data_dir)
    
    # Look for sim_* subdirectories
    sim_dirs = sorted([d for d in data_path.glob('sim_*') if d.is_dir()], reverse=True)
    
    if not sim_dirs:
        print(f"❌ No simulation folders found in {data_dir}")
        print("   Expected structure: data/sim_YYYYMMDD_HHMMSS/")
        return None, None
    
    # If multiple simulation folders, list them and use most recent
    if len(sim_dirs) > 1:
        print(f"\n📁 Found {len(sim_dirs)} simulation runs:")
        for i, sim_dir in enumerate(sim_dirs, 1):
            csv_count = len(list(sim_dir.glob('drone_*.csv')))
            print(f"   {i}. {sim_dir.name} ({csv_count} files)")
        
        selected_dir = sim_dirs[0]
        print(f"\n✅ Using most recent: {selected_dir.name}")
    else:
        selected_dir = sim_dirs[0]
        print(f"\n📁 Found simulation: {selected_dir.name}")
    
    # Load CSV files from selected simulation folder
    csv_files = sorted(selected_dir.glob('drone_*.csv'))
    
    if not csv_files:
        print(f"❌ No CSV files found in {selected_dir}")
        return None, None
    
    # Extract timestamp from folder name (sim_YYYYMMDD_HHMMSS)
    timestamp = selected_dir.name.replace('sim_', '')
    
    print(f"\n📁 Loading {len(csv_files)} data files:")
    
    drones = {}
    for csv_file in csv_files:
        # Parse filename: drone_X_role_timestamp.csv
        parts = csv_file.stem.split('_')
        instance = int(parts[1])
        role = parts[2]
        
        df = pd.read_csv(csv_file)
        drones[instance] = {
            'role': role,
            'data': df,
            'file': csv_file.name
        }
        print(f"   {csv_file.name}: {len(df)} samples, {role}")
    
    return drones, selected_dir


def load_voronoi_vertices(sim_dir):
    """Load precomputed Voronoi vertices and edges if available"""
    sim_path = Path(sim_dir)
    
    # Look for voronoi_vertices_*.csv and voronoi_edges_*.csv files
    vertex_files = list(sim_path.glob('voronoi_vertices_*.csv'))
    edge_files = list(sim_path.glob('voronoi_edges_*.csv'))
    
    if not vertex_files:
        print("ℹ️  No Voronoi vertex data found (run compute_voronoi_vertices.py first)")
        return None, None
    
    vertex_file = vertex_files[0]
    print(f"📊 Loading Voronoi vertices: {vertex_file.name}")
    
    try:
        vertices_df = pd.read_csv(vertex_file)
        print(f"   {len(vertices_df)} vertex records, {vertices_df['time'].nunique()} unique timesteps")
        
        # Load edges if available
        edges_df = None
        if edge_files:
            edge_file = edge_files[0]
            edges_df = pd.read_csv(edge_file)
            print(f"📊 Loading Voronoi edges: {edge_file.name}")
            print(f"   {len(edges_df)} edge records")
        else:
            print("ℹ️  No Voronoi edge data found")
        
        return vertices_df, edges_df
    except Exception as e:
        print(f"⚠️  Failed to load Voronoi data: {e}")
        return None, None


def analyze_game(drones):
    """Numerical analysis of game"""
    print("\n" + "="*60)
    print("  VORONOI PURSUIT-EVASION GAME ANALYSIS")
    print("="*60)
    
    # Find evader
    evader_id = None
    for drone_id, info in drones.items():
        if info['role'] == 'evader':
            evader_id = drone_id
            break
    
    if evader_id is None:
        print("❌ No evader found!")
        return
    
    evader_data = drones[evader_id]['data']
    
    # Game timing
    print("\n📊 GAME STATISTICS:")
    print("-" * 60)
    
    total_time = evader_data['time'].max()
    print(f"Total game duration: {total_time:.2f}s")
    
    # Check for Voronoi phase
    voronoi_data = evader_data[evader_data['phase'] == 'VORONOI']
    if len(voronoi_data) > 0:
        game_start = voronoi_data['time'].min()
        game_duration = voronoi_data['time'].max() - game_start
        print(f"Voronoi game start: {game_start:.2f}s")
        print(f"Voronoi game duration: {game_duration:.2f}s")
    else:
        print("⚠️  No Voronoi phase detected")
        game_start = 0
    
    # Distance analysis
    print("\n📏 DISTANCE ANALYSIS:")
    print("-" * 60)
    
    # Calculate distances between evader and each pursuer
    evader_pos_data = voronoi_data[['time', 'pos_x', 'pos_y', 'pos_z']].values
    
    min_distances = {}
    final_distances = {}
    capture_times = {}
    
    for drone_id, info in drones.items():
        if info['role'] == 'pursuer':
            pursuer_data = info['data']
            pursuer_voronoi = pursuer_data[pursuer_data['phase'] == 'VORONOI']
            
            if len(pursuer_voronoi) == 0:
                continue
            
            # Calculate distances
            distances = []
            times = []
            
            for _, evader_row in voronoi_data.iterrows():
                t = evader_row['time']
                # Find closest pursuer sample in time
                time_diff = np.abs(pursuer_voronoi['time'] - t)
                idx = time_diff.idxmin()
                pursuer_row = pursuer_voronoi.loc[idx]
                
                evader_pos = np.array([evader_row['pos_x'], evader_row['pos_y'], evader_row['pos_z']])
                pursuer_pos = np.array([pursuer_row['pos_x'], pursuer_row['pos_y'], pursuer_row['pos_z']])
                
                dist = np.linalg.norm(evader_pos - pursuer_pos)
                distances.append(dist)
                times.append(t)
            
            distances = np.array(distances)
            min_dist = distances.min()
            final_dist = distances[-1]
            
            min_distances[drone_id] = min_dist
            final_distances[drone_id] = final_dist
            
            # Check for capture (< 0.4m)
            capture_indices = np.where(distances < 0.4)[0]
            if len(capture_indices) > 0:
                capture_time = times[capture_indices[0]] - game_start
                capture_times[drone_id] = capture_time
                print(f"🎯 Pursuer {drone_id}: CAPTURED at t={capture_time:.2f}s (distance: {distances[capture_indices[0]]:.3f}m)")
            else:
                print(f"   Pursuer {drone_id}: Min distance: {min_dist:.3f}m, Final: {final_dist:.3f}m")
    
    # Velocity analysis
    print("\n🚀 VELOCITY ANALYSIS:")
    print("-" * 60)
    
    for drone_id, info in drones.items():
        voronoi_phase = info['data'][info['data']['phase'] == 'VORONOI']
        if len(voronoi_phase) > 0:
            vel_x = voronoi_phase['vel_cmd_x'].values
            vel_y = voronoi_phase['vel_cmd_y'].values
            vel_z = voronoi_phase['vel_cmd_z'].values
            
            vel_mag = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
            avg_vel = vel_mag.mean()
            max_vel = vel_mag.max()
            
            print(f"   Drone {drone_id} ({info['role']}): Avg vel: {avg_vel:.3f} m/s, Max: {max_vel:.3f} m/s")
    
    # Movement analysis
    print("\n📍 POSITION ANALYSIS:")
    print("-" * 60)
    
    for drone_id, info in drones.items():
        data = info['data']
        initial_pos = data[['pos_x', 'pos_y', 'pos_z']].iloc[0].values
        final_pos = data[['pos_x', 'pos_y', 'pos_z']].iloc[-1].values
        
        total_displacement = np.linalg.norm(final_pos - initial_pos)
        
        # Calculate total path length
        positions = data[['pos_x', 'pos_y', 'pos_z']].values
        path_segments = np.diff(positions, axis=0)
        total_path = np.sum(np.linalg.norm(path_segments, axis=1))
        
        print(f"   Drone {drone_id}: Displacement: {total_displacement:.2f}m, Path length: {total_path:.2f}m")
    
    return {
        'total_time': total_time,
        'game_start': game_start,
        'min_distances': min_distances,
        'final_distances': final_distances,
        'capture_times': capture_times,
        'evader_id': evader_id
    }


def plot_trajectories_2d(drones, stats, output_dir):
    """Plot 2D trajectories (top-down view)"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    evader_id = stats['evader_id']
    
    # Plot 1: XY plane (top-down)
    for drone_id, info in drones.items():
        data = info['data']
        color = 'red' if info['role'] == 'evader' else 'blue'
        label = f"Drone {drone_id} ({info['role']})"
        
        ax1.plot(data['pos_x'].values, data['pos_y'].values, color=color, linewidth=2, alpha=0.7, label=label)
        ax1.scatter(data['pos_x'].iloc[0], data['pos_y'].iloc[0], color=color, marker='o', s=100, edgecolors='black')
        ax1.scatter(data['pos_x'].iloc[-1], data['pos_y'].iloc[-1], color=color, marker='X', s=150, edgecolors='black')
    
    ax1.set_xlabel('X (North) [m]', fontsize=12)
    ax1.set_ylabel('Y (East) [m]', fontsize=12)
    ax1.set_title('Voronoi Game: Top-Down View (XY)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # Plot 2: XZ plane (side view)
    for drone_id, info in drones.items():
        data = info['data']
        color = 'red' if info['role'] == 'evader' else 'blue'
        
        ax2.plot(data['pos_x'].values, -data['pos_z'].values, color=color, linewidth=2, alpha=0.7)
        ax2.scatter(data['pos_x'].iloc[0], -data['pos_z'].iloc[0], color=color, marker='o', s=100, edgecolors='black')
        ax2.scatter(data['pos_x'].iloc[-1], -data['pos_z'].iloc[-1], color=color, marker='X', s=150, edgecolors='black')
    
    ax2.set_xlabel('X (North) [m]', fontsize=12)
    ax2.set_ylabel('Z (Altitude) [m]', fontsize=12)
    ax2.set_title('Voronoi Game: Side View (XZ)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = output_dir / 'game_trajectories_2d.png'
    plt.savefig(output_file, dpi=300)
    print(f"\n✅ Saved: {output_file}")


def plot_trajectories_3d(drones, stats, output_dir):
    """Plot 3D trajectories"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    for drone_id, info in drones.items():
        data = info['data']
        color = 'red' if info['role'] == 'evader' else 'blue'
        label = f"Drone {drone_id} ({info['role']})"
        
        ax.plot(data['pos_x'].values, data['pos_y'].values, -data['pos_z'].values, 
                color=color, linewidth=2, alpha=0.7, label=label)
        ax.scatter(data['pos_x'].iloc[0], data['pos_y'].iloc[0], -data['pos_z'].iloc[0], 
                   color=color, marker='o', s=100, edgecolors='black')
        ax.scatter(data['pos_x'].iloc[-1], data['pos_y'].iloc[-1], -data['pos_z'].iloc[-1], 
                   color=color, marker='X', s=150, edgecolors='black')
    
    ax.set_xlabel('X (North) [m]', fontsize=12)
    ax.set_ylabel('Y (East) [m]', fontsize=12)
    ax.set_zlabel('Z (Altitude) [m]', fontsize=12)
    ax.set_title('3D Voronoi Pursuit-Evasion Trajectories', fontsize=14, fontweight='bold')
    ax.legend()
    
    output_file = output_dir / 'game_trajectories_3d.png'
    plt.savefig(output_file, dpi=300)
    print(f"✅ Saved: {output_file}")


def plot_distances(drones, stats, output_dir):
    """Plot distances between pursuers and evader over time"""
    fig, ax = plt.subplots(figsize=(14, 7))
    
    evader_id = stats['evader_id']
    evader_data = drones[evader_id]['data']
    voronoi_data = evader_data[evader_data['phase'] == 'VORONOI']
    
    for drone_id, info in drones.items():
        if info['role'] == 'pursuer':
            pursuer_data = info['data']
            pursuer_voronoi = pursuer_data[pursuer_data['phase'] == 'VORONOI']
            
            distances = []
            times = []
            
            for _, evader_row in voronoi_data.iterrows():
                t = evader_row['time']
                time_diff = np.abs(pursuer_voronoi['time'] - t)
                idx = time_diff.idxmin()
                pursuer_row = pursuer_voronoi.loc[idx]
                
                evader_pos = np.array([evader_row['pos_x'], evader_row['pos_y'], evader_row['pos_z']])
                pursuer_pos = np.array([pursuer_row['pos_x'], pursuer_row['pos_y'], pursuer_row['pos_z']])
                
                dist = np.linalg.norm(evader_pos - pursuer_pos)
                distances.append(dist)
                times.append(t - stats['game_start'])
            
            ax.plot(np.array(times), np.array(distances), linewidth=2, label=f'Pursuer {drone_id}')
    
    ax.axhline(y=0.4, color='red', linestyle='--', linewidth=2, label='Capture Radius')
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Distance [m]', fontsize=12)
    ax.set_title('Pursuer-Evader Distances During Game', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    output_file = output_dir / 'game_distances.png'
    plt.savefig(output_file, dpi=300)
    print(f"✅ Saved: {output_file}")


def compute_voronoi_vertices(pursuer_positions):
    """
    Compute Voronoi cell vertices for given pursuer positions
    Returns vertices of the Voronoi diagram
    """
    if len(pursuer_positions) < 4:
        return None
    
    try:
        vor = Voronoi(pursuer_positions)
        return vor
    except Exception as e:
        return None


def get_farthest_voronoi_corner(evader_pos, pursuer_positions):
    """
    Find the farthest Voronoi vertex (corner) from evader position
    This is what evader strategy 1 targets
    """
    vor = compute_voronoi_vertices(pursuer_positions)
    if vor is None or len(vor.vertices) == 0:
        return None
    
    # Filter out infinite vertices
    finite_vertices = vor.vertices[np.all(np.isfinite(vor.vertices), axis=1)]
    if len(finite_vertices) == 0:
        return None
    
    # Find farthest vertex from evader
    distances = np.linalg.norm(finite_vertices - evader_pos, axis=1)
    farthest_idx = np.argmax(distances)
    
    return finite_vertices[farthest_idx]


def get_tetrahedron_edges(pursuer_positions):
    """
    Draw edges of the tetrahedron formed by 4 pursuers
    A tetrahedron has 4 vertices and 6 edges connecting them
    """
    if len(pursuer_positions) != 4:
        return []
    
    edges = []
    # For 4 vertices, draw all 6 edges (combinations of 4 choose 2)
    for i in range(4):
        for j in range(i + 1, 4):
            edges.append([pursuer_positions[i], pursuer_positions[j]])
    
    return edges


def create_animation(drones, stats, output_dir, voronoi_vertices=None, voronoi_edges=None):
    """Create animated 3D visualization with optional Voronoi vertices and edges"""
    print("\n🎬 Creating animation...")
    
    if voronoi_vertices is not None:
        print("   Using Voronoi vertex (circumcenter) visualization")
    else:
        print("   Using pursuer tetrahedron edges")
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Find max timesteps
    max_samples = max(len(info['data']) for info in drones.values())
    
    # Initialize plot elements
    lines = {}
    points = {}
    
    for drone_id, info in drones.items():
        color = 'red' if info['role'] == 'evader' else 'blue'
        lines[drone_id], = ax.plot([], [], [], color=color, linewidth=2, alpha=0.7, 
                                     label=f"Drone {drone_id} ({info['role']})")
        points[drone_id], = ax.plot([], [], [], color=color, marker='o', markersize=10)
    
    # Voronoi visualization placeholders
    voronoi_edge_collection = None
    voronoi_vertex_scatter = None
    
    # Set axis limits
    all_data = pd.concat([info['data'] for info in drones.values()])
    ax.set_xlim([all_data['pos_x'].min() - 2, all_data['pos_x'].max() + 2])
    ax.set_ylim([all_data['pos_y'].min() - 2, all_data['pos_y'].max() + 2])
    ax.set_zlim([-all_data['pos_z'].max() - 2, -all_data['pos_z'].min() + 2])
    
    ax.set_xlabel('X (North) [m]')
    ax.set_ylabel('Y (East) [m]')
    ax.set_zlabel('Z (Altitude) [m]')
    ax.legend()
    
    def animate(frame):
        nonlocal voronoi_edge_collection, voronoi_vertex_scatter
        
        # Remove old Voronoi visualizations if exists
        if voronoi_edge_collection is not None:
            voronoi_edge_collection.remove()
            voronoi_edge_collection = None
        if voronoi_vertex_scatter is not None:
            voronoi_vertex_scatter.remove()
            voronoi_vertex_scatter = None
        
        # Update drone positions and trajectories
        current_positions = {}
        current_time = None
        
        for drone_id, info in drones.items():
            data = info['data']
            if frame < len(data):
                x = data['pos_x'].iloc[:frame+1].values
                y = data['pos_y'].iloc[:frame+1].values
                z = -data['pos_z'].iloc[:frame+1].values
                
                lines[drone_id].set_data(x, y)
                lines[drone_id].set_3d_properties(z)
                
                points[drone_id].set_data([x[-1]], [y[-1]])
                points[drone_id].set_3d_properties([z[-1]])
                
                current_positions[drone_id] = np.array([x[-1], y[-1], z[-1]])
                if current_time is None:
                    current_time = data['time'].iloc[frame]
        
        # Visualize Voronoi tessellation if we're in Voronoi phase
        if len(current_positions) == 5:  # All drones present
            # Separate pursuers and evader
            pursuer_pos = []
            evader_pos = None
            
            for drone_id, info in drones.items():
                if drone_id in current_positions:
                    if info['role'] == 'pursuer':
                        pursuer_pos.append(current_positions[drone_id])
                    else:
                        evader_pos = current_positions[drone_id]
            
            if len(pursuer_pos) == 4 and evader_pos is not None:
                pursuer_pos = np.array(pursuer_pos)
                
                # Use precomputed Voronoi data if available
                if voronoi_vertices is not None and current_time is not None:
                    # Find vertices closest to current time
                    time_tolerance = 0.05  # 50ms tolerance
                    frame_vertices = voronoi_vertices[
                        np.abs(voronoi_vertices['time'] - current_time) < time_tolerance
                    ]
                    
                    if len(frame_vertices) > 0:
                        # Get Voronoi vertex (circumcenter)
                        vx = frame_vertices['x'].values[0]
                        vy = frame_vertices['y'].values[0]
                        vz = -frame_vertices['z'].values[0]  # Flip Z for display
                        voronoi_center = np.array([vx, vy, vz])
                        
                        # Draw Voronoi vertex as large yellow sphere
                        voronoi_vertex_scatter = ax.scatter([vx], [vy], [vz],
                                                            color='yellow',
                                                            s=200,
                                                            alpha=0.9,
                                                            marker='*',
                                                            label='Voronoi Vertex (Circumcenter)',
                                                            edgecolors='orange',
                                                            linewidths=2)
                        
                        # Draw lines from circumcenter to each pursuer (Voronoi structure)
                        voronoi_structure_lines = []
                        for pp in pursuer_pos:
                            voronoi_structure_lines.append([voronoi_center, pp])
                        
                        voronoi_edge_collection = Line3DCollection(voronoi_structure_lines,
                                                                    colors='magenta',
                                                                    linewidths=2.0,
                                                                    alpha=0.6,
                                                                    linestyles='dashed',
                                                                    label='Voronoi Structure')
                        ax.add_collection3d(voronoi_edge_collection)
                else:
                    # Fallback: just draw tetrahedron edges
                    edges = get_tetrahedron_edges(pursuer_pos)
                    if edges:
                        voronoi_edge_collection = Line3DCollection(edges,
                                                                    colors='cyan',
                                                                    linewidths=2.5,
                                                                    alpha=0.7,
                                                                    label='Pursuer Tetrahedron')
                        ax.add_collection3d(voronoi_edge_collection)
        
        ax.set_title(f'Voronoi Game Animation (Frame {frame}/{max_samples})', fontweight='bold')
        return list(lines.values()) + list(points.values())
    
    anim = animation.FuncAnimation(fig, animate, frames=max_samples, 
                                    interval=40, blit=False)  # 40ms = 25fps
    
    output_file = output_dir / 'game_animation.mp4'
    anim.save(str(output_file), writer='ffmpeg', fps=25, dpi=150)
    print(f"✅ Saved: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Analyze Voronoi pursuit-evasion game data')
    parser.add_argument('data_dir', nargs='?', default='data/', help='Data directory containing CSV files')
    parser.add_argument('--no-animation', action='store_true', help='Skip animation generation')
    args = parser.parse_args()
    
    # Load data
    drones, sim_dir = load_game_data(args.data_dir)
    if drones is None:
        return 1
    
    # Load Voronoi vertices and edges if available
    voronoi_vertices, voronoi_edges = load_voronoi_vertices(sim_dir)
    
    # Output directory is same as simulation directory
    output_dir = sim_dir
    print(f"\n📂 Analysis output directory: {output_dir}")
    
    # Numerical analysis
    stats = analyze_game(drones)
    if stats is None:
        return 1
    
    # Generate plots
    print("\n📊 Generating plots...")
    plot_trajectories_2d(drones, stats, output_dir)
    plot_trajectories_3d(drones, stats, output_dir)
    plot_distances(drones, stats, output_dir)
    
    # Create animation (optional)
    if not args.no_animation:
        try:
            create_animation(drones, stats, output_dir, voronoi_vertices, voronoi_edges)
        except Exception as e:
            print(f"⚠️  Animation failed: {e}")
            print("   (ffmpeg may not be installed)")
    
    print("\n" + "="*60)
    print("  ANALYSIS COMPLETE")
    print("="*60)
    print(f"\n📁 All files saved to: {output_dir}/")
    print("   - game_trajectories_2d.png")
    print("   - game_trajectories_3d.png")
    print("   - game_distances.png")
    if not args.no_animation:
        print("   - game_animation.mp4")
    print()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
