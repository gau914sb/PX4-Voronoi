#!/usr/bin/env python3
"""
Compute Voronoi Vertices from Drone Data
Takes recorded drone CSV files and computes Voronoi tessellation vertices
for each timestep during the VORONOI phase.

Outputs: voronoi_vertices_TIMESTAMP.csv in the same simulation folder

Usage:
    python3 compute_voronoi_vertices.py data/sim_TIMESTAMP/
"""

import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.spatial import Voronoi


def load_drone_data(sim_dir):
    """Load all drone CSV files from simulation directory"""
    sim_path = Path(sim_dir)
    csv_files = sorted(sim_path.glob('drone_*.csv'))
    
    if len(csv_files) == 0:
        print(f"❌ No CSV files found in {sim_dir}")
        return None
    
    print(f"📁 Loading {len(csv_files)} drone files...")
    
    drones = {}
    for csv_file in csv_files:
        data = pd.read_csv(csv_file)
        
        # Extract drone ID and role from filename
        # Format: drone_X_ROLE_TIMESTAMP.csv
        parts = csv_file.stem.split('_')
        drone_id = int(parts[1])
        role = parts[2]
        
        drones[drone_id] = {
            'role': role,
            'data': data,
            'file': csv_file
        }
        print(f"   {csv_file.name}: {len(data)} samples, {role}")
    
    return drones, sim_path


def compute_voronoi_vertices(pursuer_positions):
    """
    Compute Voronoi vertices and edges from 4 pursuer positions
    Returns vertices and edges between them that form the Voronoi cell
    """
    if len(pursuer_positions) != 4:
        return [], []
    
    try:
        vor = Voronoi(pursuer_positions)
        
        # Get center position (mean of pursuers)
        center = pursuer_positions.mean(axis=0)
        
        # Filter finite vertices within reasonable distance
        max_dist = 20.0  # meters from center
        vertices = []
        vertex_map = {}  # Map old index to new index
        
        for i, vertex in enumerate(vor.vertices):
            if np.all(np.isfinite(vertex)):
                dist = np.linalg.norm(vertex - center)
                if dist < max_dist:
                    new_idx = len(vertices)
                    vertex_map[i] = new_idx
                    vertices.append({
                        'vertex_id': new_idx,
                        'x': vertex[0],
                        'y': vertex[1],
                        'z': vertex[2],
                        'distance_from_center': dist
                    })
        
        # Extract edges between vertices (ridges)
        edges = []
        for ridge_vertices in vor.ridge_vertices:
            if -1 not in ridge_vertices:  # Skip infinite edges
                v1_idx, v2_idx = ridge_vertices
                # Check if both vertices are in our filtered set
                if v1_idx in vertex_map and v2_idx in vertex_map:
                    edges.append({
                        'v1_id': vertex_map[v1_idx],
                        'v2_id': vertex_map[v2_idx]
                    })
        
        return vertices, edges
    except Exception as e:
        print(f"⚠️  Voronoi computation failed: {e}")
        return [], []


def process_simulation(drones, sim_path):
    """Process drone data and compute Voronoi vertices for each timestep"""
    
    # Separate pursuers and evader
    pursuers = {did: info for did, info in drones.items() if info['role'] == 'pursuer'}
    evader = next((info for info in drones.values() if info['role'] == 'evader'), None)
    
    if len(pursuers) != 4:
        print(f"❌ Expected 4 pursuers, found {len(pursuers)}")
        return None
    
    if evader is None:
        print("❌ No evader found")
        return None
    
    # Get VORONOI phase data from evader (reference timeline)
    evader_data = evader['data']
    voronoi_phase = evader_data[evader_data['phase'] == 'VORONOI'].copy()
    
    if len(voronoi_phase) == 0:
        print("❌ No VORONOI phase data found")
        return None
    
    print(f"\n🧮 Computing Voronoi vertices for {len(voronoi_phase)} timesteps...")
    print(f"   Time range: {voronoi_phase['time'].min():.2f}s - {voronoi_phase['time'].max():.2f}s")
    
    # Collect all Voronoi vertex and edge data
    all_vertices = []
    all_edges = []
    
    for idx, evader_row in voronoi_phase.iterrows():
        t = evader_row['time']
        timestamp = evader_row['timestamp']
        
        # Get pursuer positions at this timestep (find closest time match)
        pursuer_positions = []
        for pursuer_id, pursuer_info in sorted(pursuers.items()):
            pursuer_data = pursuer_info['data']
            pursuer_voronoi = pursuer_data[pursuer_data['phase'] == 'VORONOI']
            
            if len(pursuer_voronoi) == 0:
                continue
            
            # Find closest timestep
            time_diff = np.abs(pursuer_voronoi['time'] - t)
            closest_idx = time_diff.idxmin()
            pursuer_row = pursuer_voronoi.loc[closest_idx]
            
            pos = np.array([
                pursuer_row['pos_x'],
                pursuer_row['pos_y'],
                pursuer_row['pos_z']
            ])
            pursuer_positions.append(pos)
        
        if len(pursuer_positions) != 4:
            continue
        
        # Compute Voronoi vertices and edges for this configuration
        vertices, edges = compute_voronoi_vertices(np.array(pursuer_positions))
        
        # Add vertices to collection with timestamp
        for v in vertices:
            all_vertices.append({
                'timestamp': timestamp,
                'time': t,
                'vertex_id': v['vertex_id'],
                'x': v['x'],
                'y': v['y'],
                'z': v['z'],
                'distance_from_center': v['distance_from_center']
            })
        
        # Add edges to collection with timestamp
        for e in edges:
            all_edges.append({
                'timestamp': timestamp,
                'time': t,
                'v1_id': e['v1_id'],
                'v2_id': e['v2_id']
            })
    
    if len(all_vertices) == 0:
        print("⚠️  No Voronoi vertices computed")
        return None, None
    
    # Convert to DataFrames
    vertices_df = pd.DataFrame(all_vertices)
    edges_df = pd.DataFrame(all_edges) if all_edges else None
    
    print(f"✅ Computed {len(vertices_df)} vertex records")
    print(f"   Unique timesteps: {vertices_df['time'].nunique()}")
    print(f"   Vertices per timestep: {vertices_df.groupby('time').size().mean():.1f} (avg)")
    
    if edges_df is not None:
        print(f"✅ Computed {len(edges_df)} edge records")
        print(f"   Edges per timestep: {edges_df.groupby('time').size().mean():.1f} (avg)")
    
    return vertices_df, edges_df


def main():
    parser = argparse.ArgumentParser(description='Compute Voronoi vertices from drone data')
    parser.add_argument('sim_dir', help='Simulation directory containing drone CSV files')
    args = parser.parse_args()
    
    print("="*60)
    print("  VORONOI VERTEX COMPUTATION")
    print("="*60)
    
    # Load drone data
    result = load_drone_data(args.sim_dir)
    if result is None:
        return 1
    
    drones, sim_path = result
    
    # Process and compute vertices and edges
    result = process_simulation(drones, sim_path)
    if result is None or result[0] is None:
        return 1
    
    vertices_df, edges_df = result
    
    # Extract timestamp from directory name
    # Format: sim_YYYYMMDD_HHMMSS
    timestamp = sim_path.name.replace('sim_', '')
    
    # Save vertices to CSV
    vertices_file = sim_path / f'voronoi_vertices_{timestamp}.csv'
    vertices_df.to_csv(vertices_file, index=False)
    print(f"\n✅ Saved vertices: {vertices_file}")
    print(f"   {len(vertices_df)} rows, {len(vertices_df.columns)} columns")
    
    # Save edges to CSV
    if edges_df is not None:
        edges_file = sim_path / f'voronoi_edges_{timestamp}.csv'
        edges_df.to_csv(edges_file, index=False)
        print(f"✅ Saved edges: {edges_file}")
        print(f"   {len(edges_df)} rows, {len(edges_df.columns)} columns")
    
    print("\n" + "="*60)
    print("  COMPUTATION COMPLETE")
    print("="*60)
    print(f"\n📁 Outputs:")
    print(f"   {vertices_file}")
    if edges_df is not None:
        print(f"   {edges_file}")
    
    return 0


if __name__ == '__main__':
    exit(main())
