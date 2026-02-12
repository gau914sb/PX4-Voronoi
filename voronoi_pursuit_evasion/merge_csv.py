#!/usr/bin/env python3
"""
Merge Individual Drone CSV Files
Combines separate CSV files from each drone into unified dataset

Usage:
    python3 merge_csv.py data/drone_1_pursuer_20260212_143022.csv ...
    python3 merge_csv.py --auto  # Auto-detect latest run
    python3 merge_csv.py --pattern "20260212_143022"  # Match timestamp
"""

import argparse
import pandas as pd
from pathlib import Path
import glob
import sys


def find_csv_sets(data_dir, pattern=None):
    """
    Find sets of CSV files from the same run
    
    Returns:
        dict: {timestamp: [file1, file2, ...]}
    """
    csv_files = list(Path(data_dir).glob('drone_*.csv'))
    
    # Group by timestamp (assumes filename format: drone_X_role_TIMESTAMP.csv)
    sets = {}
    for csv_file in csv_files:
        parts = csv_file.stem.split('_')
        if len(parts) >= 4:
            timestamp = '_'.join(parts[3:])  # Everything after role
            
            if pattern is None or pattern in timestamp:
                if timestamp not in sets:
                    sets[timestamp] = []
                sets[timestamp].append(csv_file)
    
    return sets


def merge_csv_files(csv_files, output_file=None):
    """
    Merge multiple CSV files into one
    
    Args:
        csv_files: List of CSV file paths
        output_file: Output filename (optional)
        
    Returns:
        DataFrame with merged data
    """
    print(f"\n📊 Merging {len(csv_files)} CSV files...")
    
    dataframes = []
    for csv_file in sorted(csv_files):
        print(f"   Reading: {csv_file.name}")
        df = pd.read_csv(csv_file)
        dataframes.append(df)
    
    # Concatenate all dataframes
    merged_df = pd.concat(dataframes, ignore_index=True)
    
    # Sort by timestamp, then by drone_id
    merged_df = merged_df.sort_values(['timestamp', 'drone_id'])
    
    print(f"\n✅ Merged dataset:")
    print(f"   Total rows: {len(merged_df)}")
    print(f"   Drones: {sorted(merged_df['drone_id'].unique())}")
    print(f"   Time range: {merged_df['timestamp'].min():.2f}s - {merged_df['timestamp'].max():.2f}s")
    print(f"   Duration: {merged_df['timestamp'].max() - merged_df['timestamp'].min():.2f}s")
    
    # Save if output file specified
    if output_file:
        merged_df.to_csv(output_file, index=False)
        print(f"\n💾 Saved to: {output_file}")
    
    return merged_df


def main():
    parser = argparse.ArgumentParser(
        description='Merge individual drone CSV files',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'files',
        nargs='*',
        help='CSV files to merge (if not using --auto or --pattern)'
    )
    parser.add_argument(
        '--auto',
        action='store_true',
        help='Auto-detect and merge latest run'
    )
    parser.add_argument(
        '--pattern',
        type=str,
        help='Match CSVs with this timestamp pattern'
    )
    parser.add_argument(
        '--data-dir',
        type=str,
        default='data',
        help='Data directory to search'
    )
    parser.add_argument(
        '--output',
        '-o',
        type=str,
        help='Output filename (default: auto-generated)'
    )
    
    args = parser.parse_args()
    
    data_dir = Path(__file__).parent / args.data_dir
    
    if not data_dir.exists():
        print(f"❌ Error: Data directory not found: {data_dir}")
        sys.exit(1)
    
    # Determine which files to merge
    csv_files = []
    
    if args.auto or args.pattern:
        # Auto-detect CSV sets
        sets = find_csv_sets(data_dir, args.pattern)
        
        if not sets:
            print(f"❌ No CSV files found in {data_dir}")
            sys.exit(1)
        
        if args.auto:
            # Use most recent set
            latest_timestamp = max(sets.keys())
            csv_files = sets[latest_timestamp]
            print(f"\n🔍 Auto-detected latest run: {latest_timestamp}")
        else:
            # Use pattern match
            if len(sets) > 1:
                print(f"\n⚠️  Multiple sets match pattern '{args.pattern}':")
                for ts in sets.keys():
                    print(f"   {ts} ({len(sets[ts])} files)")
                print(f"\nUsing first match: {list(sets.keys())[0]}")
            csv_files = sets[list(sets.keys())[0]]
    
    elif args.files:
        # Use explicitly provided files
        csv_files = [Path(f) for f in args.files]
    
    else:
        print("❌ Error: Provide files, --auto, or --pattern")
        parser.print_help()
        sys.exit(1)
    
    # Check all files exist
    for csv_file in csv_files:
        if not csv_file.exists():
            print(f"❌ Error: File not found: {csv_file}")
            sys.exit(1)
    
    # Generate output filename if not provided
    if args.output:
        output_file = Path(args.output)
    else:
        # Extract timestamp from first file
        parts = csv_files[0].stem.split('_')
        timestamp = '_'.join(parts[3:]) if len(parts) >= 4 else 'merged'
        output_file = data_dir / f'merged_all_drones_{timestamp}.csv'
    
    # Merge the files
    merged_df = merge_csv_files(csv_files, output_file)
    
    # Show summary statistics
    print(f"\n📈 Summary by Drone:")
    for drone_id in sorted(merged_df['drone_id'].unique()):
        drone_data = merged_df[merged_df['drone_id'] == drone_id]
        role = drone_data['role'].iloc[0]
        n_samples = len(drone_data)
        print(f"   Drone {drone_id} ({role}): {n_samples} samples")
    
    print("\n✅ Merge complete!")


if __name__ == '__main__':
    main()
