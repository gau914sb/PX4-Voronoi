#!/usr/bin/env python3
"""
Multi-Drone Concurrent Control
Launches multiple drones simultaneously using asyncio
"""

import asyncio
import argparse
import sys
from hybrid_offboard_control import HybridOffboardControl, run_mission, ros2_spin
import rclpy
import threading


async def run_drone(instance, altitude, hover_time):
    """Run a single drone mission"""
    # Calculate ports
    mavsdk_port = 50050 + instance
    mavlink_port = 14540 + instance
    
    # Create node
    node = HybridOffboardControl(
        instance=instance,
        mavsdk_port=mavsdk_port,
        mavlink_port=mavlink_port
    )
    
    # Start ROS2 spin in background thread
    spin_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run mission
    try:
        await run_mission(node, target_altitude=altitude, hover_time=hover_time)
    except Exception as e:
        node.get_logger().error(f'Mission failed: {e}')
    finally:
        node.destroy_node()
    
    return instance


async def main(num_drones, altitude, hover_time, stagger_delay):
    """Main function to coordinate multiple drones"""
    # Initialize ROS2
    rclpy.init()
    
    print(f"\n🚁 Starting {num_drones} drones concurrently...")
    print(f"   Altitude: {altitude}m")
    print(f"   Hover: {hover_time}s")
    print(f"   Stagger: {stagger_delay}s\n")
    
    # Create tasks for all drones
    tasks = []
    for i in range(1, num_drones + 1):
        # Optional: stagger launch to avoid simultaneous connections
        if stagger_delay > 0 and i > 1:
            await asyncio.sleep(stagger_delay)
        
        task = asyncio.create_task(run_drone(i, altitude, hover_time))
        tasks.append(task)
        print(f"🚀 Drone {i} launched")
    
    # Wait for all drones to complete
    results = await asyncio.gather(*tasks, return_exceptions=True)
    
    # Report results
    print(f"\n✅ All {num_drones} drones completed!")
    for i, result in enumerate(results, 1):
        if isinstance(result, Exception):
            print(f"   Drone {i}: ❌ Failed - {result}")
        else:
            print(f"   Drone {i}: ✅ Success")
    
    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Multi-drone concurrent offboard control',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-n', '--num-drones',
        type=int,
        default=3,
        help='Number of drones to control (1-10)'
    )
    parser.add_argument(
        '-a', '--altitude',
        type=float,
        default=50.0,
        help='Target altitude for all drones (meters)'
    )
    parser.add_argument(
        '-t', '--hover',
        type=float,
        default=5.0,
        help='Hover time for all drones (seconds)'
    )
    parser.add_argument(
        '-s', '--stagger',
        type=float,
        default=2.0,
        help='Delay between launching each drone (seconds)'
    )
    
    args = parser.parse_args()
    
    if args.num_drones < 1 or args.num_drones > 10:
        print("Error: Number of drones must be between 1 and 10")
        sys.exit(1)
    
    try:
        asyncio.run(main(args.num_drones, args.altitude, args.hover, args.stagger))
    except KeyboardInterrupt:
        print("\n⚠️  Mission interrupted by user")
        sys.exit(0)
