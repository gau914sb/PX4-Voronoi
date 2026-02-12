#!/usr/bin/env python3
"""
Initial Position Setup for Voronoi Pursuit-Evasion
Self-contained script - moves all 5 drones to tetrahedral formation and holds until agents take over

Usage:
    python3 initial_positions.py
"""

import asyncio
import yaml
import numpy as np
from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from std_msgs.msg import Bool as BoolMsg

from mavsdk import System
from mavsdk.offboard import PositionNedYaw


class DroneController(Node):
    """Individual drone controller for initial positioning"""
    
    def __init__(self, instance, mavsdk_port, mavlink_port, spawn_offset=None):
        super().__init__(f'drone_init_{instance}')
        
        self.instance = instance
        self.mavsdk_port = mavsdk_port
        self.mavlink_port = mavlink_port
        
        # Spawn offset for local→global coordinate conversion
        # Gazebo spawn: PX4_GZ_MODEL_POSE="0,$Y_POS" → Gazebo (X=0, Y=Y_POS) in ENU coords
        # PX4 reports in NED: Gazebo_Y (North in ENU) → NED_X (North in NED)
        # So offset is in X direction: [(instance-1)*2.0, 0, 0] in NED
        if spawn_offset is None:
            self.spawn_offset = np.array([(instance - 1) * 2.0, 0.0, 0.0])
        else:
            self.spawn_offset = np.array(spawn_offset)
        
        # State (in LOCAL frame - relative to drone's home)
        self.position_local = np.array([0.0, 0.0, 0.0])
        self.target_local = np.array([0.0, 0.0, -5.0])
        
        # MAVSDK drone
        self.drone = None
        
        # ROS2 QoS for PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'/px4_{instance}/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/px4_{instance}/fmu/in/trajectory_setpoint', qos)
        
        # Subscriber for position feedback
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{instance}/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos
        )
        
        # Control timer @ 50Hz
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(f'Drone {instance} controller initialized')
    
    def position_callback(self, msg):
        """Store position feedback (local frame from PX4)"""
        self.position_local = np.array([msg.x, msg.y, msg.z])
    
    @property
    def position_global(self):
        """Get position in global frame (local + spawn_offset)"""
        return self.position_local + self.spawn_offset
    
    @property
    def target_global(self):
        """Get target in global frame (local + spawn_offset)"""
        return self.target_local + self.spawn_offset
    
    def timestamp(self):
        """Get timestamp in microseconds"""
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def publish_offboard_mode(self):
        """Publish OffboardControlMode heartbeat"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp()
        self.offboard_mode_pub.publish(msg)
    
    def publish_setpoint(self, x, y, z, frame='global'):
        """
        Publish position setpoint
        
        Args:
            x, y, z: Target position coordinates
            frame: 'global' (world coordinates) or 'local' (relative to drone's home)
        """
        if frame == 'global':
            # Convert global to local by subtracting spawn offset
            target_global = np.array([x, y, z])
            self.target_local = target_global - self.spawn_offset
        else:
            # Already in local frame
            self.target_local = np.array([x, y, z])
        
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_local[0]), float(self.target_local[1]), float(self.target_local[2])]
        msg.yaw = 0.0
        msg.timestamp = self.timestamp()
        self.setpoint_pub.publish(msg)
    
    def control_loop(self):
        """Publish offboard heartbeat and current setpoint @ 50Hz"""
        self.publish_offboard_mode()
        # Republish stored local target
        self.publish_setpoint(self.target_local[0], self.target_local[1], self.target_local[2], frame='local')


def load_config():
    """Load configuration from YAML file"""
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def generate_tetrahedral_positions(center, radius):
    """
    Generate vertices of regular tetrahedron for pursuit-evasion
    
    One drone at apex (directly above center), three at base forming equilateral triangle
    All vertices are exactly distance R from center (inscribed in sphere of radius R)
    
    Args:
        center: Center point [x, y, z] 
        radius: Distance from center to vertices (meters)
        
    Returns:
        4x3 array of vertex positions
    """
    cx, cy, cz = center
    
    # Apex: directly above center at distance R
    apex = np.array([cx, cy, cz + radius])
    
    # Base: 3 drones at altitude (center_z - R/3) forming equilateral triangle
    base_altitude = cz - radius / 3.0
    base_radius = radius * 2.0 * np.sqrt(2.0) / 3.0  # ≈ 0.9428 * R (2√2/3)
    
    # Three drones at 0°, 120°, 240° around center
    angles = [0, 2*np.pi/3, 4*np.pi/3]
    base_vertices = []
    for angle in angles:
        x = cx + base_radius * np.cos(angle)
        y = cy + base_radius * np.sin(angle)
        base_vertices.append([x, y, base_altitude])
    
    # Return as [apex, base1, base2, base3]
    vertices = np.array([apex] + base_vertices)
    
    return vertices


async def setup_and_move_drone(controller, target_position, config):
    """
    Setup drone: arm, start offboard, move to target position
    
    Args:
        controller: DroneController node
        target_position: Target [x, y, z] in NED
        config: Configuration dictionary
        
    Returns:
        (instance, success)
    """
    try:
        # Initialize MAVSDK
        controller.get_logger().info(f'Connecting MAVSDK on port {controller.mavsdk_port}...')
        controller.drone = System(port=controller.mavsdk_port)
        await controller.drone.connect(system_address=f"udp://:{controller.mavlink_port}")
        
        # Wait for connection
        async for state in controller.drone.core.connection_state():
            if state.is_connected:
                controller.get_logger().info('MAVSDK connected!')
                break
        
        # Wait for health
        async for health in controller.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                controller.get_logger().info('Drone ready!')
                break
        
        # Start streaming setpoints before arming
        controller.get_logger().info('Streaming offboard heartbeat...')
        await asyncio.sleep(2.0)
        
        # Arm
        controller.get_logger().info('Arming...')
        await controller.drone.action.arm()
        await asyncio.sleep(1.0)
        
        # Start offboard mode
        controller.get_logger().info('Starting offboard mode...')
        await controller.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await controller.drone.offboard.start()
        controller.get_logger().info('Offboard mode active!')
        
        # Move to target position
        controller.get_logger().info(f'Moving to target: {target_position}')
        controller.publish_setpoint(target_position[0], target_position[1], target_position[2], frame='global')
        
        # Wait until reached
        threshold = config['init']['position_threshold']
        timeout = config['init']['max_wait_time']
        start_time = asyncio.get_event_loop().time()
        feedback_counter = 0
        
        while True:
            current_pos = controller.position_global  # Use GLOBAL position for comparison
            distance = np.linalg.norm(current_pos - target_position)
            
            # Print feedback every 2 seconds
            feedback_counter += 1
            if feedback_counter % 100 == 0:
                elapsed = asyncio.get_event_loop().time() - start_time
                controller.get_logger().info(
                    f'[{elapsed:.1f}s] Global: [{current_pos[0]:6.2f}, {current_pos[1]:6.2f}, {current_pos[2]:6.2f}] | '
                    f'Target: [{target_position[0]:6.2f}, {target_position[1]:6.2f}, {target_position[2]:6.2f}] | '
                    f'Distance: {distance:.2f}m'
                )
            
            if distance < threshold:
                controller.get_logger().info(f'✅ Position reached! Distance: {distance:.3f}m')
                return (controller.instance, True)
            
            # Check timeout
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                controller.get_logger().error(f'❌ Timeout after {elapsed:.1f}s! Distance: {distance:.3f}m')
                return (controller.instance, False)
            
            await asyncio.sleep(0.02)  # 50Hz
            
    except Exception as e:
        controller.get_logger().error(f'Error: {e}')
        return (controller.instance, False)


async def hold_positions_until_takeover(controllers, config, max_wait_time=None):
    """
    Hold drones at their positions until agents take over
    Monitors /voronoi/agent_active/instance_X topics
    
    Args:
        controllers: List of DroneController instances
        config: Configuration dictionary
        max_wait_time: Maximum time to wait for agents (seconds). None = wait forever
    """
    # Create takeover monitor node
    class TakeoverMonitor(Node):
        def __init__(self, instances):
            super().__init__('takeover_monitor')
            self.agent_active = {i: False for i in instances}
            self.subscribers = {}
            
            for instance in instances:
                topic = f"/voronoi/agent_active/instance_{instance}"
                self.subscribers[instance] = self.create_subscription(
                    BoolMsg,
                    topic,
                    lambda msg, inst=instance: self.takeover_callback(msg, inst),
                    10
                )
        
        def takeover_callback(self, msg, instance):
            if msg.data and not self.agent_active[instance]:
                self.agent_active[instance] = True
                self.get_logger().info(f'🔁 Agent {instance} has taken over control')
        
        def all_active(self):
            return all(self.agent_active.values())
    
    all_instances = config['pursuers']['instances'] + [config['evader']['instance']]
    monitor = TakeoverMonitor(all_instances)
    
    # Spin monitor in background thread
    def spin_monitor():
        rclpy.spin(monitor)
    
    monitor_thread = threading.Thread(target=spin_monitor, daemon=True)
    monitor_thread.start()
    
    if max_wait_time:
        print(f"\n⏳ Holding positions for {max_wait_time}s (or until agents connect)...")
        print(f"   Launch agents with: ./launch_game.sh")
        print(f"   Will auto-exit and transfer control\n")
    else:
        print(f"\n⏳ Holding positions - waiting for agents to take over...")
        print(f"   Launch game with: ./launch_game.sh")
        print(f"   Press Ctrl+C to exit early\n")
    
    start_time = asyncio.get_event_loop().time()
    
    try:
        while not monitor.all_active():
            # Controllers continue publishing via their control_loop timers
            await asyncio.sleep(0.1)
            
            # Check timeout
            if max_wait_time:
                elapsed = asyncio.get_event_loop().time() - start_time
                if elapsed > max_wait_time:
                    print(f"\n⏱️  Timeout reached ({max_wait_time}s)")
                    print(f"   Agents should now take over control")
                    print(f"   Exiting initial positions script\n")
                    break
        
        if monitor.all_active():
            print(f"\n✅ All agents active - handoff complete!")
            print(f"   Initial positions script can now exit safely\n")
        
        # Hold for 1 more second to ensure smooth transition
        await asyncio.sleep(1.0)
        
    except KeyboardInterrupt:
        print(f"\n⚠️  Manual exit - agents should already be running!")
    finally:
        monitor.destroy_node()


async def main(auto_handover_time=None):
    """Main function
    
    Args:
        auto_handover_time: Auto-exit after N seconds (for automated launch)
    """
    # Load configuration
    config = load_config()
    
    # Initialize ROS2
    rclpy.init()
    
    # Helper function for spawn offset compensation
    def get_spawn_offset(instance):
        """
        Get spawn offset for each drone instance (from launch_multi_drones.sh)
        Gazebo spawns at (0, Y_POS) in ENU → converts to (X_POS, 0) in NED
        """
        return np.array([(instance - 1) * 2.0, 0.0, 0.0])
    
    print("\n" + "="*60)
    print("  Voronoi Pursuit-Evasion: Initial Position Setup")
    print("="*60)
    
    # Calculate initial positions
    base_altitude = config['formation']['base_altitude']
    radius = config['formation']['radius']
    center = np.array([0.0, 0.0, base_altitude])
    
    # Generate tetrahedral positions for pursuers
    pursuer_positions = generate_tetrahedral_positions(center, radius)
    
    # Evader at center
    evader_position = center.copy()
    
    # Convert to NED (z is negative altitude)
    pursuer_positions[:, 2] = -pursuer_positions[:, 2]
    evader_position[2] = -evader_position[2]
    
    print(f"\n📍 Global Formation (NED coordinates):")
    print(f"   Evader (instance {config['evader']['instance']}): {evader_position}")
    for i, pos in enumerate(pursuer_positions):
        instance = config['pursuers']['instances'][i]
        print(f"   Pursuer {i+1} (instance {instance}): {pos}")
    
    print(f"\n🎯 Local Target Positions (compensated for spawn offsets):")
    for i, instance in enumerate(config['pursuers']['instances']):
        local_target = pursuer_positions[i] - get_spawn_offset(instance)
        print(f"   Pursuer {i+1} (instance {instance}): {local_target}")
    evader_instance = config['evader']['instance']
    local_target = evader_position - get_spawn_offset(evader_instance)
    print(f"   Evader (instance {evader_instance}): {local_target}")
    
    # Create controllers for all drones
    controllers = []
    target_positions = []
    
    # Pursuers
    for i, instance in enumerate(config['pursuers']['instances']):
        mavsdk_port = 50050 + instance
        mavlink_port = 14540 + instance
        controller = DroneController(instance, mavsdk_port, mavlink_port)  # Spawn offset auto-calculated
        controllers.append(controller)
        # Target is in GLOBAL frame - controller handles conversion
        target_positions.append(pursuer_positions[i])
    
    # Evader
    evader_instance = config['evader']['instance']
    mavsdk_port = 50050 + evader_instance
    mavlink_port = 14540 + evader_instance
    controller = DroneController(evader_instance, mavsdk_port, mavlink_port)  # Spawn offset auto-calculated
    controllers.append(controller)
    # Target is in GLOBAL frame - controller handles conversion
    target_positions.append(evader_position)
    
    # Spin all controllers in background threads with dedicated executors
    def spin_node(node):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    
    for controller in controllers:
        thread = threading.Thread(
            target=lambda c: spin_node(c), args=(controller,), daemon=True)
        thread.start()
    
    # Setup and move all drones concurrently
    print(f"\n🚀 Launching {len(controllers)} drones...")
    tasks = [setup_and_move_drone(c, t, config) for c, t in zip(controllers, target_positions)]
    results = await asyncio.gather(*tasks)
    
    # Check results
    print(f"\n📊 Setup Results:")
    all_success = True
    for instance, success in results:
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"   Drone {instance}: {status}")
        all_success = all_success and success
    
    if not all_success:
        print(f"\n⚠️  Some drones failed to reach positions!")
        print(f"   Check logs above for details")
        # Cleanup
        for controller in controllers:
            controller.destroy_node()
        rclpy.shutdown()
        return
    
    print(f"\n🎯 All drones at initial positions!")
    
    # Hold positions until agents take over
    await hold_positions_until_takeover(controllers, config, max_wait_time=auto_handover_time)
    
    # Cleanup
    for controller in controllers:
        controller.destroy_node()
    rclpy.shutdown()
    
    print("\n" + "="*60)
    print("  Initial setup complete - agents are in control!")
    print("  Start the game with:")
    print("  ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool \"{data: true}\"")
    print("="*60 + "\n")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Move drones to initial tetrahedral formation')
    parser.add_argument('--auto-handover', type=int, default=None,
                        help='Auto-exit after N seconds for automated launch (default: wait for agents)')
    args = parser.parse_args()
    
    try:
        asyncio.run(main(auto_handover_time=args.auto_handover))
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
        rclpy.shutdown()
