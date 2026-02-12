#!/usr/bin/env python3
"""
Unified Voronoi Agent
Single continuous agent handling initial positioning AND Voronoi game control

Phase 1 (0-40s): Move to tetrahedral formation positions
Phase 2 (40s+): Execute Voronoi pursuit-evasion velocities

No handover, no disconnection - continuous setpoint publishing

Usage:
    python3 unified_voronoi_agent.py --instance 1 --role pursuer
    python3 unified_voronoi_agent.py --instance 5 --role evader
"""

import asyncio
import argparse
import yaml
import numpy as np
import os
from pathlib import Path
import time
import threading
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


def load_config():
    """Load configuration from yaml file"""
    config_path = Path(__file__).parent / 'config.yaml'
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def generate_tetrahedral_positions(center, radius):
    """
    Generate vertices of regular tetrahedron for pursuit-evasion
    Returns 4 positions: apex + 3 base vertices forming equilateral triangle
    """
    cx, cy, cz = center
    
    # Apex: directly above center at distance R
    apex = np.array([cx, cy, cz + radius])
    
    # Base: 3 drones at altitude (center_z - R/3) forming equilateral triangle
    base_altitude = cz - radius / 3.0
    base_radius = radius * 2.0 * np.sqrt(2.0) / 3.0
    
    # Angles at 0°, 120°, 240°
    angles = [0, 2*np.pi/3, 4*np.pi/3]
    base_vertices = []
    for angle in angles:
        x = cx + base_radius * np.cos(angle)
        y = cy + base_radius * np.sin(angle)
        base_vertices.append([x, y, base_altitude])
    
    return np.array([apex] + base_vertices)


class UnifiedVoronoiAgent(Node):
    def __init__(self, instance, role, config):
        super().__init__(f'unified_agent_{instance}')
        
        self.instance = instance
        self.role = role  # 'pursuer' or 'evader'
        self.config = config
        
        # Ports
        self.mavsdk_port = 50050 + instance
        self.namespace = f'/px4_{instance}/fmu'
        
        # Spawn offset for coordinate frame conversion
        self.spawn_offset = np.array([(instance - 1) * 2.0, 0.0, 0.0])
        
        # State
        self.position_local = np.array([0.0, 0.0, 0.0])
        self.target_local = np.array([0.0, 0.0, -5.0])  # Initial hover
        self.voronoi_velocity = np.array([0.0, 0.0, 0.0])
        
        # Phase control
        self.phase = 'ARMING'  # ARMING -> POSITIONING -> VORONOI
        self.positioning_complete = False
        self.game_active = False
        self.start_time = None
        self.game_start_time = None
        self.capture_detected = False
        self.shutdown_timer = None
        
        # Data logging
        self.log_data = []
        self.csv_file = None
        self.csv_writer = None
        self._setup_logging()
        
        # MAVSDK
        self.drone = None
        
        # ROS2 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'{self.namespace}/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{self.namespace}/in/trajectory_setpoint', qos)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.namespace}/out/vehicle_local_position_v1',
            self.position_callback,
            qos
        )
        
        # Voronoi velocity subscriber
        cmd_vel_topic = f"/voronoi/cmd_vel/instance_{instance}"
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        
        # Game control subscribers
        self.game_start_sub = self.create_subscription(
            Bool, "/voronoi/start_game", self.game_start_callback, 10)
        
        self.capture_sub = self.create_subscription(
            String, "/voronoi/capture", self.capture_callback, 10)
        
        # Control timer at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(f'Unified Agent {instance} ({role}) initialized')
    
    def _setup_logging(self):
        """Setup CSV logging in timestamped simulation folder"""
        data_dir = Path(__file__).parent / 'data'
        data_dir.mkdir(exist_ok=True)
        
        # Get shared timestamp from environment (set by launch script)
        # All agents use the same timestamp so CSVs go to same folder
        timestamp = os.environ.get('VORONOI_SIM_TIMESTAMP', datetime.now().strftime('%Y%m%d_%H%M%S'))
        
        # Create simulation-specific subfolder
        sim_dir = data_dir / f'sim_{timestamp}'
        sim_dir.mkdir(exist_ok=True)
        
        filename = f'drone_{self.instance}_{self.role}_{timestamp}.csv'
        self.csv_file = sim_dir / filename
        
        f = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(f)
        self.csv_writer.writerow([
            'timestamp', 'time', 'phase',
            'pos_x', 'pos_y', 'pos_z',
            'target_x', 'target_y', 'target_z',
            'vel_cmd_x', 'vel_cmd_y', 'vel_cmd_z',
            'distance_to_target'
        ])
        f.flush()
        self.log_file_handle = f
    
    @property
    def position_global(self):
        """Current position in global frame"""
        return self.position_local + self.spawn_offset
    
    def timestamp(self):
        """Get current timestamp in microseconds"""
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def position_callback(self, msg):
        """Store position feedback"""
        self.position_local = np.array([msg.x, msg.y, msg.z])
    
    def cmd_vel_callback(self, msg):
        """Store Voronoi velocity command"""
        self.voronoi_velocity = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    
    def game_start_callback(self, msg):
        """Handle game start signal"""
        if msg.data and not self.game_active:
            self.game_active = True
            self.game_start_time = time.time()
            self.phase = 'VORONOI'
            self.get_logger().info('🎮 Switching to VORONOI control mode')
    
    def capture_callback(self, msg):
        """Handle capture event - shutdown after delay"""
        if not self.capture_detected:
            self.capture_detected = True
            self.get_logger().info(f'🎯 Game ended: {msg.data}')
            self.get_logger().info('Shutting down in 2 seconds...')
            
            # Schedule shutdown after 2 seconds
            self.shutdown_timer = self.create_timer(2.0, self.shutdown_callback)
    
    def shutdown_callback(self):
        """Gracefully shutdown the agent"""
        if hasattr(self, 'log_file_handle') and self.log_file_handle is not None:
            self.log_file_handle.close()
        self.get_logger().info('✅ Agent shutdown complete')
        raise SystemExit(0)
    
    def publish_offboard_mode(self):
        """Publish offboard mode heartbeat"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp()
        self.offboard_mode_pub.publish(msg)
    
    def publish_position_setpoint(self, pos_local):
        """Publish position setpoint in local frame"""
        msg = TrajectorySetpoint()
        msg.position = [float(pos_local[0]), float(pos_local[1]), float(pos_local[2])]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.timestamp = self.timestamp()
        self.setpoint_pub.publish(msg)
    
    def set_target_global(self, pos_global):
        """Set target position in global frame (converts to local)"""
        self.target_local = pos_global - self.spawn_offset
    
    def control_loop(self):
        """Main control loop at 50Hz"""
        # Always publish offboard heartbeat
        self.publish_offboard_mode()
        
        if self.phase == 'ARMING':
            # During arming, hold initial position
            self.publish_position_setpoint(self.target_local)
        
        elif self.phase == 'POSITIONING':
            # Move to formation position
            self.publish_position_setpoint(self.target_local)
            
            # Check if reached target
            if not self.positioning_complete:
                distance = np.linalg.norm(self.position_local - self.target_local)
                if distance < 0.3:  # Within 30cm
                    self.positioning_complete = True
                    self.get_logger().info(f'✅ Reached formation position')
        
        elif self.phase == 'VORONOI':
            # Integrate velocity to position (simple Euler integration)
            dt = 0.02  # 50Hz
            self.target_local += self.voronoi_velocity * dt
            self.publish_position_setpoint(self.target_local)
        
        # Log data
        self._log_data()
    
    def _log_data(self):
        """Log current state to CSV"""
        if not self.start_time:
            return
        
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        pos_global = self.position_global
        target_global = self.target_local + self.spawn_offset
        distance = np.linalg.norm(pos_global - target_global)
        
        self.csv_writer.writerow([
            current_time,
            elapsed,
            self.phase,
            pos_global[0], pos_global[1], pos_global[2],
            target_global[0], target_global[1], target_global[2],
            self.voronoi_velocity[0], self.voronoi_velocity[1], self.voronoi_velocity[2],
            distance
        ])
        
        # Flush every 50 rows (1 second at 50Hz)
        if len(self.log_data) % 50 == 0:
            self.log_file_handle.flush()


async def setup_drone(agent, initial_position_global):
    """Arm drone and move to initial position"""
    agent.get_logger().info(f'Connecting MAVSDK on port {agent.mavsdk_port}...')
    
    # Connect to drone
    mavlink_port = 14540 + agent.instance
    agent.drone = System(port=agent.mavsdk_port)
    await agent.drone.connect(system_address=f'udp://:{mavlink_port}')
    
    agent.get_logger().info('Waiting for drone connection...')
    async for state in agent.drone.core.connection_state():
        if state.is_connected:
            agent.get_logger().info('✅ MAVSDK connected')
            break
    
    # Wait for drone to be ready
    agent.get_logger().info('Waiting for drone to be ready...')
    async for health in agent.drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            agent.get_logger().info('✅ Drone ready')
            break
    
    # Arm
    agent.get_logger().info('Arming...')
    await agent.drone.action.arm()
    agent.get_logger().info('✅ Armed')
    
    # Enable offboard mode
    agent.get_logger().info('Setting offboard mode...')
    await agent.drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await agent.drone.offboard.start()
    agent.get_logger().info('✅ Offboard mode active')
    
    # Switch to positioning phase
    agent.phase = 'POSITIONING'
    agent.set_target_global(initial_position_global)
    agent.start_time = time.time()
    
    agent.get_logger().info(f'Moving to formation: {initial_position_global} (global)')
    agent.get_logger().info(f'Target in local frame: {agent.target_local}')


async def run_agent(instance, role, config):
    """Main async function to run unified agent"""
    
    # Calculate formation positions
    base_altitude = config['formation']['base_altitude']
    radius = config['formation']['radius']
    center = np.array([0.0, 0.0, base_altitude])
    
    # Generate tetrahedral positions (in ENU, altitude positive up)
    pursuer_positions_enu = generate_tetrahedral_positions(center, radius)
    evader_position_enu = center.copy()
    
    # Convert to NED (z negative down)
    pursuer_positions_ned = pursuer_positions_enu.copy()
    pursuer_positions_ned[:, 2] = -pursuer_positions_ned[:, 2]
    evader_position_ned = evader_position_enu.copy()
    evader_position_ned[2] = -evader_position_ned[2]
    
    # Assign initial position based on role
    if role == 'pursuer':
        pursuer_idx = config['pursuers']['instances'].index(instance)
        initial_position_global = pursuer_positions_ned[pursuer_idx]
    else:  # evader
        initial_position_global = evader_position_ned
    
    # Create agent
    agent = UnifiedVoronoiAgent(instance, role, config)
    
    # Start ROS2 spinning in background thread
    def spin_node():
        rclpy.spin(agent)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    # Setup and arm drone
    await setup_drone(agent, initial_position_global)
    
    # Monitor phases
    while True:
        await asyncio.sleep(1.0)
        
        elapsed = time.time() - agent.start_time
        
        if agent.phase == 'POSITIONING':
            pos_global = agent.position_global
            target_global = agent.target_local + agent.spawn_offset
            distance = np.linalg.norm(pos_global - target_global)
            
            if elapsed % 5 == 0:  # Log every 5 seconds
                agent.get_logger().info(
                    f'[{elapsed:.0f}s] Position: {pos_global} | '
                    f'Target: {target_global} | Distance: {distance:.2f}m'
                )
        
        elif agent.phase == 'VORONOI':
            if elapsed % 10 == 0:  # Log every 10 seconds
                agent.get_logger().info(
                    f'[{elapsed:.0f}s] VORONOI mode | Velocity: {agent.voronoi_velocity}'
                )


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Unified Voronoi Agent')
    parser.add_argument('--instance', type=int, required=True,
                        help='Drone instance number (1-5)')
    parser.add_argument('--role', type=str, required=True, choices=['pursuer', 'evader'],
                        help='Agent role')
    args = parser.parse_args()
    
    # Load config
    config = load_config()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Run agent
        asyncio.run(run_agent(args.instance, args.role, config))
    except KeyboardInterrupt:
        print(f"\n⚠️  Agent {args.instance} interrupted")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
