#!/usr/bin/env python3
"""
Drone Agent for Voronoi Pursuit-Evasion
Individual drone controller that subscribes to Voronoi velocity commands

This agent:
- Subscribes to /voronoi/cmd_vel/instance_X for velocity commands
- Controls its own drone via ROS2 setpoints
- Logs all data to individual CSV file
- Responds to game status changes

Usage:
    python3 drone_agent.py --instance 1 --role pursuer
    python3 drone_agent.py --instance 5 --role evader
"""

import asyncio
import argparse
import yaml
import numpy as np
from pathlib import Path
import sys
import os
import csv
from datetime import datetime
import threading

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


class DroneAgent(Node):
    def __init__(self, instance, role, config):
        super().__init__(f'drone_agent_{instance}')
        
        self.instance = instance
        self.role = role  # 'pursuer' or 'evader'
        self.config = config
        
        # Ports
        self.mavsdk_port = 50050 + instance
        self.mavlink_port = 14540 + instance
        self.namespace = f'/px4_{instance}/fmu'
        
        # State
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.cmd_velocity = np.array([0.0, 0.0, 0.0])
        self.game_active = False
        self.capture_detected = False
        
        # MAVSDK drone object
        self.drone = None
        
        # Data logging
        self.log_data = []
        self.start_time = None
        
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
        
        # Publisher for agent active signal (handoff coordination)
        self.agent_active_pub = self.create_publisher(
            Bool, f'/voronoi/agent_active/instance_{instance}', 10)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.namespace}/out/vehicle_local_position_v1',
            self.position_callback,
            qos
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            f'{self.namespace}/out/vehicle_attitude_v1',
            self.attitude_callback,
            qos
        )
        
        # Subscribe to Voronoi velocity commands
        cmd_vel_topic = f"{config['topics']['voronoi_cmd_vel']}/instance_{instance}"
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to game status
        self.game_status_sub = self.create_subscription(
            String,
            config['topics']['game_status'],
            self.game_status_callback,
            10
        )
        
        self.capture_sub = self.create_subscription(
            String,
            config['topics']['capture_detected'],
            self.capture_callback,
            10
        )
        
        # Control timer (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Agent active announcement timer (publish for 5 seconds after start)
        self.agent_active = False
        self.agent_active_count = 0
        self.agent_active_timer = self.create_timer(0.1, self.announce_active)
        
        # Logging timer (50Hz)
        if config['logging']['enabled']:
            log_period = 1.0 / config['logging']['frequency']
            self.log_timer = self.create_timer(log_period, self.log_callback)
        
        self.get_logger().info(f'Drone Agent {instance} ({role}) initialized')
    
    def announce_active(self):
        """Announce that agent is active to signal takeover from initial_positions.py"""
        if self.agent_active_count < 50:  # Announce for ~5 seconds
            msg = Bool()
            msg.data = True
            self.agent_active_pub.publish(msg)
            self.agent_active_count += 1
            if self.agent_active_count == 1:
                self.get_logger().info('📡 Broadcasting agent active - taking over control')
        else:
            self.agent_active_timer.cancel()  # Stop announcing after 5 seconds
    
    def position_callback(self, msg):
        """Store position and velocity"""
        self.position = np.array([msg.x, msg.y, msg.z])
        self.velocity = np.array([msg.vx, msg.vy, msg.vz])
    
    def attitude_callback(self, msg):
        """Store attitude (convert quaternion to Euler)"""
        # Simplified conversion (for logging purposes)
        # Full conversion would use quaternion_to_euler
        self.attitude = np.array([msg.q[0], msg.q[1], msg.q[2]])  # Placeholder
    
    def cmd_vel_callback(self, msg):
        """Receive velocity command from Voronoi node"""
        self.cmd_velocity = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    
    def game_status_callback(self, msg):
        """Handle game status changes"""
        if msg.data == 'ACTIVE' and not self.game_active:
            self.game_active = True
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('🎮 Game activated!')
        elif msg.data == 'CAPTURED':
            self.game_active = False
            self.get_logger().info('🏁 Game ended - capture detected')
    
    def capture_callback(self, msg):
        """Handle capture event"""
        if not self.capture_detected:
            self.capture_detected = True
            self.get_logger().info(f'📡 {msg.data}')
            # Save logged data
            self.save_log()
    
    def timestamp(self):
        """Get timestamp in microseconds"""
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def publish_offboard_mode(self):
        """Publish OffboardControlMode heartbeat"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True  # Using velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp()
        self.offboard_mode_pub.publish(msg)
    
    def publish_velocity_setpoint(self, vx, vy, vz):
        """Publish velocity setpoint"""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]  # Ignore position
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = float('nan')  # Ignore yaw
        msg.timestamp = self.timestamp()
        self.setpoint_pub.publish(msg)
    
    def control_loop(self):
        """Main control loop at 50Hz"""
        # Always publish offboard heartbeat
        self.publish_offboard_mode()
        
        if self.game_active and not self.capture_detected:
            # Publish velocity command from Voronoi
            self.publish_velocity_setpoint(
                self.cmd_velocity[0],
                self.cmd_velocity[1],
                self.cmd_velocity[2]
            )
        else:
            # Hover in place (zero velocity)
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
    
    def log_callback(self):
        """Log data at specified frequency"""
        if not self.game_active or self.start_time is None:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        timestamp = current_time - self.start_time
        
        # Compute distance to evader (for pursuers)
        distance_to_evader = 0.0  # Will be computed from positions if needed
        
        data_row = {
            'timestamp': timestamp,
            'drone_id': self.instance,
            'role': self.role,
            'x': self.position[0],
            'y': self.position[1],
            'z': self.position[2],
            'vx': self.velocity[0],
            'vy': self.velocity[1],
            'vz': self.velocity[2],
            'roll': self.attitude[0],
            'pitch': self.attitude[1],
            'yaw': self.attitude[2],
            'cmd_vx': self.cmd_velocity[0],
            'cmd_vy': self.cmd_velocity[1],
            'cmd_vz': self.cmd_velocity[2],
        }
        
        self.log_data.append(data_row)
    
    def save_log(self):
        """Save logged data to CSV file"""
        if not self.log_data:
            self.get_logger().warning('No data to save')
            return
        
        # Create data directory
        data_dir = Path(__file__).parent / self.config['logging']['directory']
        data_dir.mkdir(exist_ok=True)
        
        # Generate filename with timestamp
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = data_dir / f'drone_{self.instance}_{self.role}_{timestamp_str}.csv'
        
        # Write CSV
        with open(filename, 'w', newline='') as f:
            if self.log_data:
                writer = csv.DictWriter(f, fieldnames=self.log_data[0].keys())
                writer.writeheader()
                writer.writerows(self.log_data)
        
        self.get_logger().info(f'💾 Data saved to {filename}')


async def run_agent(instance, role, config):
    """Run the drone agent"""
    # Initialize ROS2
    rclpy.init()
    
    # Create agent node
    agent = DroneAgent(instance, role, config)
    
    # Spin ROS2 in background thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(agent),
        daemon=True
    )
    spin_thread.start()
    
    try:
        # Initialize MAVSDK
        agent.get_logger().info(f'Connecting MAVSDK on port {agent.mavsdk_port}...')
        agent.drone = System(port=agent.mavsdk_port)
        await agent.drone.connect(system_address=f"udp://:{agent.mavlink_port}")
        
        # Wait for connection
        async for state in agent.drone.core.connection_state():
            if state.is_connected:
                agent.get_logger().info('MAVSDK connected!')
                break
        
        # Wait for health
        async for health in agent.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                agent.get_logger().info('Drone ready!')
                break
        
        # Drone is already armed and in offboard mode from initial_positions.py
        # Just wait for game to start
        agent.get_logger().info('✅ Ready for game!')
        agent.get_logger().info('   Waiting for game start signal...')
        
        # Keep running until interrupted
        while rclpy.ok():
            await asyncio.sleep(0.1)
            
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted by user')
    except Exception as e:
        agent.get_logger().error(f'Error: {e}')
    finally:
        # Save any remaining data
        agent.save_log()
        agent.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Drone agent for Voronoi pursuit-evasion')
    parser.add_argument('--instance', '-i', type=int, required=True,
                        help='Drone instance number (1-5)')
    parser.add_argument('--role', '-r', type=str, required=True,
                        choices=['pursuer', 'evader'],
                        help='Drone role')
    
    args = parser.parse_args()
    
    # Load config
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    print(f"\n🚁 Drone Agent {args.instance} ({args.role})")
    print(f"   MAVLink: {14540 + args.instance}")
    print(f"   MAVSDK:  {50050 + args.instance}\n")
    
    asyncio.run(run_agent(args.instance, args.role, config))


if __name__ == '__main__':
    main()
