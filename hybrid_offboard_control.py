#!/usr/bin/env python3
"""
Hybrid MAVSDK + ROS2 Offboard Control
Uses MAVSDK for arming/mode commands, ROS2 for trajectory control

Usage:
  python3 hybrid_offboard_control.py --instance 1 --altitude 50.0 --hover 5.0
  python3 hybrid_offboard_control.py -i 2 -a 30.0 -h 3.0
"""

import asyncio
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import time
import numpy as np


class HybridOffboardControl(Node):
    def __init__(self, instance, mavsdk_port, mavlink_port):
        super().__init__(f'hybrid_offboard_{instance}')
        
        self.instance = instance
        self.mavsdk_port = mavsdk_port
        self.mavlink_port = mavlink_port
        self.namespace = f'/px4_{instance}/fmu'
        
        self.get_logger().info(f'Hybrid control for drone {instance}: MAVLink={mavlink_port}, MAVSDK gRPC={mavsdk_port}')
        
        # ROS2 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS2 Publishers (for trajectory control)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'{self.namespace}/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{self.namespace}/in/trajectory_setpoint', qos)
        
        # ROS2 Subscribers (for telemetry)
        self.status_sub = self.create_subscription(
            VehicleStatus, f'{self.namespace}/out/vehicle_status_v1', self.status_callback, qos)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, f'{self.namespace}/out/vehicle_local_position_v1', self.pos_callback, qos)
        
        # State
        self.pos = np.array([0.0, 0.0, 0.0])
        self.armed = False
        self.in_offboard = False
        
        # MAVSDK drone object (initialized in async context)
        self.drone = None
        
        # Timer for ROS2 control loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Hybrid controller initialized')
    
    def status_callback(self, msg):
        prev_armed = self.armed
        prev_offboard = self.in_offboard
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.in_offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        if self.armed != prev_armed:
            self.get_logger().info(f'Armed: {self.armed}')
        if self.in_offboard != prev_offboard:
            self.get_logger().info(f'Offboard: {self.in_offboard}')
    
    def pos_callback(self, msg):
        self.pos = np.array([msg.x, msg.y, msg.z])
    
    def timestamp(self):
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
    
    def publish_setpoint(self, x, y, z):
        """Publish trajectory setpoint"""
        # Store target for control loop
        self.target_setpoint = np.array([x, y, z])
        
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = self.timestamp()
        self.setpoint_pub.publish(msg)
    
    def control_loop(self):
        """ROS2 control loop - publishes heartbeat + setpoint at 50Hz"""
        # Always publish offboard heartbeat
        self.publish_offboard_mode()
        
        # Publish current target setpoint
        self.publish_setpoint(self.target_setpoint[0], self.target_setpoint[1], self.target_setpoint[2])


async def run_mission(node, target_altitude=50.0, hover_time=5.0):
    """Async mission using MAVSDK for commands, ROS2 for control"""
    
    # Initialize MAVSDK
    node.get_logger().info(f'Connecting MAVSDK on port {node.mavsdk_port}...')
    node.drone = System(port=node.mavsdk_port)
    await node.drone.connect(system_address=f"udp://:{node.mavlink_port}")
    
    # Wait for connection
    async for state in node.drone.core.connection_state():
        if state.is_connected:
            node.get_logger().info('MAVSDK connected!')
            break
    
    # Wait for health
    async for health in node.drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            node.get_logger().info('Drone ready!')
            break
    
    # Stream setpoints for 2 seconds (proof-of-life)
    node.get_logger().info('Streaming offboard heartbeat...')
    await asyncio.sleep(2.0)
    
    # Arm via MAVSDK
    node.get_logger().info('Arming via MAVSDK...')
    await node.drone.action.arm()
    await asyncio.sleep(1.0)
    
    # Start offboard mode via MAVSDK
    node.get_logger().info('Switching to offboard mode via MAVSDK...')
    try:
        # Set initial setpoint
        await node.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await node.drone.offboard.start()
        node.get_logger().info('Offboard mode started!')
    except OffboardError as error:
        node.get_logger().error(f'Offboard start failed: {error._result.result}')
        return
    
    # Takeoff via ROS2 trajectory control
    node.get_logger().info(f'Taking off to {target_altitude}m...')
    start_z = node.pos[2]
    duration = 20.0  # 20 seconds to climb
    start_time = time.time()
    
    while time.time() - start_time < duration:
        progress = (time.time() - start_time) / duration
        current_z = start_z + progress * (-target_altitude - start_z)
        node.publish_setpoint(0.0, 0.0, current_z)
        await asyncio.sleep(0.02)
        
        # Check if reached
        if abs(node.pos[2] - (-target_altitude)) < 2.0:
            break
    
    node.get_logger().info(f'Reached {target_altitude}m')
    
    # Hover
    node.get_logger().info(f'Hovering for {hover_time}s...')
    hover_start = time.time()
    while time.time() - hover_start < hover_time:
        node.publish_setpoint(0.0, 0.0, -target_altitude)
        await asyncio.sleep(0.02)
    
    # Land via MAVSDK
    node.get_logger().info('Landing via MAVSDK...')
    await node.drone.offboard.stop()
    await node.drone.action.land()
    
    # Wait for landing
    async for position in node.drone.telemetry.position():
        if abs(position.relative_altitude_m) < 0.5:
            break
    
    node.get_logger().info('Landed! (Auto-disarm will occur)')
    node.get_logger().info('Mission complete!')


def ros2_spin(node):
    """Spin ROS2 in a thread with dedicated executor"""
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()


async def main(instance, altitude, hover_time):
    """Main function"""
    # Initialize ROS2
    rclpy.init()
    
    # Calculate ports based on instance
    # PX4 multi-instance port pattern:
    # - MAVLink: 14540 + instance (14541, 14542, 14543...)
    # - MAVSDK gRPC: 50050 + instance (50051, 50052, 50053...)
    mavsdk_port = 50050 + instance
    mavlink_port = 14540 + instance
    
    # Create hybrid controller
    node = HybridOffboardControl(
        instance=instance,
        mavsdk_port=mavsdk_port,
        mavlink_port=mavlink_port
    )
    
    # Run ROS2 spin in executor (non-blocking)
    import threading
    spin_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run mission
    try:
        await run_mission(node, target_altitude=altitude, hover_time=hover_time)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted')
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Hybrid MAVSDK+ROS2 offboard control for PX4 multi-drone',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-i', '--instance',
        type=int,
        default=1,
        help='Drone instance number (1, 2, 3...)'
    )
    parser.add_argument(
        '-a', '--altitude',
        type=float,
        default=50.0,
        help='Target takeoff altitude in meters'
    )
    parser.add_argument(
        '-t', '--hover',
        type=float,
        default=5.0,
        help='Hover time in seconds'
    )
    args = parser.parse_args()
    
    print(f"🚁 Starting mission for Drone {args.instance}")
    print(f"   MAVLink Port: {14540 + args.instance}")
    print(f"   MAVSDK Port:  {50050 + args.instance}")
    print(f"   Altitude:     {args.altitude}m")
    print(f"   Hover Time:   {args.hover}s\n")
    
    asyncio.run(main(args.instance, args.altitude, args.hover))
