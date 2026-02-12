#!/usr/bin/env python3
"""
Single Drone Takeoff and Land Controller using ROS2/PX4.

This script controls ONE drone. For multiple drones, launch multiple instances
with different drone_instance parameters.

Test sequence:
1. Arms the drone
2. Ascends to target altitude (default 50m)
3. Hovers for specified duration (default 5s)
4. Lands

Usage (instance numbers start from 1 as per official PX4 docs):
    # Drone 1 (uses /px4_1/fmu namespace):
    ros2 run test_maneuver single_drone_takeoff_land --ros-args -p drone_instance:=1 -p target_altitude:=50.0
    
    # Drone 2 (uses /px4_2/fmu namespace):
    ros2 run test_maneuver single_drone_takeoff_land --ros-args -p drone_instance:=2 -p target_altitude:=50.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
    VehicleLocalPosition
)
import time
import numpy as np


class State:
    INIT = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER = 3
    LANDING = 4
    DONE = 5


STATE_NAMES = ["INIT", "ARMING", "TAKEOFF", "HOVER", "LANDING", "DONE"]


class SingleDroneTakeoffLand(Node):
    def __init__(self):
        super().__init__('single_drone_takeoff_land')
        
        # Declare parameters
        self.declare_parameter('drone_instance', 1)
        self.declare_parameter('target_altitude', 50.0)
        self.declare_parameter('hover_duration', 5.0)
        
        # Get parameters
        self.instance = self.get_parameter('drone_instance').get_parameter_value().integer_value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.hover_duration = self.get_parameter('hover_duration').get_parameter_value().double_value
        
        # System ID matches instance number (1, 2, 3, ...)
        self.system_id = self.instance
        
        # Namespace: all instances use /px4_N/fmu format (as per official PX4 docs)
        self.namespace = f'/px4_{self.instance}/fmu'
        
        self.get_logger().info(f'Drone Instance {self.instance} (System ID {self.system_id}) using namespace: {self.namespace}')
        self.get_logger().info(f'Target altitude: {self.target_altitude}m, Hover duration: {self.hover_duration}s')
        
        # QoS profile
        # QoS for publishers to PX4 (VOLATILE for input topics)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for subscribers from PX4 (VOLATILE - must match PX4 publisher QoS)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f'{self.namespace}/in/trajectory_setpoint', qos_pub)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{self.namespace}/in/offboard_control_mode', qos_pub)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f'{self.namespace}/in/vehicle_command', qos_pub)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, f'{self.namespace}/out/vehicle_status_v1', 
            self.status_callback, qos_sub)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, f'{self.namespace}/out/vehicle_local_position_v1',
            self.local_pos_callback, qos_sub)
        self.cmd_ack_sub = self.create_subscription(
            VehicleCommandAck, f'{self.namespace}/out/vehicle_command_ack',
            self.command_ack_callback, qos_sub)
        
        # State variables
        self.armed = False
        self.in_offboard = False
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        
        # State machine
        self.state = State.INIT
        self.state_start_time = None
        self.setpoint_counter = 0
        
        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Single Drone Controller Initialized')
    
    def status_callback(self, msg):
        prev_armed = self.armed
        prev_offboard = self.in_offboard
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.in_offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        # Debug: First callback received
        if not hasattr(self, '_status_callback_received'):
            self._status_callback_received = True
            self.get_logger().info(f'[Instance {self.instance}] First status callback: arming_state={msg.arming_state}, nav_state={msg.nav_state}')
        
        # Debug: Log state changes
        if self.armed != prev_armed:
            self.get_logger().info(f'[Instance {self.instance}] Armed state changed: {prev_armed} -> {self.armed}')
        if self.in_offboard != prev_offboard:
            self.get_logger().info(f'[Instance {self.instance}] Offboard state changed: {prev_offboard} -> {self.in_offboard}')
    
    def local_pos_callback(self, msg):
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading
    
    def command_ack_callback(self, msg):
        """Log command acknowledgments to see why arming might fail."""
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            result_str = "SUCCESS" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else f"FAILED (code {msg.result})"
            self.get_logger().info(f'[Instance {self.instance}] ARM command result: {result_str}')
        elif msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            result_str = "SUCCESS" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else f"FAILED (code {msg.result})"
            self.get_logger().info(f'[Instance {self.instance}] SET_MODE command result: {result_str}')
    
    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def state_elapsed(self):
        if self.state_start_time is None:
            return 0.0
        return time.time() - self.state_start_time
    
    def transition(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'[Instance {self.instance}] State: {STATE_NAMES[new_state]}')
    
    def publish_offboard_mode(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp()
        self.offboard_pub.publish(msg)
    
    def publish_position_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        self.trajectory_pub.publish(msg)
    
    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.param4 = float(param4)
        cmd.param5 = float(param5)
        cmd.param6 = float(param6)
        cmd.param7 = float(param7)
        cmd.target_system = self.system_id
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.confirmation = 0
        cmd.from_external = True
        cmd.timestamp = self.timestamp()
        self.cmd_pub.publish(cmd)
    
    def control_loop(self):
        """Main control loop - publish offboard heartbeat + ONE setpoint per cycle"""
        
        # CRITICAL: Always publish offboard control mode (proof-of-life heartbeat)
        # PX4 requires this stream for 1+ seconds BEFORE accepting offboard mode switch
        self.publish_offboard_mode(position=True)
        
        # DO NOT publish default setpoint here - causes conflict!
        # Each state publishes its own setpoint exactly once
        
        self.setpoint_counter += 1
        
        if self.state == State.INIT:
            # Publish init setpoint
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            
            # Stream setpoints for ~2 seconds (100 cycles at 50Hz) before attempting mode switch
            # PX4 requires proof-of-life for at least 1 second, give it 2 to be safe
            if self.setpoint_counter >= 100:
                self.transition(State.ARMING)
        
        elif self.state == State.ARMING:
            # Publish arming setpoint
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            
            # Throttle commands to ~1Hz (every 50 loops at 50Hz)
            if self.setpoint_counter % 50 == 0:
                if not self.in_offboard:
                    self.get_logger().info(f'[Instance {self.instance}] Requesting Offboard Mode (armed={self.armed}, offboard={self.in_offboard})')
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed:
                    self.get_logger().info(f'[Instance {self.instance}] Requesting Arm (armed={self.armed}, offboard={self.in_offboard})')
                    self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            if self.in_offboard and self.armed:
                self.transition(State.TAKEOFF)
        
        elif self.state == State.TAKEOFF:
            # Ramp altitude slowly (3 m/s)
            current_target = 5.0 + (3.0 * self.state_elapsed())
            target_z = min(current_target, self.target_altitude)
            
            # Publish takeoff setpoint
            self.publish_position_setpoint(0.0, 0.0, -target_z)
            
            # Check if reached altitude (with 2m tolerance)
            if abs(self.pos[2] - (-self.target_altitude)) < 2.0:
                self.transition(State.HOVER)
        
        elif self.state == State.HOVER:
            # Publish hover setpoint
            self.publish_position_setpoint(0.0, 0.0, -self.target_altitude)
            
            # Hover for specified duration
            if self.state_elapsed() > self.hover_duration:
                self.get_logger().info(f'[Instance {self.instance}] Hover complete. Landing.')
                self.transition(State.LANDING)
        
        elif self.state == State.LANDING:
            # Publish descending setpoint
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            # Check if disarmed (landed)
            if not self.armed:
                self.transition(State.DONE)
        
        elif self.state == State.DONE:
            self.get_logger().info(f'[Instance {self.instance}] Mission complete. Shutting down.')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SingleDroneTakeoffLand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('Stopping')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
