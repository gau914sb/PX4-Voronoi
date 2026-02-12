#!/usr/bin/env python3
"""
Voronoi Computation Node
Centralized Voronoi controller that computes velocity commands for all drones

This node:
- Subscribes to all 5 drone positions
- Computes Voronoi pursuit-evasion velocities at 50Hz
- Publishes velocity commands to /voronoi/cmd_vel/instance_X topics
- Detects capture condition
- Broadcasts game status

Usage:
    python3 voronoi_computation_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import numpy as np
import yaml
from pathlib import Path
import time

from voronoi_controller import pursuer_voronoi_controller, evader_voronoi_controller


class VoronoiComputationNode(Node):
    def __init__(self):
        super().__init__('voronoi_computation_node')
        
        # Load configuration
        self.config = self.load_config()
        
        # Extract config parameters
        self.pursuer_instances = self.config['pursuers']['instances']
        self.evader_instance = self.config['evader']['instance']
        self.all_instances = self.pursuer_instances + [self.evader_instance]
        
        self.gamma = self.config['voronoi']['gamma']
        self.pursuer_strategy = self.config['voronoi']['pursuer_strategy']
        self.evader_strategy = self.config['voronoi']['evader_strategy']
        self.max_velocity = self.config['voronoi']['max_velocity']
        self.capture_radius = self.config['game']['capture_radius']
        
        # Spawn offsets for coordinate frame conversion
        # Drones spawn at different positions: X = (instance - 1) * 2.0 in NED
        # See COORDINATE_FRAMES.md for details
        self.spawn_offsets = {
            instance: np.array([(instance - 1) * 2.0, 0.0, 0.0]) 
            for instance in self.all_instances
        }
        
        # State tracking (stored in LOCAL frame, converted to GLOBAL for computation)
        self.positions_local = {i: None for i in self.all_instances}
        self.game_active = False
        self.capture_detected = False
        self.captured_by = None
        self.capture_time = None
        self.shutdown_timer = None
        
        # ROS2 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for velocity commands
        self.cmd_vel_pubs = {}
        for instance in self.all_instances:
            topic = f"{self.config['topics']['voronoi_cmd_vel']}/instance_{instance}"
            self.cmd_vel_pubs[instance] = self.create_publisher(Twist, topic, 10)
        
        # Publishers for game status
        self.game_status_pub = self.create_publisher(
            String, self.config['topics']['game_status'], 10)
        self.capture_pub = self.create_publisher(
            String, self.config['topics']['capture_detected'], 10)
        
        # Subscribers for drone positions
        self.position_subs = {}
        for instance in self.all_instances:
            topic = f"/px4_{instance}/fmu/out/vehicle_local_position_v1"
            self.position_subs[instance] = self.create_subscription(
                VehicleLocalPosition,
                topic,
                lambda msg, inst=instance: self.position_callback(msg, inst),
                qos
            )
        
        # Subscriber for game start signal
        self.game_start_sub = self.create_subscription(
            Bool,
            "/voronoi/start_game",
            self.game_start_callback,
            10
        )
        
        # Control loop timer (50Hz)
        control_period = 1.0 / self.config['game']['control_frequency']
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info('Voronoi Computation Node initialized')
        self.get_logger().info(f'Pursuers: {self.pursuer_instances}')
        self.get_logger().info(f'Evader: {self.evader_instance}')
        self.get_logger().info(f'Waiting for game start signal on /voronoi/start_game')
    
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = Path(__file__).parent / "config.yaml"
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def local_to_global(self, local_pos, instance):
        """Convert position from local (drone's home) to global frame"""
        return local_pos + self.spawn_offsets[instance]
    
    def get_global_position(self, instance):
        """Get drone position in global frame"""
        local_pos = self.positions_local[instance]
        if local_pos is None:
            return None
        return self.local_to_global(local_pos, instance)
    
    def position_callback(self, msg, instance):
        """Store position data from drone (in LOCAL frame)"""
        self.positions_local[instance] = np.array([msg.x, msg.y, msg.z])
    
    def game_start_callback(self, msg):
        """Handle game start signal"""
        if msg.data and not self.game_active:
            self.game_active = True
            self.get_logger().info('🎮 GAME STARTED!')
            # Broadcast game status
            status_msg = String()
            status_msg.data = 'ACTIVE'
            self.game_status_pub.publish(status_msg)
    
    def check_all_positions_valid(self):
        """Check if we have position data from all drones"""
        for instance, pos in self.positions_local.items():
            if pos is None:
                return False
        return True
    
    def check_capture(self):
        """Check if any pursuer has captured the evader (using GLOBAL positions)"""
        if self.capture_detected:
            return True
        
        # Use GLOBAL positions for distance calculation
        evader_pos_global = self.get_global_position(self.evader_instance)
        
        for pursuer_instance in self.pursuer_instances:
            pursuer_pos_global = self.get_global_position(pursuer_instance)
            distance = np.linalg.norm(pursuer_pos_global - evader_pos_global)
            
            if distance < self.capture_radius:
                self.capture_detected = True
                self.captured_by = pursuer_instance
                self.capture_time = time.time()
                
                self.get_logger().info(f'🎯 CAPTURED!')
                self.get_logger().info(f'   Pursuer {pursuer_instance} caught evader')
                self.get_logger().info(f'   Distance: {distance:.3f}m')
                
                # Publish capture event
                capture_msg = String()
                capture_msg.data = f'CAPTURED_BY_{pursuer_instance}'
                self.capture_pub.publish(capture_msg)
                
                # Update game status
                status_msg = String()
                status_msg.data = 'CAPTURED'
                self.game_status_pub.publish(status_msg)
                
                # Schedule shutdown after 2 seconds
                self.get_logger().info('Shutting down in 2 seconds...')
                self.shutdown_timer = self.create_timer(2.0, self.shutdown_callback)
                
                return True
        
        return False
    
    def shutdown_callback(self):
        """Gracefully shutdown the computation node"""
        self.get_logger().info('✅ Computation node shutdown complete')
        raise SystemExit(0)
    
    def control_loop(self):
        """Main control loop at 50Hz"""
        if not self.game_active or self.capture_detected:
            return
        
        # Check if we have all position data
        if not self.check_all_positions_valid():
            return
        
        # Extract positions in GLOBAL frame for Voronoi computation
        pursuer_positions_global = np.array([
            self.get_global_position(i) for i in self.pursuer_instances
        ])
        evader_position_global = self.get_global_position(self.evader_instance)
        
        # Compute relative positions (pursuers relative to evader) in GLOBAL frame
        relative_positions = pursuer_positions_global - evader_position_global
        
        try:
            # Compute Voronoi control velocities
            pursuer_vels, voronoi_corners, A, c, furthest_idx, K = pursuer_voronoi_controller(
                relative_positions,
                strategy=self.pursuer_strategy,
                dimension=3,
                gamma=self.gamma,
                norm_bound=self.max_velocity
            )
            
            evader_vel = evader_voronoi_controller(
                voronoi_corners,
                K,
                strategy=self.evader_strategy,
                norm_bound=self.max_velocity
            )
            
            # Publish velocity commands
            # Note: Velocities are frame-independent (same in local and global NED frames)
            # We compute in global frame but velocities apply equally in local frame
            for i, pursuer_instance in enumerate(self.pursuer_instances):
                cmd = Twist()
                cmd.linear.x = float(pursuer_vels[i][0])
                cmd.linear.y = float(pursuer_vels[i][1])
                cmd.linear.z = float(pursuer_vels[i][2])
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pubs[pursuer_instance].publish(cmd)
            
            # Publish evader velocity
            cmd = Twist()
            cmd.linear.x = float(evader_vel[0])
            cmd.linear.y = float(evader_vel[1])
            cmd.linear.z = float(evader_vel[2])
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pubs[self.evader_instance].publish(cmd)
            
            # Check for capture
            self.check_capture()
            
        except Exception as e:
            self.get_logger().error(f'Error in Voronoi controller: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = VoronoiComputationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Voronoi computation node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
