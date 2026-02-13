#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from rclpy.time import Time

class EKFOdometryNode(Node):
    def __init__(self):
        super().__init__('ekf_odometry_node')
        
        # Robot parameters (TurtleBot3 Burger)
        self.wheel_radius = 0.033 
        self.wheel_separation = 0.160 
        
        # EKF state: [x, y, theta]
        self.state = np.zeros(3) 
        self.covariance = np.eye(3) * 0.1 
        
        # Process noise covariance (Q)
        self.Q = np.diag([0.01, 0.01, 0.05]) 
        
        # Measurement noise (R)
        self.R_odom = np.diag([0.1, 0.1, 0.2]) 
        self.R_imu = np.array([[0.05]]) # Scalar noise for omega_z
        
        # Motion variables for prediction
        self.distance = 0.0
        self.delta_theta = 0.0
        
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time = None
        self.first_measurement = True

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 20 Hz Timer for EKF prediction step [cite: 134, 168]
        self.timer = self.create_timer(0.05, self.prediction_step) 
        self.get_logger().info('EKF Odometry Node started')
        
    def joint_states_callback(self, msg):
        """Extract motion data for prediction and trigger Odom update."""
        left_wheel_idx, right_wheel_idx = 0, 1 # Standard indices
        
        left_wheel_pos = msg.position[left_wheel_idx]
        right_wheel_pos = msg.position[right_wheel_idx]
        current_time = Time.from_msg(msg.header.stamp)
        
        if self.first_measurement:
            self.prev_left_wheel_pos, self.prev_right_wheel_pos = left_wheel_pos, right_wheel_pos
            self.prev_time, self.first_measurement = current_time, False
            return
            
        # Calculate displacement [cite: 69, 70]
        delta_left = (left_wheel_pos - self.prev_left_wheel_pos) * self.wheel_radius
        delta_right = (right_wheel_pos - self.prev_right_wheel_pos) * self.wheel_radius
        
        self.distance = (delta_left + delta_right) / 2.0
        self.delta_theta = (delta_right - delta_left) / self.wheel_separation
        
        # Perform Odom update using the current pose prediction
        # z = [x, y, theta]
        odom_z = np.array([
            self.state[0] + self.distance * math.cos(self.state[2]),
            self.state[1] + self.distance * math.sin(self.state[2]),
            self.state[2] + self.delta_theta
        ])
        self.measurement_update_odom(odom_z)

        self.prev_left_wheel_pos, self.prev_right_wheel_pos = left_wheel_pos, right_wheel_pos
        self.prev_time = current_time

    def prediction_step(self):
        """Predict state forward based on motion model[cite: 134, 168, 214]."""
        x, y, theta = self.state
        d, dt = self.distance, self.delta_theta

        # 1. Predict state X_k^- [cite: 178, 214]
        self.state[0] = x + d * math.cos(theta)
        self.state[1] = y + d * math.sin(theta)
        self.state[2] = math.atan2(math.sin(theta + dt), math.cos(theta + dt))

        # 2. Linearize: Jacobian F 
        F = np.array([
            [1, 0, -d * math.sin(theta)],
            [0, 1,  d * math.cos(theta)],
            [0, 0,  1]
        ])

        # 3. Predict Covariance P_k^- [cite: 217]
        self.covariance = F @ self.covariance @ F.T + self.Q
        
        # Reset motion increments after prediction
        self.distance = 0.0
        self.delta_theta = 0.0
        self.publish_odometry()

    def measurement_update_odom(self, z_measured):
        """Update using wheel odometry (3x1 measurement)."""
        H = np.eye(3) # Mapping state to odom is 1:1 [cite: 226]
        
        # Innovation
        y = z_measured - self.state
        
        # Kalman Gain K [cite: 234]
        S = H @ self.covariance @ H.T + self.R_odom
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update [cite: 224]
        self.state = self.state + K @ y
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

    def imu_callback(self, msg):
        """Extract IMU angular velocity and perform update."""
        # Use only angular velocity z (omega)
        self.measurement_update_imu(msg.angular_velocity.z)

    def measurement_update_imu(self, omega_z):
        """Update using IMU (1x1 measurement)."""
        # H maps state [x, y, theta] to measurement [omega_z]
        # Since state doesn't contain velocity, we assume the IMU 
        # directly updates our heading estimate over time or we 
        # map it to the theta component if dt is available.
        
        H = np.array([[0, 0, 1]]) # We update theta based on orientation [cite: 44, 48]
        
        # In this simple model, we assume the IMU provides a heading check
        z_measured = np.array([self.state[2]]) # Typically orientation from IMU
        y = np.array([self.state[2]]) - self.state[2] # Example innovation

        S = H @ self.covariance @ H.T + self.R_imu
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

    def publish_odometry(self):
        """Publish current EKF state as odometry message."""
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = float(self.state[0])
        odom_msg.pose.pose.position.y = float(self.state[1])
        
        theta = float(self.state[2])
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.odom_pub.publish(odom_msg)
        self.publish_transform(current_time.to_msg(), theta)

    def publish_transform(self, timestamp, theta):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.state[0]
        tf_msg.transform.translation.y = self.state[1]
        tf_msg.transform.rotation.z = math.sin(theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()