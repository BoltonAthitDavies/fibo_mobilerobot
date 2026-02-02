#!/usr/bin/env python3
"""
EKF Odometry Fusion Node
Fuses wheel odometry from joint_states and IMU data using Extended Kalman Filter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import numpy as np
import math


class EKFOdometry(Node):
    def __init__(self):
        super().__init__('ekf_odometry_node')
        
        # Robot parameters (Turtlebot3 Burger)
        self.wheel_radius = 0.033  # meters
        self.wheel_separation = 0.160  # meters
        
        # State vector: [x, y, theta, v, omega]
        self.state = np.zeros(5)
        
        # Covariance matrix
        self.P = np.eye(5) * 0.1
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        
        # Measurement noise
        self.R_odom = np.diag([0.1, 0.1])  # For velocity measurements
        self.R_imu = np.array([[0.01]])  # For angular velocity
        
        # Previous joint states
        self.prev_joint_positions = None
        self.prev_time = None
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info('EKF Odometry node initialized')
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def predict(self, dt):
        """EKF Prediction step"""
        x, y, theta, v, omega = self.state
        
        # State prediction using motion model
        self.state[0] += v * math.cos(theta) * dt
        self.state[1] += v * math.sin(theta) * dt
        self.state[2] += omega * dt
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Jacobian of motion model
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
    
    def update_velocity(self, v_meas, omega_meas):
        """EKF Update step with velocity measurements"""
        # Measurement model: z = [v, omega]
        z = np.array([v_meas, omega_meas])
        h = np.array([self.state[3], self.state[4]])
        
        # Measurement Jacobian
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_odom
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        
        # Covariance update
        self.P = (np.eye(5) - K @ H) @ self.P
    
    def update_imu(self, omega_meas):
        """EKF Update step with IMU angular velocity"""
        # Measurement model: z = omega
        z = np.array([omega_meas])
        h = np.array([self.state[4]])
        
        # Measurement Jacobian
        H = np.zeros((1, 5))
        H[0, 4] = 1.0
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        
        # Covariance update
        self.P = (np.eye(5) - K @ H) @ self.P
    
    def joint_callback(self, msg):
        """Process joint states to compute wheel odometry"""
        current_time = self.get_clock().now()
        
        if self.prev_joint_positions is None:
            self.prev_joint_positions = msg.position
            self.prev_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Get wheel velocities
        # Assuming wheel_left_joint is index 0 and wheel_right_joint is index 1
        left_vel = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
        right_vel = msg.velocity[1] if len(msg.velocity) > 1 else 0.0
        
        # Compute linear and angular velocities
        v = self.wheel_radius * (right_vel + left_vel) / 2.0
        omega = self.wheel_radius * (right_vel - left_vel) / self.wheel_separation
        
        # Prediction step
        self.predict(dt)
        
        # Update step with wheel odometry
        self.update_velocity(v, omega)
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update previous values
        self.prev_joint_positions = msg.position
        self.prev_time = current_time
    
    def imu_callback(self, msg):
        """Process IMU data for angular velocity update"""
        # Extract angular velocity around z-axis
        omega_z = msg.angular_velocity.z
        
        # Update with IMU measurement
        self.update_imu(omega_z)
    
    def publish_odometry(self, current_time):
        """Publish odometry message and TF transform"""
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.state[2])
        
        # Set velocities
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.angular.z = self.state[4]
        
        # Set covariance (simplified)
        odom.pose.covariance[0] = self.P[0, 0]
        odom.pose.covariance[7] = self.P[1, 1]
        odom.pose.covariance[35] = self.P[2, 2]
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.state[2])
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
