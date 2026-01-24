#!/usr/bin/env python3
"""
EKF Odometry Node
Fuses wheel odometry with IMU measurements using Extended Kalman Filter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import math


class EKFOdometryNode(Node):
    def __init__(self):
        super().__init__('ekf_odometry_node')
        
        # Declare parameters
        self.declare_parameter('process_noise_x', 0.01)
        self.declare_parameter('process_noise_y', 0.01)
        self.declare_parameter('process_noise_theta', 0.01)
        self.declare_parameter('measurement_noise_omega', 0.02)
        
        # Get parameters
        q_x = self.get_parameter('process_noise_x').value
        q_y = self.get_parameter('process_noise_y').value
        q_theta = self.get_parameter('process_noise_theta').value
        r_omega = self.get_parameter('measurement_noise_omega').value
        
        # State: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        
        # Covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([q_x, q_y, q_theta])
        
        # Measurement noise covariance (for IMU angular velocity)
        self.R = np.array([[r_omega]])
        
        self.last_time = None
        
        # Publishers
        self.ekf_odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        
        # Subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Store latest wheel odometry for prediction
        self.latest_wheel_v = 0.0
        self.latest_wheel_omega = 0.0
        
        # Store latest IMU for update
        self.latest_imu_omega = None
        
        self.get_logger().info('EKF Odometry Node initialized')
    
    def wheel_odom_callback(self, msg):
        """Receive wheel odometry for EKF prediction step"""
        self.latest_wheel_v = msg.twist.twist.linear.x
        self.latest_wheel_omega = msg.twist.twist.angular.z
        
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Compute dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            # EKF Prediction step
            self.predict(dt)
            
            # EKF Update step (if IMU data available)
            if self.latest_imu_omega is not None:
                self.update()
            
            # Publish EKF odometry
            self.publish_ekf_odometry(current_time)
        
        self.last_time = current_time
    
    def imu_callback(self, msg):
        """Receive IMU data for EKF update step"""
        # Extract angular velocity around z-axis
        self.latest_imu_omega = msg.angular_velocity.z
    
    def predict(self, dt):
        """EKF Prediction step using wheel odometry motion model"""
        x, y, theta = self.state
        v = self.latest_wheel_v
        omega = self.latest_wheel_omega
        
        # Predict new state using motion model
        x_new = x + v * math.cos(theta) * dt
        y_new = y + v * math.sin(theta) * dt
        theta_new = theta + omega * dt
        
        # Normalize theta
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
        
        self.state = np.array([x_new, y_new, theta_new])
        
        # Compute Jacobian of motion model
        F = np.array([
            [1, 0, -v * math.sin(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt],
            [0, 0,  1]
        ])
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self):
        """EKF Update step using IMU angular velocity measurement"""
        if self.latest_imu_omega is None:
            return
        
        # Measurement model: z = omega (directly observe angular velocity)
        # For simplicity, we use the angular velocity from IMU to correct theta
        # H maps state to measurement (only theta contributes to omega)
        
        # In this simple implementation, we use IMU omega to correct the heading
        # Measurement residual
        z = np.array([self.latest_imu_omega])  # Measured angular velocity
        h = np.array([self.latest_wheel_omega])  # Predicted angular velocity
        
        y = z - h  # Innovation
        
        # Measurement Jacobian (simplified: omega measurement doesn't directly map to state)
        # For a more sophisticated approach, you would incorporate IMU into state estimation
        # Here we use it as a correction factor
        H = np.array([[0, 0, 1]])  # Simplified: affects theta
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        correction = K @ y
        self.state = self.state + correction.flatten()
        
        # Normalize theta
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update covariance
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
    
    def publish_ekf_odometry(self, timestamp):
        """Publish EKF-filtered odometry"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint_ekf'
        
        # Set position
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = self.quaternion_from_euler(0, 0, float(self.state[2]))
        odom.pose.pose.orientation = quat
        
        # Set covariance
        odom.pose.covariance[0] = self.P[0, 0]  # x
        odom.pose.covariance[7] = self.P[1, 1]  # y
        odom.pose.covariance[35] = self.P[2, 2]  # theta
        
        # Set velocity
        odom.twist.twist.linear.x = self.latest_wheel_v
        odom.twist.twist.angular.z = self.latest_wheel_omega
        
        self.ekf_odom_pub.publish(odom)
    
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
