#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np
import math

class EKFOdometry(Node):
    """
    Extended Kalman Filter for multi-sensor odometry fusion
    Fuses wheel odometry and IMU angular velocity
    State: [x, y, theta, vx, vy, omega]
    """
    
    def __init__(self):
        super().__init__('ekf_odometry')
        
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        
        # State covariance matrix (6x6)
        self.P = np.eye(6) * 0.1
        
        # Process noise covariance (Q)
        # Tune these based on expected system noise
        self.Q = np.diag([
            0.01,  # x process noise
            0.01,  # y process noise  
            0.1,   # theta process noise
            0.1,   # vx process noise
            0.1,   # vy process noise
            0.1    # omega process noise
        ])
        
        # Measurement noise covariances
        self.R_odom = np.diag([0.05, 0.05, 0.1, 0.1, 0.1, 0.1])  # wheel odometry
        self.R_imu = 0.05  # IMU angular velocity (scalar)
        
        # Timing
        self.last_time = None
        self.last_prediction_time = None
        
        # Publishers and subscribers
        self.ekf_odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        
        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_odom_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Flags
        self.initialized = False
        
        self.get_logger().info("EKF Odometry Node Started")
        self.get_logger().info("State: [x, y, theta, vx, vy, omega]")
    
    def wheel_odom_callback(self, msg):
        """Process wheel odometry measurements"""
        current_time = self.get_clock().now()
        
        if not self.initialized:
            self.initialize_from_odom(msg, current_time)
            return
        
        # Prediction step
        self.prediction_step(current_time)
        
        # Update step with wheel odometry
        self.update_with_wheel_odom(msg)
        
        # Publish filtered odometry
        self.publish_ekf_odometry(current_time)
        
        self.last_time = current_time
    
    def imu_callback(self, msg):
        """Process IMU measurements (angular velocity only)"""
        if not self.initialized:
            return
        
        current_time = self.get_clock().now()
        
        # Prediction step
        self.prediction_step(current_time)
        
        # Update step with IMU angular velocity
        self.update_with_imu(msg)
        
        # Publish filtered odometry  
        self.publish_ekf_odometry(current_time)
        
        self.last_time = current_time
    
    def initialize_from_odom(self, odom_msg, timestamp):
        """Initialize EKF state from first odometry message"""
        
        # Extract position and orientation
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Extract velocities
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y  
        omega = odom_msg.twist.twist.angular.z
        
        # Initialize state
        self.state = np.array([x, y, yaw, vx, vy, omega])
        
        # Initialize covariance with higher uncertainty
        self.P = np.eye(6) * 0.5
        
        self.last_time = timestamp
        self.last_prediction_time = timestamp
        self.initialized = True
        
        self.get_logger().info(f"EKF initialized at position: ({x:.3f}, {y:.3f}, {yaw:.3f})")
    
    def prediction_step(self, current_time):
        """EKF Prediction step using motion model"""
        
        if self.last_prediction_time is None:
            self.last_prediction_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_prediction_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:  # Skip if dt is invalid or too large
            return
        
        # Current state
        x, y, theta, vx, vy, omega = self.state
        
        # Motion model (constant velocity with rotation)
        # x_k+1 = x_k + vx*cos(theta)*dt - vy*sin(theta)*dt  
        # y_k+1 = y_k + vx*sin(theta)*dt + vy*cos(theta)*dt
        # theta_k+1 = theta_k + omega*dt
        # vx_k+1 = vx_k (assumed constant)
        # vy_k+1 = vy_k (assumed constant) 
        # omega_k+1 = omega_k (assumed constant)
        
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        # Predicted state
        x_pred = x + (vx * cos_theta - vy * sin_theta) * dt
        y_pred = y + (vx * sin_theta + vy * cos_theta) * dt
        theta_pred = theta + omega * dt
        
        # Normalize angle
        theta_pred = math.atan2(math.sin(theta_pred), math.cos(theta_pred))
        
        self.state = np.array([x_pred, y_pred, theta_pred, vx, vy, omega])
        
        # Jacobian of motion model (F matrix)
        F = np.eye(6)
        F[0, 2] = (-vx * sin_theta - vy * cos_theta) * dt  # dx/dtheta
        F[0, 3] = cos_theta * dt  # dx/dvx
        F[0, 4] = -sin_theta * dt  # dx/dvy
        
        F[1, 2] = (vx * cos_theta - vy * sin_theta) * dt  # dy/dtheta
        F[1, 3] = sin_theta * dt  # dy/dvx  
        F[1, 4] = cos_theta * dt  # dy/dvy
        
        F[2, 5] = dt  # dtheta/domega
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q * dt
        
        self.last_prediction_time = current_time
    
    def update_with_wheel_odom(self, odom_msg):
        """EKF Update step with wheel odometry measurement"""
        
        # Extract measurements
        z_x = odom_msg.pose.pose.position.x
        z_y = odom_msg.pose.pose.position.y
        
        quat = odom_msg.pose.pose.orientation
        _, _, z_theta = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        z_vx = odom_msg.twist.twist.linear.x
        z_vy = odom_msg.twist.twist.linear.y
        z_omega = odom_msg.twist.twist.angular.z
        
        # Measurement vector
        z = np.array([z_x, z_y, z_theta, z_vx, z_vy, z_omega])
        
        # Measurement model (identity - we observe all states directly)
        H = np.eye(6)
        
        # Innovation
        y = z - H @ self.state
        
        # Normalize angle difference
        y[2] = math.atan2(math.sin(y[2]), math.cos(y[2]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_odom
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular matrix in EKF update, skipping...")
            return
        
        # Update state and covariance
        self.state = self.state + K @ y
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_odom @ K.T
    
    def update_with_imu(self, imu_msg):
        """EKF Update step with IMU angular velocity"""
        
        # Extract angular velocity measurement (only z-axis for 2D)
        z_omega = imu_msg.angular_velocity.z
        
        # Measurement model (observe only omega)
        H = np.array([[0, 0, 0, 0, 0, 1]])  # 1x6 matrix
        
        # Innovation
        y = z_omega - H @ self.state
        
        # Innovation covariance (scalar)
        S = H @ self.P @ H.T + self.R_imu
        
        # Kalman gain (6x1 vector)
        K = self.P @ H.T / S
        
        # Update state and covariance
        self.state = self.state + K * y
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update covariance
        self.P = self.P - np.outer(K, K) * S
    
    def publish_ekf_odometry(self, timestamp):
        """Publish filtered odometry estimate"""
        
        x, y, theta, vx, vy, omega = self.state
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Velocities
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=omega)
        
        # Covariance from EKF uncertainty
        pose_cov = np.zeros(36)
        twist_cov = np.zeros(36)
        
        # Map EKF covariance to odometry covariance  
        pose_cov[0] = self.P[0, 0]  # x
        pose_cov[7] = self.P[1, 1]  # y
        pose_cov[35] = self.P[2, 2]  # yaw
        
        twist_cov[0] = self.P[3, 3]  # vx
        twist_cov[7] = self.P[4, 4]  # vy
        twist_cov[35] = self.P[5, 5]  # omega
        
        # Set high uncertainty for unused DOFs
        pose_cov[14] = 1e6  # z
        pose_cov[21] = 1e6  # roll
        pose_cov[28] = 1e6  # pitch
        
        twist_cov[14] = 1e6  # vz
        twist_cov[21] = 1e6  # angular x
        twist_cov[28] = 1e6  # angular y
        
        odom.pose.covariance = pose_cov.tolist()
        odom.twist.covariance = twist_cov.tolist()
        
        # Publish
        self.ekf_odom_pub.publish(odom)
        
        # Broadcast transform
        self.broadcast_transform(timestamp, x, y, theta)
    
    def broadcast_transform(self, timestamp, x, y, theta):
        """Broadcast EKF odom -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom_ekf'
        t.child_frame_id = 'base_link_ekf'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    ekf_odom = EKFOdometry()
    
    try:
        rclpy.spin(ekf_odom)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_odom.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()