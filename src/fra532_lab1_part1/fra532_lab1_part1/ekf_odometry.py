#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class EKFOdometry(Node):
    def __init__(self):
        super().__init__('ekf_odometry')
        
        # EKF State: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise
        self.R_odom = np.diag([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])  # wheel odometry
        self.R_imu = np.diag([0.01, 0.01, 0.005])  # IMU (ax, ay, omega)
        
        self.prev_time = None
        
        # Subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_odom_callback,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        # Publisher
        self.ekf_odom_pub = self.create_publisher(
            Odometry,
            '/ekf_odom',
            10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('EKF Odometry Node Started')

    def predict(self, dt):
        """EKF Prediction step"""
        # State transition function F
        F = np.array([
            [1, 0, 0, dt, 0, 0],     # x = x + vx*dt
            [0, 1, 0, 0, dt, 0],     # y = y + vy*dt  
            [0, 0, 1, 0, 0, dt],     # theta = theta + omega*dt
            [0, 0, 0, 1, 0, 0],      # vx = vx
            [0, 0, 0, 0, 1, 0],      # vy = vy
            [0, 0, 0, 0, 0, 1]       # omega = omega
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_wheel_odom(self, odom_msg):
        """EKF Update step with wheel odometry"""
        # Extract measurement [x, y, theta, vx, vy, omega]
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y  
        omega = odom_msg.twist.twist.angular.z
        
        z_odom = np.array([x, y, yaw, vx, vy, omega])
        
        # Measurement function (identity for wheel odometry)
        H_odom = np.eye(6)
        
        # Innovation
        y_innovation = z_odom - H_odom @ self.state
        
        # Normalize angle difference
        y_innovation[2] = math.atan2(math.sin(y_innovation[2]), math.cos(y_innovation[2]))
        
        # Innovation covariance  
        S = H_odom @ self.P @ H_odom.T + self.R_odom
        
        # Kalman gain
        K = self.P @ H_odom.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y_innovation
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ H_odom) @ self.P

    def update_imu(self, imu_msg):
        """EKF Update step with IMU"""
        # Extract IMU measurements
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        omega_z = imu_msg.angular_velocity.z
        
        z_imu = np.array([ax, ay, omega_z])
        
        # Measurement function for IMU
        # We relate accelerations to velocities derivatives and use angular velocity directly
        H_imu = np.array([
            [0, 0, 0, 0, 0, 0],    # ax measurement (we'll use it as velocity derivative approx)
            [0, 0, 0, 0, 0, 0],    # ay measurement  
            [0, 0, 0, 0, 0, 1]     # omega_z directly maps to state omega
        ])
        
        # For simplicity, we mainly use the angular velocity measurement
        # In a full implementation, we would properly model accelerometer measurements
        z_imu_simplified = np.array([omega_z])
        H_imu_simplified = np.array([[0, 0, 0, 0, 0, 1]])
        R_imu_simplified = np.array([[0.005]])
        
        # Innovation
        y_innovation = z_imu_simplified - H_imu_simplified @ self.state
        
        # Innovation covariance
        S = H_imu_simplified @ self.P @ H_imu_simplified.T + R_imu_simplified
        
        # Kalman gain
        K = self.P @ H_imu_simplified.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y_innovation
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ H_imu_simplified) @ self.P

    def wheel_odom_callback(self, msg):
        """Handle wheel odometry measurements"""
        current_time = self.get_clock().now()
        
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                # Prediction step
                self.predict(dt)
                
                # Update with wheel odometry
                self.update_wheel_odom(msg)
                
                # Publish EKF result
                self.publish_ekf_odometry(current_time)
                self.publish_tf_transform(current_time)
        
        self.prev_time = current_time

    def imu_callback(self, msg):
        """Handle IMU measurements"""
        if self.prev_time is not None:
            # Update with IMU data
            self.update_imu(msg)

    def publish_ekf_odometry(self, timestamp):
        """Publish EKF odometry result"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position = Point(
            x=self.state[0], 
            y=self.state[1], 
            z=0.0
        )
        
        # Orientation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        
        # Velocity
        odom_msg.twist.twist.linear = Vector3(
            x=self.state[3], 
            y=self.state[4], 
            z=0.0
        )
        odom_msg.twist.twist.angular = Vector3(
            x=0.0, 
            y=0.0, 
            z=self.state[5]
        )
        
        # Covariance from EKF
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0,0]   # x
        pose_cov[7] = self.P[1,1]   # y
        pose_cov[35] = self.P[2,2]  # yaw
        odom_msg.pose.covariance = pose_cov.tolist()
        
        twist_cov = np.zeros(36) 
        twist_cov[0] = self.P[3,3]   # vx
        twist_cov[7] = self.P[4,4]   # vy
        twist_cov[35] = self.P[5,5]  # vyaw
        odom_msg.twist.covariance = twist_cov.tolist()
        
        self.ekf_odom_pub.publish(odom_msg)

    def publish_tf_transform(self, timestamp):
        """Publish TF transform"""
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link_ekf'
        
        # Translation
        transform.transform.translation.x = self.state[0]
        transform.transform.translation.y = self.state[1] 
        transform.transform.translation.z = 0.0
        
        # Rotation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        transform.transform.rotation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    
    ekf_node = EKFOdometry()
    
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()