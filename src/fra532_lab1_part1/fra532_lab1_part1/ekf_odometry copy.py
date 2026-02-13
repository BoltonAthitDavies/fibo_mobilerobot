#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from rclpy.time import Time

class EKFOdometryNode(Node):
    def __init__(self):
        super().__init__('ekf_odometry_node')
        
        # Robot parameters (TurtleBot3 Burger)
        self.wheel_radius = 0.033  # m
        self.wheel_separation = 0.160  # m
        
        # EKF state vector: [x, y, theta] 
        self.state = np.zeros(3)  # [x, y, theta]
        self.covariance = np.eye(3) * 0.1  # Initial uncertainty
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.1])  # [x, y, theta] process noise
        
        # Measurement noise covariance  
        self.measure = np.zeros(1)
        self.R_odom = np.diag([0.1, 0.1, 0.2])  # Wheel odometry noise [x, y, theta]
        self.R_imu = 0.05  # IMU angular velocity noise
        
        # Previous measurements
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time = None
        self.first_measurement = True

        # Kalman gain
        self.K = np.zeros((3, 3))


        # Process jacobian
        self.F = np.eye(3)
        # Measurement jacobian
        self.H = np.eye(3)

        # Publishers and subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states', 
            self.joint_states_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback, 
            10
        )
        
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for EKF prediction step
        self.timer = self.create_timer(0.05, self.prediction_step)  # 20 Hz       
        self.get_logger().info('EKF Odometry Node started')
        
    def joint_states_callback(self, msg, odom):
        """Process joint states for wheel odometry measurement update."""
        
        # TODO: Implement wheel odometry computation
        # Extract wheel positions from joint_states
        # Compute wheel odometry (delta_x, delta_y, delta_theta)
        # Call measurement_update_odom() with wheel odometry measurement
        
        # Template structure:
        # 1. Extract wheel positions
        # 2. Compute displacement since last measurement
        # 3. Convert to robot motion (x, y, theta changes)
        # 4. Call self.measurement_update_odom(odom_measurement)
        
        # Find wheel joints
        left_wheel_idx = None
        right_wheel_idx = None
        
        # Try different naming conventions
        for i, name in enumerate(msg.name):
            if 'left' in name.lower() and 'wheel' in name.lower():
                left_wheel_idx = i
            elif 'right' in name.lower() and 'wheel' in name.lower():
                right_wheel_idx = i
                
        if left_wheel_idx is None or right_wheel_idx is None:
            # Try generic indices
            if len(msg.position) >= 2:
                left_wheel_idx = 0
                right_wheel_idx = 1
            else:
                self.get_logger().warn('Could not find wheel joints')
                return
                
        # Get current wheel positions
        left_wheel_pos = msg.position[left_wheel_idx]
        right_wheel_pos = msg.position[right_wheel_idx]
        
        current_time = Time.from_msg(msg.header.stamp)
        
        if self.first_measurement:
            self.prev_left_wheel_pos = left_wheel_pos
            self.prev_right_wheel_pos = right_wheel_pos  
            self.prev_time = current_time
            self.first_measurement = False
            return
            
        # Calculate wheel displacement
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            return
            
        delta_left = left_wheel_pos - self.prev_left_wheel_pos
        delta_right = right_wheel_pos - self.prev_right_wheel_pos
        
        # Convert to linear distances
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation

        self.F = np.array([
            [1, 0, distance * -math.sin(self.theta+delta_theta)],
            [0, 1, distance * math.cos(self.theta+delta_theta)],
            [0, 0, 1]
        ])
        covariance_pred = self.F @ self.covariance @ self.F.T + self.Q
        self.covariance = (np.eye(3) - self.K @ self.H) @ covariance_pred
        
        # Update robot pose
        if abs(delta_theta) > 1e-6:
            # Curved motion
            radius = distance / delta_theta
            dx = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            dy = radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
        else:
            # Straight motion  
            dx = distance * math.cos(self.theta)
            dy = distance * math.sin(self.theta)
            
        self.x += dx
        self.y += dy
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        linear_vel = distance / dt
        angular_vel = delta_theta / dt
        
        self.state = np.array([self.x, self.y, self.theta])


        # Publish odometry
        self.publish_odometry(msg.header.stamp, linear_vel, angular_vel)
        
        # Update previous values
        self.prev_left_wheel_pos = left_wheel_pos
        self.prev_right_wheel_pos = right_wheel_pos
        self.prev_time = current_time

        # Prepare odometry measurement
        odom_measurement = np.array([dx, dy, delta_theta])
        self.measurement_update_odom(odom_measurement)
        
    def imu_callback(self, msg):
        """Process IMU data for angular velocity measurement update."""
        
        # TODO: Implement IMU measurement update
        # Extract angular velocity from IMU
        # Update EKF with angular velocity measurement
        
        # Template structure:
        # 1. Extract angular velocity from msg.angular_velocity.z
        # 2. Call self.measurement_update_imu(angular_velocity)
        
        # Position
        x_pos = msg.pose.pose.position.x 
        y_pos = msg.pose.pose.position.y 
        z_pos = msg.pose.pose.position.z 

        # Velocity
        x_vel = msg.twist.twist.linear.x
        y_vel = msg.twist.twist.linear.y
        z_vel = msg.twist.twist.linear.z

        # Orientation  
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w
        angular_velocity = msg.angular_velocity.z
        self.measurement_update_imu(angular_velocity)
        
        
        
    def prediction_step(self):
        """EKF prediction step - predict state forward in time."""
        
        # TODO: Implement EKF prediction step
        # This is called periodically to predict the state forward
        # Based on motion model and process noise
        
        # Template structure:
        # 1. Predict state: x_pred = f(x_prev, u) 
        # 2. Predict covariance: P_pred = F * P * F.T + Q
        # 3. Update self.state and self.covariance

        
        # For now, publish current state (replace with actual EKF output)
        self.publish_odometry()
        
    def measurement_update_odom(self, odom_measurement):
        """EKF measurement update with wheel odometry."""
        
        # TODO: Implement EKF measurement update for wheel odometry
        # odom_measurement should be [delta_x, delta_y, delta_theta]
        
        # Template structure:
        # 1. Compute measurement prediction: z_pred = h(x_pred)
        # 2. Compute innovation: y = z_measured - z_pred  
        # 3. Compute Kalman gain: K = P * H.T * (H * P * H.T + R)^-1
        # 4. Update state: x = x_pred + K * y
        # 5. Update covariance: P = (I - K * H) * P
        self.state = self.state + self.K @ (odom_measurement - self.measure)
        
        
    def measurement_update_imu(self, angular_velocity):
        """EKF measurement update with IMU angular velocity."""
        
        # TODO: Implement EKF measurement update for IMU
        # angular_velocity should be scalar (rad/s)
        
        # Template structure:  
        # 1. Compute measurement prediction: theta_dot_pred = current angular velocity
        # 2. Compute innovation: y = theta_dot_measured - theta_dot_pred
        # 3. Compute Kalman gain for angular velocity measurement
        # 4. Update state and covariance
        
        self.K = self.covariance.T @ self.H.T @ np.linalg.inv(self.H @ self.covariance @ self.H.T + self.R_odom)
        self.covariance = (np.eye(3) - self.K @ self.H) @ self.covariance
        
        
    def publish_odometry(self):
        """Publish current EKF state as odometry message."""
        
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position from EKF state
        odom_msg.pose.pose.position.x = float(self.state[0])
        odom_msg.pose.pose.position.y = float(self.state[1])
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation from EKF state  
        theta = float(self.state[2])
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Covariance from EKF
        # Map 3x3 covariance to 6x6 odometry covariance
        odom_msg.pose.covariance[0] = float(self.covariance[0, 0])   # x-x
        odom_msg.pose.covariance[1] = float(self.covariance[0, 1])   # x-y  
        odom_msg.pose.covariance[6] = float(self.covariance[1, 0])   # y-x
        odom_msg.pose.covariance[7] = float(self.covariance[1, 1])   # y-y
        odom_msg.pose.covariance[35] = float(self.covariance[2, 2])  # theta-theta
        
        # TODO: Add velocity estimates from EKF (if implemented)
        # For now, set to zero
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom_msg)
        
        # Publish transform
        self.publish_transform(current_time.to_msg(), theta)
        
    def publish_transform(self, timestamp, theta):
        """Publish TF transform from odom to base_footprint."""
        
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        
        tf_msg.transform.translation.x = float(self.state[0])
        tf_msg.transform.translation.y = float(self.state[1])
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
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