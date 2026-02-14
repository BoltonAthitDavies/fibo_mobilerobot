#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from rclpy.time import Time

class ICPOdometryNode(Node):
    def __init__(self):
        super().__init__('icp_odometry_node')
        
        # ICP state: Start from EKF pose and refine it
        self.state = np.zeros(3)  # [x, y, theta] - ICP refined pose
        
        # ICP parameters
        self.max_iterations = 100
        self.convergence_threshold = 0.001
        self.max_correspondence_distance = 0.005  # meters, map resolution-aware matching
        # self.downsample_factor = 2  # Use every 2nd point
        self.downsample_factor = 1  # Use every 2nd point
        
        # Keyframe buffer parameters
        self.max_keyframes = 10  # Maximum number of keyframes to keep
        self.keyframe_distance_threshold = 0.5  # meters - minimum distance to add new keyframe
        self.keyframe_angle_threshold = 0.3  # radians (~17 degrees) - minimum angle to add new keyframe
        
        # Keyframe buffer: list of (pose, points, timestamp) tuples
        self.keyframes = []
        
        # Scan data for matching
        self.previous_scan = None
        self.previous_points = None
        self.first_scan = True
        self.last_keyframe_pose = None
        
        # EKF odometry (our initial guess) - buffered for 5 Hz consumption
        self.ekf_pose = np.zeros(3)  # [x, y, theta] from EKF
        self.ekf_available = False
        self.latest_ekf_pose = None  # Latest buffered EKF pose for 5 Hz processing
        
        # Laser scan buffer - hold latest scan for 5 Hz processing
        self.latest_laser_scan = None
        self.latest_laser_points = None
        self.new_laser_data = False  # Flag to track if fresh data arrived
        
        # Map data (point-to-map target)
        self.map_available = False
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.map_data = None  # 2D numpy array

        # Subscribers with QoS compatibility
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, laser_qos)
        self.ekf_sub = self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/icp_map', self.map_callback, map_qos)
        
        # Create 5 Hz timer for synchronized ICP processing
        self.icp_timer = self.create_timer(0.2, self.icp_process_step)  # 5 Hz = 1/0.2s
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/icp_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('ICP Odometry Node started (5 Hz synchronized, EKF-based)')

        
    def laser_callback(self, msg):
        """Buffer latest laser scan for 5 Hz synchronized processing."""
        try:
            # Convert and downsample laser scan
            current_points = self.laser_scan_to_points(msg)
            current_points = self.downsample_points(current_points)
            
            if len(current_points) < 10:
                self.get_logger().debug('Too few valid scan points, skipping')
                return
            
            # Store latest scan and mark as new data
            self.latest_laser_scan = msg
            self.latest_laser_points = current_points
            self.new_laser_data = True
            
        except Exception as e:
            self.get_logger().error(f'Error in laser callback: {str(e)}')
    
    def icp_process_step(self):
        """5 Hz timer callback for synchronized ICP processing."""
        try:
            # Check prerequisites
            if not self.ekf_available:
                self.get_logger().warn('EKF not yet available')
                return
            
            if self.latest_laser_points is None or len(self.latest_laser_points) < 10:
                self.get_logger().debug('No valid laser data buffered')
                return
            
            # Update buffer with latest EKF
            self.latest_ekf_pose = self.ekf_pose.copy()
            
            # Initialize on first laser scan
            if self.first_scan:
                self.state = self.latest_ekf_pose.copy()
                self.get_logger().info(f'Initialized ICP state from EKF: {self.state}')
                
                # Add first keyframe
                self.add_keyframe(self.state, self.latest_laser_points, self.latest_laser_scan.header.stamp)
                
                self.previous_points = self.latest_laser_points.copy()
                self.first_scan = False
                self.new_laser_data = False
                return
            
            # Process ICP only if fresh laser data
            if not self.new_laser_data:
                self.get_logger().debug('Laser data already processed, skipping this cycle')
                return
            
            if self.previous_points is None or len(self.previous_points) < 10:
                self.get_logger().warn('No valid previous scan')
                return
            
            # Use buffered EKF pose as initial guess
            initial_pose = self.latest_ekf_pose.copy()
            
            # Run ICP: match current scan to map if available, else fallback to keyframes
            if self.map_available:
                refined_pose = self.run_icp_point_to_map(self.latest_laser_points, initial_pose)
            else:
                refined_pose = self.run_icp_with_keyframes(self.latest_laser_points, initial_pose)
            
            # Update state with refined pose
            self.state = refined_pose
            
            # Add new keyframe if significant movement
            if self.should_add_keyframe(refined_pose):
                self.add_keyframe(refined_pose, self.latest_laser_points, self.latest_laser_scan.header.stamp)
            
            # Publish refined odometry at 5 Hz
            self.publish_odometry(self.latest_laser_scan.header.stamp)
            
            # Update for next iteration
            self.previous_points = self.latest_laser_points.copy()
            self.new_laser_data = False
            
        except Exception as e:
            self.get_logger().error(f'Error in ICP process step: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    
    def run_icp_with_keyframes(self, curr_points, initial_pose):
        """
        ICP algorithm using keyframe buffer and EKF pose as initial guess.
        Matches current scan against the best keyframe(s).
        
        Args:
            curr_points: Current scan points (Mx2 array) in current robot frame
            initial_pose: Initial pose guess from EKF [x, y, theta] in global frame
            
        Returns:
            refined_pose: Refined absolute pose [x, y, theta] in global frame
        """
        
        if len(self.keyframes) == 0:
            self.get_logger().warn('No keyframes available for ICP matching')
            return initial_pose
        
        # Select best keyframe(s) for matching
        best_keyframe = self.select_best_keyframe(initial_pose)
        if best_keyframe is None:
            self.get_logger().warn('No suitable keyframe found')
            return initial_pose
        
        keyframe_pose, keyframe_points, _ = best_keyframe
        
        # Start with EKF pose
        current_pose = initial_pose.copy()
        
        # Transform keyframe points to global frame
        keyframe_points_global = self.transform_points_to_global(keyframe_points, keyframe_pose)
        
        for iteration in range(self.max_iterations):
            # Transform current points to global frame using current pose estimate
            curr_points_global = self.transform_points_to_global(curr_points, current_pose)
            
            # Find correspondences in global frame
            correspondences = self.find_correspondences(keyframe_points_global, curr_points_global)
            
            if len(correspondences) < 10:
                self.get_logger().warn(f'Too few correspondences: {len(correspondences)}')
                break
            
            # Estimate pose correction
            pose_correction = self.estimate_pose_correction(
                correspondences, keyframe_points_global, curr_points_global, current_pose
            )
            
            # Apply correction
            current_pose += pose_correction
            current_pose[2] = math.atan2(math.sin(current_pose[2]), math.cos(current_pose[2]))  # Normalize angle
            
            # Check convergence
            if np.linalg.norm(pose_correction) < self.convergence_threshold:
                self.get_logger().info(f'ICP converged after {iteration + 1} iterations')
                break
        
        correction_magnitude = np.linalg.norm(current_pose - initial_pose)
        self.get_logger().info(f'ICP correction: {correction_magnitude:.4f}m, Final pose: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}]')
        
        return current_pose

    def run_icp_point_to_map(self, curr_points, initial_pose):
        """
        ICP algorithm using occupancy grid map as target (point-to-map).
        Matches current scan against nearby occupied map cells.
        """
        if not self.map_available:
            self.get_logger().warn('No map available for point-to-map ICP')
            return initial_pose

        current_pose = initial_pose.copy()

        for iteration in range(self.max_iterations):
            curr_points_global = self.transform_points_to_global(curr_points, current_pose)

            curr_matched, map_matched = self.find_map_correspondences(curr_points_global)

            if len(curr_matched) < 10:
                self.get_logger().warn(f'Too few map correspondences: {len(curr_matched)}')
                break

            pose_correction = self.estimate_pose_correction_from_matches(
                curr_matched, map_matched, current_pose
            )

            current_pose += pose_correction
            current_pose[2] = math.atan2(math.sin(current_pose[2]), math.cos(current_pose[2]))

            if np.linalg.norm(pose_correction) < self.convergence_threshold:
                self.get_logger().info(f'Point-to-map ICP converged after {iteration + 1} iterations')
                break

        correction_magnitude = np.linalg.norm(current_pose - initial_pose)
        self.get_logger().info(
            f'Point-to-map correction: {correction_magnitude:.4f}m, Final pose: '
            f'[{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}]'
        )

        return current_pose
    
    def transform_points_to_global(self, points, pose):
        """Transform points from robot frame to global frame using pose."""
        if len(points) == 0:
            return points
        
        x, y, theta = pose
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # Rotation + translation
        points_global = np.zeros_like(points)
        points_global[:, 0] = cos_t * points[:, 0] - sin_t * points[:, 1] + x
        points_global[:, 1] = sin_t * points[:, 0] + cos_t * points[:, 1] + y
        
        return points_global
    
    def find_correspondences(self, prev_points, curr_points):
        """Find nearest neighbor correspondences between point clouds."""
        correspondences = []
        
        if len(prev_points) == 0 or len(curr_points) == 0:
            return correspondences
        
        # For each current point, find closest previous point
        for i, curr_pt in enumerate(curr_points):
            distances = np.linalg.norm(prev_points - curr_pt, axis=1)
            nearest_idx = np.argmin(distances)
            
            # Only accept if distance is reasonable
            if distances[nearest_idx] < self.max_correspondence_distance:
                correspondences.append((i, nearest_idx))
        
        return correspondences
    
    def estimate_pose_correction(self, correspondences, prev_points, curr_points, current_pose):
        """
        Estimate pose correction using matched points.
        Uses SVD to find optimal rotation and translation.
        """
        
        if len(correspondences) < 3:
            return np.zeros(3)
        
        try:
            # Extract matched point pairs
            curr_matched = np.array([curr_points[i] for i, j in correspondences])
            prev_matched = np.array([prev_points[j] for i, j in correspondences])
            
            # Calculate centroids
            curr_centroid = np.mean(curr_matched, axis=0)
            prev_centroid = np.mean(prev_matched, axis=0)
            
            # Center the points
            curr_centered = curr_matched - curr_centroid
            prev_centered = prev_matched - prev_centroid
            
            # Calculate rotation using SVD
            H = curr_centered.T @ prev_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Ensure proper rotation (det(R) = 1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # Calculate angle correction
            current_theta = current_pose[2]
            target_theta = math.atan2(R[1, 0], R[0, 0])
            dtheta = target_theta - current_theta
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))  # Normalize
            
            # Calculate translation correction
            dx = prev_centroid[0] - curr_centroid[0]
            dy = prev_centroid[1] - curr_centroid[1]
            
            # Small correction gains to prevent overshooting
            correction = np.array([dx * 0.7, dy * 0.7, dtheta * 0.7])
            
            return correction
        
        except Exception as e:
            self.get_logger().warn(f'Pose correction estimation failed: {str(e)}')
            return np.zeros(3)

    def estimate_pose_correction_from_matches(self, curr_matched, map_matched, current_pose):
        """Estimate pose correction using matched point arrays."""
        if len(curr_matched) < 3:
            return np.zeros(3)

        try:
            curr_matched = np.asarray(curr_matched)
            map_matched = np.asarray(map_matched)

            curr_centroid = np.mean(curr_matched, axis=0)
            map_centroid = np.mean(map_matched, axis=0)

            curr_centered = curr_matched - curr_centroid
            map_centered = map_matched - map_centroid

            H = curr_centered.T @ map_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            current_theta = current_pose[2]
            target_theta = math.atan2(R[1, 0], R[0, 0])
            dtheta = target_theta - current_theta
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

            dx = map_centroid[0] - curr_centroid[0]
            dy = map_centroid[1] - curr_centroid[1]

            correction = np.array([dx * 0.7, dy * 0.7, dtheta * 0.7])
            return correction

        except Exception as e:
            self.get_logger().warn(f'Pose correction estimation failed: {str(e)}')
            return np.zeros(3)
    
    def downsample_points(self, points):
        """Downsample point cloud to reduce computation."""
        if len(points) == 0:
            return points
        return points[::self.downsample_factor]
    
    def laser_scan_to_points(self, laser_scan):
        """Convert LaserScan to 2D points in robot frame."""
        points = []
        angle = laser_scan.angle_min
        
        for range_val in laser_scan.ranges:
            # Filter valid ranges
            if laser_scan.range_min <= range_val <= laser_scan.range_max:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append([x, y])
            angle += laser_scan.angle_increment
        
        return np.array(points)
    
    def ekf_callback(self, msg):
        """Buffer latest EKF odometry for 5 Hz consumption.
        
        Note: EKF publishes at 20 Hz, but we only consume it every 0.2s (5 Hz) 
        in the icp_process_step() timer. This reduces redundant ICP processing.
        """
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to euler
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2 * math.atan2(qz, qw)
        
        self.ekf_pose = np.array([x, y, theta])
        
        if not self.ekf_available:
            self.get_logger().info(f'EKF odometry available at {x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°')
            self.ekf_available = True

    def map_callback(self, msg):
        """Store occupancy grid for point-to-map ICP."""
        if msg.info.width == 0 or msg.info.height == 0:
            return

        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.map_data = data
        self.map_available = True

    def world_to_map(self, x, y):
        """Convert world coordinates to map grid indices."""
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col

    def map_to_world(self, row, col):
        """Convert map grid indices to world coordinates (cell center)."""
        x = self.map_origin_x + (col + 0.5) * self.map_resolution
        y = self.map_origin_y + (row + 0.5) * self.map_resolution
        return x, y

    def is_valid_map_cell(self, row, col):
        return 0 <= row < self.map_height and 0 <= col < self.map_width

    def find_map_correspondences(self, curr_points_global):
        """Find correspondences between current points and occupied map cells."""
        if self.map_data is None or self.map_resolution is None:
            return [], []

        curr_matched = []
        map_matched = []

        search_radius_cells = max(1, int(math.ceil(self.max_correspondence_distance / self.map_resolution)))

        for curr_pt in curr_points_global:
            row, col = self.world_to_map(curr_pt[0], curr_pt[1])

            best_dist = None
            best_point = None

            for dr in range(-search_radius_cells, search_radius_cells + 1):
                for dc in range(-search_radius_cells, search_radius_cells + 1):
                    r = row + dr
                    c = col + dc
                    if not self.is_valid_map_cell(r, c):
                        continue

                    if self.map_data[r, c] < 50:
                        continue

                    mx, my = self.map_to_world(r, c)
                    dist = math.hypot(curr_pt[0] - mx, curr_pt[1] - my)

                    if dist <= self.max_correspondence_distance and (best_dist is None or dist < best_dist):
                        best_dist = dist
                        best_point = (mx, my)

            if best_point is not None:
                curr_matched.append(curr_pt)
                map_matched.append(best_point)

        return curr_matched, map_matched
    
    def publish_odometry(self, timestamp):
        """Publish ICP-refined odometry."""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'icp_odom'
        
        # Pose
        odom_msg.pose.pose.position.x = float(self.state[0])
        odom_msg.pose.pose.position.y = float(self.state[1])
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        theta = float(self.state[2])
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Covariance
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.05  # theta
        
        self.odom_pub.publish(odom_msg)
        self.publish_transform(timestamp, theta)
    
    def publish_transform(self, timestamp, theta):
        """Publish TF transform."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'icp_odom'
        
        tf_msg.transform.translation.x = float(self.state[0])
        tf_msg.transform.translation.y = float(self.state[1])
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def add_keyframe(self, pose, points, timestamp):
        """Add a new keyframe to the buffer."""
        keyframe = (pose.copy(), points.copy(), timestamp)
        self.keyframes.append(keyframe)
        
        # Remove oldest keyframe if buffer is full
        if len(self.keyframes) > self.max_keyframes:
            removed = self.keyframes.pop(0)
            self.get_logger().debug('Removed oldest keyframe from buffer')
        
        self.last_keyframe_pose = pose.copy()
        self.get_logger().info(f'Added keyframe {len(self.keyframes)}/{self.max_keyframes} at pose [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]')
    
    def should_add_keyframe(self, current_pose):
        """Decide whether to add a new keyframe based on movement thresholds."""
        if self.last_keyframe_pose is None:
            return True
        
        # Calculate distance and angle change from last keyframe
        dx = current_pose[0] - self.last_keyframe_pose[0]
        dy = current_pose[1] - self.last_keyframe_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        dtheta = abs(current_pose[2] - self.last_keyframe_pose[2])
        dtheta = min(dtheta, 2*math.pi - dtheta)  # Normalize to [0, pi]
        
        # Add keyframe if significant movement
        if distance > self.keyframe_distance_threshold or dtheta > self.keyframe_angle_threshold:
            self.get_logger().debug(f'Adding keyframe: dist={distance:.3f}m, angle={dtheta:.3f}rad')
            return True
        
        return False
    
    def select_best_keyframe(self, current_pose):
        """Select the best keyframe for ICP matching based on proximity."""
        if len(self.keyframes) == 0:
            return None
        
        # Find keyframe closest to current pose
        best_keyframe = None
        min_distance = float('inf')
        
        for keyframe in self.keyframes:
            keyframe_pose, _, _ = keyframe
            
            # Calculate distance to keyframe
            dx = current_pose[0] - keyframe_pose[0]
            dy = current_pose[1] - keyframe_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_distance:
                min_distance = distance
                best_keyframe = keyframe
        
        if best_keyframe is not None:
            keyframe_pose, _, _ = best_keyframe
            self.get_logger().debug(f'Selected keyframe at [{keyframe_pose[0]:.3f}, {keyframe_pose[1]:.3f}], distance: {min_distance:.3f}m')
        
        return best_keyframe


def main(args=None):
    rclpy.init(args=args)
    
    node = ICPOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
