import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
import math
import csv
import pandas as pd

class OdomEvaluator(Node):
    def __init__(self):
        super().__init__('odom_evaluator')
        
        # Configuration - Updated to match actual topics
        self.topics = {
            '/odom':       {'label': 'Wheel (Part3)', 'color': 'red', 'type': 'odom'},
            '/ekf_odom':   {'label': 'EKF',          'color': 'blue', 'type': 'odom'},
            '/icp_odom':   {'label': 'ICP',          'color': 'green', 'type': 'odom'},
            '/pose':       {'label': 'SLAM (Truth)', 'color': 'black', 'type': 'pose'}
        }
        
        self.data = {t: {'time': [], 'x': [], 'y': [], 'yaw': []} for t in self.topics}
        self.start_time = None
        
        # CSV Setup - Create separate files for each odometry method
        self.csv_files = {}
        self.writers = {}
        
        for topic, config in self.topics.items():
            filename = f"odom_{config['label'].lower().replace(' ', '_')}_data_seq2.csv"
            self.csv_files[topic] = open(filename, 'w', newline='')
            self.writers[topic] = csv.writer(self.csv_files[topic])
            self.writers[topic].writerow(['timestamp', 'x', 'y', 'yaw'])

        # Create subscriptions based on message type
        for topic, config in self.topics.items():
            if config['type'] == 'odom':
                self.get_logger().info(f"Subscribing to {topic} as nav_msgs/Odometry")
                self.create_subscription(Odometry, topic, lambda msg, t=topic: self.odom_cb(msg, t), 10)
            elif config['type'] == 'pose':
                self.get_logger().info(f"Subscribing to {topic} as geometry_msgs/PoseWithCovarianceStamped")
                self.create_subscription(PoseWithCovarianceStamped, topic, lambda msg, t=topic: self.slam_pose_cb(msg, t), 10)
                
        self.get_logger().info("OdomEvaluator initialized with subscriptions to all topics")

    def get_yaw(self, q):
        """Converts quaternion to yaw in degrees."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def slam_pose_cb(self, msg, topic):
        """Callback for SLAM pose messages (PoseWithCovarianceStamped)."""
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None: self.start_time = now
        
        rel_time = now - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg.pose.pose.orientation)

        self.data[topic]['time'].append(rel_time)
        self.data[topic]['x'].append(x)
        self.data[topic]['y'].append(y)
        self.data[topic]['yaw'].append(yaw)
        
        # Write to individual CSV file for this SLAM method
        self.writers[topic].writerow([rel_time, x, y, yaw])
        
        # Debug logging (every 50th message)
        if len(self.data[topic]['time']) % 50 == 1:
            self.get_logger().info(f"SLAM data received: {topic} -> x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}°")

    def odom_cb(self, msg, topic):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None: self.start_time = now
        
        rel_time = now - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg.pose.pose.orientation)

        self.data[topic]['time'].append(rel_time)
        self.data[topic]['x'].append(x)
        self.data[topic]['y'].append(y)
        self.data[topic]['yaw'].append(yaw)
        
        # Write to individual CSV file for this odometry method
        self.writers[topic].writerow([rel_time, x, y, yaw])
        
        # Debug logging (every 50th message)
        if len(self.data[topic]['time']) % 50 == 1:
            self.get_logger().info(f"Odom data received: {topic} -> x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}°")

    def generate_tables(self):
        """Generate and display tables for each odometry method."""
        print("\n" + "="*80)
        print("ODOMETRY DATA TABLES")
        print("="*80)
        
        for topic, config in self.topics.items():
            data = self.data[topic]
            if not data['x']:
                print(f"\nNo data available for {config['label']} odometry")
                continue
                
            # Create DataFrame
            df = pd.DataFrame({
                'Time (s)': data['time'],
                'X (m)': data['x'],
                'Y (m)': data['y'],
                'Yaw (deg)': data['yaw']
            })
            
            # Format numbers
            df['Time (s)'] = df['Time (s)'].round(3)
            df['X (m)'] = df['X (m)'].round(4)
            df['Y (m)'] = df['Y (m)'].round(4)
            df['Yaw (deg)'] = df['Yaw (deg)'].round(2)
            
            print(f"\n{config['label'].upper()} ODOMETRY DATA ({len(df)} samples)")
            print("-" * 60)
            
            # Show first 10 and last 10 rows if more than 20 samples
            if len(df) > 20:
                print("First 10 samples:")
                print(df.head(10).to_string(index=False))
                print("\n... ({} samples omitted) ...\n".format(len(df) - 20))
                print("Last 10 samples:")
                print(df.tail(10).to_string(index=False))
            else:
                print(df.to_string(index=False))
            
            # Statistics
            print(f"\nStatistics for {config['label']}:")
            print(f"  Duration: {df['Time (s)'].max():.2f} seconds")
            print(f"  X range: {df['X (m)'].min():.3f} to {df['X (m)'].max():.3f} m")
            print(f"  Y range: {df['Y (m)'].min():.3f} to {df['Y (m)'].max():.3f} m")
            print(f"  Yaw range: {df['Yaw (deg)'].min():.1f} to {df['Yaw (deg)'].max():.1f} deg")
            
            # Save full table to CSV
            filename = f"table_{config['label'].lower().replace(' ', '_')}_full_seq2.csv"
            df.to_csv(filename, index=False)
            print(f"  Full table saved to: {filename}")
        
        print("\n" + "="*80)

    def plot_analysis(self):
        # Close all CSV files
        for csv_file in self.csv_files.values():
            csv_file.close()
        
        # Generate tables
        self.generate_tables()
        
        fig = plt.figure(figsize=(16, 10))
        
        # 1. 2D Path Plot (Robustness & Accuracy)
        ax1 = fig.add_subplot(2, 2, 1)
        # 2. Yaw vs Time (Drift Analysis)
        ax2 = fig.add_subplot(2, 2, 2)
        # 3. Euclidean Error vs SLAM (Quantifying Accuracy)
        ax3 = fig.add_subplot(2, 1, 2)

        slam_ref = self.data['/pose']

        for t, cfg in self.topics.items():
            d = self.data[t]
            if not d['x']: continue

            # Plot 1: XY Path
            ax1.plot(d['x'], d['y'], label=cfg['label'], color=cfg['color'])
            
            # Plot 2: Yaw
            ax2.plot(d['time'], d['yaw'], color=cfg['color'], alpha=0.7)

            # Plot 3: Calculate Error relative to SLAM (Accuracy)
            if t != '/pose' and len(slam_ref['x']) > 0:
                # Simple nearest-neighbor time sync for error calculation
                errors = []
                times = []
                for i in range(len(d['time'])):
                    idx = np.argmin(np.abs(np.array(slam_ref['time']) - d['time'][i]))
                    dist = math.sqrt((d['x'][i] - slam_ref['x'][idx])**2 + 
                                     (d['y'][i] - slam_ref['y'][idx])**2)
                    errors.append(dist)
                    times.append(d['time'][i])
                ax3.plot(times, errors, label=f"{cfg['label']} Error", color=cfg['color'])

        ax1.set_title("2D Plane Trajectory"); ax1.legend(); ax1.axis('equal'); ax1.grid(True)
        ax2.set_title("Yaw Heading (Degrees)"); ax2.set_ylabel("Deg"); ax2.grid(True)
        ax3.set_title("Absolute Trajectory Error (ATE) relative to SLAM"); ax3.set_ylabel("Error (m)"); ax3.legend(); ax3.grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = OdomEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Generating Analysis and Tables...")
        node.plot_analysis()
    finally:
        # Ensure all files are closed
        for csv_file in node.csv_files.values():
            if not csv_file.closed:
                csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()