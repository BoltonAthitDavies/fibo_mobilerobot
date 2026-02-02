#!/usr/bin/env python3
"""
Plot trajectories from recorded odometry data
Usage: python3 plot_trajectories.py <bag_file>
"""

import sys
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3


def read_messages_from_bag(bag_path, topic_name):
    """Read messages from a ROS2 bag file"""
    conn = sqlite3.connect(bag_path)
    cursor = conn.cursor()
    
    # Get topic id
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    topic_id_row = cursor.fetchone()
    
    if topic_id_row is None:
        print(f"Topic {topic_name} not found in bag file")
        return []
    
    topic_id = topic_id_row[0]
    
    # Get message type
    cursor.execute("SELECT type FROM topics WHERE id=?", (topic_id,))
    topic_type = cursor.fetchone()[0]
    msg_type = get_message(topic_type)
    
    # Get messages
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,)
    )
    
    messages = []
    for timestamp, data in cursor:
        msg = deserialize_message(data, msg_type)
        messages.append((timestamp, msg))
    
    conn.close()
    return messages


def extract_trajectory(messages):
    """Extract x, y coordinates from odometry messages"""
    x_coords = []
    y_coords = []
    
    for timestamp, msg in messages:
        x_coords.append(msg.pose.pose.position.x)
        y_coords.append(msg.pose.pose.position.y)
    
    return x_coords, y_coords


def plot_trajectories(bag_file):
    """Plot all trajectories from the bag file"""
    fig, ax = plt.subplots(figsize=(12, 10))
    
    topics = [
        ('/wheel_odom', 'Wheel Odometry', 'red'),
        ('/ekf_odom', 'EKF Odometry', 'blue'),
        ('/icp_odom', 'ICP Odometry', 'green'),
    ]
    
    for topic, label, color in topics:
        messages = read_messages_from_bag(bag_file, topic)
        if messages:
            x, y = extract_trajectory(messages)
            ax.plot(x, y, label=label, color=color, linewidth=2)
            print(f"Plotted {len(messages)} points for {label}")
        else:
            print(f"No data found for {topic}")
    
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('Trajectory Comparison', fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=300)
    print("Plot saved as trajectory_comparison.png")
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 plot_trajectories.py <bag_file.db3>")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    plot_trajectories(bag_file)
