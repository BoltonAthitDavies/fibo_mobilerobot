#!/usr/bin/env python3
"""
Node structure checker for FRA532 Lab 1
Helps debug which nodes are running and their topics
"""

import rclpy
from rclpy.node import Node
import subprocess
import sys

def run_command(cmd):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {e}"

def check_nodes():
    """Check which nodes are currently running."""
    print("=" * 60)
    print("ACTIVE ROS2 NODES")
    print("=" * 60)
    
    output = run_command("ros2 node list")
    if output:
        nodes = output.split('\n')
        part1_nodes = [n for n in nodes if 'ekf' in n.lower() or 'wheel' in n.lower()]
        part2_nodes = [n for n in nodes if 'icp' in n.lower()]
        other_nodes = [n for n in nodes if n not in part1_nodes and n not in part2_nodes]
        
        print(f"Part 1 Nodes ({len(part1_nodes)}):")
        for node in part1_nodes:
            print(f"  ✓ {node}")
        
        print(f"\nPart 2 Nodes ({len(part2_nodes)}):")
        for node in part2_nodes:
            print(f"  ✓ {node}")
        
        print(f"\nOther Nodes ({len(other_nodes)}):")
        for node in other_nodes:
            print(f"  • {node}")
    else:
        print("No nodes found or ROS2 not running!")

def check_topics():
    """Check key topics for the lab."""
    print("\n" + "=" * 60)
    print("KEY TOPICS STATUS")
    print("=" * 60)
    
    key_topics = {
        "/scan": "LiDAR data (required for ICP)",
        "/joint_states": "Wheel encoder data (required for EKF)",
        "/imu": "IMU data (optional for EKF)",
        "/odom": "Basic wheel odometry",
        "/ekf_odom": "EKF refined odometry (Part 1 output)",
        "/icp_pose": "ICP pose estimates (Part 2 output)",
        "/icp_refined_odom": "ICP refined odometry (Part 2 output)"
    }
    
    # Get all topics
    all_topics = run_command("ros2 topic list").split('\n') if run_command("ros2 topic list") else []
    
    for topic, description in key_topics.items():
        if topic in all_topics:
            # Check topic info
            info = run_command(f"ros2 topic info {topic}")
            pub_count = info.count("Publisher count:")
            sub_count = info.count("Subscriber count:")
            print(f"  ✓ {topic:<20} - {description}")
            if "Publisher count: 0" in info:
                print(f"    ⚠️  No publishers!")
        else:
            print(f"  ✗ {topic:<20} - {description} (NOT FOUND)")

def check_tf_tree():
    """Check TF transforms."""
    print("\n" + "=" * 60)
    print("TRANSFORM FRAMES")
    print("=" * 60)
    
    frames = run_command("ros2 run tf2_tools view_frames.py --all-frames-pose")
    if "map" in frames:
        print("  ✓ map frame available")
    else:
        print("  ✗ map frame missing")
    
    if "odom" in frames:
        print("  ✓ odom frame available") 
    else:
        print("  ✗ odom frame missing")
    
    if "base_footprint" in frames:
        print("  ✓ base_footprint frame available")
    else:
        print("  ✗ base_footprint frame missing")

def main():
    print("FRA532 Lab 1 - Node Structure Checker")
    print("Checking current ROS2 system status...\n")
    
    try:
        check_nodes()
        check_topics()
        check_tf_tree()
        
        print("\n" + "=" * 60)
        print("RECOMMENDATIONS")
        print("=" * 60)
        
        # Check if Part 1 is running
        nodes = run_command("ros2 node list").split('\n') if run_command("ros2 node list") else []
        ekf_running = any('ekf' in n.lower() for n in nodes)
        icp_running = any('icp' in n.lower() for n in nodes)
        
        if not ekf_running and not icp_running:
            print("  • No lab nodes detected. Start with:")
            print("    ros2 launch fra532_lab1_part1 ekf_odometry.launch.py")
        elif ekf_running and not icp_running:
            print("  • Part 1 (EKF) is running. To add Part 2 (ICP):")
            print("    ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py")
        elif not ekf_running and icp_running:
            print("  • Part 2 (ICP) running without Part 1. Consider:")
            print("    ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py launch_ekf:=true")
        else:
            print("  • Both parts running - system looks good!")
            print("  • Monitor topics: ros2 topic echo /icp_refined_odom")
        
    except KeyboardInterrupt:
        print("\nStatus check interrupted.")
    except Exception as e:
        print(f"Error during status check: {e}")

if __name__ == '__main__':
    main()