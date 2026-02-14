#!/usr/bin/env python3

import re
import numpy as np

def preprocess_imu_string(imu_str):
    """
    Parse IMU message string and extract values.
    
    Args:
        imu_str: String representation of sensor_msgs.msg.Imu
        
    Returns:
        dict: Parsed IMU data
    """
    
    # Extract timestamp
    timestamp_match = re.search(r'stamp=.*?Time\(sec=(\d+), nanosec=(\d+)\)', imu_str)
    if timestamp_match:
        sec = int(timestamp_match.group(1))
        nanosec = int(timestamp_match.group(2))
        timestamp = sec + nanosec * 1e-9
    else:
        timestamp = 0.0
    
    # Extract frame_id
    frame_match = re.search(r"frame_id='([^']+)'", imu_str)
    frame_id = frame_match.group(1) if frame_match else 'unknown'
    
    # Extract orientation quaternion
    quat_match = re.search(r'orientation=.*?Quaternion\(x=([-\d.e]+), y=([-\d.e]+), z=([-\d.e]+), w=([-\d.e]+)\)', imu_str)
    if quat_match:
        qx, qy, qz, qw = [float(x) for x in quat_match.groups()]
    else:
        qx = qy = qz = qw = 0.0
    
    # Extract angular velocity
    angular_vel_match = re.search(r'angular_velocity=.*?Vector3\(x=([-\d.e]+), y=([-\d.e]+), z=([-\d.e]+)\)', imu_str)
    if angular_vel_match:
        wx, wy, wz = [float(x) for x in angular_vel_match.groups()]
    else:
        wx = wy = wz = 0.0
    
    # Extract linear acceleration
    linear_acc_match = re.search(r'linear_acceleration=.*?Vector3\(x=([-\d.e]+), y=([-\d.e]+), z=([-\d.e]+)\)', imu_str)
    if linear_acc_match:
        ax, ay, az = [float(x) for x in linear_acc_match.groups()]
    else:
        ax = ay = az = 0.0
    
    return {
        'timestamp': timestamp,
        'frame_id': frame_id,
        'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
        'angular_velocity': {'x': wx, 'y': wy, 'z': wz},
        'linear_acceleration': {'x': ax, 'y': ay, 'z': az}
    }

def imu_to_csv_row(imu_data):
    """Convert IMU data dict to CSV row string."""
    return f"{imu_data['timestamp']:.9f},{imu_data['orientation']['x']:.6f},{imu_data['orientation']['y']:.6f},{imu_data['orientation']['z']:.6f},{imu_data['orientation']['w']:.6f},{imu_data['angular_velocity']['x']:.6f},{imu_data['angular_velocity']['y']:.6f},{imu_data['angular_velocity']['z']:.6f},{imu_data['linear_acceleration']['x']:.6f},{imu_data['linear_acceleration']['y']:.6f},{imu_data['linear_acceleration']['z']:.6f}"

def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def main():
    # Example IMU string
    imu_string = """sensor_msgs.msg.Imu(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1769097853, nanosec=193095911), frame_id='imu_link'), orientation=geometry_msgs.msg.Quaternion(x=0.006038070190697908, y=-0.00905844010412693, z=0.0009028510539792478, w=0.999936044216156), orientation_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]), angular_velocity=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular_velocity_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]), linear_acceleration=geometry_msgs.msg.Vector3(x=0.23104046285152435, y=0.11671733111143112, z=10.164583206176758), linear_acceleration_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]))"""
    
    # Parse the IMU data
    imu_data = preprocess_imu_string(imu_string)
    
    # Display results
    print("Parsed IMU Data:")
    print("=" * 50)
    print(f"Timestamp: {imu_data['timestamp']:.9f}")
    print(f"Frame ID: {imu_data['frame_id']}")
    
    print("\nOrientation (Quaternion):")
    print(f"  x: {imu_data['orientation']['x']:.6f}")
    print(f"  y: {imu_data['orientation']['y']:.6f}")
    print(f"  z: {imu_data['orientation']['z']:.6f}")
    print(f"  w: {imu_data['orientation']['w']:.6f}")
    
    # Convert to Euler angles
    roll, pitch, yaw = quaternion_to_euler(
        imu_data['orientation']['x'], 
        imu_data['orientation']['y'], 
        imu_data['orientation']['z'], 
        imu_data['orientation']['w']
    )
    print(f"\nOrientation (Euler - radians):")
    print(f"  Roll:  {roll:.6f}")
    print(f"  Pitch: {pitch:.6f}")
    print(f"  Yaw:   {yaw:.6f}")
    
    print(f"\nOrientation (Euler - degrees):")
    print(f"  Roll:  {np.degrees(roll):.2f}°")
    print(f"  Pitch: {np.degrees(pitch):.2f}°")
    print(f"  Yaw:   {np.degrees(yaw):.2f}°")
    
    print("\nAngular Velocity (rad/s):")
    print(f"  x: {imu_data['angular_velocity']['x']:.6f}")
    print(f"  y: {imu_data['angular_velocity']['y']:.6f}")
    print(f"  z: {imu_data['angular_velocity']['z']:.6f}")
    
    print("\nLinear Acceleration (m/s²):")
    print(f"  x: {imu_data['linear_acceleration']['x']:.6f}")
    print(f"  y: {imu_data['linear_acceleration']['y']:.6f}")
    print(f"  z: {imu_data['linear_acceleration']['z']:.6f}")
    
    print("\nCSV Header:")
    print("timestamp,qx,qy,qz,qw,wx,wy,wz,ax,ay,az")
    print("\nCSV Row:")
    print(imu_to_csv_row(imu_data))

if __name__ == "__main__":
    main()