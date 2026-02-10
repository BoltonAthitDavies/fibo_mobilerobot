#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from datetime import datetime
import argparse

def load_trajectory_data(file_path):
    """Load trajectory data from text file."""
    try:
        data = np.loadtxt(file_path, skiprows=1)  # Skip header
        if len(data.shape) == 1:  # Single row
            data = data.reshape(1, -1)
        return data
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        return None

def plot_trajectories():
    """Plot and compare trajectories from different methods."""
    
    # Create output directory
    output_dir = 'trajectory_comparison'
    os.makedirs(output_dir, exist_ok=True)
    
    # Define expected trajectory file patterns
    trajectory_types = {
        'Wheel Odometry': 'wheel_odom*.txt',
        'EKF Odometry': 'ekf_odom*.txt', 
        'ICP Odometry': 'icp_odom*.txt',
        'SLAM': 'slam_trajectory*.txt'
    }
    
    # Search for trajectory files in multiple directories
    search_dirs = [
        '.',
        'slam_results',
        'results',
        'part1_results', 
        'part2_results',
        'trajectories'
    ]
    
    plt.figure(figsize=(15, 10))
    
    found_trajectories = {}
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    color_idx = 0
    
    # Search for each trajectory type
    for traj_name, pattern in trajectory_types.items():
        trajectory_found = False
        
        for search_dir in search_dirs:
            if not os.path.exists(search_dir):
                continue
                
            files = glob.glob(os.path.join(search_dir, pattern))
            
            if files:
                # Use the most recent file if multiple found
                latest_file = max(files, key=os.path.getctime)
                data = load_trajectory_data(latest_file)
                
                if data is not None:
                    # Assume columns are [timestamp, x, y] or [x, y]
                    if data.shape[1] >= 3:
                        x_data, y_data = data[:, 1], data[:, 2]
                    else:
                        x_data, y_data = data[:, 0], data[:, 1]
                    
                    plt.plot(x_data, y_data, 
                            color=colors[color_idx % len(colors)],
                            linewidth=2, 
                            label=f'{traj_name} ({os.path.basename(latest_file)})')
                    
                    found_trajectories[traj_name] = {
                        'file': latest_file,
                        'x': x_data,
                        'y': y_data
                    }
                    
                    trajectory_found = True
                    color_idx += 1
                    break
        
        if not trajectory_found:
            print(f"Warning: No trajectory files found for {traj_name}")
    
    # Configure plot
    plt.xlabel('X Position [m]', fontsize=12)
    plt.ylabel('Y Position [m]', fontsize=12)
    plt.title('Trajectory Comparison - FRA532 Lab 1', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend(fontsize=10)
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(output_dir, f'trajectory_comparison_{timestamp}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    
    print(f"Trajectory comparison plot saved: {plot_file}")
    
    # Create individual sequence plots if SLAM results exist
    if 'SLAM' in found_trajectories:
        create_sequence_plots(output_dir)
    
    plt.show()

def create_sequence_plots(output_dir):
    """Create separate plots for each dataset sequence."""
    
    sequences = ['seq00', 'seq01', 'seq02']
    sequence_names = {
        'seq00': 'Empty Hallway',
        'seq01': 'Non-Empty with Sharp Turns', 
        'seq02': 'Non-Empty with Non-Aggressive Motion'
    }
    
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    
    for idx, seq in enumerate(sequences):
        ax = axes[idx]
        
        # Look for trajectory files containing sequence identifier
        trajectory_files = glob.glob(f'slam_results/*{seq}*.txt') + \
                          glob.glob(f'*{seq}*trajectory*.txt')
        
        if trajectory_files:
            latest_file = max(trajectory_files, key=os.path.getctime)
            data = load_trajectory_data(latest_file)
            
            if data is not None:
                if data.shape[1] >= 3:
                    x_data, y_data = data[:, 1], data[:, 2]
                else:
                    x_data, y_data = data[:, 0], data[:, 1]
                
                ax.plot(x_data, y_data, 'b-', linewidth=2, alpha=0.8)
                ax.scatter(x_data[0], y_data[0], color='green', s=100, 
                          marker='o', label='Start', zorder=5)
                ax.scatter(x_data[-1], y_data[-1], color='red', s=100, 
                          marker='X', label='End', zorder=5)
        
        ax.set_title(f'{sequence_names[seq]}', fontsize=12, fontweight='bold')
        ax.set_xlabel('X Position [m]')
        ax.set_ylabel('Y Position [m]') 
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend()
    
    plt.tight_layout()
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(output_dir, f'sequence_trajectories_{timestamp}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    
    print(f"Sequence trajectories plot saved: {plot_file}")
    plt.show()

def compute_trajectory_metrics():
    """Compute and display trajectory metrics."""
    
    print("\n" + "="*60)
    print("TRAJECTORY ANALYSIS")
    print("="*60)
    
    # Look for SLAM trajectory files
    slam_files = glob.glob('slam_results/slam_trajectory*.txt')
    
    if not slam_files:
        print("No SLAM trajectory files found for analysis.")
        return
    
    for slam_file in slam_files:
        print(f"\nAnalyzing: {os.path.basename(slam_file)}")
        
        data = load_trajectory_data(slam_file)
        if data is None:
            continue
        
        # Extract trajectory data
        if data.shape[1] >= 3:
            timestamps, x_data, y_data = data[:, 0], data[:, 1], data[:, 2]
        else:
            x_data, y_data = data[:, 0], data[:, 1]
            timestamps = np.arange(len(x_data))
        
        # Compute metrics
        total_distance = np.sum(np.sqrt(np.diff(x_data)**2 + np.diff(y_data)**2))
        max_x, min_x = np.max(x_data), np.min(x_data)
        max_y, min_y = np.max(y_data), np.min(y_data)
        
        start_pos = (x_data[0], y_data[0])
        end_pos = (x_data[-1], y_data[-1])
        loop_closure_error = np.sqrt((end_pos[0] - start_pos[0])**2 + 
                                   (end_pos[1] - start_pos[1])**2)
        
        print(f"  Total Distance Traveled: {total_distance:.2f} m")
        print(f"  Workspace Bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")  
        print(f"  Start Position: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
        print(f"  End Position: ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
        print(f"  Loop Closure Error: {loop_closure_error:.3f} m")
        print(f"  Number of Poses: {len(x_data)}")

def main():
    parser = argparse.ArgumentParser(description='Compare and analyze trajectories from FRA532 Lab 1')
    parser.add_argument('--plot', action='store_true', help='Generate trajectory comparison plots')
    parser.add_argument('--analyze', action='store_true', help='Compute trajectory metrics')
    parser.add_argument('--all', action='store_true', help='Run all analysis and plotting')
    
    args = parser.parse_args()
    
    if args.all or (not args.plot and not args.analyze):
        print("Running complete trajectory analysis...")
        plot_trajectories()
        compute_trajectory_metrics()
    else:
        if args.plot:
            plot_trajectories()
        if args.analyze:
            compute_trajectory_metrics()

if __name__ == '__main__':
    main()