#!/usr/bin/env python3
"""
Position Error vs Time Plot
Plots x and y errors over time for EKF performance analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError as e:
    print(f"Error: rosbags library not found! ({e})")
    print("Install it with: pip install --break-system-packages rosbags matplotlib numpy")
    print("Or use the venv: /home/shevi/ekf_ws/.venv/bin/python plot_position_error.py bags/ekf_bag")
    sys.exit(1)


def quaternion_to_yaw(qw, qx, qy, qz):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def read_bag(bag_path):
    """Read data from ROS2 bag."""
    truth_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    ekf_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    
    bag_path = Path(bag_path)
    if not bag_path.exists():
        print(f"Error: Bag path {bag_path} does not exist!")
        sys.exit(1)
    
    print(f"Reading bag from: {bag_path}")
    
    # Create a type store
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with Reader(bag_path) as reader:        
        start_time = None
        
        for connection, timestamp, rawdata in reader.messages():
            # Deserialize message
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
            if start_time is None:
                start_time = timestamp
            
            t = (timestamp - start_time) / 1e9
            
            if connection.topic == '/truth/pose':
                truth_data['t'].append(t)
                truth_data['x'].append(msg.pose.position.x)
                truth_data['y'].append(msg.pose.position.y)
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.w, msg.pose.orientation.x,
                    msg.pose.orientation.y, msg.pose.orientation.z
                )
                truth_data['yaw'].append(yaw)
            
            elif connection.topic == '/ekf/pose':
                ekf_data['t'].append(t)
                ekf_data['x'].append(msg.pose.pose.position.x)
                ekf_data['y'].append(msg.pose.pose.position.y)
                yaw = quaternion_to_yaw(
                    msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
                )
                ekf_data['yaw'].append(yaw)
    
    # Convert to numpy arrays
    for key in truth_data:
        truth_data[key] = np.array(truth_data[key])
    for key in ekf_data:
        ekf_data[key] = np.array(ekf_data[key])
    
    print(f"Read {len(truth_data['t'])} truth poses")
    print(f"Read {len(ekf_data['t'])} EKF poses")
    
    return truth_data, ekf_data


def interpolate_truth(truth_data, ekf_times):
    """Interpolate truth data to match EKF timestamps."""
    truth_x_interp = np.interp(ekf_times, truth_data['t'], truth_data['x'])
    truth_y_interp = np.interp(ekf_times, truth_data['t'], truth_data['y'])
    
    return truth_x_interp, truth_y_interp


def plot_position_errors(truth_data, ekf_data):
    """Create position error vs time plots."""
    # Interpolate truth to EKF times
    truth_x, truth_y = interpolate_truth(truth_data, ekf_data['t'])
    
    # Compute errors (absolute values)
    x_error = np.abs(ekf_data['x'] - truth_x)
    y_error = np.abs(ekf_data['y'] - truth_y)
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # X Error plot
    ax1.plot(ekf_data['t'], x_error, 'b-', linewidth=1.5, label='|x - x̂|')
    ax1.axhline(y=np.mean(x_error), color='r', linestyle='--', 
                linewidth=2, label=f'Mean: {np.mean(x_error):.4f} m', alpha=0.7)
    ax1.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Error [m]', fontsize=13, fontweight='bold')
    ax1.set_title('X Position Error vs Time', fontsize=15, fontweight='bold', pad=10)
    ax1.legend(loc='best', fontsize=11)
    ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax1.set_ylim(bottom=0)
    
    # Y Error plot
    ax2.plot(ekf_data['t'], y_error, 'g-', linewidth=1.5, label='|y - ŷ|')
    ax2.axhline(y=np.mean(y_error), color='r', linestyle='--', 
                linewidth=2, label=f'Mean: {np.mean(y_error):.4f} m', alpha=0.7)
    ax2.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax2.set_ylabel('Error [m]', fontsize=13, fontweight='bold')
    ax2.set_title('Y Position Error vs Time', fontsize=15, fontweight='bold', pad=10)
    ax2.legend(loc='best', fontsize=11)
    ax2.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax2.set_ylim(bottom=0)
    
    # Add subtle background
    ax1.set_facecolor('#f8f8f8')
    ax2.set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return fig


def plot_combined_errors(truth_data, ekf_data):
    """Create combined position error plot."""
    # Interpolate truth to EKF times
    truth_x, truth_y = interpolate_truth(truth_data, ekf_data['t'])
    
    # Compute errors (absolute values)
    x_error = np.abs(ekf_data['x'] - truth_x)
    y_error = np.abs(ekf_data['y'] - truth_y)
    
    # Create single plot
    plt.figure(figsize=(12, 6))
    
    plt.plot(ekf_data['t'], x_error, 'b-', linewidth=2, label='|x - x̂|', alpha=0.8)
    plt.plot(ekf_data['t'], y_error, 'g-', linewidth=2, label='|y - ŷ|', alpha=0.8)
    
    # Add mean lines
    plt.axhline(y=np.mean(x_error), color='b', linestyle='--', 
                linewidth=1.5, label=f'X Mean: {np.mean(x_error):.4f} m', alpha=0.5)
    plt.axhline(y=np.mean(y_error), color='g', linestyle='--', 
                linewidth=1.5, label=f'Y Mean: {np.mean(y_error):.4f} m', alpha=0.5)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel('Error [m]', fontsize=14, fontweight='bold')
    plt.title('Position Error vs Time', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.ylim(bottom=0)
    
    # Add subtle background
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return plt.gcf()


def compute_statistics(truth_data, ekf_data):
    """Compute and print error statistics."""
    # Interpolate truth to EKF times
    truth_x, truth_y = interpolate_truth(truth_data, ekf_data['t'])
    
    # Compute errors
    x_error = ekf_data['x'] - truth_x
    y_error = ekf_data['y'] - truth_y
    x_error_abs = np.abs(x_error)
    y_error_abs = np.abs(y_error)
    
    print("\n" + "="*60)
    print("POSITION ERROR STATISTICS")
    print("="*60)
    print(f"\n|x - x̂| (Absolute X Error):")
    print(f"  Mean:  {np.mean(x_error_abs):.4f} m")
    print(f"  Std:   {np.std(x_error_abs):.4f} m")
    print(f"  Max:   {np.max(x_error_abs):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(x_error**2)):.4f} m")
    
    print(f"\n|y - ŷ| (Absolute Y Error):")
    print(f"  Mean:  {np.mean(y_error_abs):.4f} m")
    print(f"  Std:   {np.std(y_error_abs):.4f} m")
    print(f"  Max:   {np.max(y_error_abs):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(y_error**2)):.4f} m")
    
    pos_error = np.sqrt(x_error**2 + y_error**2)
    print(f"\nTotal Position Error:")
    print(f"  Mean:  {np.mean(pos_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(pos_error**2)):.4f} m")
    print(f"  Max:   {np.max(pos_error):.4f} m")
    print("="*60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_position_error.py <bag_directory> [--combined]")
        print("Example: python3 plot_position_error.py bags/ekf_bag")
        print("         python3 plot_position_error.py bags/ekf_bag --combined")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    combined = '--combined' in sys.argv
    
    # Read bag data
    truth_data, ekf_data = read_bag(bag_path)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    # Compute statistics
    compute_statistics(truth_data, ekf_data)
    
    # Create plots
    if combined:
        print("Creating combined error plot...")
        fig = plot_combined_errors(truth_data, ekf_data)
        output_file = 'position_error_combined.png'
        output_pdf = 'position_error_combined.pdf'
    else:
        print("Creating separate error plots...")
        fig = plot_position_errors(truth_data, ekf_data)
        output_file = 'position_error_vs_time.png'
        output_pdf = 'position_error_vs_time.pdf'
    
    # Save figures
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Plot saved to: {output_file}")
    
    plt.savefig(output_pdf, bbox_inches='tight', facecolor='white')
    print(f"PDF saved to: {output_pdf}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
