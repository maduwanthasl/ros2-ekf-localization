#!/usr/bin/env python3
"""
EKF Results Plotter
Reads a ROS2 bag and plots:
1. XY trajectory comparison (truth vs EKF)
2. Position errors (x, y) over time with ±2σ bounds
3. Yaw error over time
4. GPS measurements scatter plot
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
except ImportError:
    print("Error: rosbags library not found!")
    print("Install it with: pip install rosbags")
    sys.exit(1)


def quaternion_to_yaw(qw, qx, qy, qz):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def read_bag(bag_path):
    """Read data from ROS2 bag."""
    truth_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    ekf_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    gps_data = {'t': [], 'x': [], 'y': []}
    
    bag_path = Path(bag_path)
    if not bag_path.exists():
        print(f"Error: Bag path {bag_path} does not exist!")
        sys.exit(1)
    
    print(f"Reading bag from: {bag_path}")
    
    with Reader(bag_path) as reader:
        # Get start time
        connections = [c for c in reader.connections]
        start_time = None
        
        for connection, timestamp, rawdata in reader.messages():
            msg = deserialize_cdr(rawdata, connection.msgtype)
            
            if start_time is None:
                start_time = timestamp
            
            t = (timestamp - start_time) / 1e9  # Convert to seconds
            
            if connection.topic == '/truth/pose':
                truth_data['t'].append(t)
                truth_data['x'].append(msg.pose.position.x)
                truth_data['y'].append(msg.pose.position.y)
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z
                )
                truth_data['yaw'].append(yaw)
            
            elif connection.topic == '/ekf/pose':
                ekf_data['t'].append(t)
                ekf_data['x'].append(msg.pose.position.x)
                ekf_data['y'].append(msg.pose.position.y)
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z
                )
                ekf_data['yaw'].append(yaw)
            
            elif connection.topic == '/gps':
                gps_data['t'].append(t)
                gps_data['x'].append(msg.pose.position.x)
                gps_data['y'].append(msg.pose.position.y)
    
    # Convert to numpy arrays
    for key in truth_data:
        truth_data[key] = np.array(truth_data[key])
    for key in ekf_data:
        ekf_data[key] = np.array(ekf_data[key])
    for key in gps_data:
        gps_data[key] = np.array(gps_data[key])
    
    print(f"Read {len(truth_data['t'])} truth poses")
    print(f"Read {len(ekf_data['t'])} EKF poses")
    print(f"Read {len(gps_data['t'])} GPS measurements")
    
    return truth_data, ekf_data, gps_data


def interpolate_truth(truth_data, ekf_times):
    """Interpolate truth data to match EKF timestamps."""
    truth_x_interp = np.interp(ekf_times, truth_data['t'], truth_data['x'])
    truth_y_interp = np.interp(ekf_times, truth_data['t'], truth_data['y'])
    truth_yaw_interp = np.interp(ekf_times, truth_data['t'], truth_data['yaw'])
    
    return truth_x_interp, truth_y_interp, truth_yaw_interp


def compute_errors_and_covariance(truth_data, ekf_data):
    """Compute errors and estimate covariance."""
    # Interpolate truth to EKF times
    truth_x, truth_y, truth_yaw = interpolate_truth(truth_data, ekf_data['t'])
    
    # Compute errors
    x_error = ekf_data['x'] - truth_x
    y_error = ekf_data['y'] - truth_y
    yaw_error = wrap_angle(ekf_data['yaw'] - truth_yaw)
    
    # Compute standard deviations (using sliding window)
    window_size = 50
    x_std = []
    y_std = []
    yaw_std = []
    
    for i in range(len(x_error)):
        start = max(0, i - window_size // 2)
        end = min(len(x_error), i + window_size // 2)
        
        x_std.append(np.std(x_error[start:end]))
        y_std.append(np.std(y_error[start:end]))
        yaw_std.append(np.std(yaw_error[start:end]))
    
    return {
        't': ekf_data['t'],
        'x_error': x_error,
        'y_error': y_error,
        'yaw_error': yaw_error,
        'x_std': np.array(x_std),
        'y_std': np.array(y_std),
        'yaw_std': np.array(yaw_std)
    }


def plot_results(truth_data, ekf_data, gps_data, errors):
    """Create comprehensive plots."""
    fig = plt.figure(figsize=(15, 10))
    
    # 1. XY Trajectory - Clean publication style
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(truth_data['x'], truth_data['y'], 'k-', linewidth=2.5, label='Ground Truth', alpha=0.9)
    ax1.plot(ekf_data['x'], ekf_data['y'], 'b-', linewidth=2, label='EKF Estimate', alpha=0.8)
    ax1.scatter(gps_data['x'], gps_data['y'], c='red', s=15, alpha=0.4, marker='x', label='GPS Measurements')
    
    # Mark start and end points
    ax1.plot(truth_data['x'][0], truth_data['y'][0], 'go', markersize=10, label='Start', markeredgecolor='darkgreen', markeredgewidth=2)
    ax1.plot(truth_data['x'][-1], truth_data['y'][-1], 'rs', markersize=10, label='End', markeredgecolor='darkred', markeredgewidth=2)
    
    ax1.set_xlabel('X [m]', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
    ax1.set_title('2D Trajectory (Top-Down View)', fontsize=14, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.axis('equal')
    
    # 2. X Error with ±2σ bounds
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(errors['t'], errors['x_error'], 'b-', linewidth=1, label='X Error')
    ax2.fill_between(errors['t'], 
                      -2 * errors['x_std'], 
                      2 * errors['x_std'],
                      alpha=0.3, color='blue', label='±2σ')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Error (m)')
    ax2.set_title('X Position Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Y Error with ±2σ bounds
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(errors['t'], errors['y_error'], 'g-', linewidth=1, label='Y Error')
    ax3.fill_between(errors['t'], 
                      -2 * errors['y_std'], 
                      2 * errors['y_std'],
                      alpha=0.3, color='green', label='±2σ')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Error (m)')
    ax3.set_title('Y Position Error')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Yaw Error with ±2σ bounds
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(errors['t'], np.degrees(errors['yaw_error']), 'r-', linewidth=1, label='Yaw Error')
    ax4.fill_between(errors['t'], 
                      -2 * np.degrees(errors['yaw_std']), 
                      2 * np.degrees(errors['yaw_std']),
                      alpha=0.3, color='red', label='±2σ')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw Error (deg)')
    ax4.set_title('Yaw Error')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. Position Error Magnitude
    ax5 = plt.subplot(2, 3, 5)
    pos_error = np.sqrt(errors['x_error']**2 + errors['y_error']**2)
    ax5.plot(errors['t'], pos_error, 'purple', linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Position Error (m)')
    ax5.set_title('Position Error Magnitude')
    ax5.grid(True, alpha=0.3)
    
    # 6. Error Statistics
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')
    
    stats_text = f"""
    EKF Performance Statistics
    ================================
    
    X Error:
      Mean: {np.mean(errors['x_error']):.4f} m
      Std:  {np.std(errors['x_error']):.4f} m
      RMSE: {np.sqrt(np.mean(errors['x_error']**2)):.4f} m
    
    Y Error:
      Mean: {np.mean(errors['y_error']):.4f} m
      Std:  {np.std(errors['y_error']):.4f} m
      RMSE: {np.sqrt(np.mean(errors['y_error']**2)):.4f} m
    
    Yaw Error:
      Mean: {np.mean(np.degrees(errors['yaw_error'])):.4f}°
      Std:  {np.std(np.degrees(errors['yaw_error'])):.4f}°
      RMSE: {np.sqrt(np.mean(errors['yaw_error']**2)) * 180/np.pi:.4f}°
    
    Position Error:
      Mean: {np.mean(pos_error):.4f} m
      Max:  {np.max(pos_error):.4f} m
      RMSE: {np.sqrt(np.mean(pos_error**2)):.4f} m
    """
    
    ax6.text(0.1, 0.5, stats_text, transform=ax6.transAxes,
             fontfamily='monospace', fontsize=9, verticalalignment='center')
    
    plt.tight_layout()
    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_ekf_results.py <bag_directory>")
        print("Example: python3 plot_ekf_results.py bags/ekf_bag")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    
    # Read bag data
    truth_data, ekf_data, gps_data = read_bag(bag_path)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    # Compute errors
    errors = compute_errors_and_covariance(truth_data, ekf_data)
    
    # Create plots
    fig = plot_results(truth_data, ekf_data, gps_data, errors)
    
    # Save figure
    output_file = 'ekf_results.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_file}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
