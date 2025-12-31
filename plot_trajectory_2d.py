#!/usr/bin/env python3
"""
2D Trajectory Plot - Publication Quality
Plots ground truth vs EKF estimate for report
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
    print("Or use the venv: /home/shevi/ekf_ws/.venv/bin/python plot_trajectory_2d.py bags/ekf_bag")
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
    gps_data = {'t': [], 'x': [], 'y': []}
    cmd_data = {'t': [], 'v': [], 'w': []}
    
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
            
            elif connection.topic == '/gps':
                gps_data['t'].append(t)
                gps_data['x'].append(msg.pose.position.x)
                gps_data['y'].append(msg.pose.position.y)
            
            elif connection.topic == '/cmd_vel':
                cmd_data['t'].append(t)
                cmd_data['v'].append(msg.linear.x)
                cmd_data['w'].append(msg.angular.z)
    
    # Convert to numpy arrays
    for key in truth_data:
        truth_data[key] = np.array(truth_data[key])
    for key in ekf_data:
        ekf_data[key] = np.array(ekf_data[key])
    for key in gps_data:
        gps_data[key] = np.array(gps_data[key])
    for key in cmd_data:
        cmd_data[key] = np.array(cmd_data[key])
    
    print(f"Read {len(truth_data['t'])} truth poses")
    print(f"Read {len(ekf_data['t'])} EKF poses")
    print(f"Read {len(gps_data['t'])} GPS measurements")
    
    return truth_data, ekf_data, gps_data, cmd_data


def compute_dead_reckoning(cmd_data, initial_x=0.0, initial_y=0.0, initial_yaw=0.0):
    """Compute dead reckoning trajectory from velocity commands."""
    if len(cmd_data['t']) == 0:
        return {'t': np.array([]), 'x': np.array([]), 'y': np.array([])}
    
    x = [initial_x]
    y = [initial_y]
    yaw = initial_yaw
    
    dt = 0.02  # 50 Hz
    
    for i in range(len(cmd_data['v'])):
        v = cmd_data['v'][i]
        w = cmd_data['w'][i]
        
        # Integrate
        x.append(x[-1] + v * np.cos(yaw) * dt)
        y.append(y[-1] + v * np.sin(yaw) * dt)
        yaw += w * dt
    
    return {
        't': cmd_data['t'],
        'x': np.array(x[1:]),  # Skip initial
        'y': np.array(y[1:])
    }


def plot_2d_trajectory(truth_data, ekf_data, gps_data, dead_reckoning=None):
    """Create publication-quality 2D trajectory plot."""
    plt.figure(figsize=(10, 8))
    
    # Plot trajectories
    plt.plot(truth_data['x'], truth_data['y'], 
             'k-', linewidth=2.5, label='Ground Truth', zorder=3)
    
    plt.plot(ekf_data['x'], ekf_data['y'], 
             'b-', linewidth=2, label='EKF Estimate', alpha=0.85, zorder=2)
    
    if dead_reckoning is not None and len(dead_reckoning['x']) > 0:
        plt.plot(dead_reckoning['x'], dead_reckoning['y'], 
                 'r--', linewidth=2, label='Dead Reckoning', alpha=0.7, zorder=1)
    
    # Plot GPS measurements as scatter
    plt.scatter(gps_data['x'], gps_data['y'], 
                c='orange', s=25, alpha=0.4, marker='x', 
                label='GPS Measurements', zorder=0)
    
    # Mark start point
    plt.plot(truth_data['x'][0], truth_data['y'][0], 
             'go', markersize=12, label='Start', 
             markeredgecolor='darkgreen', markeredgewidth=2.5, zorder=4)
    
    # Mark end point
    plt.plot(truth_data['x'][-1], truth_data['y'][-1], 
             'rs', markersize=12, label='End', 
             markeredgecolor='darkred', markeredgewidth=2.5, zorder=4)
    
    # Labels and formatting
    plt.xlabel('X [m]', fontsize=14, fontweight='bold')
    plt.ylabel('Y [m]', fontsize=14, fontweight='bold')
    plt.title('2D Trajectory (Top-Down View)', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.axis('equal')
    
    # Add subtle background
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return plt.gcf()


def compute_statistics(truth_data, ekf_data):
    """Compute error statistics."""
    # Interpolate truth to EKF times
    truth_x = np.interp(ekf_data['t'], truth_data['t'], truth_data['x'])
    truth_y = np.interp(ekf_data['t'], truth_data['t'], truth_data['y'])
    
    # Compute errors
    x_error = ekf_data['x'] - truth_x
    y_error = ekf_data['y'] - truth_y
    pos_error = np.sqrt(x_error**2 + y_error**2)
    
    print("\n" + "="*50)
    print("EKF PERFORMANCE STATISTICS")
    print("="*50)
    print(f"\nPosition Error:")
    print(f"  Mean:  {np.mean(pos_error):.4f} m")
    print(f"  Std:   {np.std(pos_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(pos_error**2)):.4f} m")
    print(f"  Max:   {np.max(pos_error):.4f} m")
    print(f"\nX Error:")
    print(f"  Mean:  {np.mean(x_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(x_error**2)):.4f} m")
    print(f"\nY Error:")
    print(f"  Mean:  {np.mean(y_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(y_error**2)):.4f} m")
    print("="*50 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory_2d.py <bag_directory>")
        print("Example: python3 plot_trajectory_2d.py bags/ekf_bag")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    
    # Read bag data
    truth_data, ekf_data, gps_data, cmd_data = read_bag(bag_path)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    # Compute dead reckoning (optional)
    dead_reckoning = None
    if len(cmd_data['t']) > 0:
        print("Computing dead reckoning trajectory...")
        dead_reckoning = compute_dead_reckoning(
            cmd_data, 
            truth_data['x'][0], 
            truth_data['y'][0], 
            truth_data['yaw'][0]
        )
    
    # Compute statistics
    compute_statistics(truth_data, ekf_data)
    
    # Create plot
    fig = plot_2d_trajectory(truth_data, ekf_data, gps_data, dead_reckoning)
    
    # Save figure
    output_file = 'trajectory_2d.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Plot saved to: {output_file}")
    
    # Also save as PDF for publication
    output_pdf = 'trajectory_2d.pdf'
    plt.savefig(output_pdf, bbox_inches='tight', facecolor='white')
    print(f"PDF saved to: {output_pdf}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
