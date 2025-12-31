#!/usr/bin/env python3
"""
Heading (Yaw) Error vs Time Plot
Plots yaw/heading error over time for EKF performance analysis
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
    print("Or use the venv: /home/shevi/ekf_ws/.venv/bin/python plot_yaw_error.py bags/ekf_bag")
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


def interpolate_truth_yaw(truth_data, ekf_times):
    """Interpolate truth yaw to match EKF timestamps."""
    truth_yaw_interp = np.interp(ekf_times, truth_data['t'], truth_data['yaw'])
    return truth_yaw_interp


def plot_yaw_error(truth_data, ekf_data, use_degrees=True):
    """Create yaw error vs time plot."""
    # Interpolate truth to EKF times
    truth_yaw = interpolate_truth_yaw(truth_data, ekf_data['t'])
    
    # Compute error and wrap to [-pi, pi]
    yaw_error = wrap_angle(ekf_data['yaw'] - truth_yaw)
    
    # Convert to degrees if requested
    if use_degrees:
        yaw_error = np.degrees(yaw_error)
        yaw_error_abs = np.abs(yaw_error)
        unit = 'deg'
        unit_symbol = '°'
    else:
        yaw_error_abs = np.abs(yaw_error)
        unit = 'rad'
        unit_symbol = ' rad'
    
    # Create figure
    plt.figure(figsize=(12, 6))
    
    plt.plot(ekf_data['t'], yaw_error_abs, 'r-', linewidth=1.5, label='|θ - θ̂|')
    
    # Add mean line
    mean_error = np.mean(yaw_error_abs)
    plt.axhline(y=mean_error, color='b', linestyle='--', 
                linewidth=2, label=f'Mean: {mean_error:.4f}{unit_symbol}', alpha=0.7)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel(f'Error [{unit}]', fontsize=14, fontweight='bold')
    plt.title('Heading (Yaw) Error vs Time', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.ylim(bottom=0)
    
    # Add subtle background
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return plt.gcf(), yaw_error


def plot_yaw_error_signed(truth_data, ekf_data, use_degrees=True):
    """Create signed yaw error vs time plot (shows positive/negative errors)."""
    # Interpolate truth to EKF times
    truth_yaw = interpolate_truth_yaw(truth_data, ekf_data['t'])
    
    # Compute error and wrap to [-pi, pi]
    yaw_error = wrap_angle(ekf_data['yaw'] - truth_yaw)
    
    # Convert to degrees if requested
    if use_degrees:
        yaw_error = np.degrees(yaw_error)
        unit = 'deg'
        unit_symbol = '°'
    else:
        unit = 'rad'
        unit_symbol = ' rad'
    
    # Create figure
    plt.figure(figsize=(12, 6))
    
    plt.plot(ekf_data['t'], yaw_error, 'purple', linewidth=1.5, label='θ - θ̂')
    
    # Add zero line
    plt.axhline(y=0, color='k', linestyle='-', linewidth=1, alpha=0.5)
    
    # Add mean line
    mean_error = np.mean(yaw_error)
    plt.axhline(y=mean_error, color='b', linestyle='--', 
                linewidth=2, label=f'Mean: {mean_error:.4f}{unit_symbol}', alpha=0.7)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel(f'Error [{unit}]', fontsize=14, fontweight='bold')
    plt.title('Heading (Yaw) Error vs Time (Signed)', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    
    # Add subtle background
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return plt.gcf(), yaw_error


def compute_statistics(truth_data, ekf_data):
    """Compute and print yaw error statistics."""
    # Interpolate truth to EKF times
    truth_yaw = interpolate_truth_yaw(truth_data, ekf_data['t'])
    
    # Compute error
    yaw_error = wrap_angle(ekf_data['yaw'] - truth_yaw)
    yaw_error_abs = np.abs(yaw_error)
    
    print("\n" + "="*60)
    print("HEADING (YAW) ERROR STATISTICS")
    print("="*60)
    print(f"\n|θ - θ̂| (Absolute Yaw Error):")
    print(f"  Mean:  {np.mean(yaw_error_abs):.4f} rad = {np.degrees(np.mean(yaw_error_abs)):.4f}°")
    print(f"  Std:   {np.std(yaw_error_abs):.4f} rad = {np.degrees(np.std(yaw_error_abs)):.4f}°")
    print(f"  Max:   {np.max(yaw_error_abs):.4f} rad = {np.degrees(np.max(yaw_error_abs)):.4f}°")
    print(f"  RMSE:  {np.sqrt(np.mean(yaw_error**2)):.4f} rad = {np.degrees(np.sqrt(np.mean(yaw_error**2))):.4f}°")
    
    print(f"\nSigned Error (θ - θ̂):")
    print(f"  Mean:  {np.mean(yaw_error):.4f} rad = {np.degrees(np.mean(yaw_error)):.4f}°")
    print(f"  Std:   {np.std(yaw_error):.4f} rad = {np.degrees(np.std(yaw_error)):.4f}°")
    print("="*60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_yaw_error.py <bag_directory> [--radians] [--signed]")
        print("Example: python3 plot_yaw_error.py bags/ekf_bag")
        print("         python3 plot_yaw_error.py bags/ekf_bag --radians")
        print("         python3 plot_yaw_error.py bags/ekf_bag --signed")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    use_radians = '--radians' in sys.argv
    use_signed = '--signed' in sys.argv
    
    # Read bag data
    truth_data, ekf_data = read_bag(bag_path)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    # Compute statistics
    compute_statistics(truth_data, ekf_data)
    
    # Create plot
    if use_signed:
        print("Creating signed yaw error plot...")
        fig, yaw_error = plot_yaw_error_signed(truth_data, ekf_data, use_degrees=not use_radians)
        output_file = 'yaw_error_signed.png'
        output_pdf = 'yaw_error_signed.pdf'
    else:
        print("Creating absolute yaw error plot...")
        fig, yaw_error = plot_yaw_error(truth_data, ekf_data, use_degrees=not use_radians)
        output_file = 'yaw_error_vs_time.png'
        output_pdf = 'yaw_error_vs_time.pdf'
    
    # Save figures
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Plot saved to: {output_file}")
    
    plt.savefig(output_pdf, bbox_inches='tight', facecolor='white')
    print(f"PDF saved to: {output_pdf}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
