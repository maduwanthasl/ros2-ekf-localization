#!/usr/bin/env python3
"""
Sensor Measurements vs EKF Estimate
Shows GPS measurements overlaid with EKF estimates
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
    print("Or use the venv: /home/shevi/ekf_ws/.venv/bin/python plot_sensor_vs_ekf.py bags/ekf_bag")
    sys.exit(1)


def quaternion_to_yaw(qw, qx, qy, qz):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def read_bag(bag_path, read_imu=False):
    """Read data from ROS2 bag."""
    ekf_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    gps_data = {'t': [], 'x': [], 'y': []}
    imu_data = {'t': [], 'yaw': []}
    
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
            
            if connection.topic == '/ekf/pose':
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
            
            elif connection.topic == '/imu_yaw' and read_imu:
                imu_data['t'].append(t)
                imu_data['yaw'].append(msg.data)
    
    # Convert to numpy arrays
    for key in ekf_data:
        ekf_data[key] = np.array(ekf_data[key])
    for key in gps_data:
        gps_data[key] = np.array(gps_data[key])
    for key in imu_data:
        imu_data[key] = np.array(imu_data[key])
    
    print(f"Read {len(ekf_data['t'])} EKF poses")
    print(f"Read {len(gps_data['t'])} GPS measurements")
    if read_imu:
        print(f"Read {len(imu_data['t'])} IMU measurements")
    
    return ekf_data, gps_data, imu_data


def plot_gps_vs_ekf(ekf_data, gps_data):
    """Plot GPS measurements vs EKF estimates (Option A)."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # X position
    ax1.scatter(gps_data['t'], gps_data['x'], c='orange', s=40, alpha=0.6, 
                marker='x', linewidths=2, label='GPS Measurements', zorder=2)
    ax1.plot(ekf_data['t'], ekf_data['x'], 'b-', linewidth=2.5, 
             label='EKF Estimate', alpha=0.9, zorder=3)
    ax1.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax1.set_ylabel('X Position [m]', fontsize=13, fontweight='bold')
    ax1.set_title('GPS vs EKF: X Position', fontsize=15, fontweight='bold', pad=10)
    ax1.legend(loc='best', fontsize=12, framealpha=0.95)
    ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax1.set_facecolor('#f8f8f8')
    
    # Y position
    ax2.scatter(gps_data['t'], gps_data['y'], c='orange', s=40, alpha=0.6, 
                marker='x', linewidths=2, label='GPS Measurements', zorder=2)
    ax2.plot(ekf_data['t'], ekf_data['y'], 'g-', linewidth=2.5, 
             label='EKF Estimate', alpha=0.9, zorder=3)
    ax2.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax2.set_ylabel('Y Position [m]', fontsize=13, fontweight='bold')
    ax2.set_title('GPS vs EKF: Y Position', fontsize=15, fontweight='bold', pad=10)
    ax2.legend(loc='best', fontsize=12, framealpha=0.95)
    ax2.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax2.set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    return fig


def plot_imu_vs_ekf(ekf_data, imu_data, use_degrees=True):
    """Plot IMU yaw measurements vs EKF estimates (Option B)."""
    # Convert to degrees if requested
    if use_degrees:
        ekf_yaw = np.degrees(ekf_data['yaw'])
        imu_yaw = np.degrees(imu_data['yaw'])
        unit = 'deg'
        unit_symbol = '°'
    else:
        ekf_yaw = ekf_data['yaw']
        imu_yaw = imu_data['yaw']
        unit = 'rad'
        unit_symbol = ' rad'
    
    plt.figure(figsize=(12, 6))
    
    plt.scatter(imu_data['t'], imu_yaw, c='red', s=20, alpha=0.4, 
                marker='o', label='IMU Measurements', zorder=2)
    plt.plot(ekf_data['t'], ekf_yaw, 'b-', linewidth=2.5, 
             label='EKF Estimate', alpha=0.9, zorder=3)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel(f'Yaw [{unit}]', fontsize=14, fontweight='bold')
    plt.title('IMU vs EKF: Heading (Yaw)', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    return plt.gcf()


def plot_combined_gps_vs_ekf(ekf_data, gps_data):
    """Plot GPS vs EKF on single plot with both X and Y."""
    plt.figure(figsize=(12, 6))
    
    # X position
    plt.scatter(gps_data['t'], gps_data['x'], c='orange', s=40, alpha=0.5, 
                marker='x', linewidths=2, label='GPS X', zorder=2)
    plt.plot(ekf_data['t'], ekf_data['x'], 'b-', linewidth=2.5, 
             label='EKF X', alpha=0.9, zorder=3)
    
    # Y position
    plt.scatter(gps_data['t'], gps_data['y'], c='red', s=40, alpha=0.5, 
                marker='+', linewidths=2, label='GPS Y', zorder=2)
    plt.plot(ekf_data['t'], ekf_data['y'], 'g-', linewidth=2.5, 
             label='EKF Y', alpha=0.9, zorder=3)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel('Position [m]', fontsize=14, fontweight='bold')
    plt.title('GPS Measurements vs EKF Estimate', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95, ncol=2)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    return plt.gcf()


def compute_statistics(ekf_data, gps_data):
    """Compute statistics on GPS noise."""
    # Interpolate EKF to GPS times
    ekf_x_at_gps = np.interp(gps_data['t'], ekf_data['t'], ekf_data['x'])
    ekf_y_at_gps = np.interp(gps_data['t'], ekf_data['t'], ekf_data['y'])
    
    # GPS innovation (difference between GPS and EKF at measurement times)
    x_innovation = gps_data['x'] - ekf_x_at_gps
    y_innovation = gps_data['y'] - ekf_y_at_gps
    
    print("\n" + "="*60)
    print("GPS MEASUREMENT STATISTICS")
    print("="*60)
    print(f"\nGPS X Innovation (GPS - EKF):")
    print(f"  Mean:  {np.mean(x_innovation):.4f} m")
    print(f"  Std:   {np.std(x_innovation):.4f} m")
    print(f"  This shows GPS noise level")
    
    print(f"\nGPS Y Innovation (GPS - EKF):")
    print(f"  Mean:  {np.mean(y_innovation):.4f} m")
    print(f"  Std:   {np.std(y_innovation):.4f} m")
    print(f"  This shows GPS noise level")
    
    print(f"\nEKF smooths {len(ekf_data['t'])} samples from {len(gps_data['t'])} GPS measurements")
    print(f"Update rate: EKF ~{1/np.mean(np.diff(ekf_data['t'])):.1f} Hz, GPS ~{1/np.mean(np.diff(gps_data['t'])):.1f} Hz")
    print("="*60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_sensor_vs_ekf.py <bag_directory> [--imu] [--combined]")
        print("\nOptions:")
        print("  (default)    : GPS vs EKF (separate X/Y subplots)")
        print("  --combined   : GPS vs EKF (X and Y on same plot)")
        print("  --imu        : IMU yaw vs EKF yaw")
        print("\nExamples:")
        print("  python3 plot_sensor_vs_ekf.py bags/ekf_bag")
        print("  python3 plot_sensor_vs_ekf.py bags/ekf_bag --combined")
        print("  python3 plot_sensor_vs_ekf.py bags/ekf_bag --imu")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    use_imu = '--imu' in sys.argv
    use_combined = '--combined' in sys.argv
    
    # Read bag data
    ekf_data, gps_data, imu_data = read_bag(bag_path, read_imu=use_imu)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    # Create plots
    if use_imu:
        if len(imu_data['t']) == 0:
            print("Error: No IMU data found in bag!")
            sys.exit(1)
        print("Creating IMU vs EKF plot...")
        fig = plot_imu_vs_ekf(ekf_data, imu_data)
        output_file = 'imu_vs_ekf.png'
        output_pdf = 'imu_vs_ekf.pdf'
    elif use_combined:
        print("Creating combined GPS vs EKF plot...")
        compute_statistics(ekf_data, gps_data)
        fig = plot_combined_gps_vs_ekf(ekf_data, gps_data)
        output_file = 'gps_vs_ekf_combined.png'
        output_pdf = 'gps_vs_ekf_combined.pdf'
    else:
        print("Creating GPS vs EKF plot...")
        compute_statistics(ekf_data, gps_data)
        fig = plot_gps_vs_ekf(ekf_data, gps_data)
        output_file = 'gps_vs_ekf.png'
        output_pdf = 'gps_vs_ekf.pdf'
    
    # Save figures
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Plot saved to: {output_file}")
    
    plt.savefig(output_pdf, bbox_inches='tight', facecolor='white')
    print(f"PDF saved to: {output_pdf}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
