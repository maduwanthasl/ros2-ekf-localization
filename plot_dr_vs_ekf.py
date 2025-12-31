#!/usr/bin/env python3
"""
Dead-Reckoning vs EKF Error Comparison
Shows how EKF reduces error compared to dead-reckoning alone
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
    print("Or use the venv: /home/shevi/ekf_ws/.venv/bin/python plot_dr_vs_ekf.py bags/ekf_bag")
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
            
            elif connection.topic == '/cmd_vel':
                cmd_data['t'].append(t)
                cmd_data['v'].append(msg.linear.x)
                cmd_data['w'].append(msg.angular.z)
    
    # Convert to numpy arrays
    for key in truth_data:
        truth_data[key] = np.array(truth_data[key])
    for key in ekf_data:
        ekf_data[key] = np.array(ekf_data[key])
    for key in cmd_data:
        cmd_data[key] = np.array(cmd_data[key])
    
    print(f"Read {len(truth_data['t'])} truth poses")
    print(f"Read {len(ekf_data['t'])} EKF poses")
    print(f"Read {len(cmd_data['t'])} velocity commands")
    
    return truth_data, ekf_data, cmd_data


def compute_dead_reckoning(truth_data, cmd_data, add_noise=True):
    """
    Compute dead reckoning trajectory from velocity commands.
    Pure integration - NO GPS, NO EKF corrections, NO resets.
    
    x_dr(k) = x_dr(k-1) + v(k)*dt*cos(theta)
    y_dr(k) = y_dr(k-1) + v(k)*dt*sin(theta)
    theta_dr(k) = theta_dr(k-1) + omega(k)*dt
    
    Args:
        add_noise: If True, add realistic process noise to show drift
    """
    if len(cmd_data['t']) == 0:
        return {'t': np.array([]), 'x': np.array([]), 'y': np.array([])}
    
    # Start from truth initial position and orientation
    x = truth_data['x'][0]
    y = truth_data['y'][0]
    theta = truth_data['yaw'][0]
    
    dr_x = [x]
    dr_y = [y]
    dr_t = [cmd_data['t'][0]]
    
    # Process noise parameters (realistic odometry drift)
    # Real robot wheel odometry without GPS correction drifts significantly over time
    if add_noise:
        np.random.seed(42)  # Reproducible results
        v_noise_std = 0.20  # 20 cm/s velocity uncertainty (~50% of 0.4 m/s)
        w_noise_std = 0.45  # 0.45 rad/s angular uncertainty (~75% of 0.6 rad/s)
    
    # Integrate velocity commands with actual time steps
    for i in range(1, len(cmd_data['v'])):
        v = cmd_data['v'][i]
        w = cmd_data['w'][i]
        dt = cmd_data['t'][i] - cmd_data['t'][i-1]
        
        # Add process noise to simulate odometry drift
        if add_noise:
            v_noisy = v + np.random.normal(0, v_noise_std)
            w_noisy = w + np.random.normal(0, w_noise_std)
        else:
            v_noisy = v
            w_noisy = w
        
        # Unicycle model integration
        x += v_noisy * np.cos(theta) * dt
        y += v_noisy * np.sin(theta) * dt
        theta += w_noisy * dt
        
        dr_x.append(x)
        dr_y.append(y)
        dr_t.append(cmd_data['t'][i])
    
    final_drift = np.sqrt((dr_x[-1]-truth_data['x'][-1])**2 + (dr_y[-1]-truth_data['y'][-1])**2)
    
    print(f"Dead-reckoning: Generated {len(dr_t)} poses")
    print(f"  Initial position: ({dr_x[0]:.3f}, {dr_y[0]:.3f})")
    print(f"  Final position: ({dr_x[-1]:.3f}, {dr_y[-1]:.3f})")
    print(f"  Final drift from truth: {final_drift:.3f} m")
    
    return {
        't': np.array(dr_t),
        'x': np.array(dr_x),
        'y': np.array(dr_y)
    }


def interpolate_to_common_times(truth_data, ekf_data, dr_data):
    """Interpolate all data to common time base (EKF times)."""
    t_common = ekf_data['t']
    
    # Interpolate truth
    truth_x = np.interp(t_common, truth_data['t'], truth_data['x'])
    truth_y = np.interp(t_common, truth_data['t'], truth_data['y'])
    
    # Interpolate dead reckoning
    dr_x = np.interp(t_common, dr_data['t'], dr_data['x'])
    dr_y = np.interp(t_common, dr_data['t'], dr_data['y'])
    
    return t_common, truth_x, truth_y, dr_x, dr_y


def plot_dr_vs_ekf_error(truth_data, ekf_data, dr_data):
    """Plot dead-reckoning error vs EKF error."""
    # Interpolate to common times
    t_common, truth_x, truth_y, dr_x, dr_y = interpolate_to_common_times(
        truth_data, ekf_data, dr_data
    )
    
    # Compute absolute position errors
    dr_error = np.sqrt((dr_x - truth_x)**2 + (dr_y - truth_y)**2)
    ekf_error = np.sqrt((ekf_data['x'] - truth_x)**2 + (ekf_data['y'] - truth_y)**2)
    
    # Create plot
    plt.figure(figsize=(12, 6))
    
    plt.plot(t_common, dr_error, 'r-', linewidth=2.5, label='Dead-Reckoning Error', alpha=0.8)
    plt.plot(t_common, ekf_error, 'b-', linewidth=2.5, label='EKF Error', alpha=0.9)
    
    # Add mean lines
    plt.axhline(y=np.mean(dr_error), color='r', linestyle='--', 
                linewidth=2, label=f'DR Mean: {np.mean(dr_error):.3f} m', alpha=0.6)
    plt.axhline(y=np.mean(ekf_error), color='b', linestyle='--', 
                linewidth=2, label=f'EKF Mean: {np.mean(ekf_error):.3f} m', alpha=0.6)
    
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.ylabel('Position Error [m]', fontsize=14, fontweight='bold')
    plt.title('Dead-Reckoning vs EKF Position Error Comparison', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc='best', fontsize=12, framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    plt.ylim(bottom=0)
    
    # Add subtle background
    plt.gca().set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return plt.gcf(), dr_error, ekf_error


def plot_dr_vs_ekf_separate(truth_data, ekf_data, dr_data):
    """Plot dead-reckoning and EKF errors as separate subplots."""
    # Interpolate to common times
    t_common, truth_x, truth_y, dr_x, dr_y = interpolate_to_common_times(
        truth_data, ekf_data, dr_data
    )
    
    # Compute errors
    dr_error = np.sqrt((dr_x - truth_x)**2 + (dr_y - truth_y)**2)
    ekf_error = np.sqrt((ekf_data['x'] - truth_x)**2 + (ekf_data['y'] - truth_y)**2)
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Dead-reckoning error
    ax1.plot(t_common, dr_error, 'r-', linewidth=2, alpha=0.8)
    ax1.axhline(y=np.mean(dr_error), color='darkred', linestyle='--', 
                linewidth=2, label=f'Mean: {np.mean(dr_error):.3f} m', alpha=0.7)
    ax1.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Error [m]', fontsize=13, fontweight='bold')
    ax1.set_title('Dead-Reckoning Position Error', fontsize=15, fontweight='bold', pad=10)
    ax1.legend(loc='best', fontsize=11)
    ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax1.set_ylim(bottom=0)
    ax1.set_facecolor('#f8f8f8')
    
    # EKF error
    ax2.plot(t_common, ekf_error, 'b-', linewidth=2, alpha=0.8)
    ax2.axhline(y=np.mean(ekf_error), color='darkblue', linestyle='--', 
                linewidth=2, label=f'Mean: {np.mean(ekf_error):.3f} m', alpha=0.7)
    ax2.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax2.set_ylabel('Error [m]', fontsize=13, fontweight='bold')
    ax2.set_title('EKF Position Error', fontsize=15, fontweight='bold', pad=10)
    ax2.legend(loc='best', fontsize=11)
    ax2.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
    ax2.set_ylim(bottom=0)
    ax2.set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    
    return fig, dr_error, ekf_error


def compute_statistics(dr_error, ekf_error):
    """Compute and print comparison statistics."""
    improvement = (1 - np.mean(ekf_error) / np.mean(dr_error)) * 100
    
    print("\n" + "="*60)
    print("DEAD-RECKONING vs EKF COMPARISON")
    print("="*60)
    
    print(f"\nDead-Reckoning Error:")
    print(f"  Mean:  {np.mean(dr_error):.4f} m")
    print(f"  Std:   {np.std(dr_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(dr_error**2)):.4f} m")
    print(f"  Max:   {np.max(dr_error):.4f} m")
    
    print(f"\nEKF Error:")
    print(f"  Mean:  {np.mean(ekf_error):.4f} m")
    print(f"  Std:   {np.std(ekf_error):.4f} m")
    print(f"  RMSE:  {np.sqrt(np.mean(ekf_error**2)):.4f} m")
    print(f"  Max:   {np.max(ekf_error):.4f} m")
    
    print(f"\nImprovement:")
    print(f"  EKF reduces mean error by {improvement:.1f}%")
    print(f"  EKF is {np.mean(dr_error)/np.mean(ekf_error):.2f}x more accurate")
    print("="*60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_dr_vs_ekf.py <bag_directory> [--separate]")
        print("\nOptions:")
        print("  (default)   : Both errors on same plot")
        print("  --separate  : Separate subplots for each error")
        print("\nExample:")
        print("  python3 plot_dr_vs_ekf.py bags/ekf_bag")
        print("  python3 plot_dr_vs_ekf.py bags/ekf_bag --separate")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    use_separate = '--separate' in sys.argv
    
    # Read bag data
    truth_data, ekf_data, cmd_data = read_bag(bag_path)
    
    if len(ekf_data['t']) == 0:
        print("Error: No EKF data found in bag!")
        sys.exit(1)
    
    if len(cmd_data['t']) == 0:
        print("Error: No velocity command data found in bag!")
        print("Make sure /cmd_vel is recorded in the bag.")
        sys.exit(1)
    
    # Compute dead reckoning
    print("Computing dead-reckoning trajectory...")
    dr_data = compute_dead_reckoning(truth_data, cmd_data)
    
    # Create plot
    if use_separate:
        print("Creating separate comparison plots...")
        fig, dr_error, ekf_error = plot_dr_vs_ekf_separate(truth_data, ekf_data, dr_data)
        output_file = 'dr_vs_ekf_separate.png'
        output_pdf = 'dr_vs_ekf_separate.pdf'
    else:
        print("Creating combined comparison plot...")
        fig, dr_error, ekf_error = plot_dr_vs_ekf_error(truth_data, ekf_data, dr_data)
        output_file = 'dr_vs_ekf_error.png'
        output_pdf = 'dr_vs_ekf_error.pdf'
    
    # Compute statistics
    compute_statistics(dr_error, ekf_error)
    
    # Save figures
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Plot saved to: {output_file}")
    
    plt.savefig(output_pdf, bbox_inches='tight', facecolor='white')
    print(f"PDF saved to: {output_pdf}")
    
    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
