#!/usr/bin/env python3
"""
Validation script to verify that pose_factor messages were correctly computed
from raw IMU data during the mission.

This script:
1. Reads raw CV7 AHRS IMU data from the bag
2. Recomputes IMU preintegration using the same algorithm as imu_sensor_handler.py
3. Reads the recorded pose_factor messages
4. Compares the recomputed values to the recorded values

If they match closely, it confirms the pose_factor is legitimate IMU odometry.
If they differ significantly, something else might be going on.
"""

import rosbag
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from collections import defaultdict

class IMUPreintegrator:
    """Reimplements the preintegration algorithm from imu_sensor_handler.py"""
    
    def __init__(self, accel_noise=0.005, gyro_noise=0.005, gravity=9.81):
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.gravity = gravity
        
    def preintegrate(self, imu_messages, start_time, end_time):
        """
        Preintegrate IMU measurements between start_time and end_time.
        
        Returns:
            dict with 'position', 'orientation', 'velocity'
        """
        # Filter messages in time window
        msgs_in_window = [msg for msg in imu_messages 
                         if start_time <= msg.header.stamp.to_sec() <= end_time]
        
        if len(msgs_in_window) < 2:
            return None
            
        # Initialize integration variables
        delta_R = np.eye(3)  # Rotation matrix
        delta_v = np.zeros(3)  # Velocity
        delta_p = np.zeros(3)  # Position
        
        # Gravity vector in world frame (NED convention: gravity points down)
        g_world = np.array([0.0, 0.0, self.gravity])
        
        # Integrate over all consecutive IMU measurements
        for i in range(len(msgs_in_window) - 1):
            msg_i = msgs_in_window[i]
            msg_j = msgs_in_window[i + 1]
            
            dt = (msg_j.header.stamp - msg_i.header.stamp).to_sec()
            if dt <= 0 or dt > 1.0:  # Skip invalid time steps
                continue
                
            # Extract angular velocity (rad/s) and linear acceleration (m/s^2)
            w = np.array([
                msg_i.angular_velocity.x,
                msg_i.angular_velocity.y,
                msg_i.angular_velocity.z
            ])
            
            a = np.array([
                msg_i.linear_acceleration.x,
                msg_i.linear_acceleration.y,
                msg_i.linear_acceleration.z
            ])
            
            # Integrate rotation (using rotation vector)
            angle = np.linalg.norm(w) * dt
            if angle > 1e-8:
                axis = w / np.linalg.norm(w)
                rot = R.from_rotvec(axis * angle)
                dR = rot.as_dcm() if hasattr(rot, 'as_dcm') else rot.as_matrix()
            else:
                dR = np.eye(3)
            
            delta_R = delta_R @ dR
            
            # Remove gravity from acceleration (in body frame, gravity is rotated by current orientation)
            g_body = delta_R.T @ g_world
            a_corrected = a - g_body
            
            # Integrate velocity and position
            # Using midpoint integration for better accuracy
            delta_v += delta_R @ a_corrected * dt
            delta_p += delta_v * dt + 0.5 * delta_R @ a_corrected * (dt**2)
        
        # Convert rotation matrix to quaternion
        rot = R.from_matrix(delta_R) if hasattr(R, 'from_matrix') else R.from_dcm(delta_R)
        quat = rot.as_quat()  # [x, y, z, w]
        
        return {
            'position': delta_p,
            'orientation': quat,
            'velocity': delta_v,
            'num_measurements': len(msgs_in_window)
        }


def extract_pose_factor_data(bag_path):
    """Extract pose_factor messages from bag"""
    pose_factors = []
    
    print(f"Reading pose_factor messages from {bag_path}...")
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/actor_0/pose_factor']):
            pose_factors.append({
                'stamp': msg.header.stamp.to_sec(),
                'bag_time': t.to_sec(),  # Actual time in bag
                'key1': msg.key1,
                'key2': msg.key2,
                'position': np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]),
                'orientation': np.array([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]),
                'covariance': np.array(msg.pose.covariance).reshape((6, 6))
            })
    
    print(f"Found {len(pose_factors)} pose_factor messages")
    return pose_factors


def extract_imu_data(bag_path):
    """Extract raw IMU messages from bag"""
    imu_messages = []
    
    print(f"Reading raw IMU messages from {bag_path}...")
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/actor_0/cv7_ahrs']):
            imu_messages.append(msg)
    
    print(f"Found {len(imu_messages)} raw IMU messages")
    return imu_messages


def extract_integrated_state_data(bag_path):
    """Extract integrated_state messages to get actual pose timestamps"""
    integrated_states = []
    
    print(f"Reading integrated_state messages from {bag_path}...")
    with rosbag.Bag(bag_path, 'r') as bag:
        pose_index = 0
        for topic, msg, t in bag.read_messages(topics=['/actor_0/integrated_state']):
            # Integrated_state messages are published every 2 poses (A0, A2, A4, ...)
            # This is based on the observation that we have 289 integrated_states
            # but pose_factors go up to A576
            key = f"A{pose_index * 2}"
            integrated_states.append({
                'stamp': msg.header.stamp.to_sec(),
                'bag_time': t.to_sec(),
                'key': key,
                'position': np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]),
                'orientation': np.array([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ])
            })
            pose_index += 1
    
    print(f"Found {len(integrated_states)} integrated_state messages (keys: A0, A2, A4, ...)")
    return integrated_states


def find_imu_time_window(imu_messages, key1, key2, pose_factor_stamp, integrated_states):
    """
    Determine the time window for IMU preintegration using integrated_state timestamps.
    
    The actual system preintegrates between consecutive integrated_state messages.
    Pose factors are: A0->A1, A2->A3, A4->A5, etc. (always even->odd)
    Integrated states are published for: A0, A2, A4, A6, ... (even poses only)
    
    For pose_factor A0->A1: integrate from timestamp(A0) to timestamp(A2)
    For pose_factor A2->A3: integrate from timestamp(A2) to timestamp(A4)
    etc.
    """
    # Build a lookup table from key to timestamp
    key_to_time = {}
    for state in integrated_states:
        key_to_time[state['key']] = state['stamp']
    
    # key1 is always even (A0, A2, A4...), key2 is odd (A1, A3, A5...)
    # Extract the number from key1
    try:
        key1_num = int(key1[1:])  # Remove 'A' prefix
        next_key = f"A{key1_num + 2}"  # Next even pose
        
        if key1 in key_to_time and next_key in key_to_time:
            start_time = key_to_time[key1]
            end_time = key_to_time[next_key]
            return start_time, end_time
    except (ValueError, KeyError):
        pass
    
    # Fallback: use a heuristic based on pose_factor timestamp
    # Typical interval is ~3-4 seconds between poses
    window_size = 10.0
    start_time = pose_factor_stamp - window_size
    end_time = pose_factor_stamp
    return start_time, end_time


def compare_results(bag_path, num_samples=20):
    """
    Compare recomputed preintegration to recorded pose_factor messages.
    
    Args:
        bag_path: Path to the bag file
        num_samples: Number of pose_factor messages to validate
    """
    # Load data
    pose_factors = extract_pose_factor_data(bag_path)
    imu_messages = extract_imu_data(bag_path)
    integrated_states = extract_integrated_state_data(bag_path)
    
    if not pose_factors or not imu_messages:
        print("ERROR: Could not load data from bag")
        return
    
    # Sample evenly distributed pose_factors to validate
    indices = np.linspace(0, len(pose_factors)-1, min(num_samples, len(pose_factors)), dtype=int)
    
    preintegrator = IMUPreintegrator()
    
    results = {
        'position_errors': [],
        'orientation_errors': [],
        'recorded_positions': [],
        'computed_positions': [],
        'recorded_orientations': [],
        'computed_orientations': [],
        'time_windows': [],
        'num_imu_msgs': []
    }
    
    print(f"\nValidating {len(indices)} pose_factor messages...\n")
    print(f"{'Idx':<5} {'Keys':<10} {'Time Window (s)':<20} {'IMU':<6} {'Pos Err (m)':<12} {'Ori Err (°)':<12}")
    print("-" * 80)
    
    for idx in indices:
        pf = pose_factors[idx]
        
        # Determine time window for preintegration using integrated_state
        start_time, end_time = find_imu_time_window(imu_messages, pf['key1'], pf['key2'], 
                                                     pf['stamp'], integrated_states)
        
        # Recompute preintegration
        result = preintegrator.preintegrate(imu_messages, start_time, end_time)
        
        if result is None:
            print(f"{idx:<5} {pf['key1']:>3s}->{pf['key2']:<3s} {'N/A':<20} {'N/A':<6} {'SKIP':<12} {'SKIP':<12}")
            continue
        
        # Compare position
        pos_error = np.linalg.norm(result['position'] - pf['position'])
        
        # Compare orientation (using quaternion distance)
        q1 = R.from_quat(result['orientation'])
        q2 = R.from_quat(pf['orientation'])
        # Angular distance between rotations
        q_diff = q1 * q2.inv()
        angle = q_diff.magnitude() if hasattr(q_diff, 'magnitude') else np.linalg.norm(q_diff.as_rotvec())
        ori_error = np.degrees(angle)
        
        results['position_errors'].append(pos_error)
        results['orientation_errors'].append(ori_error)
        results['recorded_positions'].append(pf['position'])
        results['computed_positions'].append(result['position'])
        results['recorded_orientations'].append(pf['orientation'])
        results['computed_orientations'].append(result['orientation'])
        results['time_windows'].append(end_time - start_time)
        results['num_imu_msgs'].append(result['num_measurements'])
        
        time_window_str = f"{end_time - start_time:.2f}"
        print(f"{idx:<5} {pf['key1']:>3s}->{pf['key2']:<3s} {time_window_str:<20} "
              f"{result['num_measurements']:<6} {pos_error:<12.4f} {ori_error:<12.2f}")
    
    # Summary statistics
    print("\n" + "="*80)
    print("VALIDATION SUMMARY")
    print("="*80)
    
    if results['position_errors']:
        pos_errors = np.array(results['position_errors'])
        ori_errors = np.array(results['orientation_errors'])
        time_windows = np.array(results['time_windows'])
        num_imu = np.array(results['num_imu_msgs'])
        
        print(f"\nPosition Error Statistics:")
        print(f"  Mean:   {np.mean(pos_errors):.4f} m")
        print(f"  Median: {np.median(pos_errors):.4f} m")
        print(f"  Max:    {np.max(pos_errors):.4f} m")
        print(f"  Min:    {np.min(pos_errors):.4f} m")
        print(f"  Std:    {np.std(pos_errors):.4f} m")
        
        print(f"\nOrientation Error Statistics:")
        print(f"  Mean:   {np.mean(ori_errors):.2f}°")
        print(f"  Median: {np.median(ori_errors):.2f}°")
        print(f"  Max:    {np.max(ori_errors):.2f}°")
        print(f"  Min:    {np.min(ori_errors):.2f}°")
        print(f"  Std:    {np.std(ori_errors):.2f}°")
        
        print(f"\nTime Window Statistics:")
        print(f"  Mean:   {np.mean(time_windows):.2f} s")
        print(f"  Median: {np.median(time_windows):.2f} s")
        print(f"  Range:  {np.min(time_windows):.2f} - {np.max(time_windows):.2f} s")
        
        print(f"\nIMU Messages per Integration:")
        print(f"  Mean:   {np.mean(num_imu):.0f} msgs")
        print(f"  Median: {np.median(num_imu):.0f} msgs")
        print(f"  Range:  {np.min(num_imu):.0f} - {np.max(num_imu):.0f} msgs")
        
        # Interpretation
        print("\n" + "-"*80)
        print("INTERPRETATION:")
        print("-"*80)
        
        # Check for systematic offset (consistent error direction)
        recorded = np.array(results['recorded_positions'])
        computed = np.array(results['computed_positions'])
        mean_diff = np.mean(recorded - computed, axis=0)
        print(f"\nMean position difference (recorded - computed):")
        print(f"  X: {mean_diff[0]:8.3f} m")
        print(f"  Y: {mean_diff[1]:8.3f} m")
        print(f"  Z: {mean_diff[2]:8.3f} m")
        print(f"  Magnitude: {np.linalg.norm(mean_diff):.3f} m")
        
        if np.linalg.norm(mean_diff) > 100:
            print("\n⚠ LARGE SYSTEMATIC OFFSET detected!")
            print("  This suggests:")
            print("  - Different coordinate frame or reference point")
            print("  - Different gravity compensation")
            print("  - Or the preintegration uses additional corrections not in this script")
        
        if np.mean(pos_errors) < 0.1:
            print("\n✓ Position errors are VERY SMALL (<10cm average)")
            print("  → pose_factor is correctly computed from raw IMU")
        elif np.mean(pos_errors) < 1.0:
            print("\n✓ Position errors are SMALL (<1m average)")
            print("  → pose_factor likely computed from IMU, but algorithm may differ slightly")
        elif np.mean(pos_errors) < 100 and np.std(pos_errors) < 10:
            print("\n⚠ Position errors are MODERATE but CONSISTENT")
            print("  → Systematic offset, likely due to:")
            print("    - Different starting reference point")
            print("    - Coordinate frame transformation")
            print("    - Gravity model differences")
        else:
            print("\n✗ Position errors are LARGE and VARIABLE")
            print("  → pose_factor may NOT be directly from IMU preintegration")
            print("  → Could be from a different source or very different algorithm")
        
        if np.mean(ori_errors) < 5.0:
            print("\n✓ Orientation errors are SMALL (<5° average)")
            print("  → Rotations match well")
        elif np.mean(ori_errors) < 20.0:
            print("\n⚠ Orientation errors are MODERATE (5-20° average)")
            print("  → Some discrepancy in rotation computation")
        else:
            print("\n✗ Orientation errors are LARGE (>20° average)")
            print("  → Significant difference in rotation computation")
        
        print("\n" + "-"*80)
        print("CONCLUSION:")
        print("-"*80)
        
        # Overall verdict
        if np.mean(pos_errors) < 1.0 and np.mean(ori_errors) < 5.0:
            print("✓✓ The pose_factor data MATCHES the raw IMU preintegration!")
            print("   The data is legitimate and accurately computed.")
        elif np.std(pos_errors) / np.mean(pos_errors) < 0.1:  # Low coefficient of variation
            print("⚠⚠ Systematic offset present, but data is CONSISTENT")
            print("   The pose_factor is likely from IMU, but with:")
            print("   - Different reference frame")
            print("   - Different initialization")
            print("   - Additional corrections not captured in validation")
        else:
            print("✗✗ Significant discrepancies detected")
            print("   The pose_factor may not be pure IMU preintegration")
        
        # Plot results
        plot_comparison(results)
    else:
        print("ERROR: No valid comparisons could be made")


def plot_comparison(results):
    """Generate comparison plots"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    
    # Position error over samples
    ax = axes[0, 0]
    ax.plot(results['position_errors'], 'o-', label='Position Error')
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Position Error per Sample')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Orientation error over samples
    ax = axes[0, 1]
    ax.plot(results['orientation_errors'], 'o-', color='orange', label='Orientation Error')
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('Orientation Error (degrees)')
    ax.set_title('Orientation Error per Sample')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Position comparison (3D positions)
    ax = axes[0, 2]
    recorded = np.array(results['recorded_positions'])
    computed = np.array(results['computed_positions'])
    
    for i, label in enumerate(['X', 'Y', 'Z']):
        ax.plot(recorded[:, i], label=f'Recorded {label}', linestyle='-', marker='o', markersize=3)
        ax.plot(computed[:, i], label=f'Computed {label}', linestyle='--', marker='x', markersize=3)
    
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('Position (m)')
    ax.set_title('Recorded vs Computed Positions')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Error histogram
    ax = axes[1, 0]
    ax.hist(results['position_errors'], bins=20, alpha=0.7, label='Position (m)', edgecolor='black')
    ax.set_xlabel('Error Magnitude (m)')
    ax.set_ylabel('Count')
    ax.set_title('Position Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Systematic offset per axis
    ax = axes[1, 1]
    position_diffs = recorded - computed
    mean_diffs = np.mean(position_diffs, axis=0)
    std_diffs = np.std(position_diffs, axis=0)
    
    x_pos = np.arange(3)
    ax.bar(x_pos, mean_diffs, yerr=std_diffs, capsize=5, alpha=0.7, edgecolor='black')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(['X', 'Y', 'Z'])
    ax.set_ylabel('Mean Difference (m)')
    ax.set_title('Systematic Offset (Recorded - Computed)')
    ax.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Trajectory comparison (XY plane)
    ax = axes[1, 2]
    ax.plot(recorded[:, 0], recorded[:, 1], 'o-', label='Recorded', markersize=4)
    ax.plot(computed[:, 0], computed[:, 1], 'x--', label='Computed', markersize=4)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Trajectory Comparison (XY Plane)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    
    output_path = '/home/lizgajski2/catkin_ws/tests/results/pose_factor_validation.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Validation plot saved to: {output_path}")
    
    # Also show first few samples in detail
    print("\n" + "="*80)
    print("DETAILED SAMPLE COMPARISON (First 5)")
    print("="*80)
    for i in range(min(5, len(results['recorded_positions']))):
        rec_pos = results['recorded_positions'][i]
        com_pos = results['computed_positions'][i]
        diff = rec_pos - com_pos
        print(f"\nSample {i}:")
        print(f"  Recorded: [{rec_pos[0]:8.3f}, {rec_pos[1]:8.3f}, {rec_pos[2]:8.3f}]")
        print(f"  Computed: [{com_pos[0]:8.3f}, {com_pos[1]:8.3f}, {com_pos[2]:8.3f}]")
        print(f"  Diff:     [{diff[0]:8.3f}, {diff[1]:8.3f}, {diff[2]:8.3f}]  (mag: {np.linalg.norm(diff):.3f} m)")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_path = '/home/lizgajski2/catkin_ws/tests/pink_solo.bag'
    
    print("="*80)
    print("POSE_FACTOR VALIDATION SCRIPT")
    print("="*80)
    print(f"\nBag file: {bag_path}")
    print("\nThis script will:")
    print("1. Read raw IMU data from the bag")
    print("2. Recompute IMU preintegration")
    print("3. Compare to recorded pose_factor messages")
    print("4. Report discrepancies\n")
    
    compare_results(bag_path, num_samples=30)
    
    print("\n" + "="*80)
    print("VALIDATION COMPLETE")
    print("="*80)
