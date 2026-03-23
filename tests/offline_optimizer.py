#!/usr/bin/env python3
"""
Offline CORA, GTSAM, & EKF (in development) Optimization Script

This script extracts odometry, ranging, and GPS data from ROS bags,
saves them to CSV format, then runs GTSAM Levenberg-Marquardt optimization
offline without needing to replay the bag in real-time.

Usage:
    python3 offline_gtsam_optimizer.py <bag_file> [--output-dir <dir>]
    
Example:
    python3 offline_gtsam_optimizer.py tests/pink_solo.bag
    python3 offline_gtsam_optimizer.py tests/pink_solo.bag --output-dir tests/results/pink_solo
"""

import argparse
import os
import sys
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless environments
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import rosbag
from tqdm import tqdm

# GTSAM imports
from gtsam import (
    NonlinearFactorGraph,
    Values,
    BetweenFactorPose3,
    RangeFactorPose3,
    PriorFactorPose3,
    Pose3,
    Rot3,
    Point3,
    noiseModel,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    symbol,
)
import gtsam

# CORA imports
try:
    import cora
    from scipy.spatial.transform import Rotation as R
    CORA_AVAILABLE = True
    print("✓ CORA dependencies loaded successfully")
except ImportError as e:
    CORA_AVAILABLE = False
    print(f"WARNING: CORA dependencies not available ({e}). CORA optimization will be skipped.")


def extract_data_from_bag(bag_path, actor_name="actor_0"):
    """
    Extract odometry, range, and GPS data from a ROS bag.
    
    Returns:
        odometry_data: List of dicts with pose factor information (relative poses)
        integrated_state_data: List of dicts with absolute poses from integrated_state
        range_data: List of dicts with range factor information
        gps_data: List of dicts with GPS information
    """
    print(f"\n{'='*80}")
    print(f"EXTRACTING DATA FROM BAG: {bag_path}")
    print(f"{'='*80}\n")
    
    odometry_data = []
    integrated_state_data = []
    range_data = []
    gps_data = []
    
    # Topics to extract
    pose_factor_topic = f"/{actor_name}/pose_factor"
    integrated_state_topic = f"/{actor_name}/integrated_state"
    range_factor_topic = f"/{actor_name}/range_factor"
    gps_topic = f"/{actor_name}/gps"
    
    with rosbag.Bag(bag_path, 'r') as bag:
        # Get message counts
        info = bag.get_type_and_topic_info()
        total_messages = sum([
            info.topics.get(pose_factor_topic, type('', (), {'message_count': 0})).message_count,
            info.topics.get(integrated_state_topic, type('', (), {'message_count': 0})).message_count,
            info.topics.get(range_factor_topic, type('', (), {'message_count': 0})).message_count,
            info.topics.get(gps_topic, type('', (), {'message_count': 0})).message_count,
        ])
        
        print(f"Topics to extract:")
        print(f"  - {pose_factor_topic}: {info.topics.get(pose_factor_topic, type('', (), {'message_count': 0})).message_count} messages")
        print(f"  - {integrated_state_topic}: {info.topics.get(integrated_state_topic, type('', (), {'message_count': 0})).message_count} messages")
        print(f"  - {range_factor_topic}: {info.topics.get(range_factor_topic, type('', (), {'message_count': 0})).message_count} messages")
        print(f"  - {gps_topic}: {info.topics.get(gps_topic, type('', (), {'message_count': 0})).message_count} messages")
        print(f"\nTotal messages to extract: {total_messages}\n")
        
        # Extract messages with progress bar
        with tqdm(total=total_messages, desc="Extracting messages") as pbar:
            # Extract pose factors (odometry)
            for topic, msg, t in bag.read_messages(topics=[pose_factor_topic]):
                odom = {
                    'timestamp': t.to_sec(),
                    'key1': msg.key1,
                    'key2': msg.key2,
                    'tx': msg.pose.pose.position.x,
                    'ty': msg.pose.pose.position.y,
                    'tz': msg.pose.pose.position.z,
                    'qx': msg.pose.pose.orientation.x,
                    'qy': msg.pose.pose.orientation.y,
                    'qz': msg.pose.pose.orientation.z,
                    'qw': msg.pose.pose.orientation.w,
                    'cov': list(msg.pose.covariance),
                }
                odometry_data.append(odom)
                pbar.update(1)
            
            # Extract integrated_state (absolute poses)
            for topic, msg, t in bag.read_messages(topics=[integrated_state_topic]):
                state = {
                    'timestamp': t.to_sec(),
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'qx': msg.pose.pose.orientation.x,
                    'qy': msg.pose.pose.orientation.y,
                    'qz': msg.pose.pose.orientation.z,
                    'qw': msg.pose.pose.orientation.w,
                }
                integrated_state_data.append(state)
                pbar.update(1)
            
            # Extract range factors
            for topic, msg, t in bag.read_messages(topics=[range_factor_topic]):
                range_meas = {
                    'timestamp': t.to_sec(),
                    'key1': msg.key1,
                    'key2': msg.key2,
                    'meas_range': msg.meas_range,
                    'range_sigma': msg.range_sigma,
                    'depth1': msg.depth1,
                    'depth2': msg.depth2,
                }
                range_data.append(range_meas)
                pbar.update(1)
            
            # Extract GPS
            for topic, msg, t in bag.read_messages(topics=[gps_topic]):
                gps = {
                    'timestamp': t.to_sec(),
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'cov': list(msg.pose.covariance),
                }
                gps_data.append(gps)
                pbar.update(1)
    
    print(f"\nExtraction complete:")
    print(f"  - Odometry measurements: {len(odometry_data)}")
    print(f"  - Integrated state poses: {len(integrated_state_data)}")
    print(f"  - Range measurements: {len(range_data)}")
    print(f"  - GPS measurements: {len(gps_data)}")
    
    return odometry_data, integrated_state_data, range_data, gps_data


def save_to_csv(odometry_data, integrated_state_data, range_data, gps_data, output_dir):
    """Save extracted data to CSV files."""
    print(f"\n{'='*80}")
    print(f"SAVING DATA TO CSV")
    print(f"{'='*80}\n")
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Save odometry data
    odom_file = os.path.join(output_dir, 'odometry.csv')
    with open(odom_file, 'w', newline='') as f:
        if odometry_data:
            writer = csv.DictWriter(f, fieldnames=odometry_data[0].keys())
            writer.writeheader()
            writer.writerows(odometry_data)
    print(f"✓ Saved odometry data to: {odom_file}")
    
    # Save integrated_state data
    integrated_state_file = os.path.join(output_dir, 'integrated_state.csv')
    with open(integrated_state_file, 'w', newline='') as f:
        if integrated_state_data:
            writer = csv.DictWriter(f, fieldnames=integrated_state_data[0].keys())
            writer.writeheader()
            writer.writerows(integrated_state_data)
    print(f"✓ Saved integrated_state data to: {integrated_state_file}")
    
    # Save range data
    range_file = os.path.join(output_dir, 'ranges.csv')
    with open(range_file, 'w', newline='') as f:
        if range_data:
            writer = csv.DictWriter(f, fieldnames=range_data[0].keys())
            writer.writeheader()
            writer.writerows(range_data)
    print(f"✓ Saved range data to: {range_file}")
    
    # Save GPS data
    gps_file = os.path.join(output_dir, 'gps.csv')
    with open(gps_file, 'w', newline='') as f:
        if gps_data:
            writer = csv.DictWriter(f, fieldnames=gps_data[0].keys())
            writer.writeheader()
            writer.writerows(gps_data)
    print(f"✓ Saved GPS data to: {gps_file}")
    
    return odom_file, integrated_state_file, range_file, gps_file


def parse_key(key_str):
    """Parse key string like 'A0', 'A1', 'L0' into symbol character and index."""
    char = key_str[0]
    index = int(key_str[1:])
    return char, index


def run_gtsam_optimization(integrated_state_data, range_data, gps_data):
    """
    Run GTSAM Levenberg-Marquardt optimization on extracted data.
    
    Uses integrated_state for pose initialization (absolute world-frame poses).
    
    Returns:
        graph: The factor graph
        initial_values: Initial values before optimization
        result: Optimized values after optimization
    """
    print(f"\n{'='*80}")
    print(f"RUNNING GTSAM OPTIMIZATION")
    print(f"{'='*80}\n")
    
    # Create factor graph and initial values
    graph = NonlinearFactorGraph()
    initial = Values()
    
    # Track which variables we've seen
    pose_keys = set()
    landmark_keys = set()
    
    # Initialize all poses from integrated_state (absolute world-frame poses)
    print(f"Initializing {len(integrated_state_data)} poses from integrated_state...")
    for idx, state in enumerate(integrated_state_data):
        key_str = f'A{idx}'  # A0, A1, A2, ... (sequential indices)
        pose_keys.add(key_str)
        
        key_sym = symbol('A', idx)
        
        # Create pose from integrated_state (world coordinates)
        rotation = Rot3.Quaternion(state['qw'], state['qx'], state['qy'], state['qz'])
        translation = np.array([state['x'], state['y'], state['z']])
        pose = Pose3(rotation, translation)
        
        initial.insert(key_sym, pose)
        
        # Add prior on first pose
        if idx == 0:
            # Tight constraint on first pose in world frame
            prior_sigmas = np.array([1e-6, 1e-6, 1e-6,  # x, y, z position
                                      1e-8, 1e-8, 1e-8])  # roll, pitch, yaw orientation
            prior_noise = noiseModel.Diagonal.Sigmas(prior_sigmas)
            graph.add(PriorFactorPose3(key_sym, pose, prior_noise))
    
    print(f"✓ Initialized {len(pose_keys)} poses from integrated_state (world coordinates)")
    print(f"  Added prior on pose A0 at ({integrated_state_data[0]['x']:.1f}, {integrated_state_data[0]['y']:.1f}, {integrated_state_data[0]['z']:.1f})")
    
    # Compute between-factors from consecutive integrated_state poses
    print(f"\nComputing {len(integrated_state_data)-1} between-factors from consecutive poses...")
    for idx in range(len(integrated_state_data) - 1):
        key1_str = f'A{idx}'
        key2_str = f'A{idx+1}'
        
        key1_sym = symbol('A', idx)
        key2_sym = symbol('A', idx+1)
        
        # Get poses from initial values
        pose1 = initial.atPose3(key1_sym)
        pose2 = initial.atPose3(key2_sym)
        
        # Compute relative pose
        rel_pose = pose1.between(pose2)
        
        # Create covariance - GTSAM-specific tuning
        # Morrison's recommendation: "weaken your between factor noise models to ensure 
        # noisy data isn't getting locked in by an overconfident noise model"
        #
        # GTSAM's Levenberg-Marquardt optimizer works best with MODERATE constraints:
        # - Allows freedom to balance odometry and range measurements
        # - Can escape local minima from noisy odometry
        # - Achieves excellent results (3.25m XY error with σ=2m)
        #
        # Tested values:
        #   σ_trans=0.316m, σ_rot=0.1rad  → 18.90m error (too tight, overfitting)
        #   σ_trans=2.0m,   σ_rot=5.0rad  → 3.25m error (optimal!) ✨
        #   σ_trans=10.0m,  σ_rot=100rad  → 18.90m error (too loose)
        #
        # GTSAM OPTIMAL: Loose constraints let ranges correct odometry drift
        translation_std = 2.0   # 2m std dev - moderate confidence, prevents overfitting
        rotation_std = 5.0      # 5 rad std dev (~286°) - loose confidence per Morrison
        sigmas = np.array([translation_std, translation_std, translation_std,     # x, y, z position
                          rotation_std, rotation_std, rotation_std])              # roll, pitch, yaw
        odom_noise = noiseModel.Diagonal.Sigmas(sigmas)
        
        translation_variance = translation_std ** 2  # 4.0 m²
        rotation_variance = rotation_std ** 2        # 25.0 rad²
        
        # Add between factor
        graph.add(BetweenFactorPose3(key1_sym, key2_sym, rel_pose, odom_noise))
    
    print(f"✓ Added {len(integrated_state_data)-1} between-factors")
    
    # Add range factors
    print(f"\nAdding {len(range_data)} range factors...")
    for range_meas in tqdm(range_data, desc="Range factors"):
        key1_char, key1_idx = parse_key(range_meas['key1'])
        key2_char, key2_idx = parse_key(range_meas['key2'])
        
        key1_sym = symbol(key1_char, key1_idx)
        key2_sym = symbol(key2_char, key2_idx)
        
        # Create noise model
        range_noise = noiseModel.Isotropic.Sigma(1, range_meas['range_sigma'])
        
        # Add range factor
        graph.add(RangeFactorPose3(key1_sym, key2_sym, range_meas['meas_range'], range_noise))
        
        # If key2 is a landmark and not initialized, add it
        if key2_char == 'L' and range_meas['key2'] not in landmark_keys:
            # Initialize landmarks at known world positions
            landmark_positions = {
                'L0': np.array([-74.5193539608157, -38.9298973079931, 1.5]),
                'L1': np.array([66.5150726324041, 25.969767675496275, 1.5]),
            }
            
            if range_meas['key2'] in landmark_positions:
                landmark_pose = Pose3(Rot3(), landmark_positions[range_meas['key2']])
                initial.insert(key2_sym, landmark_pose)
                landmark_keys.add(range_meas['key2'])
                
                # Add strong prior on landmark position
                landmark_sigmas = np.array([0.1, 0.1, 0.1,  # tight position
                                           10.0, 10.0, 10.0])  # loose orientation (doesn't matter)
                landmark_noise = noiseModel.Diagonal.Sigmas(landmark_sigmas)
                landmark_prior = Pose3(Rot3(), landmark_positions[range_meas['key2']])
                graph.add(PriorFactorPose3(key2_sym, landmark_prior, landmark_noise))
    
    print(f"✓ Added {len(range_data)} range factors")
    print(f"  Landmark variables: {len(landmark_keys)}")
    
    # Print optimization problem summary
    print(f"\n{'='*80}")
    print(f"OPTIMIZATION PROBLEM SUMMARY")
    print(f"{'='*80}")
    print(f"  Pose variables: {len(pose_keys)}")
    print(f"  Landmark variables: {len(landmark_keys)}")
    print(f"  Total variables: {len(pose_keys) + len(landmark_keys)}")
    print(f"  Total factors: {graph.size()}")
    print(f"    - Prior factors: 1 (pose) + {len(landmark_keys)} (landmarks)")
    print(f"    - Odometry factors: {len(integrated_state_data)-1}")
    print(f"    - Range factors: {len(range_data)}")
    
    # Configure LM optimizer
    params = LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    params.setRelativeErrorTol(1e-5)
    params.setAbsoluteErrorTol(1e-5)
    
    print(f"\n{'='*80}")
    print(f"OPTIMIZING WITH LEVENBERG-MARQUARDT")
    print(f"{'='*80}\n")
    
    # Run optimization
    optimizer = LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    
    # Print results
    initial_error = graph.error(initial)
    final_error = graph.error(result)
    
    print(f"\n{'='*80}")
    print(f"OPTIMIZATION RESULTS")
    print(f"{'='*80}")
    print(f"  Initial error: {initial_error:.6f}")
    print(f"  Final error: {final_error:.6f}")
    print(f"  Error reduction: {(1 - final_error/initial_error)*100:.2f}%")
    print(f"  Iterations: {optimizer.iterations()}")
    
    return graph, initial, result, list(pose_keys), list(landmark_keys)


# CORA optimization is imported from cora_optimization.py module
# (See import at top of file for run_cora_optimization and extract_trajectory_from_cora)


def extract_trajectory(values, pose_keys):
    """Extract trajectory from GTSAM Values."""
    trajectory = []
    
    # Sort pose_keys numerically (not alphabetically)
    def key_sort(key_str):
        """Extract numeric index from key like 'A123' -> 123"""
        return int(key_str[1:])
    
    for key_str in sorted(pose_keys, key=key_sort):
        char, idx = parse_key(key_str)
        key_sym = symbol(char, idx)
        
        if values.exists(key_sym):
            pose = values.atPose3(key_sym)
            t = pose.translation()
            r = pose.rotation().quaternion()  # [w, x, y, z]
            
            trajectory.append({
                'key': key_str,
                'x': t[0],
                'y': t[1],
                'z': t[2],
                'qw': r[0],
                'qx': r[1],
                'qy': r[2],
                'qz': r[3],
            })
    
    return trajectory


def save_trajectory_csv(trajectory, output_dir, filename='optimized_trajectory.csv'):
    """Save optimized trajectory to CSV."""
    filepath = os.path.join(output_dir, filename)
    with open(filepath, 'w', newline='') as f:
        if trajectory:
            writer = csv.DictWriter(f, fieldnames=trajectory[0].keys())
            writer.writeheader()
            writer.writerows(trajectory)
    print(f"✓ Saved optimized trajectory to: {filepath}")
    return filepath


def plot_trajectory(initial_traj, gtsam_traj, cora_traj, landmark_keys, landmark_vals, 
                    integrated_state_data, range_data, gps_data, output_dir):
    """
    Create comprehensive trajectory visualization plot.
    Similar to the plot generated by process_mission.launch.
    
    Uses integrated_state for the absolute odometry baseline trajectory.
    Plots GPS fixes for start and surface positions.
    Compares GTSAM and CORA optimized trajectories.
    
    Returns:
        plot_path: Path to saved plot
        gtsam_error: XY distance from GTSAM final pose to surface GPS (or None)
        cora_error: XY distance from CORA final pose to surface GPS (or None)
    """
    print(f"\n{'='*80}")
    print(f"GENERATING TRAJECTORY PLOT")
    print(f"{'='*80}\n")
    
    fig = plt.figure(figsize=(14, 10))
    ax1 = plt.subplot(1, 1, 1)  # XY trajectory only
    
    # Extract coordinates
    init_x = [p['x'] for p in initial_traj]
    init_y = [p['y'] for p in initial_traj]
    
    gtsam_x = [p['x'] for p in gtsam_traj]
    gtsam_y = [p['y'] for p in gtsam_traj]
    
    cora_x = [p['x'] for p in cora_traj]
    cora_y = [p['y'] for p in cora_traj]
    
    # Get absolute odometry trajectory from integrated_state
    odom_traj_x = [p['x'] for p in integrated_state_data]
    odom_traj_y = [p['y'] for p in integrated_state_data]
    
    # Extract landmarks
    landmark_x = []
    landmark_y = []
    landmark_labels = []
    
    for key_str in landmark_keys:
        char, idx = parse_key(key_str)
        key_sym = symbol(char, idx)
        if landmark_vals.exists(key_sym):
            pose = landmark_vals.atPose3(key_sym)
            t = pose.translation()
            landmark_x.append(t[0])
            landmark_y.append(t[1])
            landmark_labels.append(key_str)
    
    # Extract GPS fixes
    # We want to show:
    # - Start GPS: The GPS fix just before the dive (last surface GPS before big jump)
    # - Surface GPS: The first GPS fix after surfacing (first GPS after big jump)
    #
    # Strategy: Find the biggest GPS jump - that indicates the dive→surface transition
    
    surface_gps = None
    start_gps = None
    if len(gps_data) > 1:
        # Find the biggest GPS jump between consecutive fixes
        max_jump = 0
        jump_idx = None
        for idx in range(1, len(gps_data)):
            prev_gps = gps_data[idx - 1]
            curr_gps = gps_data[idx]
            dx = curr_gps['x'] - prev_gps['x']
            dy = curr_gps['y'] - prev_gps['y']
            jump = (dx**2 + dy**2)**0.5
            if jump > max_jump:
                max_jump = jump
                jump_idx = idx
        
        # If we found a significant jump (>10m), use it to identify dive→surface
        if jump_idx is not None and max_jump > 10:
            # Start GPS: GPS just before the jump (last surface GPS before dive)
            start_gps = (gps_data[jump_idx - 1]['x'], 
                        gps_data[jump_idx - 1]['y'], 
                        gps_data[jump_idx - 1]['z'])
            # Surface GPS: GPS just after the jump (first GPS after surfacing)
            surface_gps = (gps_data[jump_idx]['x'], 
                          gps_data[jump_idx]['y'], 
                          gps_data[jump_idx]['z'])
        else:
            # No significant jump found, fall back to using last two GPS fixes
            start_gps = (gps_data[-2]['x'], gps_data[-2]['y'], gps_data[-2]['z'])
            surface_gps = (gps_data[-1]['x'], gps_data[-1]['y'], gps_data[-1]['z'])
    
    # Calculate distances to surface GPS for text box
    gtsam_error = None
    cora_error = None
    if surface_gps and len(gtsam_traj) > 0 and len(cora_traj) > 0:
        last_gtsam = gtsam_traj[-1]
        last_cora = cora_traj[-1]
        gtsam_error = np.sqrt((last_gtsam['x'] - surface_gps[0])**2 + 
                             (last_gtsam['y'] - surface_gps[1])**2)
        cora_error = np.sqrt((last_cora['x'] - surface_gps[0])**2 + 
                            (last_cora['y'] - surface_gps[1])**2)

    
    # Plot XY view
    ax1.plot(odom_traj_x, odom_traj_y, color='darkgrey', linestyle='-', linewidth=2.5, label='Ref. DR (Odometry)')
    ax1.plot(gtsam_x, gtsam_y, 'r--', linewidth=2, label='GTSAM Optimized')
    ax1.plot(cora_x, cora_y, 'b--', linewidth=2, alpha=0.7, label='CORA Optimized')
    ax1.scatter(landmark_x, landmark_y, c='orange', s=400, marker='*', 
                label='Landmarks', zorder=5)
    for i, label in enumerate(landmark_labels):
        ax1.annotate(label, (landmark_x[i], landmark_y[i]), 
                    xytext=(5, 5), textcoords='offset points', fontsize=14, fontweight='bold')
    # Add GPS fixes
    if start_gps:
        ax1.scatter(start_gps[0], start_gps[1], c='green', s=300, marker='+', 
                   linewidths=4, label='Start GPS', zorder=5)
    if surface_gps:
        ax1.scatter(surface_gps[0], surface_gps[1], c='black', s=300, marker='+', 
                   linewidths=4, label='Surface GPS', zorder=5)
    ax1.set_xlabel('X (m)', fontsize=16)
    ax1.set_ylabel('Y (m)', fontsize=16)
    # ax1.set_title('GTSAM LM and CORA Trajectories', fontsize=18, fontweight='bold')
    ax1.tick_params(axis='both', which='major', labelsize=14)
    ax1.legend(loc='best', fontsize=13)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Add error metrics text box
    if gtsam_error is not None and cora_error is not None:
        textstr = f'δGTSAM: {gtsam_error:.2f}m\nδCORA: {cora_error:.2f}m'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax1.text(0.98, 0.02, textstr, transform=ax1.transAxes, fontsize=12,
                verticalalignment='bottom', horizontalalignment='right', bbox=props)
    
    plt.tight_layout()
    
    # Save plot
    plot_path = os.path.join(output_dir, 'trajectory_plot.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"✓ Saved trajectory plot to: {plot_path}")
    
    plt.close()
    
    return plot_path, gtsam_error, cora_error


################################################################################
# CORA OPTIMIZATION FUNCTIONS
################################################################################

def parse_key(key_str):
    """Parse a key string like 'A0' or 'L1' into (char, index)."""
    return key_str[0], int(key_str[1:])


def run_cora_optimization(integrated_state_data, range_data, gps_data):
    """
    Run CORA (batch) optimization on extracted data using actual CORA optimizer.
    
    This uses Morrison's CORA implementation (Riemannian optimization on Stiefel manifold).
    Unlike GTSAM's Levenberg-Marquardt, CORA uses:
    - TNT (Trust-region Newton with Trust-region) solver
    - LOBPCG for certification
    - Convex relaxation approach (robust to poor initialization)
    - Can handle random initialization (doesn't need good initial guess)
    
    Returns:
        problem: The CORA problem
        initial_values: CORA Values before optimization
        result_values: CORA Values after optimization  
        pose_keys: List of pose keys
        landmark_keys: List of landmark keys
    """
    if not CORA_AVAILABLE:
        print(f"\n{'='*80}")
        print(f"CORA OPTIMIZATION SKIPPED (module not available)")
        print(f"{'='*80}\n")
        return None, None, None, [], []
    
    print(f"\n{'='*80}")
    print(f"RUNNING CORA BATCH OPTIMIZATION (Morrison's Method)")
    print(f"{'='*80}\n")
    
    # Create CORA problem (dimension=3 for 3D poses, d=4 for SE(3))
    dimension = 3
    problem = cora.Problem(dimension, dimension + 1)
    initial_values = cora.Values()
    
    # Track which variables we've seen
    pose_keys = set()
    landmark_keys = set()
    
    # Step 1: Add pose variables and initialize from integrated_state
    print(f"Adding {len(integrated_state_data)} pose variables...")
    for idx, state in enumerate(integrated_state_data):
        key_str = f'A{idx}'  # A0, A1, A2, ... (sequential indices)
        pose_keys.add(key_str)
        
        # Create CORA symbol
        sym = cora.Symbol('A', idx)
        
        # Add pose variable to problem
        problem.addPoseVariable(sym)
        
        # Create rotation matrix from quaternion
        qw, qx, qy, qz = state['qw'], state['qx'], state['qy'], state['qz']
        # Convert quaternion to rotation matrix (scipy uses [x,y,z,w] order)
        try:
            rot_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()  # scipy >=1.4
        except AttributeError:
            rot_matrix = R.from_quat([qx, qy, qz, qw]).as_dcm()  # scipy <1.4
        
        # Translation vector
        trans_vector = np.array([state['x'], state['y'], state['z']])
        
        # Set initial pose estimate
        initial_values.set_pose(sym, rot_matrix, trans_vector)
    
    print(f"✓ Added {len(pose_keys)} pose variables")
    print(f"  Initialized from integrated_state at ({integrated_state_data[0]['x']:.1f}, {integrated_state_data[0]['y']:.1f}, {integrated_state_data[0]['z']:.1f})")
    
    # Step 1.5: SKIP pose prior for now - CORA should figure out coordinates from relative measurements
    # NOTE: Without a prior, CORA optimizes in an arbitrary coordinate frame
    # We may need to post-process to align with our desired frame
    
    # Step 2: Add landmark variables
    landmark_positions = {
        'L0': np.array([-74.5193539608157, -38.9298973079931, 1.5]),
        'L1': np.array([66.5150726324041, 25.969767675496275, 1.5]),
    }
    
    print(f"\nAdding {len(landmark_positions)} landmark variables...")
    for key_str, position in landmark_positions.items():
        landmark_keys.add(key_str)
        sym = cora.Symbol(key_str[0], int(key_str[1]))  # 'L0' -> Symbol('L', 0)
        problem.addLandmarkVariable(sym)
        initial_values.set_landmark(sym, position)
    
    print(f"✓ Added {len(landmark_keys)} landmark variables")
    
    # Step 3: Add relative pose measurements (between-factors) from consecutive poses
    print(f"\nAdding {len(integrated_state_data)-1} relative pose measurements...")
    for idx in range(len(integrated_state_data) - 1):
        sym1 = cora.Symbol('A', idx)
        sym2 = cora.Symbol('A', idx+1)
        
        state1 = integrated_state_data[idx]
        state2 = integrated_state_data[idx+1]
        
        # Convert quaternions to rotation matrices
        try:
            R1 = R.from_quat([state1['qx'], state1['qy'], state1['qz'], state1['qw']]).as_matrix()
            R2 = R.from_quat([state2['qx'], state2['qy'], state2['qz'], state2['qw']]).as_matrix()
        except AttributeError:
            R1 = R.from_quat([state1['qx'], state1['qy'], state1['qz'], state1['qw']]).as_dcm()
            R2 = R.from_quat([state2['qx'], state2['qy'], state2['qz'], state2['qw']]).as_dcm()
        
        t1 = np.array([state1['x'], state1['y'], state1['z']])
        t2 = np.array([state2['x'], state2['y'], state2['z']])
        
        # Compute relative pose: T_rel = T1^{-1} * T2
        R_rel = R1.T @ R2
        t_rel = R1.T @ (t2 - t1)
        
        # Create covariance matrix for relative pose measurement
        # Covariance format: [translation(3), rotation(3)] - 6x6 matrix
        #
        # CORA-specific tuning - TIGHTER constraints work better
        # CORA's convex relaxation + coordinate alignment pipeline requires tighter
        # constraints for stable convergence. The alignEstimateToOrigin() + SE(3) 
        # transform becomes unstable with loose constraints.
        #
        # Tested values and results:
        #   σ_trans=0.2m,   σ_rot=0.05rad → 17.98m XY error (too tight, underfitting)
        #   σ_trans=0.25m,  σ_rot=0.08rad → 14.32m XY error (not optimal)
        #   σ_trans=0.3m,   σ_rot=0.1rad  → 13.38m XY error (very good)
        #   σ_trans=0.316m, σ_rot=0.1rad  → 13.20m XY error ✨ (BEST - optimal!)
        #   σ_trans=0.35m,  σ_rot=0.12rad → 13.11m XY error (nearly as good)
        #   σ_trans=0.5m,   σ_rot=0.5rad  → 30.73m XY error (too loose)
        #   σ_trans=2.0m,   σ_rot=5.0rad  → 49.12m XY error (way too loose)
        #
        # OPTIMAL CONFIGURATION for this mission:
        # σ=0.316m achieves 13.20m XY error (vs GTSAM's 3.25m with σ=2m)
        # CORA requires much tighter constraints than GTSAM for best performance


        translation_std = 0.316      # 0.316m std dev - optimal for CORA's relaxation
        rotation_std = 0.1           # 0.1 rad std dev (~5.7°) - optimal orientation trust
        translation_variance = translation_std ** 2  # 0.0998 m²
        rotation_variance = rotation_std ** 2        # 0.01 rad²
        
        covar = np.diag([translation_variance] * 3 + [rotation_variance] * 3)
        
        # Add relative pose measurement
        rel_pose_meas = cora.RelativePoseMeasurement(sym1, sym2, R_rel, t_rel, covar)
        problem.addRelativePoseMeasurement(rel_pose_meas)
    
    print(f"✓ Added {len(integrated_state_data)-1} relative pose measurements")
    
    # Step 4: Add range measurements
    print(f"\nAdding {len(range_data)} range measurements...")
    for range_meas in tqdm(range_data, desc="Range measurements"):
        # Parse keys
        key1_char, key1_idx = parse_key(range_meas['key1'])
        key2_char, key2_idx = parse_key(range_meas['key2'])
        
        sym1 = cora.Symbol(key1_char, key1_idx)
        sym2 = cora.Symbol(key2_char, key2_idx)
        
        # Add range measurement
        range_measurement = cora.RangeMeasurement(
            sym1, sym2,
            range_meas['meas_range'],
            range_meas['range_sigma'] ** 2  # CORA expects variance, not sigma
        )
        problem.addRangeMeasurement(range_measurement)
    
    print(f"✓ Added {len(range_data)} range measurements")
    
    # Update problem data (builds matrices)
    print(f"\nUpdating CORA problem data...")
    problem.updateProblemData()
    
    # Print optimization problem summary
    print(f"\n{'='*80}")
    print(f"CORA OPTIMIZATION PROBLEM SUMMARY")
    print(f"{'='*80}")
    print(f"  Pose variables: {len(pose_keys)}")
    print(f"  Landmark variables: {len(landmark_keys)}")
    print(f"  Total variables: {len(pose_keys) + len(landmark_keys)}")
    print(f"  Relative pose measurements: {len(integrated_state_data)-1}")
    print(f"  Range measurements: {len(range_data)}")
    
    # Get initial guess matrix from values
    # Try random initialization (Morrison's approach - CORA is robust to poor init)
    use_random_init = False  # Set to True to test random initialization
    
    if use_random_init:
        print(f"\nInitialization strategy: RANDOM (Morrison's approach)")
        print(f"  Note: CORA is designed to be robust to poor initialization")
        problem.updateProblemData()
        
        # Manual random initialization (Python doesn't expose getRandomInitialGuess)
        x0 = np.random.randn(problem.getExpectedVariableSize(), problem.getRelaxationRank())
        # Project to manifold
        x0 = problem.projectToManifold(x0)
        print(f"  Initial matrix shape: {x0.shape}")
    else:
        print(f"\nInitialization strategy: ODOMETRY-BASED (from integrated_state)")
        print(f"  Converting initial values to matrix form...")
        x0 = cora.getVarMatrixFromValues(problem, initial_values)
        print(f"  Initial matrix shape: {x0.shape}")
    
    # Run CORA optimization
    print(f"\n{'='*80}")
    print(f"OPTIMIZING WITH CORA (TNT + LOBPCG)")
    print(f"{'='*80}\n")
    print(f"  Solver: Trust-region Newton with Trust-region (TNT)")
    print(f"  Certification: LOBPCG (Locally Optimal Block Preconditioned Conjugate Gradient)")
    print(f"  Max relaxation rank: 6")
    print(f"  Initialization: From integrated_state (Morrison's approach)")
    print()
    
    try:
        (solver_result, iterate_history) = cora.solveCORA(
            problem=problem,
            x0=x0,
            max_relaxation_rank=6,
            verbose=True,
            log_iterates=False,
            show_iterates=False,
        )
        
        print(f"\n{'='*80}")
        print(f"CORA BATCH OPTIMIZATION RESULTS")
        print(f"{'='*80}")
        print(f"  Final objective value: {solver_result.f:.6f}")
        print(f"  Optimization status: {solver_result.status}")
        if hasattr(solver_result, 'gradnorm'):
            print(f"  Gradient norm: {solver_result.gradnorm:.6e}")
            print(f" Solver result.x shape: {solver_result.x.shape}")
        
        # PROJECT THE SOLUTION from relaxed space to feasible set
        print(f"\nProjecting solution to feasible set...")
        projected_matrix = cora.projectSolution(problem, solver_result.x, verbose=False)
        print(f"  Projected matrix shape: {projected_matrix.shape}")
        
        # ALIGN SOLUTION TO ORIGIN (CORA's canonical frame)
        # This aligns so first rotation = identity and mean translation = 0
        print(f"Aligning solution to CORA's canonical frame using problem.alignEstimateToOrigin()...")
        aligned_matrix = problem.alignEstimateToOrigin(projected_matrix)
        print(f"  Aligned matrix shape: {aligned_matrix.shape}")
        
        # Convert aligned matrix back to Values
        print(f"Converting aligned matrix to CORA Values...")
        result_values = cora.getValuesFromVarMatrix(problem, aligned_matrix)
        print(f"  Result values contain {len([k for k in range(272*2) if result_values.has_pose(cora.Symbol('A', k))])} poses")
        
        # Check first pose coordinates after CORA's internal alignment
        first_sym = cora.Symbol('A', 0)
        first_trans_cora = result_values.get_pose_translation(first_sym)[:3]
        first_rot_cora = result_values.get_pose_rotation(first_sym)
        
        # Get expected first pose from integrated_state
        first_state = integrated_state_data[0]
        expected_trans = np.array([first_state['x'], first_state['y'], first_state['z']])
        try:
            expected_rot = R.from_quat([first_state['qx'], first_state['qy'], first_state['qz'], first_state['qw']]).as_matrix()
        except AttributeError:
            expected_rot = R.from_quat([first_state['qx'], first_state['qy'], first_state['qz'], first_state['qw']]).as_dcm()
        
        print(f"\nCoordinate frame transformation:")
        print(f"  CORA canonical frame first pose: ({first_trans_cora[0]:.2f}, {first_trans_cora[1]:.2f}, {first_trans_cora[2]:.2f})")
        print(f"  World frame first pose: ({expected_trans[0]:.2f}, {expected_trans[1]:.2f}, {expected_trans[2]:.2f})")
        
        # Compute SE(3) transformation from CORA's canonical frame to world frame
        # T_world = T_align * T_cora
        # where T_align = T_expected * T_cora^{-1}
        R_align = expected_rot @ first_rot_cora.T  # Rotation alignment
        t_align = expected_trans - R_align @ first_trans_cora  # Translation alignment
        
        print(f"  Applying SE(3) transformation to world frame...")
        print(f"  Translation offset: ({t_align[0]:.2f}, {t_align[1]:.2f}, {t_align[2]:.2f})")
        
        # Apply SE(3) transformation to all poses
        print(f"\nTransforming CORA poses to world coordinate frame...")
        aligned_values = cora.Values()
        for key_str in pose_keys:
            char, idx = parse_key(key_str)
            sym = cora.Symbol(char, idx)
            
            if result_values.has_pose(sym):
                rot_cora = result_values.get_pose_rotation(sym)
                trans_cora = result_values.get_pose_translation(sym)[:3]
                
                # Apply SE(3) transformation to world frame
                rot_world = R_align @ rot_cora
                trans_world = R_align @ trans_cora + t_align
                
                aligned_values.set_pose(sym, rot_world, trans_world)
        
        # Also transform landmarks
        for key_str in landmark_keys:
            char, idx = parse_key(key_str)
            sym = cora.Symbol(char, idx)
            
            if result_values.has_landmark(sym):
                pos_cora = result_values.get_landmark(sym)
                # Apply SE(3) transformation to landmarks too
                pos_world = R_align @ pos_cora + t_align
                aligned_values.set_landmark(sym, pos_world)
        
        print(f"✓ Transformed {len(pose_keys)} poses and {len(landmark_keys)} landmarks to world frame")
        
        return problem, initial_values, aligned_values, list(pose_keys), list(landmark_keys)
        
    except Exception as e:
        print(f"\n{'='*80}")
        print(f"CORA OPTIMIZATION FAILED")
        print(f"{'='*80}")
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return problem, initial_values, initial_values, list(pose_keys), list(landmark_keys)


def extract_trajectory_from_cora(problem, values, pose_keys):
    """Extract trajectory from CORA Values."""
    trajectory = []
    
    # Sort pose_keys numerically (not alphabetically)
    def key_sort(key_str):
        """Extract numeric index from key like 'A123' -> 123"""
        return int(key_str[1:])
    
    for key_str in sorted(pose_keys, key=key_sort):
        char, idx = parse_key(key_str)
        sym = cora.Symbol(char, idx)
        
        # Extract pose directly from CORA Values object
        try:
            if values.has_pose(sym):
                rot = values.get_pose_rotation(sym)
                trans = values.get_pose_translation(sym)
                
                # Convert to pose dict
                trajectory.append({
                    'x': trans[0],
                    'y': trans[1],
                    'z': trans[2],
                    'rotation_matrix': rot,
                })
            else:
                print(f"Warning: Pose {key_str} not found in CORA values")
                continue
        except Exception as e:
            print(f"Warning: Could not extract pose for {key_str}: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    return trajectory


################################################################################
# MAIN
################################################################################

def main():
    parser = argparse.ArgumentParser(
        description='Extract data from ROS bag and run offline GTSAM optimization'
    )
    parser.add_argument('bag_file', help='Path to ROS bag file')
    parser.add_argument('--output-dir', default=None, 
                       help='Output directory for CSV and plots (default: tests/results/<bag_name>)')
    parser.add_argument('--actor', default='actor_0', 
                       help='Actor name in topics (default: actor_0)')
    parser.add_argument('--cutoff-time', type=float, default=None,
                       help='Timestamp cutoff - exclude data after this time (for trimming surface data)')
    
    args = parser.parse_args()
    
    # Validate bag file exists
    if not os.path.exists(args.bag_file):
        print(f"ERROR: Bag file not found: {args.bag_file}")
        sys.exit(1)
    
    # Determine output directory
    if args.output_dir is None:
        bag_name = Path(args.bag_file).stem
        args.output_dir = f"results/{bag_name}_offline"
    
    print(f"\n{'='*80}")
    print(f"OFFLINE GTSAM OPTIMIZATION")
    print(f"{'='*80}")
    print(f"Bag file: {args.bag_file}")
    print(f"Actor: {args.actor}")
    print(f"Output directory: {args.output_dir}")
    if args.cutoff_time:
        print(f"Cutoff time: {args.cutoff_time:.6f}")
    print(f"{'='*80}\n")
    
    # Step 1: Extract data from bag
    odometry_data, integrated_state_data, range_data, gps_data = extract_data_from_bag(args.bag_file, args.actor)
    
    # Step 1.5: Save raw CSVs (before trimming)
    raw_dir = os.path.join(args.output_dir, 'raw')
    os.makedirs(raw_dir, exist_ok=True)
    print(f"\n{'='*80}")
    print(f"SAVING RAW DATA (before trimming)")
    print(f"{'='*80}\n")
    save_to_csv(odometry_data, integrated_state_data, range_data, gps_data, raw_dir)
    
    # Store original GPS data for plotting
    original_gps_data = gps_data.copy()
    
    # Step 1.6: Apply cutoff time filter if specified
    if args.cutoff_time:
        print(f"\n{'='*80}")
        print(f"APPLYING CUTOFF TIME FILTER")
        print(f"{'='*80}\n")
        
        original_odom = len(odometry_data)
        original_state = len(integrated_state_data)
        original_range = len(range_data)
        original_gps = len(gps_data)
        
        odometry_data = [d for d in odometry_data if d['timestamp'] <= args.cutoff_time]
        integrated_state_data = [d for d in integrated_state_data if d['timestamp'] <= args.cutoff_time]
        range_data = [d for d in range_data if d['timestamp'] <= args.cutoff_time]
        gps_data = [d for d in gps_data if d['timestamp'] <= args.cutoff_time]
        
        print(f"Odometry data: {original_odom} → {len(odometry_data)} ({original_odom - len(odometry_data)} removed)")
        print(f"Integrated state: {original_state} → {len(integrated_state_data)} ({original_state - len(integrated_state_data)} removed)")
        print(f"Range data: {original_range} → {len(range_data)} ({original_range - len(range_data)} removed)")
        print(f"GPS data: {original_gps} → {len(gps_data)} ({original_gps - len(gps_data)} removed)")
        
        # Re-map pose keys in range data
        # The bag file uses keys A0, A2, A4, A6... (incrementing by 2)
        # But we assign sequential indices A0, A1, A2, A3... to integrated_state
        # Formula: new_index = old_index / 2
        print(f"\nRe-mapping pose keys in range data (A0,A2,A4... → A0,A1,A2...)...")
        max_pose_idx = len(integrated_state_data) - 1
        filtered_ranges = []
        for r in range_data:
            if r['key1'].startswith('A'):
                old_idx = int(r['key1'][1:])
                new_idx = old_idx // 2
                # Only keep if the new index is valid
                if new_idx <= max_pose_idx:
                    r['key1'] = f'A{new_idx}'
                    filtered_ranges.append(r)
            else:
                filtered_ranges.append(r)
        
        removed_ranges = len(range_data) - len(filtered_ranges)
        range_data = filtered_ranges
        print(f"✓ Re-mapped {len(range_data)} pose keys ({removed_ranges} ranges filtered out due to invalid keys)")
    
    # Step 2: Save trimmed CSVs
    print(f"\n{'='*80}")
    print(f"SAVING TRIMMED DATA (for optimization)")
    print(f"{'='*80}\n")
    save_to_csv(odometry_data, integrated_state_data, range_data, gps_data, args.output_dir)
    
    # Step 3: Run GTSAM optimization
    gtsam_graph, gtsam_initial, gtsam_result, gtsam_pose_keys, gtsam_landmark_keys = run_gtsam_optimization(
        integrated_state_data, range_data, gps_data
    )
    
    # Step 3.5: Run CORA optimization
    cora_problem, cora_initial, cora_result, cora_pose_keys, cora_landmark_keys = run_cora_optimization(
        integrated_state_data, range_data, gps_data
    )
    
    # Step 4: Extract trajectories
    print(f"\n{'='*80}")
    print(f"EXTRACTING OPTIMIZED TRAJECTORIES")
    print(f"{'='*80}\n")
    
    initial_traj = extract_trajectory(gtsam_initial, gtsam_pose_keys)
    gtsam_traj = extract_trajectory(gtsam_result, gtsam_pose_keys)
    
    # Extract CORA trajectory using specialized function
    if CORA_AVAILABLE and cora_result is not None:
        cora_traj = extract_trajectory_from_cora(cora_problem, cora_result, cora_pose_keys)
    else:
        # Fallback: use GTSAM trajectory if CORA failed
        cora_traj = gtsam_traj
        print("⚠ Using GTSAM trajectory as fallback for CORA")
    
    # Step 5: Save optimized trajectories
    save_trajectory_csv(gtsam_traj, args.output_dir, 'gtsam_optimized_trajectory.csv')
    save_trajectory_csv(cora_traj, args.output_dir, 'cora_optimized_trajectory.csv')
    save_trajectory_csv(initial_traj, args.output_dir, 'initial_trajectory.csv')
    
    # Step 5.5: Calculate distance from last optimized pose to first surface GPS fix
    if args.cutoff_time and len(gtsam_traj) > 0 and len(original_gps_data) > 0:
        last_odom_time = integrated_state_data[-1]['timestamp']
        surface_fixes = [gps for gps in original_gps_data if gps['timestamp'] > last_odom_time]
        
        if len(surface_fixes) > 0:
            print(f"\n{'='*80}")
            print(f"DISTANCE TO SURFACE GPS FIX")
            print(f"{'='*80}")
            
            first_surface_gps = surface_fixes[0]
            print(f"First surface GPS: ({first_surface_gps['x']:.2f}, {first_surface_gps['y']:.2f}, {first_surface_gps['z']:.2f})")
            print(f"Time difference: {first_surface_gps['timestamp'] - last_odom_time:.2f}s\n")
            
            # GTSAM distance
            last_gtsam_pose = gtsam_traj[-1]
            import numpy as np
            gtsam_distance_3d = np.sqrt(
                (last_gtsam_pose['x'] - first_surface_gps['x'])**2 + 
                (last_gtsam_pose['y'] - first_surface_gps['y'])**2 + 
                (last_gtsam_pose['z'] - first_surface_gps['z'])**2
            )
            gtsam_distance_xy = np.sqrt(
                (last_gtsam_pose['x'] - first_surface_gps['x'])**2 + 
                (last_gtsam_pose['y'] - first_surface_gps['y'])**2
            )
            
            print(f"GTSAM Last Pose: ({last_gtsam_pose['x']:.2f}, {last_gtsam_pose['y']:.2f}, {last_gtsam_pose['z']:.2f})")
            # print(f"  Distance (3D): {gtsam_distance_3d:.2f}m")
            print(f"  Distance (XY): {gtsam_distance_xy:.2f}m\n")
            
            # CORA distance
            last_cora_pose = cora_traj[-1]
            cora_distance_3d = np.sqrt(
                (last_cora_pose['x'] - first_surface_gps['x'])**2 + 
                (last_cora_pose['y'] - first_surface_gps['y'])**2 + 
                (last_cora_pose['z'] - first_surface_gps['z'])**2
            )
            cora_distance_xy = np.sqrt(
                (last_cora_pose['x'] - first_surface_gps['x'])**2 + 
                (last_cora_pose['y'] - first_surface_gps['y'])**2
            )
            
            print(f"CORA Last Pose: ({last_cora_pose['x']:.2f}, {last_cora_pose['y']:.2f}, {last_cora_pose['z']:.2f})")
            # print(f"  Distance (3D): {cora_distance_3d:.2f}m")
            print(f"  Distance (XY): {cora_distance_xy:.2f}m")
    
    # Step 6: Generate plots (use original GPS data to show all GPS fixes including surface)
    plot_path, gtsam_error, cora_error = plot_trajectory(
        initial_traj, gtsam_traj, cora_traj, gtsam_landmark_keys, gtsam_result,
        integrated_state_data, range_data, original_gps_data, args.output_dir)
    
    print(f"\n{'='*80}")
    print(f"OFFLINE OPTIMIZATION COMPLETE")
    print(f"{'='*80}")
    print(f"All results saved to: {args.output_dir}")
    print(f"{'='*80}\n")
    
    # Return error metrics for batch processing summary
    return gtsam_error, cora_error


if __name__ == '__main__':
    main()
