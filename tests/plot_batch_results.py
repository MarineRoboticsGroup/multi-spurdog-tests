#!/usr/bin/env python3
"""
Plot CORA batch solution vs odometry to visualize optimization results.
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def plot_batch_comparison(mission_name, results_dir):
    """Plot odometry vs CORA batch solution."""
    
    # Read CSV files
    odom_file = os.path.join(results_dir, f'{mission_name}_odometry.csv')
    cora_batch_file = os.path.join(results_dir, f'{mission_name}_cora_batch.csv')
    
    if not os.path.exists(odom_file):
        print(f"ERROR: Odometry file not found: {odom_file}")
        return
    
    if not os.path.exists(cora_batch_file):
        print(f"ERROR: CORA batch file not found: {cora_batch_file}")
        return
    
    # Load data
    odom_df = pd.read_csv(odom_file)
    cora_df = pd.read_csv(cora_batch_file)
    
    print(f"Loaded {len(odom_df)} odometry poses")
    print(f"Loaded {len(cora_df)} CORA batch poses")
    
    # Create figure with 4 subplots
    fig = plt.figure(figsize=(16, 12))
    
    # 1. XY plot
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(odom_df['x'], odom_df['y'], 'b-', label='Odometry', alpha=0.7, linewidth=2)
    ax1.plot(cora_df['x'], cora_df['y'], 'r-', label='CORA Batch', alpha=0.7, linewidth=2)
    ax1.scatter(odom_df['x'].iloc[0], odom_df['y'].iloc[0], c='blue', s=100, marker='o', label='Odom Start')
    ax1.scatter(cora_df['x'].iloc[0], cora_df['y'].iloc[0], c='red', s=100, marker='o', label='CORA Start')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. XZ plot
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(odom_df['x'], odom_df['z'], 'b-', label='Odometry', alpha=0.7, linewidth=2)
    ax2.plot(cora_df['x'], cora_df['z'], 'r-', label='CORA Batch', alpha=0.7, linewidth=2)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('XZ Trajectory (Depth Profile)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 3D plot
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    ax3.plot(odom_df['x'], odom_df['y'], odom_df['z'], 'b-', label='Odometry', alpha=0.7, linewidth=2)
    ax3.plot(cora_df['x'], cora_df['y'], cora_df['z'], 'r-', label='CORA Batch', alpha=0.7, linewidth=2)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_zlabel('Z (m)')
    ax3.set_title('3D Trajectory')
    ax3.legend()
    
    # 4. Difference plot
    ax4 = fig.add_subplot(2, 2, 4)
    # Compute Euclidean distance between corresponding poses
    diff = ((cora_df['x'] - odom_df['x'])**2 + 
            (cora_df['y'] - odom_df['y'])**2 + 
            (cora_df['z'] - odom_df['z'])**2)**0.5
    ax4.plot(diff.values, 'g-', linewidth=2)
    ax4.set_xlabel('Pose Index')
    ax4.set_ylabel('Euclidean Distance (m)')
    ax4.set_title(f'CORA vs Odometry Difference (Mean: {diff.mean():.2f}m, Max: {diff.max():.2f}m)')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f'{mission_name} - CORA Batch Optimization Results', fontsize=16)
    plt.tight_layout()
    
    # Save plot
    output_file = os.path.join(results_dir, f'{mission_name}_batch_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_file}")
    
    # Print statistics
    print(f"\n=== Statistics ===")
    print(f"Odometry range: X=[{odom_df['x'].min():.2f}, {odom_df['x'].max():.2f}], "
          f"Y=[{odom_df['y'].min():.2f}, {odom_df['y'].max():.2f}], "
          f"Z=[{odom_df['z'].min():.2f}, {odom_df['z'].max():.2f}]")
    print(f"CORA range: X=[{cora_df['x'].min():.2f}, {cora_df['x'].max():.2f}], "
          f"Y=[{cora_df['y'].min():.2f}, {cora_df['y'].max():.2f}], "
          f"Z=[{cora_df['z'].min():.2f}, {cora_df['z'].max():.2f}]")
    print(f"Average trajectory difference: {diff.mean():.2f}m")
    print(f"Maximum trajectory difference: {diff.max():.2f}m")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        mission_name = 'pink_solo'
        results_dir = '/home/lizgajski2/catkin_ws/tests/results/pink_solo'
    else:
        mission_name = sys.argv[1]
        results_dir = f'/home/lizgajski2/catkin_ws/tests/results/{mission_name}'
    
    plot_batch_comparison(mission_name, results_dir)
