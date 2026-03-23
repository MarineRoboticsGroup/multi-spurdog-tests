#!/usr/bin/env python3
"""
Systematic noise model parameter sweep for CORA and GTSAM.

This script tests multiple configurations to address:
1. Fair comparison: Use GTSAM's parameters for both optimizers
2. Optimal tuning: Find best parameters for each optimizer
3. Hypothesis: CORA might benefit from loose rotation, tight translation

Tests configurations on pink_solo mission (fastest to iterate on).
"""

import subprocess
import sys
import re
from pathlib import Path

# Test configurations
# Format: (name, translation_std, rotation_std, description)
CONFIGURATIONS = [
    # ===== FAIR COMPARISON: GTSAM's parameters =====
    ("gtsam_params", 2.0, 5.0, "GTSAM's optimal params (fair comparison)"),
    
    # ===== CURRENT OPTIMAL =====
    ("current_optimal", 0.316, 0.1, "Current CORA optimal (13.20m error)"),
    
    # ===== HYPOTHESIS: Loose rotation, tight translation =====
    ("loose_rot_0.5", 0.316, 0.5, "Moderate rotation looseness (28.6°)"),
    ("loose_rot_1.0", 0.316, 1.0, "Loose rotation (57.3°)"),
    ("loose_rot_2.0", 0.316, 2.0, "Very loose rotation (114.6°)"),
    ("loose_rot_3.0", 0.316, 3.0, "Extremely loose rotation (171.9°)"),
    ("loose_rot_5.0", 0.316, 5.0, "GTSAM-style rotation (286°)"),
    
    # ===== ALTERNATIVE: Moderate looseness for both =====
    ("moderate_both", 0.5, 0.5, "Moderate looseness both"),
    ("moderate_trans", 0.5, 1.0, "Moderate trans, loose rot"),
    
    # ===== EXTREMES for reference =====
    ("very_tight", 0.1, 0.05, "Very tight (reference lower bound)"),
    ("very_loose", 2.0, 5.0, "Very loose (GTSAM params duplicate)"),
]

# Test mission
TEST_MISSION = {
    "name": "pink_solo",
    "bag": "tests/pink_solo.bag",
    "cutoff": 1764048438.567943
}


def modify_optimizer_script(script_path, trans_std, rot_std):
    """
    Temporarily modify optimizer script with new noise parameters.
    Returns backup path.
    """
    backup_path = f"{script_path}.backup"
    
    # Read original
    with open(script_path, 'r') as f:
        lines = f.readlines()
    
    # Save backup
    with open(backup_path, 'w') as f:
        f.writelines(lines)
    
    # Find and modify CORA noise model lines
    # Look for lines around 730 in CORA section
    modified = False
    for i, line in enumerate(lines):
        # Find translation_std in CORA section (has specific value 0.316)
        if 'translation_std = 0.316' in line and 'CORA' in ''.join(lines[max(0,i-30):i]):
            lines[i] = f'        translation_std = {trans_std}      # Modified by sweep: {trans_std}m\n'
            modified = True
            print(f"  Modified line {i+1}: translation_std = {trans_std}")
        # Find rotation_std right after
        elif modified and 'rotation_std = 0.1' in line:
            lines[i] = f'        rotation_std = {rot_std}           # Modified by sweep: {rot_std:.1f}rad\n'
            print(f"  Modified line {i+1}: rotation_std = {rot_std}")
            break
    
    if not modified:
        print("  WARNING: Could not find CORA noise model lines to modify!")
    
    # Write modified
    with open(script_path, 'w') as f:
        f.writelines(lines)
    
    return backup_path


def restore_optimizer_script(script_path, backup_path):
    """Restore original optimizer script from backup."""
    with open(backup_path, 'r') as f:
        content = f.read()
    with open(script_path, 'w') as f:
        f.write(content)
    Path(backup_path).unlink()


def run_optimizer(script_path, bag_file, cutoff_time):
    """
    Run optimizer and extract error metrics.
    Returns (gtsam_error, cora_error) or (None, None) if failed.
    """
    cmd = [
        "python3", script_path,
        bag_file,
        "--cutoff-time", str(cutoff_time)
    ]
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300  # 5 min timeout
        )
        
        if result.returncode != 0:
            print(f"  ✗ Optimizer failed with return code {result.returncode}")
            return None, None
        
        # Parse output for error metrics
        output = result.stdout + result.stderr
        
        # Extract GTSAM error
        gtsam_match = re.search(r'GTSAM Last Pose:.*?Distance \(XY\): ([\d.]+)m', output, re.DOTALL)
        cora_match = re.search(r'CORA Last Pose:.*?Distance \(XY\): ([\d.]+)m', output, re.DOTALL)
        
        gtsam_error = float(gtsam_match.group(1)) if gtsam_match else None
        cora_error = float(cora_match.group(1)) if cora_match else None
        
        return gtsam_error, cora_error
        
    except subprocess.TimeoutExpired:
        print(f"  ✗ Optimizer timed out after 5 minutes")
        return None, None
    except Exception as e:
        print(f"  ✗ Error running optimizer: {e}")
        return None, None


def print_results_table(results):
    """Print formatted results table."""
    print("\n" + "="*120)
    print("NOISE MODEL PARAMETER SWEEP RESULTS")
    print("="*120)
    print(f"\nTested on: {TEST_MISSION['name']} mission")
    print("-"*120)
    print(f"{'Configuration':<20} {'Trans σ':<10} {'Rot σ':<12} {'CORA Error':<15} {'GTSAM Error':<15} {'vs GTSAM':<15} {'Description'}")
    print(f"{'':20} {'(m)':<10} {'(rad/deg)':<12} {'(m)':<15} {'(m)':<15} {'Optimal':<15}")
    print("-"*120)
    
    # Sort by CORA error
    sorted_results = sorted(results, key=lambda x: x[3] if x[3] is not None else float('inf'))
    
    for config_name, trans_std, rot_std, cora_err, gtsam_err, desc in sorted_results:
        rot_deg = rot_std * 57.2958  # Convert to degrees
        rot_str = f"{rot_std:.1f} / {rot_deg:.1f}°"
        
        cora_str = f"{cora_err:.2f}m" if cora_err is not None else "FAILED"
        gtsam_str = f"{gtsam_err:.2f}m" if gtsam_err is not None else "FAILED"
        
        # Compare to GTSAM's optimal (3.25m)
        if cora_err is not None:
            ratio = cora_err / 3.25
            comparison = f"{ratio:.2f}x"
        else:
            comparison = "N/A"
        
        # Highlight best result
        marker = "✨" if cora_err is not None and cora_err == min([r[3] for r in results if r[3] is not None]) else "  "
        
        print(f"{marker}{config_name:<18} {trans_std:<10.3f} {rot_str:<12} {cora_str:<15} {gtsam_str:<15} {comparison:<15} {desc}")
    
    print("-"*120)
    
    # Find best configuration
    best = min([r for r in results if r[3] is not None], key=lambda x: x[3], default=None)
    if best:
        print(f"\n✨ BEST CONFIGURATION: {best[0]}")
        print(f"   Translation σ: {best[1]:.3f}m")
        print(f"   Rotation σ: {best[2]:.3f} rad ({best[2]*57.2958:.1f}°)")
        print(f"   CORA Error: {best[3]:.2f}m")
        print(f"   GTSAM Error: {best[4]:.2f}m")
        print(f"   Improvement over current: {((13.20 - best[3])/13.20 * 100):.1f}%")
    
    print("\n" + "="*120)
    
    # Key insights
    print("\nKEY INSIGHTS:")
    print("-"*120)
    
    # Check if GTSAM params work for CORA
    gtsam_config = next((r for r in results if r[0] == "gtsam_params"), None)
    if gtsam_config and gtsam_config[3] is not None:
        print(f"1. FAIR COMPARISON (same parameters):")
        print(f"   With GTSAM's params (σ_t=2.0m, σ_r=5.0rad):")
        print(f"   - CORA error: {gtsam_config[3]:.2f}m")
        print(f"   - GTSAM error: {gtsam_config[4]:.2f}m")
        print(f"   - Ratio: {gtsam_config[3]/gtsam_config[4]:.2f}x worse")
    
    # Check hypothesis
    loose_rot_configs = [r for r in results if "loose_rot" in r[0] and r[3] is not None]
    if loose_rot_configs:
        best_loose_rot = min(loose_rot_configs, key=lambda x: x[3])
        print(f"\n2. LOOSE ROTATION HYPOTHESIS:")
        print(f"   Best with tight translation + loose rotation:")
        print(f"   - Config: {best_loose_rot[0]} (σ_t={best_loose_rot[1]:.3f}m, σ_r={best_loose_rot[2]:.3f}rad)")
        print(f"   - CORA error: {best_loose_rot[3]:.2f}m")
        print(f"   - Improvement over current: {((13.20 - best_loose_rot[3])/13.20 * 100):.1f}%")
    
    print("="*120 + "\n")


def main():
    print("="*120)
    print("NOISE MODEL PARAMETER SWEEP FOR CORA")
    print("="*120)
    print(f"\nMission: {TEST_MISSION['name']}")
    print(f"Configurations to test: {len(CONFIGURATIONS)}")
    print(f"Estimated time: {len(CONFIGURATIONS) * 2:.0f} minutes")
    print("="*120 + "\n")
    
    script_path = "tests/offline_optimizer.py"
    results = []
    
    for i, (config_name, trans_std, rot_std, desc) in enumerate(CONFIGURATIONS, 1):
        print(f"\n[{i}/{len(CONFIGURATIONS)}] Testing: {config_name}")
        print(f"  Description: {desc}")
        print(f"  Parameters: σ_trans={trans_std}m, σ_rot={rot_std:.1f}rad ({rot_std*57.2958:.1f}°)")
        
        # Modify script
        backup_path = modify_optimizer_script(script_path, trans_std, rot_std)
        
        try:
            # Run optimizer
            gtsam_err, cora_err = run_optimizer(
                script_path,
                TEST_MISSION['bag'],
                TEST_MISSION['cutoff']
            )
            
            if cora_err is not None:
                print(f"  ✓ Success: CORA={cora_err:.2f}m, GTSAM={gtsam_err:.2f}m")
            else:
                print(f"  ✗ Failed to extract errors")
            
            results.append((config_name, trans_std, rot_std, cora_err, gtsam_err, desc))
            
        finally:
            # Restore original script
            restore_optimizer_script(script_path, backup_path)
    
    # Print results
    print_results_table(results)
    
    # Save results to CSV
    csv_path = "tests/results/noise_model_sweep_results.csv"
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
    
    with open(csv_path, 'w') as f:
        f.write("config_name,trans_std_m,rot_std_rad,rot_std_deg,cora_error_m,gtsam_error_m,description\n")
        for config_name, trans_std, rot_std, cora_err, gtsam_err, desc in results:
            rot_deg = rot_std * 57.2958
            cora_str = f"{cora_err:.2f}" if cora_err is not None else "FAILED"
            gtsam_str = f"{gtsam_err:.2f}" if gtsam_err is not None else "FAILED"
            f.write(f"{config_name},{trans_std},{rot_std},{rot_deg:.1f},{cora_str},{gtsam_str},{desc}\n")
    
    print(f"Results saved to: {csv_path}")


if __name__ == '__main__':
    main()
