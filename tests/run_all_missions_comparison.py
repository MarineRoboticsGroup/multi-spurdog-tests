#!/usr/bin/env python3
"""
Run all missions through both optimizers and generate comparison table.

This script:
1. Runs all missions WITHOUT priors (offline_optimizer.py)
2. Runs all missions WITH priors (offline_optimizer_with_priors.py)
3. Prints a comparison table of δGTSAM and δCORA errors
"""

import sys
import importlib.util
from pathlib import Path

# Mission data: (name, bag_file, cutoff_time)
MISSIONS = [
    ("pink_solo", "tests/pink_solo.bag", 1764048438.567943),
    ("past_shep", "tests/past_shep.bag", 1764043197.378492),
    ("lush_erle", "tests/lush_erle.bag", 1764049657.220171),
    ("sour_axle", "tests/sour_axle.bag", 1764040146.629958),
    ("damp_beer", "tests/damp_beer.bag", 1764041947.780546),
]


def load_optimizer_module(script_path):
    """Dynamically load optimizer script as a module."""
    spec = importlib.util.spec_from_file_location("optimizer", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_optimizer(optimizer_module, bag_file, cutoff_time):
    """
    Run optimizer on a mission.
    
    Returns:
        (gtsam_error, cora_error) or (None, None) if failed
    """
    # Create mock args for the optimizer
    class Args:
        def __init__(self):
            self.bag_file = bag_file
            self.output_dir = None  # Use default
            self.actor = 'actor_0'
            self.cutoff_time = cutoff_time
    
    args = Args()
    
    # Temporarily replace sys.argv to pass args to optimizer
    old_argv = sys.argv
    sys.argv = ['optimizer', bag_file, '--cutoff-time', str(cutoff_time)]
    
    try:
        # Call the optimizer's main function directly
        result = optimizer_module.main()
        if result is None:
            return None, None
        gtsam_error, cora_error = result
        return gtsam_error, cora_error
    except Exception as e:
        print(f"Error running optimizer: {e}")
        import traceback
        traceback.print_exc()
        return None, None
    finally:
        sys.argv = old_argv


def print_comparison_table(results_without_priors, results_with_priors):
    """Print a formatted comparison table."""
    print("\n" + "="*100)
    print("OPTIMIZER COMPARISON: WITH vs WITHOUT PRIORS")
    print("="*100)
    print("\nXY Distance to Surface GPS Fix (meters)")
    print("-"*100)
    
    # Table header
    print(f"{'Mission':<15} {'WITHOUT PRIORS':<35} {'WITH PRIORS':<35} {'Change':<15}")
    print(f"{'':15} {'δGTSAM':<15} {'δCORA':<15} {'δGTSAM':<15} {'δCORA':<15} {'GTSAM':<7} {'CORA':<7}")
    print("-"*100)
    
    # Sort missions by name for consistent display
    mission_names = sorted(results_without_priors.keys())
    
    for mission in mission_names:
        no_prior = results_without_priors.get(mission, (None, None))
        with_prior = results_with_priors.get(mission, (None, None))
        
        gtsam_no = no_prior[0]
        cora_no = no_prior[1]
        gtsam_yes = with_prior[0]
        cora_yes = with_prior[1]
        
        # Format values
        gtsam_no_str = f"{gtsam_no:.2f}m" if gtsam_no is not None else "N/A"
        cora_no_str = f"{cora_no:.2f}m" if cora_no is not None else "N/A"
        gtsam_yes_str = f"{gtsam_yes:.2f}m" if gtsam_yes is not None else "N/A"
        cora_yes_str = f"{cora_yes:.2f}m" if cora_yes is not None else "N/A"
        
        # Calculate change (positive = improvement, negative = degradation)
        gtsam_change = ""
        cora_change = ""
        if gtsam_no is not None and gtsam_yes is not None:
            diff = gtsam_no - gtsam_yes
            gtsam_change = f"{diff:+.2f}m" if diff != 0 else "0.00m"
        if cora_no is not None and cora_yes is not None:
            diff = cora_no - cora_yes
            cora_change = f"{diff:+.2f}m" if diff != 0 else "0.00m"
        
        print(f"{mission:<15} {gtsam_no_str:<15} {cora_no_str:<15} {gtsam_yes_str:<15} {cora_yes_str:<15} {gtsam_change:<7} {cora_change:<7}")
    
    print("-"*100)
    print("\nNotes:")
    print("  • δGTSAM/δCORA: XY distance from final optimized pose to surface GPS fix")
    print("  • Change: Positive = improvement (WITH priors is better), Negative = degradation")
    print("  • WITHOUT priors → tests/results/<mission>_offline/")
    print("  • WITH priors → tests/results_with_priors/<mission>_offline/")
    print("="*100 + "\n")


def main():
    print("="*100)
    print("RUNNING ALL MISSIONS THROUGH BOTH OPTIMIZERS (WITH & WITHOUT PRIORS)")
    print("="*100)
    print()
    
    # Load optimizer modules
    offline_opt = load_optimizer_module("tests/offline_optimizer.py")
    offline_opt_priors = load_optimizer_module("tests/offline_optimizer_with_priors.py")
    
    # Storage for results
    results_without_priors = {}
    results_with_priors = {}
    
    # PART 1: Run WITHOUT priors
    print("="*100)
    print("PART 1: RUNNING MISSIONS WITHOUT PRIORS")
    print("="*100)
    print()
    
    for name, bag, cutoff in MISSIONS:
        print("-"*100)
        print(f"Processing: {name} (WITHOUT priors)")
        print("-"*100)
        
        gtsam_err, cora_err = run_optimizer(offline_opt, bag, cutoff)
        results_without_priors[name] = (gtsam_err, cora_err)
        
        if gtsam_err is not None:
            print(f"✓ {name} completed successfully (WITHOUT priors)")
            print(f"  δGTSAM: {gtsam_err:.2f}m, δCORA: {cora_err:.2f}m")
        else:
            print(f"✗ {name} failed (WITHOUT priors)")
        print()
    
    print("="*100)
    print("PART 1 COMPLETE: All missions processed WITHOUT priors")
    print("Results saved to: tests/results/<mission>_offline/")
    print("="*100)
    print()
    
    # PART 2: Run WITH priors
    print("="*100)
    print("PART 2: RUNNING MISSIONS WITH PRIORS")
    print("="*100)
    print()
    
    for name, bag, cutoff in MISSIONS:
        print("-"*100)
        print(f"Processing: {name} (WITH priors)")
        print("-"*100)
        
        gtsam_err, cora_err = run_optimizer(offline_opt_priors, bag, cutoff)
        results_with_priors[name] = (gtsam_err, cora_err)
        
        if gtsam_err is not None:
            print(f"✓ {name} completed successfully (WITH priors)")
            print(f"  δGTSAM: {gtsam_err:.2f}m, δCORA: {cora_err:.2f}m")
        else:
            print(f"✗ {name} failed (WITH priors)")
        print()
    
    print("="*100)
    print("PART 2 COMPLETE: All missions processed WITH priors")
    print("Results saved to: tests/results_with_priors/<mission>_offline/")
    print("="*100)
    print()
    
    # PART 3: Print comparison table
    print_comparison_table(results_without_priors, results_with_priors)
    
    print("="*100)
    print("ALL MISSIONS COMPLETE")
    print("="*100)


if __name__ == '__main__':
    main()
