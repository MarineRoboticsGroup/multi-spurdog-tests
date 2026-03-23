#!/bin/bash
# Run all missions through offline optimizers (both with and without priors)
# and generate comparison table

echo "================================================================"
echo "RUNNING ALL MISSIONS THROUGH OFFLINE OPTIMIZERS"
echo "================================================================"
echo ""

# Mission data (mission_name:bag_file:cutoff_time)
missions=(
    "pink_solo:tests/pink_solo.bag:1764048438.567943"
    "past_shep:tests/past_shep.bag:1764043197.378492"
    "lush_erle:tests/lush_erle.bag:1764049657.220171"
    "sour_axle:tests/sour_axle.bag:1764040146.629958"
    "damp_beer:tests/damp_beer.bag:1764041947.780546"
)

# Arrays to store results
declare -A gtsam_no_prior
declare -A cora_no_prior
declare -A gtsam_with_prior
declare -A cora_with_prior

# PART 1: Run WITHOUT priors
echo "================================================================"
echo "PART 1: RUNNING MISSIONS WITHOUT PRIORS"
echo "================================================================"
echo ""

for mission_data in "${missions[@]}"; do
    IFS=':' read -r name bag cutoff <<< "$mission_data"
    
    echo "----------------------------------------------------------------"
    echo "Processing: $name (WITHOUT priors)"
    echo "----------------------------------------------------------------"
    
    # Run optimizer and capture output
    output=$(python3 tests/offline_optimizer.py "$bag" --cutoff-time "$cutoff" 2>&1)
    
    if [ $? -eq 0 ]; then
        echo "✓ $name completed successfully (WITHOUT priors)"
        
        # Extract GTSAM error (look for "GTSAM Last Pose" section)
        gtsam_err=$(echo "$output" | grep -A 1 "GTSAM Last Pose:" | grep "Distance (XY):" | sed 's/.*Distance (XY): \([0-9.]*\)m/\1/')
        cora_err=$(echo "$output" | grep -A 1 "CORA Last Pose:" | grep "Distance (XY):" | sed 's/.*Distance (XY): \([0-9.]*\)m/\1/')
        
        gtsam_no_prior[$name]=$gtsam_err
        cora_no_prior[$name]=$cora_err
        
        echo "  δGTSAM: ${gtsam_err}m, δCORA: ${cora_err}m"
    else
        echo "✗ $name failed (WITHOUT priors)"
        gtsam_no_prior[$name]="N/A"
        cora_no_prior[$name]="N/A"
    fi
    echo ""
done

echo "================================================================"
echo "PART 1 COMPLETE: All missions processed WITHOUT priors"
echo "Results saved to: tests/results/<mission>_offline/"
echo "================================================================"
echo ""

# PART 2: Run WITH priors
echo "================================================================"
echo "PART 2: RUNNING MISSIONS WITH PRIORS"
echo "================================================================"
echo ""

for mission_data in "${missions[@]}"; do
    IFS=':' read -r name bag cutoff <<< "$mission_data"
    
    echo "----------------------------------------------------------------"
    echo "Processing: $name (WITH priors)"
    echo "----------------------------------------------------------------"
    
    # Run optimizer and capture output
    output=$(python3 tests/offline_optimizer_with_priors.py "$bag" --cutoff-time "$cutoff" 2>&1)
    
    if [ $? -eq 0 ]; then
        echo "✓ $name completed successfully (WITH priors)"
        
        # Extract GTSAM error
        gtsam_err=$(echo "$output" | grep -A 1 "GTSAM Last Pose:" | grep "Distance (XY):" | sed 's/.*Distance (XY): \([0-9.]*\)m/\1/')
        cora_err=$(echo "$output" | grep -A 1 "CORA Last Pose:" | grep "Distance (XY):" | sed 's/.*Distance (XY): \([0-9.]*\)m/\1/')
        
        gtsam_with_prior[$name]=$gtsam_err
        cora_with_prior[$name]=$cora_err
        
        echo "  δGTSAM: ${gtsam_err}m, δCORA: ${cora_err}m"
    else
        echo "✗ $name failed (WITH priors)"
        gtsam_with_prior[$name]="N/A"
        cora_with_prior[$name]="N/A"
    fi
    echo ""
done

echo "================================================================"
echo "PART 2 COMPLETE: All missions processed WITH priors"
echo "Results saved to: tests/results_with_priors/<mission>_offline/"
echo "================================================================"
echo ""

# PART 3: Print comparison table
echo "===================================================================================================="
echo "OPTIMIZER COMPARISON: WITH vs WITHOUT PRIORS"
echo "===================================================================================================="
echo ""
echo "XY Distance to Surface GPS Fix (meters)"
echo "----------------------------------------------------------------------------------------------------"
printf "%-15s %-35s %-35s %-15s\n" "Mission" "WITHOUT PRIORS" "WITH PRIORS" "Change"
printf "%-15s %-15s %-15s %-15s %-15s %-7s %-7s\n" "" "δGTSAM" "δCORA" "δGTSAM" "δCORA" "GTSAM" "CORA"
echo "----------------------------------------------------------------------------------------------------"

# Print results for each mission (sorted)
for mission_data in "${missions[@]}"; do
    IFS=':' read -r name bag cutoff <<< "$mission_data"
    
    gtsam_no="${gtsam_no_prior[$name]}"
    cora_no="${cora_no_prior[$name]}"
    gtsam_yes="${gtsam_with_prior[$name]}"
    cora_yes="${cora_with_prior[$name]}"
    
    # Format values
    gtsam_no_str="${gtsam_no}m"
    cora_no_str="${cora_no}m"
    gtsam_yes_str="${gtsam_yes}m"
    cora_yes_str="${cora_yes}m"
    
    # Calculate change (positive = improvement)
    if [[ "$gtsam_no" != "N/A" && "$gtsam_yes" != "N/A" ]]; then
        gtsam_change=$(echo "$gtsam_no - $gtsam_yes" | bc)
        gtsam_change_str=$(printf "%+.2fm" "$gtsam_change")
    else
        gtsam_change_str="N/A"
    fi
    
    if [[ "$cora_no" != "N/A" && "$cora_yes" != "N/A" ]]; then
        cora_change=$(echo "$cora_no - $cora_yes" | bc)
        cora_change_str=$(printf "%+.2fm" "$cora_change")
    else
        cora_change_str="N/A"
    fi
    
    printf "%-15s %-15s %-15s %-15s %-15s %-7s %-7s\n" \
        "$name" "$gtsam_no_str" "$cora_no_str" "$gtsam_yes_str" "$cora_yes_str" "$gtsam_change_str" "$cora_change_str"
done

echo "----------------------------------------------------------------------------------------------------"
echo ""
echo "Notes:"
echo "  • δGTSAM/δCORA: XY distance from final optimized pose to surface GPS fix"
echo "  • Change: Positive = improvement (WITH priors is better), Negative = degradation"
echo "  • WITHOUT priors → tests/results/<mission>_offline/"
echo "  • WITH priors → tests/results_with_priors/<mission>_offline/"
echo "===================================================================================================="
echo ""

echo "================================================================"
echo "ALL MISSIONS COMPLETE"
echo "================================================================"
