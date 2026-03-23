#!/bin/bash
# Script to copy mission rosbags to tests directory with mission names
#
# Usage: ./copy_mission_bags.sh [mission_name]
#   If no mission_name is provided, all missions will be copied
#
# Available missions:
#   sour_axle, damp_beer, past_shep, pink_solo, lush_erle, all

TESTS_DIR="/home/lizgajski2/catkin_ws/tests"
DATA_DIR="$TESTS_DIR/Fall 2025 Single Agent Data"

# Mission mapping: mission_name -> (folder_name, bag_filename)
declare -A MISSIONS=(
    ["sour_axle"]="251124-2156F-SOUR-AXEL"
    ["damp_beer"]="251124-2231S-DAMP-BEER"
    ["past_shep"]="251124-2244S-PAST-SHEP"
    ["pink_solo"]="251125-0010Q-PINK-SOLO"
    ["lush_erle"]="251125-0035J-LUSH-ERLE"
)

copy_mission() {
    local mission_name=$1
    local folder_name=${MISSIONS[$mission_name]}
    
    if [ -z "$folder_name" ]; then
        echo "Error: Unknown mission name '$mission_name'"
        echo "Available missions: ${!MISSIONS[@]}"
        return 1
    fi
    
    # Find the LOG_SEASCOUT directory
    local mission_dir="$DATA_DIR/$folder_name"
    local log_dir=$(find "$mission_dir" -type d -name "LOG_SEASCOUT*" | grep -v "_tmp" | head -1)
    
    if [ -z "$log_dir" ]; then
        echo "Error: Could not find LOG_SEASCOUT directory in $mission_dir"
        return 1
    fi
    
    # Find the .bag file
    local bag_file=$(find "$log_dir" -name "*.bag" | head -1)
    
    if [ -z "$bag_file" ]; then
        echo "Error: Could not find .bag file in $log_dir"
        return 1
    fi
    
    # Copy and rename
    local dest_file="$TESTS_DIR/${mission_name}.bag"
    
    echo "Copying: $bag_file"
    echo "     To: $dest_file"
    
    cp "$bag_file" "$dest_file"
    
    if [ $? -eq 0 ]; then
        echo "✓ Successfully copied $mission_name"
        # Print bag info
        echo "  Bag size: $(du -h "$dest_file" | cut -f1)"
        return 0
    else
        echo "✗ Failed to copy $mission_name"
        return 1
    fi
}

# Main script
if [ $# -eq 0 ] || [ "$1" == "all" ]; then
    echo "Copying all mission bags..."
    echo "============================="
    for mission in "${!MISSIONS[@]}"; do
        copy_mission "$mission"
        echo ""
    done
else
    copy_mission "$1"
fi

echo ""
echo "Done! You can now run the estimator with:"
echo "  roslaunch $TESTS_DIR/process_mission.launch mission_name:=<name>"
echo ""
echo "Available missions: ${!MISSIONS[@]}"
