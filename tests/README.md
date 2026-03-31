# Fall 2025 Mission Data Processing

This directory contains tools for processing Fall 2025 single agent mission data with CORA and GTSAM estimators.

## Mission List

- **sour_axle** - 251124-2156F-SOUR-AXEL
- **damp_beer** - 251124-2231S-DAMP-BEER  
- **past_shep** - 251124-2244S-PAST-SHEP
- **pink_solo** - 251125-0010Q-PINK-SOLO
- **lush_erle** - 251125-0035J-LUSH-ERLE

## Offline Batch Optimization (Recommended)

**For post-mission analysis with validated noise models**, use the offline optimizer:

```bash
# Basic usage (processes entire bag)
python3 tests/offline_optimizer.py tests/pink_solo.bag

# With cutoff timestamp (exclude ascent data)
python3 tests/offline_optimizer.py tests/pink_solo.bag --cutoff-time 1764048438.567943
```

**What it does:**
- Runs both GTSAM and CORA batch optimizers on the complete dataset
- Uses validated noise models from extensive testing
- Automatically detects GPS start/surface fixes
- Generates trajectory plots comparing odometry, GTSAM, and CORA
- Saves results to `tests/results/<mission>_offline/`

**Key advantages:**
- **Validated performance**: Noise models tuned across 5 missions (see `NOISE_MODEL_FINDINGS.md`)
- **Proper data preprocessing**: Excludes ascent odometry that can have large jumps
- **Ground truth comparison**: Measures distance to surface GPS fix
- **Batch optimization**: Both algorithms see all data at once (not incremental)

**Results Summary:**
| Mission | Duration | GTSAM Error | CORA Error | Notes |
|---------|----------|-------------|------------|-------|
| pink_solo | 17.1min | 3.25m | 13.20m | Baseline |
| past_shep | 14.9min | 3.36m | 11.65m | Best CORA |
| lush_erle | 11.3min | 3.45m | 31.53m | CORA outlier |
| sour_axle | 11.2min | 18.42m | 179.47m | Long GPS gap |
| damp_beer | 5.9min | 18.19m | 108.75m | Short mission |

See `NOISE_MODEL_FINDINGS.md` for detailed analysis.

---

## Directory Structure

```
tests/
├── offline_optimizer.py           # Batch optimizer with validated noise models (RECOMMENDED)
├── copy_mission_bags.sh          # Helper script to copy bags
├── process_mission.launch         # Launch file for online/real-time processing
├── README.md                      # This file
├── validate_pose_factor.py       # Historical: IMU data validation script
├── compare_cv7_logs.py           # Historical: CV7 log comparison script
├── Fall 2025 Single Agent Data/  # Raw mission data
├── results/                       # Generated results (created automatically)
│   ├── sour_axle_offline/        # Offline optimizer results
│   ├── damp_beer_offline/
│   ├── past_shep_offline/
│   ├── pink_solo_offline/
│   ├── lush_erle_offline/
│   ├── sour_axle/                # Online processor results (if used)
│   └── ...
└── *.bag                         # Copied mission bags
```

## Notes

### Offline Optimizer
- Processes complete mission data in batch mode
- Uses validated noise models: GTSAM (σ=2m, 5rad), CORA (σ=0.316m, 0.1rad)
- Automatically handles GPS selection and data trimming
- Recommended for final analysis and results

### Online Processor
- Estimators run in real-time as the bag plays back
- Bag playback rate can be adjusted: `roslaunch process_mission.launch mission_name:=pink_solo playback_rate:=10.0`
- For faster processing, increase the rate (default is 10.0x realtime)
- The mission processor automatically generates plots on shutdown (Ctrl+C)
- Useful for testing incremental/online algorithms

## Online Processing (Real-time) - NOT WORKING YET

**For real-time processing during mission playback**, use the ROS launch file. This runs estimators incrementally as data arrives, simulating online operation.

### 1. Copy Mission Bags

Copy all mission bags with standardized names:
```bash
cd /home/lizgajski2/catkin_ws/tests
./copy_mission_bags.sh all
```

Or copy a single mission:
```bash
./copy_mission_bags.sh sour_axle
```

### 2. Process a Mission (Online)

After copying the bags, process a mission with both GTSAM and CORA estimators in real-time:

```bash
# Source your workspace
source /home/lizgajski2/catkin_ws/devel/setup.bash

# Run the processor
roslaunch /home/lizgajski2/catkin_ws/tests/process_mission.launch mission_name:=sour_axle
```

**Note**: The rosbag will start paused. Press **SPACE** to begin playback.

**Online vs Offline:**
- **Online (process_mission.launch)**: Estimators update incrementally as data arrives, simulating real-time operation during a mission
- **Offline (offline_optimizer.py)**: Batch optimization with all data available, using validated noise models for best accuracy

### 3. View Results

Results are saved to: `/home/lizgajski2/catkin_ws/tests/results/<mission_name>/`

Each mission will generate:
- **<mission>_comparison.png** - Comparison plots (2D XY, XZ, YZ, and 3D)
- **<mission>_odometry.csv** - Raw odometry trajectory
- **<mission>_gtsam.csv** - GTSAM optimized poses
- **<mission>_cora.csv** - CORA optimized poses
- **<mission>_gps.csv** - GPS fixes
- **<mission>_ranges.csv** - Range measurements

## Process All Missions

To process all missions sequentially:

```bash
cd /home/lizgajski2/catkin_ws/tests

# Copy all bags
./copy_mission_bags.sh all

# Process each mission
for mission in sour_axle damp_beer past_shep pink_solo lush_erle; do
    echo "Processing $mission..."
    roslaunch process_mission.launch mission_name:=$mission
    sleep 2
done
```

## Plot Contents

Each comparison plot includes:

1. **Odometry trajectory** (black) - Raw dead-reckoning from IMU/DVL
2. **GTSAM optimized** (blue) - Factor graph optimization with Levenberg-Marquardt
3. **CORA optimized** (red) - Certifiably correct range-aided SLAM
4. **GPS Start** (cyan triangle) - Dive entry position
5. **GPS End** (magenta triangle) - Surface position
6. **Landmarks** (red stars) - Acoustic beacon positions with range uncertainty circles

## Troubleshooting

### Mission processor not found
The mission processor needs to be built:
```bash
cd /home/lizgajski2/catkin_ws
catkin build spurdog_acomms
source devel/setup.bash
```

### No data in plots
- Check that the rosbag topics match expected names (`/actor_0/range_factor`, `/actor_0/pose_factor`, etc.)
- Verify bag playback with: `rostopic echo /actor_0/range_factor`
- Check ROS logs: `~/.ros/log/latest/`

### Estimator errors
- CORA requires the CORA C++ library to be built. Check `/home/lizgajski2/catkin_ws/src/multi-spurdog/cora/build/`
- GTSAM requires proper GTSAM installation (included with ROS noetic)

