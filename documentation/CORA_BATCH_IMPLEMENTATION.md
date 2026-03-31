# CORA Batch Solver Implementation

## Overview

This document describes the clean, straightforward batch-solving implementation for CORA, correcting previous misunderstandings about how CORA should be used.

## Key Understandings

### CORA Design (from paper and code analysis)

1. **CORA is a BATCH solver**: Designed to solve the COMPLETE RA-SLAM problem ONCE with ALL data
2. **NOT incremental**: Should NOT be called repeatedly during data collection
3. **Random landmark initialization**: Landmarks should be initialized RANDOMLY (not from range estimates)
4. **Odometry-based pose initialization**: Poses can be initialized from odometry chain

### GTSAM Levenberg-Marquardt

1. **Also a BATCH optimizer**: LM blends Gauss-Newton with trust region
2. **Not iSAM2**: This codebase does NOT use iSAM2 (incremental algorithm)
3. **Can be used incrementally**: But solves the full graph each time

## Implementation Changes

### CoraEstimator Class

**Storage Phase (during mission)**:
```python
self._all_odometry_measurements: List[OdometryMeasurement] = []
self._all_range_measurements: List[RangeMeasurement] = []
self._all_pose_priors: List[Union[Pose2D, Pose3D]] = []
```

**Key Methods**:

1. `_specific_add_range()`: Stores range measurements (doesn't add to problem yet)
2. `_specific_add_odometry()`: Stores odometry measurements  
3. `update()`: Does NOTHING (CORA doesn't solve incrementally)
4. `solve_batch()`: **Main solving method** - called at end of mission

**Batch Solving Process** (`solve_batch()`):

```python
# Step 1: Build fresh problem with ALL measurements
self._rebuild_problem_from_scratch()

# Step 2: Create initial guess
#   - Poses: from odometry (current_estimate)
#   - Landmarks: RANDOM (per CORA paper!)
x0 = self._create_initial_guess()

# Step 3: Solve once with all data
(results, _) = cora.solveCORA(
    problem=self._problem,
    x0=x0,
    max_relaxation_rank=self._problem.getRelaxationRank(),
    verbose=True
)

# Step 4: Extract solution and update estimates
self._extract_solution()
```

### Initialization Strategy (from CORA paper experiments)

**Poses**: Initialize from odometry chain
```python
# Use odometry-based positions/orientations from current_estimate
for key, pose in self.current_estimate.pose_map.items():
    R = get_rotation_matrix_from_quat(pose.orientation)
    init_values.set_pose(key, R, pose.position)
```

**Landmarks**: Initialize RANDOMLY
```python
# Random in [-10, 10] range (per CORA paper experiments)
for key in self.current_estimate.point_map.keys():
    random_pos = np.random.rand(dimension) * 20 - 10
    init_values.set_landmark(key, random_pos)
```

This matches the CORA paper experiments code:
```cpp
// From cora/examples/paper_experiments.cpp line 487
x0.block(landmark_start_idx, 0, 1, dim) = CORA::Matrix::Random(1, dim) * 10;
```

### Workflow

**During Mission** (data collection):
- Odometry arrives → stored in `_all_odometry_measurements`
- Ranges arrive → stored in `_all_range_measurements`  
- `update()` is called → does nothing (batch solver)
- Poses/landmarks initialized for GTSAM compatibility → tracked in `current_estimate`

**At End of Mission** (solving):
- `solve_batch()` called from shutdown hook
- Rebuild complete problem from stored data
- Create initial guess (odometry poses + random landmarks)
- Solve once with CORA
- Extract optimized poses/landmarks
- Publish final results

## Testing

### Compare GTSAM LM vs CORA

Both are batch optimizers, but:
- **GTSAM LM**: Can solve incrementally (each update solves full graph)
- **CORA**: Designed for single batch solve with complete data

To compare:
1. Run mission with both estimators
2. GTSAM will update throughout (incremental batch solves)
3. CORA will solve once at end (true batch)
4. Compare final trajectories

### Expected Results

- **Number of poses**: Should match between GTSAM and CORA
- **Landmarks**: May differ initially (GTSAM uses range-based, CORA uses random)
- **Convergence**: CORA should converge to similar result as GTSAM (both are batch optimizers)

## Files Modified

1. `/home/lizgajski2/catkin_ws/src/multi-spurdog/spurdog_acomms/src/estimator/cora_estimator.py`
   - Complete rewrite for batch solving
   - Added `solve_batch()` method
   - Storage-based measurement tracking

2. `/home/lizgajski2/catkin_ws/src/multi-spurdog/spurdog_acomms/src/estimator/estimator_manager.py`
   - Added `solve_batch()` forwarding method

3. `/home/lizgajski2/catkin_ws/src/multi-spurdog/spurdog_acomms/src/estimator/ros/manager.py`
   - Added `solve_batch()` ROS wrapper

4. `/home/lizgajski2/catkin_ws/src/multi-spurdog/spurdog_acomms/src/estimator_node.py`
   - Added shutdown hook to call `solve_batch()` for CORA
   - Publishes final optimized poses after batch solve

## Usage

### Launch Files

No changes needed - existing launch files work:
```bash
roslaunch spurdog_acomms estimator_node.launch mode:=CORA
```

CORA will:
1. Collect data silently during mission
2. Solve batch problem at shutdown
3. Publish final optimized poses

### Programmatic Usage

```python
# Create CORA estimator
estimator = CoraEstimator(EstimatorMode.CORA, dimension=3)

# Add measurements during mission
estimator.add_odometry(odom_measurement)
estimator.add_range(range_measurement)

# At end, solve batch
success = estimator.solve_batch()

# Extract results
for key in estimator.current_estimate.pose_map:
    pose = estimator.get_pose_from_estimator(key)
    print(f"{key}: {pose.position}")
```

## References

- CORA Paper: Section on RA-SLAM batch optimization
- CORA Experiments: `cora/examples/paper_experiments.cpp` (getOdomInitialization, line 487)
- GTSAM: Levenberg-Marquardt batch optimizer
