# Offline Batch Estimator Walk Through

**Date**: March 3, 2026  
**Script**: `tests/offline_optimizer.py`  
**Purpose**: Batch optimization of underwater vehicle trajectories using GTSAM and CORA
**Commands & Timestamps**: 
    # Basic usage (processes entire bag)
    python3 tests/offline_optimizer.py tests/pink_solo.bag

    # With cutoff timestamp (recommended - excludes ascent data)
    python3 tests/offline_optimizer.py tests/pink_solo.bag --cutoff-time 1764048438.567943

    # Other validated missions:
    python3 tests/offline_optimizer.py tests/past_shep.bag --cutoff-time 1764043197.378492
    python3 tests/offline_optimizer.py tests/lush_erle.bag --cutoff-time 1764049657.220171
    python3 tests/offline_optimizer.py tests/sour_axle.bag --cutoff-time 1764040146.629958
    python3 tests/offline_optimizer.py tests/damp_beer.bag --cutoff-time 1764041947.780546

---

## Overview

This document explains the methodology used in the offline batch optimizer for processing mission data. The optimizer runs two different estimators (GTSAM and CORA) on the same data and compares their performance against GPS ground truth.

**NOTE**: GPS fixes are **NOT** added to the factor graphs during optimization. They are **ONLY** used to evaluate the final optimized trajectory by comparing the last pose to the surface GPS fix.

---

## Data Sources

### Input Data from ROS Bag:
1. **`/actor_0/integrated_state`** - Absolute world-frame poses from IMU integration
   - Used for pose initialization
   - Used to compute relative odometry measurements
   - Contains position (x, y, z) and orientation (quaternion)
   
2. **`/actor_0/range_factor`** - Acoustic range measurements to landmarks
   - Ranges between vehicle poses and landmarks (e.g., A42 ↔ L0)
   - Each measurement has range value and uncertainty (sigma)
   
3. **`/actor_0/gps`** - GPS fixes (surface only)
   - **NOT added to factor graph**
   - **ONLY used for ground truth comparison**
   - Used to identify start/surface positions and calculate final error

### Data Preprocessing:
- **Cutoff time filtering**: Excludes ascent odometry (can have 97m jumps)
- **Key remapping**: Bag uses A0, A2, A4... → remapped to A0, A1, A2... for sequential indexing
- **GPS selection**: Start GPS before dive, Surface GPS after surfacing

---

## GTSAM Levenberg-Marquardt Optimization

### Algorithm
- **Optimizer**: Levenberg-Marquardt (second-order, blends Gauss-Newton + trust region)
- **Framework**: Direct nonlinear optimization on SE(3) manifold
- **Convergence**: Iterative refinement until error reduction < threshold

### Initialization

#### Pose Initialization:
- **Method**: All poses initialized from `integrated_state` messages
- **Frame**: World-frame absolute poses (not relative)
- **Count**: Typically 100-300 poses per mission (e.g., 272 poses for pink_solo)
- **Keys**: Sequential A0, A1, A2, A3, ... A271
- **Data**:
  ```python
  pose = Pose3(
      rotation=Rot3.Quaternion(qw, qx, qy, qz),  # From integrated_state
      translation=[x, y, z]                      # From integrated_state
  )
  ```

#### Landmark Initialization:
- **Method**: Known world positions from survey
- **Count**: 2 landmarks (L0, L1)
- **Positions**:
  - L0: (-74.52, -38.93, 1.5) meters
  - L1: (66.52, 25.97, 1.5) meters
- **Frame**: Same world frame as poses

### Factor Graph Construction

#### 1. Prior Factor on First Pose (1 factor)
- **Purpose**: Anchors the optimization, prevents drift
- **Applied to**: Pose A0 (first pose)
- **Noise Model**:
  ```python
  position_sigma = 1e-6 m      # Very tight (effectively fixed)
  orientation_sigma = 1e-8 rad  # Extremely tight (prevents rotation drift)
  ```
- **Rationale**: Morrison's advice - "try to pin the initial orientation" because range measurements don't constrain rotation

#### 2. Prior Factors on Landmarks (2 factors)
- **Purpose**: Anchors landmarks at surveyed positions
- **Applied to**: L0 and L1
- **Noise Model**:
  ```python
  position_sigma = 0.1 m       # Tight (±10cm survey accuracy)
  orientation_sigma = 10.0 rad  # Loose (orientation doesn't matter for landmarks)
  ```

#### 3. Between-Factors / Odometry Constraints (271 factors for 272 poses)
- **Purpose**: Encode relative motion between consecutive poses
- **Computed from**: Consecutive `integrated_state` poses
- **Formula**: `rel_pose = pose1.between(pose2)`  (T_rel = T₁⁻¹ × T₂)
- **Noise Model** (GTSAM-specific tuning):
  ```python
  translation_sigma = 2.0 m       # Moderate confidence
  rotation_sigma = 5.0 rad        # Loose confidence (~286°)
  ```
- **Rationale**: Morrison's recommendation to "weaken noise models" prevents overfitting to noisy odometry. Achieves **3.25m error** with these loose constraints.

#### 4. Range Factors (hundreds of factors)
- **Purpose**: Constrain distance between poses and landmarks
- **Example**: A42 ↔ L0 with measured range 85.3m ± 2.0m
- **Noise Model**: `sigma = range_sigma` from bag (typically 2-5m)
- **Added for**: All acoustic measurements in the bag

### Optimization Process
1. Build factor graph with all factors
2. Initialize all variables (poses + landmarks)
3. Run Levenberg-Marquardt optimizer
   - Max iterations: 100
   - Convergence thresholds: relative error < 1e-5, absolute error < 1e-5
4. Extract optimized poses

### Key Characteristics
- **World-frame optimization**: First pose prior anchors everything in world coordinates
- **Moderate constraints**: σ=2m translation allows flexibility to balance odometry and ranges
- **Excellent performance**: 3-3.5m XY error on well-constrained missions

---

## CORA Batch Optimization (Convex Relaxation)

### Algorithm
- **Optimizer**: TNT (Trust-region Newton on Riemannian manifold)
- **Framework**: Convex relaxation on Stiefel manifold + certification
- **Approach**: Optimizes in arbitrary frame, then aligns to world frame
- **Robustness**: Can handle random initialization (doesn't need good initial guess)

### Initialization

#### Pose Initialization:
- **Method**: Initialized from `integrated_state` messages (same as GTSAM)
- **Frame**: World-frame absolute poses
- **Count**: Same as GTSAM (e.g., 272 poses)
- **Keys**: Sequential A0, A1, A2, ... A271
- **Data**:
  ```python
  rotation_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()
  translation = [x, y, z]
  initial_values.set_pose(symbol, rotation_matrix, translation)
  ```
- **Note**: CORA could also use random initialization (Morrison's approach), but we use odometry for fair comparison with GTSAM

#### Landmark Initialization:
- **Method**: Known world positions (same as GTSAM)
- **Count**: 2 landmarks (L0, L1)
- **Positions**: Same as GTSAM
- **Data**:
  ```python
  initial_values.set_landmark(symbol, position)
  ```

### Problem Construction

#### 1. NO Prior Factors Used (By Design Choice in offline_optimizer.py)
- **Implementation Choice**: This offline batch script does NOT add pose or landmark priors to CORA
- **Why**: CORA's convex relaxation is robust to initialization and doesn't require anchoring (Morrison's paper demonstrates this)
- **Note**: CORA DOES support priors via `addPosePrior()` and `addLandmarkPrior()`, but they're optional
- **Morrison's ROS implementation**: 
  - Main branch `cora_estimator.py` DOES implement `add_pose_prior()` (lines 197-214)
  - Does NOT implement `add_landmark_prior()` (interface doesn't have this method)
  - Only poses get priors in Morrison's system, landmarks are initialized from range measurements
- **Result**: Without priors, CORA optimizes in arbitrary coordinate frame
- **Post-processing**: Alignment required to transform to world coordinates

#### 2. Relative Pose Measurements (271 measurements)
- **Purpose**: Encode relative motion between consecutive poses
- **Computed from**: Consecutive `integrated_state` poses (same as GTSAM between-factors)
- **Formula**: 
  ```python
  R_rel = R₁ᵀ @ R₂           # Relative rotation
  t_rel = R₁ᵀ @ (t₂ - t₁)    # Relative translation
  ```
- **Noise Model** (CORA-specific tuning):
  ```python
  translation_sigma = 0.316 m    # Very tight (0.1 m² variance)
  rotation_sigma = 0.1 rad       # Very tight (~5.7°, 0.01 rad² variance)
  ```
- **Rationale**: CORA's convex relaxation + coordinate alignment pipeline requires much tighter constraints than GTSAM. Achieves **13.20m error** with these tight constraints.

#### 3. Range Measurements (hundreds of measurements)
- **Purpose**: Same as GTSAM - constrain distance between poses and landmarks
- **Data**: Same measurements as GTSAM
- **Noise**: Variance (sigma²) instead of sigma
- **Added for**: All acoustic measurements in the bag

### Optimization Process
1. Add pose variables (no priors!)
2. Add landmark variables (no priors!)
3. Add relative pose measurements
4. Add range measurements
5. Update problem data (builds matrices)
6. Solve with TNT + LOBPCG certification
7. Extract solution (in arbitrary frame)
8. Align solution to world frame (post-processing)

### Key Characteristics
- **Arbitrary-frame optimization**: No pose prior, optimizes freely
- **Tight constraints**: σ=0.316m translation prevents alignment instability
- **Good performance**: 11-13m XY error on well-constrained missions (3-4× worse than GTSAM)
- **Struggles with**: Short missions, poor geometry, long GPS gaps

---

## GPS Ground Truth Comparison

### ⚠️ GPS NOT in Factor Graph

**CRITICAL**: GPS fixes are **NEVER** added to the optimization factor graph. They serve **ONLY** as ground truth for evaluation.

### GPS Usage:
1. **Start GPS**: Identified as last GPS before dive (for visualization)
2. **Surface GPS**: Identified as first GPS after surfacing (for ground truth)
3. **Error Calculation**: Distance from last optimized pose to surface GPS
   ```python
   error_xy = sqrt((pose_x - gps_x)² + (pose_y - gps_y)²)
   error_3d = sqrt((pose_x - gps_x)² + (pose_y - gps_y)² + (pose_z - gps_z)²)
   ```

### Why GPS is Not Used in Optimization:
1. **Sparse measurements**: GPS only available at surface (start/end), not underwater
2. **Different reference frame**: GPS might be in different coordinate system
3. **Unknown offset**: Time delay between last pose and surface GPS
4. **Clean test**: Evaluates pure range-aided SLAM without GPS aiding

---

## Noise Model Differences: GTSAM vs CORA

### Why Different Noise Models?

The two optimizers have fundamentally different sensitivities to constraint tightness:

| Property | GTSAM (LM) | CORA (TNT) |
|----------|------------|------------|
| **Optimal σ_trans** | 2.0 m | 0.316 m |
| **Optimal σ_rot** | 5.0 rad | 0.1 rad |
| **Constraint philosophy** | Loose (prevents overfitting) | Tight (prevents misalignment) |
| **Morrison's advice applies?** | ✅ YES | ❌ NO |
| **Best performance** | 3.25m XY error | 13.20m XY error |

### GTSAM (Loose Constraints):
- **Why loose works**: LM optimizer balances odometry and ranges naturally
- **Benefit**: Can escape local minima from noisy odometry
- **Result**: 3-3.5m XY error on well-constrained missions (excellent!)

### CORA (Tight Constraints):
- **Why tight works**: Convex relaxation + coordinate alignment needs stability
- **Multi-step process**: Optimize → project → align → transform to world
- **Instability with loose**: σ=2m/5rad → 49m error (alignment fails)
- **Result**: 11-13m XY error on well-constrained missions (good, but 3-4× worse)

See `NOISE_MODEL_FINDINGS.md` for detailed experimental results across 5 missions.

---

## Summary

### Factor Graph Components:

| Component | GTSAM | CORA |
|-----------|-------|------|
| **Pose initialization** | From integrated_state | From integrated_state |
| **Landmark initialization** | Known positions | Known positions |
| **First pose prior** | ✅ YES (very tight) | ❌ NOT USED* |
| **Landmark priors** | ✅ YES (tight position) | ❌ NOT USED* |
| **Between-factors / Odometry** | ✅ YES (loose σ=2m, 5rad) | ✅ YES** (tight σ=0.316m, 0.1rad) |
| **Range factors** | ✅ YES (σ from bag) | ✅ YES (σ² from bag) |
| **GPS factors** | ❌ NO | ❌ NO |

*CORA supports priors via `addPosePrior()` and `addLandmarkPrior()`. Morrison's ROS implementation (main branch) DOES use `add_pose_prior()` but NOT `add_landmark_prior()`. This offline batch script currently uses neither to demonstrate CORA's robustness without anchoring, but further testing will be done, adding the pose priors to investiagte if CORAs results will be more comparable to GTSAM.

**CORA calls them "relative pose measurements" instead of "between-factors", but they're mathematically equivalent.

### GPS Role:
- ❌ NOT added to factor graph during optimization
- ✅ ONLY used for ground truth comparison after optimization
- ✅ Used to calculate final XY error metric

### Performance Summary:
- **GTSAM**: 3-3.5m error on good missions (excellent ✨)
- **CORA**: 11-13m error on good missions (good, but 3-4× worse)
- **Both struggle**: Short missions, poor geometry, long GPS gaps (18-180m errors)

### Key Insight:
Different optimization algorithms require different noise model tuning. Morrison's advice to "weaken noise models" applies specifically to GTSAM's Levenberg-Marquardt, NOT to CORA's convex relaxation approach.

---

## Example Command

```bash
# Run with cutoff timestamp to exclude ascent data
python3 tests/offline_optimizer.py tests/pink_solo.bag --cutoff-time 1764048438.567943
```

Results saved to: `tests/results/pink_solo_offline/`
- `gtsam_trajectory.csv` - GTSAM optimized poses
- `cora_trajectory.csv` - CORA optimized poses
- `odometry.csv` - Raw integrated_state trajectory
- `ranges.csv` - All range measurements
- `gps.csv` - GPS fixes (for ground truth only)
- `trajectory_plot.png` - Visualization comparing all trajectories
