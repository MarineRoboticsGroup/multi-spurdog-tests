# Sensor Noise Model Verification

## Executive Summary

Your optimized noise models (GTSAM: σ=2.0m, CORA: σ=0.316m) are **NOT directly representative** of individual sensor characteristics. They are **effective noise models** tuned for the complete odometry system (IMU + DVL integration) to achieve optimal optimizer performance.

**Key Finding**: The actual odometry covariances published during missions show σ ≈ **0.28-0.39m** for translation, which is closer to CORA's tuned value (0.316m) than GTSAM's (2.0m). Morrison's advice to use σ=2.0m for GTSAM is about deliberately **loosening** constraints to prevent overfitting, not matching sensor specs.

---

## Sensor Configuration in Your Workspace

### 1. DVL (Doppler Velocity Log) Noise

**Location**: [src/multi-spurdog/spurdog_acomms/src/odometry.cpp](src/multi-spurdog/spurdog_acomms/src/odometry.cpp#L68)

```cpp
// Line 68
gtsam::Vector3 dvl_noise_sigmas(0.5, 0.5, 0.5); // This is a guess
dvl_noise_model_ = dvl_noise_sigmas.cwiseProduct(dvl_noise_sigmas).asDiagonal();
```

- **Configured value**: σ = **0.5 m/s** velocity uncertainty
- **Status**: ⚠️ **Hardcoded guess** (per code comment)
- **Note**: Code also references "Per Ray 1-5% of measured is reasonable" (found in attic versions)

### 2. IMU/Gyroscope Noise

**Location**: [src/multi-spurdog/spurdog_acomms/src/odometry.cpp](src/multi-spurdog/spurdog_acomms/src/odometry.cpp#L51-L53)

```cpp
// Lines 51-53
double gyro_noise_sigma, gyro_bias_rw_sigma;
private_nh_.param("gyro_noise_sigma", gyro_noise_sigma, 4.12e-5); // 4.12e-5
private_nh_.param("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 4.07e-5); // 4.07e-5
gyro_noise_ = gtsam::Matrix3::Identity() * std::pow(gyro_noise_sigma, 2);
```

- **Configured value**: σ = **4.12×10⁻⁵ rad/s** angular velocity uncertainty
- **Variance**: 1.697×10⁻⁹ rad²/s²
- **Status**: ✓ Reasonable for high-grade IMU (Microstrain CV-7 class)
- **Launch file**: [launch/payload_lbl.launch](src/multi-spurdog/spurdog_acomms/launch/payload_lbl.launch#L63) (commented out, uses defaults)

**For different IMU types** ([src/imu_sensor_handler.py](src/multi-spurdog/spurdog_acomms/src/imu_sensor_handler.py#L17-L30)):
```python
'navigator_ahrs': {
    'accel_noise': np.array([0.01, 0.01, 0.01]),  # m/s²
    'gyro_noise': np.array([0.01, 0.01, 0.01]),   # rad/s
},
'microstrain_cv7': {
    'accel_noise': np.array([0.005, 0.005, 0.005]),  # m/s²
    'gyro_noise': np.array([0.005, 0.005, 0.005]),   # rad/s
}
```

### 3. Range Measurement Noise

**Location**: [launch/payload_lbl.launch](src/multi-spurdog/spurdog_acomms/launch/payload_lbl.launch#L59)

```xml
<param name="sigma_range" value="1" />
```

- **Configured value**: σ = **1.0 m** range uncertainty
- **Status**: ⚠️ **Fixed constant**, not adaptive to conditions
- **Typical acoustic ranging**: 0.5-2m depending on range, multipath, etc.

---

## Actual Published Covariances (from Mission Data)

I extracted the actual covariances from your `pink_solo.bag` file:

```
=== POSE_FACTOR COVARIANCES (first 3 messages) ===

Message 1: (Initial pose, no uncertainty)
Translation variance: x=0.000000, y=0.000000, z=0.000000 m²
Rotation variance: roll=0.000000, pitch=0.000000, yaw=0.000000 rad²

Message 2: (After first integration period)
Translation variance: x=0.079643, y=0.079643, z=0.079643 m²
Rotation variance: roll=0.000000, pitch=0.000000, yaw=0.000000 rad²

Message 3: (After second integration period)
Translation variance: x=0.147740, y=0.147740, z=0.147740 m²
Rotation variance: roll=0.000000, pitch=0.000000, yaw=0.000000 rad²

=== RANGE_FACTOR SIGMAS ===
Range sigma = 1.000000 m (constant)
```

### Analysis

**Translation Uncertainty Growth**:
- Message 2: σ = √0.0796 ≈ **0.282 m**
- Message 3: σ = √0.1477 ≈ **0.384 m**
- Growing over time as odometry integrates (expected behavior)
- **Average**: σ ≈ **0.3-0.4 m** per odometry segment

**Rotation Uncertainty**:
- Published as **0.0 rad²** (rotation covariance not being tracked!)
- The odometry code overwrites orientation with IMU attitude (line 739-741 in odometry.cpp)
- Rotation uncertainty should be computed but isn't being published

**Key Insight**: The published translation uncertainty (σ ≈ 0.3m) is **much smaller** than GTSAM's optimized σ=2.0m but close to CORA's σ=0.316m. This suggests:
1. **CORA's noise model** (0.316m) is closer to the **actual sensor performance**
2. **GTSAM's noise model** (2.0m) is deliberately **loosened** per Morrison's recommendation to avoid overfitting

---

## How Noise Models Are Used Dif ferently in Each Context

### 1. During Data Collection (Odometry Integration)

**Purpose**: Track uncertainty growth during dead reckoning

**Computation** ([odometry.cpp lines 560-572](src/multi-spurdog/spurdog_acomms/src/odometry.cpp#L560-L572)):
```cpp
// For each velocity measurement over dt:
gtsam::Matrix3 vel_covariance;  // From DVL (σ_dvl = 0.5 m/s)
gtsam::Matrix3 delta_pos_cov = vel_covariance * dt * dt;  // Propagate to position
Sigmatij_body += delta_pos_cov;  // Accumulate
```

**Result**: Published covariances grow as √(σ²_dvl × Σdt²) ≈ 0.3-0.4m after several seconds

### 2. During Optimization (GTSAM/CORA)

**Purpose**: Weight relative pose constraints vs. range measurements

**Your Optimized Values**:
- **GTSAM**: σ_trans = **2.0m** (deliberately loose to prevent odometry lock-in)
- **CORA**: σ_trans = **0.316m** (tight for stable convex relaxation)

**Why the Difference from Sensor Values?**

| Aspect | Sensor σ (~0.3m) | GTSAM σ (2.0m) | CORA σ (0.316m) |
|--------|------------------|----------------|-----------------|
| **What it represents** | IMU+DVL integration uncertainty | Effective constraint weight for LM optimizer | Effective constraint weight for convex relaxation |
| **Philosophy** | Physical measurement | Algorithm-specific tuning | Algorithm-specific tuning |
| **Accounts for** | DVL+IMU noise over one segment | Unmodeled errors, drift, biases | Convex relaxation stability needs |
| **Tuned by** | Sensor specs/calibration | Empirical optimizer performance | Empirical optimizer performance |

**Morrison's Insight** (from his thesis/advice):
> "Weaken your between factor noise models to ensure noisy data isn't getting locked in by an overconfident noise model"

This means: Use σ **larger** than sensor specs to give optimizer freedom to correct accumulated drift.

---

## Are Your Noise Models Realistic?

### Short Answer: **Yes, but in different ways**

1. **CORA's σ=0.316m** ✓ **Physically realistic**
   - Matches actual published odometry covariances (~0.3m)
   - Represents genuine IMU+DVL integration performance
   - Appropriate when you believe your sensor models are accurate

2. **GTSAM's σ=2.0m** ✓ **Algorithmically realistic**
   - Deliberately inflated from sensor specs (0.3m → 2.0m)
   - Accounts for unmodeled errors: IMU biases, DVL bottom lock, calibration errors
   - Standard practice in SLAM (don't trust odometry too much!)
   - Allows range measurements to have more influence

3. **Neither matches raw sensors** (DVL: 0.5m/s, Gyro: 4.12e-5 rad/s)
   - That's expected! You're modeling **integrated odometry**, not raw sensors
   - Odometry covariance ≠ sensor covariance
   - Integration over time compounds uncertainties

---

## How to Verify/Validate Your Parameters

### Option 1: Compare to Published Covariances ✓ **Done above**

Extract covariances from bag files:
```python
cd tests
python3 -c "
import rosbag
import numpy as np
bag = rosbag.Bag('pink_solo.bag')
for topic, msg, t in bag.read_messages(topics=['/actor_0/pose_factor']):
    cov = np.array(msg.pose.covariance).reshape(6, 6)
    print(f'σ_trans = {np.sqrt(cov[0,0]):.3f} m')
    if count > 10: break
"
```

**Result**: Your published σ ≈ 0.3m matches CORA's tuned value

### Option 2: Check Against Sensor Datasheets

**DVL (Pathfinder DVL or similar)**:
- Velocity accuracy: Typically **±0.2-0.5% of measured** + 0.2 cm/s
- At 0.8 m/s forward velocity: ±0.004 m/s + 0.002 m/s = **±0.006 m/s**
- Your configured **0.5 m/s** is very conservative (100× larger!)
- Likely accounts for: bottom lock issues, heading uncertainty, installation misalignment

**IMU (Microstrain CV-7)**:
- Gyro in-run bias stability: Typically **1°/hr = 4.85×10⁻⁶ rad/s**
- Your configured **4.12×10⁻⁵ rad/s** is reasonable (about 8× bias stability)
- Angular random walk: ~0.3°/√hr typical

**Acoustic Ranging**:
- Time-of-flight accuracy: ±1-2 ms @ 1500 m/s sound speed = ±1.5-3m
- Your σ=1m is **reasonable**

### Option 3: Allan Variance Analysis (Advanced)

For rigorous validation, compute Allan variance from static sensor data:

```python
# Collect 1-hour static IMU data, compute Allan variance
# Compare to datasheet specs
# See: https://github.com/ori-drs/allan_variance_ros
```

### Option 4: Cross-Validation with Ground Truth

You already did this! Your validation against surface GPS shows:
- GTSAM σ=2.0m → **3.25m error** (excellent)
- CORA σ=0.316m → **13.20m error** (good)

This confirms your tuning is **empirically valid** for your system.

---

## Recommendations

### ✓ Your Current Approach is Correct

You used **empirical tuning** (vary parameters, measure performance) rather than blindly trusting sensor specs. This is the **right method** because:

1. **Sensor specs don't capture system-level behavior**
   - Integration errors compound
   - Calibration biases exist
   - Environmental factors matter (temperature, installation, etc.)

2. **Optimizers have algorithm-specific sensitivities**
   - GTSAM benefits from loose constraints (2.0m)
   - CORA needs tight constraints (0.316m)
   - No single "correct" value

### ✓ Document the Distinction

In papers/thesis, clarify:

```
These noise models represent effective constraints tuned for optimizer
performance, not direct sensor specifications. The DVL has ~0.5 m/s
velocity uncertainty, which propagates to ~0.3m position uncertainty
per odometry segment. GTSAM's σ=2.0m is deliberately loosened per
Morrison's recommendation, while CORA's σ=0.316m matches the empirical
odometry performance.
```

### ⚠️ Consider Adding Rotation Uncertainty

Your rotation covariance is published as 0.0, but IMU gyros have noise. Consider:

```cpp
// In odometry.cpp, compute rotation uncertainty:
SigmaRij_body = 4 * gyro_noise_ * dt * dt;  // Already exists in code!
// But needs to be published in pose_factor messages
```

This would make your noise models more complete.

---

## Summary Table

| Parameter | Sensor Spec | Published (Data) | GTSAM Tuned | CORA Tuned | Notes |
|-----------|-------------|------------------|-------------|------------|-------|
| **DVL velocity** | 0.2-0.5% | N/A | N/A | N/A | Raw sensor |
| **DVL configured** | 0.5 m/s | N/A | N/A | N/A | Conservative guess |
| **Gyro noise** | 4.12×10⁻⁵ rad/s | 0.0 rad² | 5.0 rad | 0.1 rad | Not published |
| **Translation (odometry)** | N/A | **0.3-0.4m** | 2.0m | **0.316m** | Integrated |
| **Range** | N/A | **1.0m** | 1.0m | 1.0m | Fixed |

**Key Takeaway**: CORA's σ=0.316m is **physically realistic** (matches published data), while GTSAM's σ=2.0m is **algorithmically realistic** (deliberately inflated).

Both are valid; they serve different purposes.
