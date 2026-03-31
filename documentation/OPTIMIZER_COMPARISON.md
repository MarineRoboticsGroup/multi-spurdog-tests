# GTSAM vs CORA: Optimizer Comparison

## DEPRECATION NOTICE

**This document is largely superseded by `NOISE_MODEL_FINDINGS.md`**, which contains validated results from 5 missions with properly tuned noise models.

This file is kept for historical reference to document the evolution of understanding about optimizer-specific tuning requirements.

---

## Summary
**OPTIMIZER-SPECIFIC TUNING** - Different algorithms require different noise models for optimal performance.

**Key Finding**: Morrison's advice to "weaken noise models" applies to **GTSAM's Levenberg-Marquardt** but **NOT to CORA's convex relaxation**. Each optimizer must be tuned independently.

**Validated Configuration (see NOISE_MODEL_FINDINGS.md for details):**
- **GTSAM**: σ_trans = 2.0m, σ_rot = 5.0 rad (moderate/loose) → **3.25m XY error** ✨
- **CORA**: σ_trans = 0.316m, σ_rot = 0.1 rad (tight) → **13.20m XY error** ✨

**For current results and detailed analysis, see `NOISE_MODEL_FINDINGS.md`.**

---

## 1. Initialization Strategy
**IDENTICAL** - Both use the same initialization approach

| Aspect | GTSAM | CORA |
|--------|-------|------|
| **Pose Initialization** | From `integrated_state` (absolute world-frame poses) | From `integrated_state` (absolute world-frame poses) |
| **Number of Poses** | 272 poses (A0, A2, A4, ..., A542) | 272 poses (A0, A2, A4, ..., A542) |
| **Landmark Initialization** | Known world positions: L0=(-74.52, -38.93, 1.5), L1=(66.52, 25.97, 1.5) | Known world positions: L0=(-74.52, -38.93, 1.5), L1=(66.52, 25.97, 1.5) |
| **First Pose Location** | (74.5, -191.3, -1.0) | (74.5, -191.3, -1.0) |

---

## 2. Between-Factor Creation
**IDENTICAL** - Both compute relative poses the same way

| Aspect | GTSAM | CORA |
|--------|-------|------|
| **Method** | `pose1.between(pose2)` | Manual: `R_rel = R1.T @ R2`, `t_rel = R1.T @ (t2 - t1)` |
| **Result** | Relative transformation T_rel = T1^{-1} * T2 | Relative transformation T_rel = T1^{-1} * T2 |
| **Number** | 271 between-factors | 271 relative pose measurements |
| **Source** | Consecutive `integrated_state` poses | Consecutive `integrated_state` poses |

**Note**: GTSAM uses built-in `between()` method, CORA computes manually, but both produce mathematically identical relative poses.

---

## 3. Noise Models
**OPTIMIZER-SPECIFIC** - Different values tuned for each algorithm

### Between-Factors (Odometry)

**GTSAM (Levenberg-Marquardt):**
```python
# Moderate/loose constraints work best for GTSAM
translation_std = 2.0   # 2m standard deviation (4 m² variance)
rotation_std = 5.0      # 5 rad standard deviation (~286°, 25 rad² variance)
```
- **Rationale**: Morrison's recommendation to "weaken noise models" prevents overfitting
- **Performance**: 3.25m XY error (excellent!)
- **Why it works**: LM optimizer can escape local minima, balances odometry and ranges

**CORA (Riemannian Optimization):**
```python
# UPDATED: Tightest constraints work best for CORA
translation_std = 0.316m  # 0.316m standard deviation (0.1 m² variance)
rotation_std = 0.1        # 0.1 rad standard deviation (~5.7°, 0.01 rad² variance)
```
- **Rationale**: Convex relaxation requires very tight constraints for stability
- **Performance**: 13.20m XY error (validated across multiple missions)
- **Why tightest**: `alignEstimateToOrigin()` + SE(3) transform unstable with loose constraints

### Key Difference Explanation

| Optimizer | Optimal σ_trans | Optimal σ_rot | Why? |
|-----------|-----------------|---------------|------|
| **GTSAM** | 2.0m | 5.0 rad | Direct SE(3) optimization, anchored by prior, benefits from freedom |
| **CORA** | 0.316m | 0.1 rad | Convex relaxation → arbitrary frame → alignment; needs very tight constraints |

**Critical Insight**: CORA optimizes in an arbitrary coordinate frame (no pose prior), then uses `alignEstimateToOrigin()` to transform to a canonical frame, then applies manual SE(3) transform to world frame. This multi-step coordinate transformation pipeline becomes unstable with loose constraints.

### Experimental Evidence (from NOISE_MODEL_FINDINGS.md)

| Noise Model | GTSAM Result | CORA Result |
|-------------|--------------|-------------|
| σ=0.316m, 0.1rad (very tight) | 18.90m | 13.20m |
| σ=2.0m, 5.0rad (loose) | 3.25m | 49.12m |

**Conclusion**: Morrison's advice applies to GTSAM, not CORA. Each optimizer needs algorithm-specific tuning. See `NOISE_MODEL_FINDINGS.md` for comprehensive validation across 5 missions.

### Prior Factors

**GTSAM - First Pose Prior:**
```python
prior_sigmas = np.array([1e-6, 1e-6, 1e-6,   # x, y, z position (meters)
                         1e-8, 1e-8, 1e-8])  # roll, pitch, yaw orientation (radians)
```
- Very tight constraint on first pose (near-zero uncertainty)

**GTSAM - Landmark Priors:**
```python
landmark_sigmas = np.array([0.1, 0.1, 0.1,      # tight position
                            10.0, 10.0, 10.0])  # loose orientation
```

**CORA:**
- **NO pose prior** on first pose
- **NO landmark priors**
- Optimizes in arbitrary coordinate frame, then uses `alignEstimateToOrigin()` + SE(3) transform

**DIFFERENT** - GTSAM anchors first pose and landmarks, CORA optimizes freely then aligns

---

## 4. Optimization Algorithm
**EQUIVALENT** - Both use similar second-order methods

| Aspect | GTSAM | CORA |
|--------|-------|------|
| **Algorithm** | Levenberg-Marquardt | Trust-region Newton (TNT) |
| **Order** | Second-order | Second-order |
| **Manifold** | SE(3) manifold-aware | Riemannian optimization on Stiefel manifold |
| **Approach** | Direct nonlinear optimization | Convex relaxation + rank projection |

---

## 5. Post-Processing
**DIFFERENT** - CORA requires alignment, GTSAM doesn't

**GTSAM:**
- Optimizes in world frame (anchored by first pose prior)
- No post-processing needed
- Results directly in world coordinates

**CORA:**
- Optimizes in arbitrary frame (no pose prior)
- `projectSolution()` - projects from relaxed to feasible set
- `alignEstimateToOrigin()` - aligns to CORA's canonical frame (first rotation = identity, mean translation = 0)
- Manual SE(3) transform - transforms from CORA's canonical frame to world frame

---

## Impact on Results

### Current Results (with Morrison-recommended noise models):
Run the optimizer to see updated results with the new noise models.

### Expected Behavior:
With looser constraints (σ_trans = 2m, σ_rot = 5 rad):
1. **Less overfitting** - Won't lock in noisy odometry too tightly
2. **More influence from ranges** - Range measurements have more weight
3. **Better generalization** - Should handle noisy sections better
4. **Slightly larger errors** - May sacrifice some accuracy for robustness

The comparison should now properly reflect Morrison's philosophy of avoiding overconfident noise models.

---

## Recommendations

**USE NOISE_MODEL_FINDINGS.md** - This file is historical reference only.

**Current validated configuration:**
- **GTSAM**: σ_trans = 2.0m, σ_rot = 5.0 rad (loose) → 3-3.5m error on well-constrained missions
- **CORA**: σ_trans = 0.316m, σ_rot = 0.1 rad (very tight) → 11-13m error on well-constrained missions

These values are validated across 5 missions. See `NOISE_MODEL_FINDINGS.md` for:
- Per-mission results and analysis
- Data preprocessing guidelines (exclude ascent data)
- Understanding of when CORA struggles (short missions, poor geometry)
- Ground truth comparison methodology

---

## Current Status
**Status**: **HISTORICAL DOCUMENT** - For current results and methodology, see:
- `NOISE_MODEL_FINDINGS.md` - Validated results across 5 missions
- `tests/offline_optimizer.py` - Batch optimizer with validated noise models
- `tests/README.md` - Usage instructions for offline optimizer
