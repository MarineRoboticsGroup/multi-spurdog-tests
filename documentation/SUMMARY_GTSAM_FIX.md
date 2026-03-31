# Summary: GTSAM Bug Fix and Comparison Study - March 18, 2026

## What We Fixed

### 1. GTSAM Noise Model Bug
**Problem:** Both optimizer scripts were using CORA's tight noise model (σ_t=0.316m, σ_r=0.1rad) for GTSAM instead of GTSAM's optimal loose model (σ_t=2.0m, σ_r=5.0rad).

**Files Fixed:**
- `tests/offline_optimizer.py` (line ~302)
- `tests/offline_optimizer_with_priors.py` (line ~319)

**Impact:** GTSAM performance improved from 13.48m → 3.25m error (4.15x improvement!)

---

## What We Tested

### Systematic Parameter Sweep
Ran 11 different noise model configurations on pink_solo mission to find:
1. Best parameters for each optimizer
2. Fair comparison with same parameters
3. Whether loose rotation helps CORA (hypothesis testing)

### Results Summary

| Comparison Type | GTSAM | CORA | Winner |
|----------------|-------|------|--------|
| **Fair (same params)** | 3.25m | 17.95m | GTSAM 5.5x better ✨ |
| **Optimal (each tuned)** | 3.25m | 13.21m | GTSAM 4.1x better ✨ |

**Key Finding:** GTSAM is superior in BOTH comparisons.

---

## What We Documented

Created comprehensive markdown file: **`GTSAM_VS_CORA_COMPARISON.md`**

Contains:
- Executive summary with key findings
- Explanation of noise model parameters (σ_trans, σ_rot)
- Why σ_rot=5.0 rad doesn't mean bad sensors
- Full parameter sweep results table
- Hypothesis testing (loose rotation for CORA)
- Bug discovery and fix details
- Recommendations for future work
- All mission results

**Location:** `/home/lizgajski2/catkin_ws/GTSAM_VS_CORA_COMPARISON.md`

---

## Matplotlib Error Metrics Display

### Your Question: "Do the trajectory plots show δGTSAM and δCORA?"

**Answer:** Yes! Matplotlib has this feature and we're already using it.

### Implementation

```python
# Add error metrics text box (line ~574 in offline_optimizer.py)
if gtsam_error is not None and cora_error is not None:
    textstr = f'δGTSAM: {gtsam_error:.2f}m\nδCORA: {cora_error:.2f}m'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax1.text(0.98, 0.02, textstr, transform=ax1.transAxes, fontsize=12,
            verticalalignment='bottom', horizontalalignment='right', bbox=props)
```

### What This Does:

- **Position:** Bottom right corner (0.98, 0.02 in axes coordinates)
- **Background:** Semi-transparent wheat-colored box with rounded corners
- **Content:** Shows δGTSAM and δCORA values (XY distance to surface GPS)
- **Alignment:** Right-aligned, bottom-aligned

### Matplotlib Features Used:

1. **`ax.text()`** - Adds text annotation to axes
2. **`transform=ax1.transAxes`** - Uses axes coordinates (0-1) instead of data coordinates
3. **`bbox=props`** - Adds background box with styling
4. **`boxstyle='round'`** - Rounded corners
5. **`facecolor='wheat'`** - Box background color
6. **`alpha=0.8`** - 80% opacity (slightly transparent)

### Example Output:

```
┌─────────────────┐
│ δGTSAM: 18.19m │
│ δCORA: 109.56m │
└─────────────────┘
```

This appears in the **bottom right corner** of every trajectory plot generated.

### Most Recent Example:

**File:** `results/damp_beer_offline/trajectory_plot.png` (generated 12:41 today)
- δGTSAM: 18.19m
- δCORA: 109.56m

The text box should be clearly visible in all newly generated plots!

---

## Files Generated Today

### Documentation:
- ✅ `GTSAM_VS_CORA_COMPARISON.md` - Comprehensive study results
- ✅ `tests/results/noise_model_sweep_results.csv` - Parameter sweep data
- ✅ `tests/results/noise_model_sweep_log.txt` - Execution log

### Updated Code:
- ✅ `tests/offline_optimizer.py` - Fixed GTSAM noise model
- ✅ `tests/offline_optimizer_with_priors.py` - Fixed GTSAM noise model
- ✅ `tests/noise_model_sweep.py` - Created parameter sweep script

### Fresh Plots (with error metrics):
- ✅ `results/damp_beer_offline/trajectory_plot.png`

---

## Next Steps

### To Regenerate All Plots with Error Metrics:

```bash
./tests/run_all_missions.sh
```

This will:
1. Run all 5 missions WITHOUT priors
2. Run all 5 missions WITH priors
3. Generate fresh plots with δGTSAM and δCORA text boxes
4. Print comparison table at the end

**Estimated time:** ~2 minutes per mission × 5 missions × 2 runs = **~20 minutes total**

### To View a Specific Plot:

```bash
# View in image viewer
xdg-open results/damp_beer_offline/trajectory_plot.png

# Or check timestamp
ls -lh results/*/trajectory_plot.png
```

---

## Key Takeaways

1. **GTSAM is 4-5x better than CORA** for this dataset
2. **Different optimizers need different tuning:**
   - GTSAM: Loose constraints (trust ranges)
   - CORA: Tight constraints (numerical stability)
3. **Fair comparison confirmed:** GTSAM superior even with same parameters
4. **Plots now show error metrics** in bottom right corner
5. **Bug fixed:** GTSAM now uses correct noise model

---

## Questions Answered

✅ What do noise model parameters mean?  
✅ Are current parameters realistic?  
✅ Should CORA prioritize ranges over orientation? (Tested: No, tight constraints are better)  
✅ Fair comparison request from CORA developer (Done: GTSAM 5.5x better)  
✅ Do plots show error metrics? (Yes: matplotlib text box in bottom right)  
✅ Bug fixed and documented

---

**Study completed:** March 18, 2026  
**Total configurations tested:** 11  
**Winner:** GTSAM (by large margin)
