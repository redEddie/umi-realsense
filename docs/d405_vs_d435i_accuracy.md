# Why D405 gives lower SLAM-anchor accuracy than D435i

Record of an experiment in this project (2026-06): we built the full UMI pipeline
(map → world-anchor via ArUco → camera-IMU/TCP → gripper action) and compared the
**world-anchor precision** (`calibrate_slam_tag` translation-spread of the origin
tag, lower = better) between the two cameras.

## Result

| Camera | Stereo baseline | IMU | Map / tracking | **Anchor spread (median)** |
|---|---|---|---|---|
| **D435i** | 50.15 mm | yes | single map, 0 track-loss | **0.38 cm** |
| **D405** (origin tag ~0.7 m) | 18.19 mm | **no** | single map, 0 track-loss | **7.1 cm** |
| **D405** (origin tag ~0.4 m, "close") | 18.19 mm | **no** | single map, 0 track-loss | **11.9 cm** (worse) |
| **D405** localize (drift-removal attempt) | — | — | re-localization fails | **only 2 tracked frames** |

(baselines/IMU from `calibration/factory_calib_*.json`; spreads from `aruco/calibrate_slam_tag.py`.)

Tracking was *robust* on both (single map, no track-loss) — the difference is in
**metric pose accuracy**, not tracking stability.

## Sample footage
Short clips (~6 s, 2x downscaled) from the actual recordings:
- D405 (color): [`clips/d405_sample.mp4`](clips/d405_sample.mp4) — close-range view,
  the origin marker fills much of the frame; relatively little far background.
- D435i (left IR): [`clips/d435i_sample.mp4`](clips/d435i_sample.mp4) — wider scene
  context with more distant background structure.

## Why D405 is less accurate

1. **Short stereo baseline (18 mm vs 50 mm).** Stereo metric accuracy scales with
   baseline. At the same distance D405's disparity is ~2.8x smaller, so triangulated
   depth (hence camera-pose scale) is noisier → the trajectory carries scale/drift
   error that shows up as the 7–12 cm anchor spread.

2. **No IMU.** D435i has a BMI055; even when we run STEREO-only, having the option of
   inertial constraints (and generally a camera designed for VIO) matters. D405 has no
   motion module at all → no inertial drift correction is possible.

3. **Close range hurt, not helped (counter-intuitive).** Moving the marker closer
   (0.7 m → 0.4 m) made it *worse* (7.1 → 11.9 cm). At close range each frame sees a
   small patch, so far-apart keyframes share few common landmarks → weak global
   constraints → more accumulated drift. (Marker corner precision improved, but the
   dominant error is the SLAM trajectory, not the tag pose.)

4. **Re-localization (the usual drift fix) doesn't hold on D405.** UMI computes the
   anchor from a trajectory re-localized against the globally-optimized map (lower
   drift than the online mapping trajectory). On D405 re-localization barely tracked
   (2 OK frames) — narrow context + no IMU make relocalization fragile.

## Takeaway / recommendation

- **Environment-SLAM-based 6DoF pose tracking needs baseline + IMU.** Use **D435i or
  D455** for the pose/anchor (D435i measured 0.38 cm here).
- **D405's strength is close-range depth + RGB-stereo observation**, not wide-area
  metric SLAM. Use it for close observation if needed, not for the SLAM anchor.
- The pipeline itself is camera-agnostic (`--cam d405|d435i`); only the *accuracy*
  differs. We therefore switched the SLAM/anchor camera back to D435i.
