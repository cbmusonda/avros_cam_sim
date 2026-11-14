# 360° camera layout analysis (4x ZED X)

Question: where to put two 2.2 mm ZED X (front/back) and two 4 mm ZED X (sides) for 360° coverage with no gaps, for IGVC AutoNav.
Plain Python 3 (no dependencies). Geometry is copied from `../main.js` (car boxes, Ouster OS1-64 position).

## Result

Put all four cameras on a **crown on a short mast directly above the lidar**, lenses at **1.50 m**, optical centres 12 cm from the mast axis.
The sim's **"360° crown (recommended)"** preset loads this layout.

| camera | lens | xyz in base_link (m) | yaw | pitch (down) |
|---|---|---|---|---|
| zed_front | 2.2 mm | (0.209, 0, 1.50) | 0° | 30° |
| zed_left | 4 mm | (0.089, 0.12, 1.50) | 90° | 37° |
| zed_right | 4 mm | (0.089, -0.12, 1.50) | -90° | 37° |
| zed_rear | 2.2 mm | (-0.031, 0, 1.50) | 180° | 37° |

- Ground fully covered, with stereo depth, all around from ~0.6 m to ~4.25 m off the robot footprint; beyond that only 1-2° slivers on the seam lines (±56°, ±126°).
- 15-25° overlap between neighbours at 2 m (needed for extrinsic calibration).
- Front top edge ~9° above horizon (far lane lines; tolerates nosing down off a 15% ramp).
- The crown sits above the Ouster's +21.2° top beam cone: 0% of lidar beams blocked by the cameras. Put the mast pole at the platform's rear edge (x ≈ -0.18): a 25 mm tube shadows ~5° straight behind, which the rear camera covers.
- Insensitive to ±3° pitch errors and to moving the crown ±10 cm fore/aft. Keep side yaw at exactly 90° (85° shrinks the ring to 2.95 m).

Height trade-off (pitches re-optimised per height, `results/sweep3.txt`):

| lens height | full stereo ground ring | seam overlap @ 2 m |
|---|---|---|
| 1.15 m | 0.5 – 3.1 m | 8–10° |
| 1.30 m | 0.5 – 3.6 m | 12–15° |
| 1.45–1.50 m | 0.5–0.6 – 4.2 m | 16–18° |
| 1.60 m | 0.5 – 4.6 m | 19–25° |
| 1.75 m | 0.6 – 5.2 m | 24–28° |

Why not body-mounted: rectified FOVs sum to 2×105.2° + 2×71.5° ≈ 353° < 360°, so a horizontal ring can't close; pitching down widens the ground coverage, which only works well when the cameras are high and close together. Spread around the body, the corner seams never close (best body layout: 85% worst-ring coverage 1–6 m; current URDF + rear camera: 48%). Low 4 mm side cameras also lose depth inside their 1.0 m minimum range.

## Assumptions / limits

- FOV: rectified rectilinear on the 5.76 x 3.6 mm sensor (2.2 mm → 105.2 x 78.6°, matches Stereolabs' 105 x 78 rectified; 4 mm → 71.5 x 48.5°, verify against the SDK for your unit). Stereo = point in both eyes (120 mm baseline) and inside depth range (2.2 mm 0.3–12 m, 4 mm 1.0–20 m).
- Flat ground, boxy robot, no lens distortion, no camera-on-camera or mast/E-stop-light occlusion.
- IGVC numbers (10 ft nominal lanes, 3 in lines, 2 ft potholes, ≤15% ramps, 6 ft height limit) from the 2026 rules summary; check against the official PDF.

## Files

- `camsim.py` – geometry, camera/stereo visibility, coverage and lidar-shadow scoring
- `ring.py` – full-ring analysis on offset contours of the footprint
- `base.py` – current layout + rear camera baseline
- `opt.py <body|free|mast>` – first random-search + local optimiser (writes `best_*.json`)
- `opt2.py <crown|hybrid> <H>` – crown / hybrid optimiser (writes `r2_*.json`); `rings_run.py` compares these
- `opt3.py <H>` – maximises the full-ring distance for a crown at height H
- `verify.py` – final check + sensitivity of the recommended layout
- `results/` – outputs of the runs above
