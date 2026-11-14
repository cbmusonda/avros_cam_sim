# 360° camera layout analysis (4x ZED X)

Question: where to put two 2.2 mm ZED X (front/back) and two 4 mm ZED X (sides) for 360° coverage with no gaps, for IGVC AutoNav.
Plain Python 3 (no dependencies). Geometry is copied from `../main.js` (car boxes, Ouster OS1-64 position).

## Result

Put all four cameras on a **crown on a short mast directly above the lidar**, lenses at **1.50 m**, optical centres 12 cm from the mast axis.
The sim's **"360° crown (recommended)"** preset loads this layout.

| camera | lens | xyz in base_link (m) | yaw | pitch (down) |
|---|---|---|---|---|
| zed_front | 2.2 mm | (0.209, 0, 1.50) | 0° | 34° |
| zed_left | 4 mm | (0.089, 0.12, 1.50) | 90° | 40° |
| zed_right | 4 mm | (0.089, -0.12, 1.50) | -90° | 40° |
| zed_rear | 2.2 mm | (-0.031, 0, 1.50) | 180° | 33° |

- Ground fully covered, with stereo depth, all around from ~0.55 m to ~4.2 m off the robot footprint; beyond that only 1-2° slivers on the seam lines (±56°, ±126°).
- 15-25° overlap between neighbours at 2 m (needed for extrinsic calibration).
- Front top edge ~5° above horizon (far lane lines; tolerates nosing down off a 15% ramp).
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

## Lane-line detectability (pitches above)

Pitches were first chosen for coverage only (30/37/37). `lines.py` adds a lane-line model: pixels across a 3 in line
(full res and 960x600), stereo-valid, in IGVC zones (lines alongside, lines ahead in a 10 ft lane, 5 ft-radius curves, potholes),
and re-optimises pitches per height. Front 34° / rear 33° / sides 40° raises lines-alongside detectability from 91% to 100%
and lines-ahead from 91% to ~95-99%, while the 360° ring stays ~0.55-4.2 m. Results: `results/lines_sweep.txt`.

| lens height | lines alongside | lines ahead | 360° ring | lines ≥6 px ahead to |
|---|---|---|---|---|
| 0.8 m | 77% | 63% | open | 1.1 m |
| 1.15 m | 100% | 86% | 0.6–2.9 m | 6.8 m |
| 1.30 m | 100% | 91% | 0.6–3.5 m | 7.2 m |
| 1.45 m | 100% | 95% | 0.6–3.8 m | 7.5 m |
| 1.60 m | 100% | 99% | 0.6–4.4 m | 7.8 m |
| 1.75 m | 100% | 100% | 0.6–4.6 m | 8.0 m |

Prior IGVC design reports (2012–2025, read via the www.igvc.org mirror) agree: top AutoNav teams mostly mount cameras on a mast at
1.2–1.8 m (e.g. USNA 1.78 m, Georgia Tech 1.65 m, Oklahoma 1.68 m, NMIMS 1.52 m, Boise State 1.2 m at 30° down), and Oklahoma's 2025
winner used four cameras, one per face, on a tall central mast. Low mounts (0.4–0.9 m) repeatedly reported blind spots and can't see
behind ~40 in barrels. Practical notes from those reports: stiff mast (sway was the most common complaint), adjustable height/pitch,
per-frame ground-plane/IMU tilt correction, sun hoods / auto-exposure, and run line detection at ≥960x600.

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
- `verify.py` – final check + sensitivity of the coverage-only layout
- `lines.py` – lane-line detectability model and height/pitch sweep
- `results/` – outputs of the runs above
