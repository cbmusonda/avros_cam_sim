# AVROS Camera Sim

Browser-based 3D simulator for choosing where to mount the ZED X cameras on the AVROS / IGVC robot.
Static site (three.js r160 vendored in `vendor/`), no build step.

- Car body, IMU and VLP-16 from `avros_bringup/urdf/avros.urdf.xacro` (ROS frame: x forward, y left, z up, origin = base_link on ground).
- Cameras start at the URDF mounts: front (2.2 mm, 15° down), left/right (4 mm, ±90°).
- Click a camera, drag the gizmo (T = move, R = rotate) or use the sliders. FOV volume, ground footprint and a live camera-POV inset update as you move.
- FOV is locked to the Stereolabs ZED X datasheet (2.2 mm: 110 x 80 x 120 deg, 4 mm: 75 x 50 x 83 deg H x V x D). Only position/orientation is adjustable.
- **URDF origins** prints `<origin xyz rpy>` lines to paste back into the xacro. Export/Import JSON saves layouts; state is also kept in localStorage.

## Run locally
    python3 -m http.server 8000   # then open http://localhost:8000

(ES modules need http, not file://.)

## Publish
Push to GitHub, then Settings > Pages > Deploy from branch `main` / root.
