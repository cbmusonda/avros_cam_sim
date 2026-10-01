# AVROS Camera Sim

Browser-based 3D simulator for choosing where to mount the ZED X cameras on the AVROS / IGVC robot.
Static site (three.js r160 vendored in `vendor/`), no build step.

- Car body, IMU and VLP-16 from `avros_bringup/urdf/avros.urdf.xacro` (ROS frame: x forward, y left, z up, origin = base_link on ground).
- Cameras start at the URDF mounts: front (2.2 mm, 15° down), left/right (4 mm, ±90°).
- Click a camera, drag the gizmo (T = move, R = rotate) or use the sliders. FOV volume, ground footprint and a live camera-POV inset update as you move.
- Lens presets use the ZED X sensor (5.76 x 3.6 mm). "calculated" is ideal rectilinear; "datasheet" values are approximate Stereolabs figures, verify them.
- **Lidar overlay (VLP-16):** the lidar is movable (x/y/z only, kept level). Click it or use the "Lidar (VLP-16)" section's sliders / gizmo. It draws the 16 beams (-15 to +15 deg, 2 deg steps) out to 30 m (min range 0.4 m): coloured rings where each beam hits the ground, faint beam lines every 45 deg, and hit dots on the car body (red) and reference figures (yellow). The car body blocks beams. The panel lists the blind radius (steepest beam), each ring's ground radius, ring gaps and % of each ring blocked by the car. Toggle rings/beams under Scene.
- **URDF origins** prints `<origin xyz rpy>` lines (cameras and the velodyne) to paste back into the xacro. Export/Import JSON saves layouts (including the lidar); state is also kept in localStorage.

## Run locally
    python3 -m http.server 8000   # then open http://localhost:8000

(ES modules need http, not file://.)

## Publish
Push to GitHub, then Settings > Pages > Deploy from branch `main` / root.
