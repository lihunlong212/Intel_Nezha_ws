# AprilTag realtime tuner

This package is deliberately independent from the mission camera, route, PID, and
dispatch nodes. It opens the camera directly and does not publish flight-control
topics. Stop `demo1` and `drone_camera_node` before running it because only one
process can own `/dev/video0`.

Build and run:

```bash
colcon build --symlink-install --packages-select apriltag_tuner_pkg
source install/setup.bash
ros2 launch apriltag_tuner_pkg apriltag_tuner.launch.py \
  camera_device:=/dev/video0 target_id:=-1
```

GUI controls:

- `target mode`: `0` accepts any 36h11 ID, `1` only accepts `target id`.
- `corner refine`: `0` none, `1` subpixel, `2` contour, `3` AprilTag.
- `CLAHE x10`, `sharpen x10`, and `blur radius` tune preprocessing.
- Remaining controls map to OpenCV `DetectorParameters`.
- Green outlines are decoded tags, the red dot is the selected tag, and yellow
  outlines are rejected quadrilateral candidates.

Keyboard:

- `s`: save current values to `output_file` (default
  `~/.config/nezha/apriltag_detector.yaml`). The production camera automatically
  reads this same file on its next startup.
- `r`: restore robust starting values.
- `space`: pause/resume the camera image while retaining live slider adjustment.
- `q` or `Esc`: quit.

The saved target mode and target ID are only a tuning record. Production pickup ID
selection still comes from the active order; only detector and preprocessing values
are applied by the production camera.
