# Wireless LiDAR and local SLAM

## Architecture

The PC connects directly to the external COIN-D6 bridge using its own TCP socket (initial port 9000). LiDAR bytes NEVER travel through the AGV controller. The AGV keeps its existing TCP 4211 connection for commands and telemetry. Acquisition and scan matching use separate workers and a bounded latest-scan handoff. Combining already-received AGV telemetry with LiDAR happens on the PC; this does not add AGV network requests. Shared Wi-Fi airtime and PC CPU are still finite resources.

USB/serial support has been removed from the LiDAR module. BIN replay remains available for offline verification. The serial dependency used by the existing arm code is unrelated to LiDAR.

## Prepare the scan

1. Open **LiDAR / SLAM**, select **TCP - live**, enter the external LiDAR IP and port, then **Connect** and **Start LiDAR**. The initial IP 192.168.4.1 is provisional, not a discovered device address.
2. Mount defaults: X = +0.115 m, Y = 0 m, measured from the robot axle centre. Confirm yaw physically. X points forward, Y left, yaw counterclockwise. Sensor clockwise angles are converted before applying the mount transform.
3. The self-return filter excludes points inside the 23 x 23 cm footprint plus a configurable margin (initially 0.02 m). **Excluded sector centre / width** masks an additional sector originating at the LiDAR, expressed in robot directions: 0 degrees forward, +90 left, 180 rear. Width 0 disables only the angular mask. Set the actual obstructed width from a stationary scan; the rear 180 degrees are NOT automatically discarded. Excluded measurements are unknown, never free space.
4. Apply mount, range and mask settings with **New map / apply calibration**. They are saved in `lidar_settings.json`. Recalibration starts a new map so differently calibrated points cannot mix.
5. Enable **Use AGV odometry prediction** and select **Left encoder + IMU** or **Right encoder + IMU**, choosing the reliable encoder. Apply the configuration to save it. The optional two-encoder mode retains the original AGV odometry; do not choose it when an encoder is faulty.

## Translation and in-place rotation

Single-encoder predictions are computed from calibrated wheel travel and IMU increments at telemetry rate. Motor polarity and channel swapping are undone before classifying opposite wheel directions. When motor telemetry reports an in-place rotation, the predicted centre holds X/Y and updates only yaw from the IMU, ignoring encoder travel. This mode remains active during zero-PWM settling until a new nonzero translation command is observed. During translation, the wheel-to-centre correction accounts for small heading changes. Rotation direction is never inferred from the faulty encoder.

This is an explicit in-place-motion assumption, not a measurement that the chassis did not slip. Moving the chassis by hand after a rotation is not supported by this motor-based classification. ICP may still correct the pose if measured surfaces disagree with the prediction. The original AGV two-wheel odometry display is unchanged; the selected single-encoder prediction applies to SLAM and the dedicated calibration run.

The LiDAR itself moves along an arc when the centre rotates because it is 11.5 cm ahead of that centre. Transforming its points into the robot frame before scan matching accounts for this offset.

A selected single-encoder mode requires valid IMU data. IMU discontinuities or odometry resets clear the active map rather than bridging an unknown motion segment. Historical replay never uses live odometry. A scan needs an odometry sample within 200 ms of its receive timestamp.

## Ground distance calibration: 50 cm

In **AGV > Calibrate > Ground distance calibration - 50 cm**:

1. Choose **Left** or **Right**. Mark the robot's initial centre position on the floor. Ensure fresh IMU, AGV power, PC ownership and a clear straight path.
2. Press **START 50 cm - encoder + IMU**. This starts the existing guarded single-encoder motion primitive with IMU heading hold. STOP remains available.
3. After stopping, read the indicated distance and measure the actual travel with a tape. Enter the measured distance, which can be less or more than 50 cm.
4. Press **APPLY + SAVE distance scale**. Only the selected wheel scale changes: `new_scale = old_scale * actual_cm / indicated_cm`. Ticks per revolution and the other wheel scale remain unchanged. At least 49 cm indicated is required, allowing a 1 cm stopping tolerance around the 50 cm test. IMU loss, heading deviations above 5 degrees, reboot and active movement reject application.
5. `settings.json` is replaced atomically and contains the new scale plus `_distance_calibration`: side, requested/indicated/measured distances, correction ratio, resulting scales and save timestamp. The existing startup loader restores geometry; the UI restores the last calibration report/side. A save failure keeps the old in-memory scale and the measurement for retry.
6. Repeat the 50 cm test to verify. For example, 50 cm indicated and 45 cm actual gives a 0.9 correction. Do not enter 50 merely because it was the requested distance.

Wheel/encoder revolution calibration remains separate. Perform it first, then ground distance calibration, then check IMU direction and LiDAR mount/sector.

## Finish and save

**Finish scan + save map/image** sends STOP only to the LiDAR, stops admitting new scans, waits for the pending scan-matching result, and opens the save dialog. It does not command the AGV to stop: stop robot motion with the global STOP before ending the experiment.

Choose a filename (a timestamped name is proposed). Two files are produced:

- `.npz`: `map_xy_m` (N x 2 metres), `trajectory_xy_yaw` (N x 3 metres/radians), mount, source and `metadata_json` containing UTC time, range/mask settings, odometry mode and AGV geometry captured for this map. Read with `numpy.load(..., allow_pickle=False)`.
- `.png`: labelled map and robot-centre trajectory, in metres, suitable for reviewing and sharing.

The map remains visible after finishing or cancelling the save dialog; use **Save map + trajectory + PNG** to retry. A later AGV reset does not erase a finished map. **New map / apply calibration** starts a new map; press Start LiDAR again if it was stopped. Saving an active map exports the latest completed result. Raw BIN/JSONL recording is optional and preserves unfiltered data for subsequent reprocessing.

The image is a point-map plot, NOT a ROS occupancy map or a navigation-ready free-space grid. The NPZ stores data for subsequent analysis; this application does not yet load saved maps for localization.

## Test sequence for the first hardware session

1. Confirm external LiDAR IP/port and independent connection; watch valid packets and CRC errors while the AGV is stationary.
2. Select the reliable encoder and verify IMU sign with a small left rotation.
3. Calibrate wheel revolutions, then perform and repeat the 50 cm ground test.
4. With mapping disabled, check a known wall distance and orientation; adjust the self-return margin and angular sector. Keep useful environmental returns outside the blocked sector.
5. Start a new map with the same Left/Right encoder selected. Test stationary scans, a slow 50 cm translation, then a slow in-place rotation. Check that the centre does not translate in the prediction and that walls do not duplicate badly.
6. Global STOP, finish scan, save NPZ/PNG, and retain the raw capture for diagnosis.

## Limits and references

This is experimental local point-to-map ICP with bounded memory. It has no loop closure, global optimization, automatic relocalization, motion compensation within a revolution or navigation. Host receive times are not synchronized sensor timestamps. Repetitive geometry, moving objects, slip, Wi-Fi latency, aggressive masks and fast motion can defeat matching. Large gaps require a new map. Software tests do not establish physical map accuracy.

Reference review:

- [WPILib differential-drive odometry](https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/differential-drive-odometry.html): gyro/encoder odometry, pose conventions, resetting and drift. Its standard model uses both encoders; the single-encoder/in-place policy here is a project-specific adaptation.
- [ROS laser_filters angular exclusion implementation](https://github.com/ros-perception/laser_filters/blob/ros2/include/laser_filters/angular_bounds_filter_in_place.h): excluding angular measurements instead of treating them as valid ranges. This app implements its own mask and does not depend on ROS.
- [Nav2 map server](https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/README.md): occupancy-grid image/YAML persistence. Our measured-surface point map is deliberately not represented as an occupancy grid.

Run `python -m unittest discover -s tests -v` with the project environment. Tests cover binary protocol/TCP, masks, Left/Right predictions, rotation with false encoder data, 50 cm calibration and atomic persistence, ICP, asynchronous finish, NPZ/PNG saving and existing AGV/arm behaviour.
