# Future steps

Consolidated open items across everything touched 2026-08-10. Ordered
roughly by "blocks other things" first. Cross-references point at the doc
with the full trace — read there before starting, several of these already
have false starts recorded that are worth not repeating.

## Blocking / high-value

1. **RTX LiDAR azimuth clip (±90° instead of 360°)**. Root cause identified:
   `g1_sim/rtx_publisher.py` reads `IsaacCreateRTXLidarScanBuffer`, deprecated
   in this Isaac Sim 6.0.1 build; the sim's own log says to use
   `GenericModelOutput` instead. This blocks meaningful RViz/detection
   verification for the whole RTX pipeline, warehouse scene included.
   → `Tasks.md` "Live verification — 2026-08-10", `Verify_RTX.md`.

2. **Finish the warehouse pipeline's process 2 + 3 live run.**
   `g1_patrol.py` is written but has never been run together with
   `g1_warehouse_sim.py`. Rosbag capture command is documented but the full
   3-minute capture has never actually been executed. Do both, then build
   the bag→video script (`cv_bridge` + OpenCV `VideoWriter` — confirmed
   available in the ROS2 Jazzy Python env, just not wired up yet).
   → `Verify_Warehouse.md`.

3. **Pedestrian/target placement vs. blind radius.** Both the plain
   `g1_rtx_sim.py` scene's static `PEDESTRIANS` targets and the warehouse
   scene's IRA-spawned humans can end up inside the sensor's ~9.6 m blind
   radius depending on spawn area config — the LiDAR will never see them.
   Check IRA's `spawn_area`/`navigation_area` config in
   `g1_sim/ira_actors.py`'s YAML once real detection testing starts.
   → `Tasks.md` Task 5 background, `Plan.md`.

## Locomotion / control

4. **Turning and strafing untested live.** Only forward (`vx=0.5`) has been
   exercised against a real Isaac Sim run; `vy`/`wz` are structurally
   identical in the observation contract but unverified.
5. **50 Hz vs. 60 Hz control-rate mismatch.** The WBC policy was trained at
   50 Hz control; `g1_rtx_sim.py`/`g1_warehouse_sim.py` run physics at 60 Hz
   and the control-tick check effectively fires every physics step (~20%
   faster than trained). Stable so far, but not an exact reproduction.
6. **Lateral drift.** MuJoCo validation showed ~0.68 m of sideways drift
   over 17s of a pure-forward command. Not investigated — genuine gait
   asymmetry vs. harness artifact, unknown.
7. **Real localization**, if `g1_patrol.py`'s open-loop dead-reckoning patrol
   turns out not to be good enough (it will drift and not close its loop).
   Needs real odometry or SLAM output feeding a closed-loop controller
   instead of fixed timers.
   → `docs/decoupled_wbc_findings.md` §7.

## Detection

8. **OpenPCDet PointPillar integration.** Real pretrained weights
   (`pointpillar_7728.pth`) confirmed working standalone; two integration
   paths scoped (wrap OpenPCDet directly as a new backend vs. reshape our
   model to match its architecture) but neither implemented.
   → `Tasks.md` "OpenPCDet comparison".
9. **Detection against a populated warehouse scene** hasn't been tried at
   all — only the plain flat-ground/simple-room scenes so far. Once the
   azimuth clip (#1) is fixed, the warehouse scene with IRA humans is the
   natural next test target for the CenterPoint/PointPillar backends.

## SLAM

10. **Ultra-Fusion sensor profile.** Runs correctly via Docker, but no
    config exists for a legged, wheel-less robot with our real Mid-360/D435
    extrinsics — needs authoring from `config/m3dgr/uf_m3dgr_ros2_lio.yaml`
    plus actual calibration data we don't have yet.
    → `docs/ultra_fusion_findings.md` §5.
11. **Live/bagged DDS test.** Ultra-Fusion's `--net=host` cross-distro DDS
    bridging (container's Humble ROS2 seeing host Jazzy topics) is a
    well-established pattern per its docs but was never actually exercised
    against real sensor data this session.

## Infra / environment

12. **GEAR-SONIC (`gear_sonic_deploy`) is parked, not finished.** TensorRT
    10.13 is installed and the C++ build got through dependency resolution
    before the user redirected to `decoupled_wbc`. If SONIC's fuller
    whole-body capability (manipulation + locomotion, mocap-driven) is
    wanted later, that TensorRT install and the checkout under
    `third_party/GR00T-WholeBodyControl/gear_sonic_deploy/` are still there,
    untouched.
13. **Disk is critically tight machine-wide** (~5 GB free / 355 GB, 99%
    used, mostly from causes unrelated to this project). Any further large
    asset/Docker-image pulls should check `df -h /` first and expect to hit
    the floor faster than feels intuitive for a 355 GB disk.
14. **`record_g1_bag.sh` (real-robot bag script) still references
    `/utlidar/cloud` and `/g1/odom`**, neither of which this sim pipeline
    publishes — it was written for real hardware and is correct for that
    use case, just don't reach for it when testing the sim pipeline (use
    the raw `ros2 bag record` command in `Verify_Warehouse.md` instead, or
    write a sim-specific wrapper if this becomes a frequent operation).
