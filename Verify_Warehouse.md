# Verify — warehouse SDG pipeline (3 processes)

Status as of 2026-08-10: **Process 1 verified working. Process 2 written,
not yet run live. Process 3 not yet finalized.** Update this file's
checkboxes as each piece gets exercised for real — don't mark something ✅
from reading the code alone, the whole point of this file is the distinction
between "written" and "verified."

---

## The 3 processes

| # | What | Script | Kind |
|---|---|---|---|
| 1 | Warehouse scene: G1 + WBC locomotion + LiDAR/camera/IMU + IRA humans/Nova Carters | `scripts/g1_warehouse_sim.py` | Isaac Sim (own process, owns the stage) |
| 2 | G1 patrol commander — publishes `/g1/cmd_vel` in a timed loop | `scripts/g1_patrol.py` | plain rclpy node |
| 3 | Rosbag capture | `ros2 bag record` (command below) | separate OS process |

All three run **simultaneously**, in three separate terminals.

## Prerequisites

```bash
pkill -9 -f "python.*scripts/"      # kill any stragglers - only one Isaac Sim fits in 16GB
free -h                              # want several GB available
df -h /                              # this machine has been disk-tight all session - check before every run
```

## Process 1 — the sim

```bash
conda activate isaac
cd ~/Projects/thesis/G1_sim
python scripts/g1_warehouse_sim.py --headless --num-humans 2 --num-carters 1
```

Useful flags: `--steps N` (stop after N physics steps instead of running
forever), `--no-ira` (warehouse + G1 only, no humans/carters — useful to
isolate whether a problem is IRA-related), `--no-locomotion` (robot holds
its default pose instead of using the WBC bridge), `--num-humans 0
--num-carters 0` (warehouse with IRA's navmesh/setup path exercised but no
actors spawned).

**Expected startup log** (in order):

```
[WH] IRA config      : .../assets/ira_warehouse_config.yaml  (2 humans, 1 carters)
[ira_actors] navmesh baked in <N> frames (<T>s)      # N should be a few hundred, T a few seconds
[ira_actors] setup_simulation still running (...)     # may print a few times, this is normal
[ira_actors] setup OK in <T>s
[WH] G1 prim valid   : True  pelvis valid: True
[WH] articulation    : 29 DOF
[WH] IRA actors      : yes
[WH] WBC bridge      : loaded (decoupled_wbc Balance/Walk, gains overridden)
[WH] lidar prims     : 4
[WH] state graph     : ...  (/tf, /g1/joint_states, /clock)
[WH] imu graph       : ...  (/g1/imu, prim=...)
[WH] cmd_vel graph   : ...  (/g1/cmd_vel)
[WH] camera graph    : ...
[WH] timeline        : playing=True
[WH] running
```

Every ~50 WBC updates it also prints `pelvis_z` — watch this for a collapse
(falling toward 0) vs. a stable stance (~0.7-0.75 m).

### Agent verification (process 1)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list                              # expect: /livox/mid360/points /g1/camera/rgb
                                              # /g1/camera/depth /g1/camera/camera_info
                                              # /g1/joint_states /g1/imu /tf /tf_static /clock
ros2 topic hz /livox/mid360/points           # ~10-11 Hz
ros2 topic hz /g1/imu                        # should publish at physics rate or close to it
ros2 topic echo /g1/joint_states --once      # 29 joint names/positions, non-NaN
```

- [ ] All topics above present in `ros2 topic list`
- [ ] `[ira_actors] navmesh baked in <N> frames` printed with N < the
      `--navmesh-max-frames` budget (default 3000; real bake measured ~700)
- [ ] `[WH] IRA actors : yes` (not `no` — `no` means IRA setup failed and the
      scene silently fell back to warehouse-with-no-actors; check the log
      above it for the real error)
- [ ] pelvis_z stays in a stable band (~0.65-0.8 m) across the run, not
      trending toward 0

### Human verification (process 1)

Launch RViz in a separate terminal (see the RViz fix note in `README.md`
first if it crashes with a `libpthread`/`GLIBC_PRIVATE` error — that's a
snap-confinement environment issue, not the sim):

```bash
source /opt/ros/jazzy/setup.bash
env -u GTK_PATH -u GIO_MODULE_DIR -u LOCPATH -u SNAP -u SNAP_LIBRARY_PATH \
  rviz2 -d rviz/g1.rviz
```

- [ ] Fixed Frame is `World` (already fixed in `rviz/g1.rviz`) and a robot
      model / TF tree is visible, not empty
- [ ] Point cloud shows something structured near the robot, not empty or a
      single flat line (watch for the known ±90° azimuth clip — see
      `Tasks.md`; a full ring is not expected yet)
- [ ] Camera image display shows the warehouse interior, not black/garbage
- [ ] (with IRA humans/carters) watch for moving humanoid figures and Nova
      Carter robots wandering the warehouse in the 3D view — this is the
      main visual confirmation that IRA actually spawned dynamic actors, not
      just the static warehouse mesh

---

## Process 2 — G1 patrol commander

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/thesis/G1_sim
python3 scripts/g1_patrol.py                          # default: 4-leg square loop, forever
python3 scripts/g1_patrol.py --once                    # one loop then exit - use this to test first
python3 scripts/g1_patrol.py --forward-speed 0.35 --leg-duration 6 --loop-legs 4
```

This is **open-loop dead reckoning** — no localization exists yet, so it
walks forward N seconds then turns, repeating a polygon. It will drift and
not close the loop exactly. That's expected, not a bug to chase (see
`FUTURE_STEPS.md`).

### Agent verification (process 2)

```bash
ros2 topic echo /g1/cmd_vel --once            # non-zero linear.x while sim's log shows forward legs
```

- [ ] `ros2 topic hz /g1/cmd_vel` shows ~10 Hz while running
- [ ] Process 1's own log (`[WH] wbc cmd=(...)`) shows the commanded vx/vy/wz
      changing between forward and turn phases, matching process 2's own
      "leg N: forward"/"leg N: turn" log lines

### Human verification (process 2)

- [ ] In RViz, watch the G1 actually walk forward then rotate in place,
      repeating — not just twitching or standing still
- [ ] `pelvis_z` in process 1's log stays stable through the turns, not just
      the straight legs

**Not yet run live this session** — the script is written and process 1's
WBC bridge is independently verified to walk on a real cmd_vel command
(see `docs/decoupled_wbc_findings.md`), but the two haven't been run
together yet. Do that before trusting this section's checkboxes.

---

## Process 3 — rosbag capture

No wrapper script yet (the existing `scripts/record_g1_bag.sh` is for the
**real robot**, different topics — don't use it here). Run directly:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/thesis/G1_sim
mkdir -p bags
ros2 bag record \
  --output bags/warehouse_$(date +%Y%m%d_%H%M%S) \
  --storage mcap --compression-mode file --compression-format zstd \
  /livox/mid360/points \
  /g1/camera/rgb /g1/camera/depth /g1/camera/camera_info \
  /g1/joint_states /g1/imu \
  /tf /tf_static /clock \
  /g1/cmd_vel
```

For the full 3-minute capture: start process 1, wait for `[WH] running` in
its log, start process 2, then start this recording command and let it run
for 180s (`timeout 180 ros2 bag record ...` or just Ctrl-C it after 3
minutes).

### Agent verification (process 3)

```bash
ros2 bag info bags/warehouse_<timestamp>
```

- [ ] Duration ≈ 180s (whatever window you actually captured)
- [ ] Every topic above appears with a nonzero message count
- [ ] `/g1/camera/rgb` and `/g1/camera/depth` message counts are roughly
      `duration * camera_publish_rate`, not near-zero (near-zero means the
      camera graph wasn't actually publishing during the capture window)

### Human verification (process 3)

**Not yet built**: a script to render the bag's camera topic to an `.mp4`
so the recorded run can actually be watched afterward (`cv_bridge` +
OpenCV `VideoWriter` was scoped but not finished this session — see
`FUTURE_STEPS.md`). Until that exists, the human-verifiable option is:

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play bags/warehouse_<timestamp>
# in another terminal, with the RViz config from Process 1 already open:
# should show the recorded run replaying - camera image, point cloud, TF
```

- [ ] Bag plays back without errors
- [ ] RViz shows the same things it showed live (camera feed, point cloud,
      moving robot) when subscribed during playback
