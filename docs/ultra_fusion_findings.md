# Ultra-Fusion — investigation findings (2026-08-10)

Repo: `~/Projects/thesis/g1_perception_ws/src/Ultra-Fusion` (clone of
`sjtuyinjie/Ultra-Fusion`, already present from a prior interrupted session).

## 0. Prior-session state check

Prior agent had only run `git clone` — `git status` is clean, `git log` shows
no local commits/edits, and there are zero build/install artifacts referencing
Ultra-Fusion anywhere under `g1_perception_ws/build` or `g1_perception_ws/install`.
No install script had been run. Nothing to resume; started fresh.

## 1. Source availability — still not released

README.md (current, 931 lines) still states explicitly:

> This repository currently releases **executable binaries and demos**. Full
> source code will be released after paper acceptance.

Confirmed by directory search: **zero** `CMakeLists.txt` or `package.xml`
anywhere in the repo. Only `Dockerfile`, `Dockerfile.ros2`, packaging/deb-install
scripts under `scripts/`, docs, and images. The earlier research summary is
still accurate on this point — no source drop has happened.

## 2. Install path — Docker is now clearly the intended/working path

Two Dockerfiles exist:
- `Dockerfile` → `FROM ubuntu:20.04`, ROS1 Noetic
- `Dockerfile.ros2` → `FROM ubuntu:22.04`, ROS2 Humble

**Key realization: the ROS1/Humble-vs-Jazzy distro mismatch noted in prior
research only applies to *native* install.** Docker containers bring their own
userspace (Ubuntu 20.04/22.04 + Humble) on top of the host kernel — the host
running Jazzy/Ubuntu 24.04 does not block this. Running via `--net=host`
lets the Humble container's DDS layer see topics published by host-side
Jazzy nodes (the README's own `docker run` examples already use `--net=host`).
This is the officially recommended path, not a workaround.

## 3. Environment check

```
df -h /            → 355G total, 13G free before starting (97% used — pre-existing, not from this task)
docker --version   → Docker version 29.7.2
nvidia-smi          → RTX 4060 Laptop, 8188 MiB
nvidia-container-toolkit 1.19.1-1 installed (dpkg -l confirms)
```
Docker + NVIDIA Container Toolkit are present and usable. GPU is not actually
required for `uf_node` (CPU-only Ceres solver), so this is moot for Ultra-Fusion
itself but confirms the toolchain is healthy.

## 4. What was attempted (ROS2 Humble path, matching our Jazzy/D435/Mid-360 setup)

1. **Image tag correction**: README says `maotiandocker/ultrafusion-ros2:0.2.1`,
   but that tag **does not exist** on Docker Hub (`docker manifest inspect`
   → "no such manifest"). Docker Hub API confirms only tag `0.2.0` is published
   (966 MB compressed / 4.29 GB on disk after extraction, pushed 2026-06-29).
   README is stale on the exact tag; noted as a real (minor) inconsistency.
2. Pulled `maotiandocker/ultrafusion-ros2:0.2.0` — succeeded. Disk 13G → 8.4G
   free (net growth ~4.6 GB after accounting for prior Docker layer cache).
   Stayed well above the 2 GB stop threshold throughout.
3. Ran `./scripts/install_ultrafusion_ros2_deb.sh` inside the container to
   install `ultrafusion-ros2_0.2.2` (deb itself is tiny, 3.6 MB, downloaded from
   GitHub Releases, SHA256 verified by the script). **First attempt failed**:
   ```
   ultrafusion-ros2 depends on ros-humble-ros2service; however: Package ros-humble-ros2service is not installed.
   ultrafusion-ros2 depends on ros-humble-std-srvs; however: Package ros-humble-std-srvs is not installed.
   ```
   Root cause: the published `0.2.0` image predates the current
   `Dockerfile.ros2` in the repo, which has a later `apt-get install
   ros-humble-ros2service ros-humble-std-srvs python3-yaml` layer (added to
   support the v0.2.2 `map_pcd` Trigger-service feature) that was never baked
   into a re-pushed image. This is a genuine image/deb version-skew bug in the
   upstream repo, not a Jazzy/Humble ABI problem.
4. Fixed by `apt-get install ros-humble-ros2service ros-humble-std-srvs
   python3-yaml` **inside the Humble container**, i.e. bringing the container
   in line with the repo's own current `Dockerfile.ros2` spec — not a
   cross-distro hack, not touching the host, not force-installing anything
   version-mismatched.
5. Re-ran the deb installer → succeeded cleanly:
   ```
   Ultra-Fusion ROS2 v0.2.2 installed.
     Binary : uf_node  (/opt/ultrafusion/bin/uf_node)
   ```
6. **Verified the binary actually runs**, not just installs: `ldd` shows no
   unresolved shared libraries, and launching a real profile
   (`uf_m3dgr_ros2_lvwio.yaml`) produces correct, non-crashing behavior —
   it parses all YAML parameters, subscribes to its configured topics,
   logs a sane "waiting for sensor readiness" warning (expected — no bag was
   playing), and shuts down cleanly on SIGTERM with a valid (empty) TUM
   trajectory write. No segfault, no ABI error, no missing-symbol error.

**Conclusion: Ultra-Fusion (ROS2 Humble, v0.2.2) installs and runs correctly
inside Docker on this Jazzy/Ubuntu 24.04 host.**

Not yet tested in this session (out of scope for "does it run" but needed
before real use):
- Feeding it live topics from actual G1 Mid-360/D435 ROS2 Jazzy drivers via
  `--net=host` (no sensor nodes were running during this check — DDS
  cross-distro compatibility for standard `sensor_msgs`/`geometry_msgs` types
  is a well-established pattern but wasn't exercised end-to-end here).
- RViz2 GUI verification (needs `-e DISPLAY` + X11 mounts, not attempted).
- Playing an actual M3DGR or custom rosbag through it.

## 5. Sensor-profile note for our rig (Mid-360 + D435, no wheel odom)

None of the released config shortcuts match our sensor suite exactly — G1 is
legged, not wheeled, so `m3dgr_*` (which defaults to `lvwio`, requiring a
`wheel_topic: /odom`) is not directly usable. Per README §3 ("Adapt to Your
Device"), the right approach is:
- Copy `/opt/ultrafusion/config/m3dgr/uf_m3dgr_ros2_lio.yaml` (or `_vio`/no
  RGB-D if D435 depth is used) as the closest starting profile — `wheel: 0`,
  `use_lidar: 1`, `use_image: 1`.
- Point `common.lid_topic`/`imu_topic` at the Mid-360 driver topics and
  `common.image0_topic`/`image1_topic` at the D435 color/aligned-depth topics.
- Fill `mapping.extrinsic_T/R` (LiDAR→IMU) and `extrinsic_TIC/RIC` (camera→IMU)
  with our actual G1 sensor mount extrinsics, and a real camera intrinsics
  YAML (`cam0_calib`) for the D435 — none of this exists yet in this repo and
  would need to be authored before a real run.

## 6. Disk / Docker feasibility summary

| Item | Value |
| --- | --- |
| Free disk before | 13 GB (pre-existing 97% full disk — not from this task) |
| Free disk after image pull + install | 8.4 GB |
| ultrafusion-ros2 image size on disk | 4.29 GB (966 MB compressed) |
| .deb package size | 3.6 MB |
| Docker / NVIDIA Container Toolkit | present, working (Docker 29.7.2, toolkit 1.19.1-1) |
| Stop threshold (per task) | 2 GB — never approached |

Root filesystem is still at 98% (8.4 GB / 355 GB free) after this work — most
of that pre-existing pressure is unrelated to Ultra-Fusion (not investigated
further per task scope). Any future large downloads (rosbags, additional
Docker layers, a `colcon build` workspace) should check `df -h /` first.

## 7. Verdict

**Not a dead end.** Ultra-Fusion ROS2 v0.2.2 installs and runs on this machine
via Docker (`maotiandocker/ultrafusion-ros2:0.2.0` image + latest `.deb`,
with two packages manually added to match the repo's current `Dockerfile.ros2`).
No distro/ABI mismatch was hit because Docker sidesteps the host being Jazzy —
the container runs its own Humble userspace. Source code is still unreleased
(binaries/deb only), so no native Jazzy build is possible, and none was
attempted per the task's constraints.

Remaining work before it's actually useful for G1: author a custom YAML
profile + camera intrinsics + extrinsics for Mid-360+D435 (no released profile
matches a legged, wheel-less rig), and validate against live/bagged G1 sensor
data over `--net=host` DDS.

## 8. Alternatives (ROS2 Jazzy-native, public source) — for later comparison

Not installed/attempted — listed per task instructions as options if the
Docker/Ultra-Fusion path stalls on the profile-authoring or live-DDS steps
above:

- **FAST-LIO2** (`hku-mars/FAST_LIO`) — LiDAR-inertial only (no visual), very
  lightweight, widely used with Livox sensors specifically (has native Livox
  driver support), public ROS2 branches exist. Good baseline if visual fusion
  isn't required.
- **Point-LIO** (`hku-mars/Point-LIO`) — same lineage as FAST-LIO2, higher-rate
  point-wise updates, also Livox-native.
- **FAST-LIVO2** (`hku-mars/FAST-LIVO2`) — LiDAR-inertial-**visual** odometry,
  closest functional match to Ultra-Fusion's LVIO mode for our exact sensor
  triplet (LiDAR + camera + IMU), public source, active repo.
  Would need a ROS2 Jazzy port check (originally ROS1/some ROS2 ports exist
  in forks — verify before committing).
- **LIO-SAM** (`TixiaoShan/LIO-SAM`) — LiDAR-inertial with SAM (smoothing and
  mapping) backend, mainstream, has ROS2 ports (e.g. community forks); no
  native visual fusion.
- **direct_lidar_inertial_odometry (DLIO)** (`vectr-ucla/direct_lidar_inertial_odometry`)
  — LiDAR-inertial, ROS2 support exists, lighter-weight than LIO-SAM.
- **LVI-SAM** (`TixiaoShan/LVI-SAM`) — LiDAR-visual-inertial, combines LIO-SAM
  + VINS-Mono; ROS1-centric, ROS2 ports less mature — would need more
  verification than FAST-LIVO2.

If visual fusion isn't a hard requirement, FAST-LIO2/Point-LIO are the
lowest-risk fallback (mature, Livox-native, minimal dependencies). If the
D435 camera fusion matters, FAST-LIVO2 is the closest public-source analog to
what Ultra-Fusion offers.
