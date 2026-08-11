"""Optional humans + Nova Carters via Isaac Replicator Agent (IRA), for the
warehouse SDG scene.

IRA (``isaacsim.replicator.agent.core``, extension version 1.6.8 on this
install - noticeably different API from the 5.1.0-era docs/``sdg_scheduler.py``
CLI those docs describe) is entirely config-driven: build a YAML matching its
``RootConfig`` schema, then call two async entry points:

    isaacsim.replicator.agent.core.api.load_config_file(path)
    await isaacsim.replicator.agent.core.api.setup_simulation()

``setup_simulation()`` opens the environment USD **as the new root stage**
(``StageManager.wait_for_stage_open``, a full stage replace - not a
reference), then loads characters, then robots, baking a navmesh in between.
Because it owns the stage, IRA must run *before* anything else is added to
the scene - the caller adds G1 and its sensors afterward, on the resulting
stage.

**Real bug found and worked around**: ``EnvironmentLoader.load()`` calls
``omni.metropolis.pipeline.simulation_util.ensure_navmesh_ready()``, which
polls ``inav.get_navmesh()`` for a **hard-coded 100 frames** with no config
knob to raise it. On the real ``warehouse.usd`` (291 meshes) baking
genuinely succeeds but needs ~600-700 frames (~7-8s headless on this
machine), so every attempt failed with "NavMesh building failed after 101
frames" even though baking was progressing fine. Confirmed by direct
`omni.anim.navigation.core` testing outside IRA: ``start_navmesh_baking()``
can only be made to return ``True`` via IRA's own pipeline (a standalone
reference-add of the warehouse USD returns ``can_bake=False`` - IRA's
environment/collider setup does something extra first that makes baking
possible at all), and once baking is running it finishes in under 700
frames, comfortably inside a raised budget. ``environment_loader.py`` does
``from ...simulation_util import ensure_navmesh_ready``, which copies the
name into its own module namespace, so the fix has to patch that module's
attribute specifically - patching the origin module has no effect.
"""

from __future__ import annotations

import time
from pathlib import Path

DEFAULT_MAX_NAVMESH_FRAMES = 3000  # ~30x the stock cap; real bake measured ~700 frames

WAREHOUSE_REL = "Isaac/Environments/Simple_Warehouse/warehouse.usd"  # IRA-relative form (no leading slash)


def write_config(
    path: Path,
    *,
    warehouse_rel: str = WAREHOUSE_REL,
    num_humans: int = 2,
    num_carters: int = 1,
    seed: int = 42,
    duration_s: float = 200.0,
) -> Path:
    """Write a minimal IRA YAML config: N wandering characters + N wandering
    Nova Carters in the given warehouse. No ``sensor``/``replicator`` section
    - we don't want IRA's own writer-driven capture loop, only the actors it
    spawns; our own G1 sensors do the real ROS2 publishing.
    """
    import yaml

    config = {
        "isaacsim.replicator.agent": {
            "version": "1.6.0",
            "seed": seed,
            "simulation_duration": duration_s,
            "environment": {"base_stage_asset_path": warehouse_rel},
        }
    }
    if num_humans > 0:
        config["isaacsim.replicator.agent"]["character"] = {
            "groups": {
                "humans": {
                    "num": num_humans,
                    "asset_path": "Isaac/People/Characters/",
                    "routines": [
                        {
                            "wander": {
                                "weight": 1.0,
                                "repeat": 1,
                                "walk": {
                                    "speed_range": [0.8, 1.2],
                                    "distance_range": [3.0, 8.0],
                                    "navigation_areas": [],
                                },
                                "idle": [
                                    {"animation": "idle", "weight": 1.0, "time_range": [2.0, 4.0]}
                                ],
                            }
                        }
                    ],
                }
            }
        }
    if num_carters > 0:
        config["isaacsim.replicator.agent"]["robot"] = {
            "groups": {
                "carters": {
                    "num": num_carters,
                    "config_file_path": "nova_carter.yaml",
                    "routines": [
                        {
                            "wander": {
                                "weight": 1.0,
                                "repeat": 1,
                                "move": {"distance_range": [5.0, 10.0], "navigation_areas": []},
                                "idle": {"time_range": [2.0, 4.0]},
                            }
                        }
                    ],
                }
            }
        }

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return path


def enable_extension() -> None:
    """Enable the IRA core extension. Must happen before importing its api
    module, same requirement as the ROS2 bridge extension elsewhere in this
    codebase."""
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.replicator.agent.core", True)
    for _ in range(30):
        omni.kit.app.get_app().update()


def _patch_navmesh_frame_budget(max_frames: int = DEFAULT_MAX_NAVMESH_FRAMES) -> None:
    """See module docstring - raises the hard-coded 100-frame navmesh bake
    poll cap that makes ``EnvironmentLoader.load()`` fail on real warehouse
    meshes. Patches the name as imported into
    ``isaacsim.replicator.agent.core.scene_assembly.environment_loader``,
    not the origin ``omni.metropolis.pipeline.simulation_util`` module."""
    import carb
    import omni.kit.app
    import isaacsim.replicator.agent.core.scene_assembly.environment_loader as environment_loader

    async def _ensure_navmesh_ready_patched() -> bool:
        import omni.anim.navigation.core as nav

        inav = nav.acquire_interface()
        if inav is None:
            carb.log_error("[ira_actors] NavMesh interface not available")
            return False
        if inav.get_navmesh() is not None:
            return True
        if not inav.start_navmesh_baking():
            carb.log_error("[ira_actors] start_navmesh_baking() returned False")
            return False
        frame_count = 0
        t0 = time.time()
        while inav.get_navmesh() is None:
            await omni.kit.app.get_app().next_update_async()
            frame_count += 1
            if frame_count > max_frames:
                carb.log_error(
                    f"[ira_actors] navmesh bake exceeded raised budget of {max_frames} frames "
                    f"({time.time()-t0:.1f}s) - stock IRA cap is 100, this needs raising further"
                )
                return False
        print(f"[ira_actors] navmesh baked in {frame_count} frames ({time.time()-t0:.1f}s)")
        return True

    environment_loader.ensure_navmesh_ready = _ensure_navmesh_ready_patched


async def setup(
    config_path: Path,
    *,
    max_navmesh_frames: int = DEFAULT_MAX_NAVMESH_FRAMES,
) -> bool:
    """Load ``config_path`` and run IRA's full setup (environment + navmesh +
    characters + robots) on the current stage - which becomes a *new* stage,
    replacing whatever was open before. Returns ``True`` on success, ``False``
    on any failure (config validation, navmesh bake, asset load) so the
    caller can fall back to a plain warehouse load with no actors rather than
    crashing the whole pipeline over an IRA-specific failure.
    """
    from isaacsim.replicator.agent.core import api as ira_api

    _patch_navmesh_frame_budget(max_navmesh_frames)

    if not ira_api.load_config_file(str(config_path)):
        print(f"[ira_actors] FAILED to load config {config_path}")
        return False

    try:
        await ira_api.setup_simulation()
    except Exception as e:
        print(f"[ira_actors] FAILED setup_simulation(): {e!r}")
        return False

    return True


def discover_prims_at(stage, scope_path: str) -> list[str]:
    """Return the direct children of ``scope_path``, or ``[]`` if it doesn't
    exist (e.g. the carters scope when ``--num-carters 0``)."""
    prim = stage.GetPrimAtPath(scope_path)
    if not prim.IsValid():
        return []
    return [child.GetPath().pathString for child in prim.GetChildren()]


def discover_actor_prims(
    stage,
    humans_scope: str = "/World/Characters/humans",
    carters_scope: str = "/World/Robots/carters",
) -> list[str]:
    """Return the root Xform of every spawned human and Nova Carter.

    IRA names actors ``humans_0``, ``humans_1``, ... and (for a single
    carter) plain ``Nova_Carter`` with no index suffix, so the count-based
    naming isn't predictable from ``--num-humans``/``--num-carters`` alone -
    read it back from the stage instead of guessing the pattern.
    """
    return discover_prims_at(stage, humans_scope) + discover_prims_at(stage, carters_scope)


def find_imu_prim(stage, root_path: str):
    """Find an existing IMU sensor prim already under ``root_path``.

    Nova Carter comes from IRA with its own IMU already set up (per NVIDIA's
    Replicator Agent robot-properties docs) - creating a *second* one via
    ``IMU.create()`` applies a physics schema to the rigid body IRA's own
    wander controller already holds a tensor view into, which invalidates
    that view and spams "Simulation view object is invalidated" on every
    tick thereafter (confirmed live with 4 carters, 2026-08-10). Read the
    stock sensor instead of authoring a new one.
    """
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        type_name = prim.GetTypeName()
        if "imu" in type_name.lower() or "imu" in prim.GetName().lower():
            return prim.GetPath().pathString
    return None


def attach_carter_imu_publishers(
    stage,
    carter_prim_paths: list[str],
) -> list[str]:
    """Publish each Nova Carter's existing IMU sensor (IRA's own, not one we
    create) as ``sensor_msgs/Imu`` on ``/carter_N/imu``."""
    from g1_sim.rtx_camera import attach_imu_publisher

    graph_paths = []
    for i, carter_path in enumerate(carter_prim_paths):
        imu_prim = find_imu_prim(stage, carter_path)
        if imu_prim is None:
            print(f"[ira_actors] carter {i} IMU  : not found under {carter_path}, skipping")
            continue
        topic = f"/carter_{i}/imu"
        graph_path = attach_imu_publisher(
            imu_prim,
            graph_path=f"/ActionGraph/CarterImuROS2_{i}",
            topic=topic,
            frame_id=f"carter_{i}_imu",
        )
        graph_paths.append(graph_path)
        print(f"[ira_actors] carter {i} IMU  : {topic}  prim={imu_prim}")
    return graph_paths


def attach_actor_tf_publishers(
    actor_prim_paths: list[str],
    graph_path: str = "/ActionGraph/ActorsTF",
    topic_name: str = "/tf",
) -> str | None:
    """Publish one TF frame per human/Nova Carter (root pose only).

    Deliberately targets each actor's top-level Xform, not any nested
    SkelRoot/articulation - ``ROS2PublishTransformTree`` only walks into an
    articulation tree when the *target* prim itself is an articulation root,
    so pointing it at the plain wrapper Xform gives exactly one frame per
    actor instead of exploding into every skeleton bone or wheel joint.
    """
    if not actor_prim_paths:
        return None

    import omni.graph.core as og

    nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishActorsTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    connections = [
        ("OnTick.outputs:tick", "PublishActorsTF.inputs:execIn"),
        ("Context.outputs:context", "PublishActorsTF.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishActorsTF.inputs:timeStamp"),
    ]
    values = [
        ("PublishActorsTF.inputs:topicName", topic_name),
        # targetPrims is a "target" (relationship) input, not a plain data
        # attribute - it has to go through SET_VALUES in this same edit()
        # call. A separate og.Controller.set() after the fact (the first
        # attempt) silently no-ops on relationship inputs, leaving
        # targetPrims unset - which is what caused the "[PoseTree] target
        # getObjectType eInvalid" spam (confirmed live, 2026-08-10): the node
        # was running with no targets at all. Matches how
        # attach_robot_state_publishers() below sets targetPrims/parentPrim.
        ("PublishActorsTF.inputs:targetPrims", actor_prim_paths),
        # Leaving parentPrim blank ("use World" per the node's own docs) was
        # the first attempt and produced a second, parallel spam: "[PoseTree]
        # parent getObjectType eInvalid for '/World'" on every tick, even
        # after targetPrims was fixed (confirmed live, 2026-08-10) - the
        # blank-default path doesn't actually resolve cleanly in practice.
        # Setting it explicitly is what already works for the G1 publisher.
        ("PublishActorsTF.inputs:parentPrim", ["/World"]),
    ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connections,
            og.Controller.Keys.SET_VALUES: values,
        },
    )
    return graph_path


def run_setup_blocking(
    simulation_app,
    config_path: Path,
    *,
    max_navmesh_frames: int = DEFAULT_MAX_NAVMESH_FRAMES,
    timeout_s: float = 240.0,
) -> bool:
    """Synchronous wrapper: pumps ``simulation_app.update()`` until
    :func:`setup` (a coroutine) finishes or ``timeout_s`` elapses. Standalone
    Isaac Sim scripts have no running asyncio event loop of their own, so
    this is the same "ensure_future + drive the app manually" pattern used
    elsewhere for one-off async Isaac Sim calls.
    """
    import asyncio

    task = asyncio.ensure_future(setup(config_path, max_navmesh_frames=max_navmesh_frames))
    t0 = time.time()
    last_log = t0
    while not task.done():
        simulation_app.update()
        if time.time() - last_log > 10.0:
            print(f"[ira_actors] setup_simulation still running ({time.time()-t0:.1f}s)")
            last_log = time.time()
        if time.time() - t0 > timeout_s:
            print(f"[ira_actors] FAILED: setup exceeded {timeout_s}s timeout")
            return False

    exc = task.exception()
    if exc is not None:
        print(f"[ira_actors] FAILED: {exc!r}")
        return False

    ok = task.result()
    print(f"[ira_actors] setup {'OK' if ok else 'FAILED'} in {time.time()-t0:.1f}s")
    return ok
