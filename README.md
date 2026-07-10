# piper_description_rbnx

Robonix package wrapping the **AgileX Piper** arm URDF +
`robot_state_publisher`. Stand-in for `system.soma` (URDF + RSP),
which is on robonix's v0.2 roadmap. Until soma ships, this package
owns the joint-driven TF tree for the Piper arm:

```
base_link → link1 → link2 → link3 → link4 → link5 → link6
```

driven from `/arm/joint_states_single` via `robot_state_publisher`.

This package does not publish camera hand-eye calibration. In the
current vertical-grasp deploy, yolo_grasp consumes
`rbnx-boot/hand-eye-data/2d_homography.npy` to map RGB bbox pixels
directly into `arm/base_link` XY, then combines that with the configured
desktop height.

## Boot ordering

This package needs to boot **after `piper_ctl`** (which publishes
`/arm/joint_states_single`). Without joint feedback, RSP stays quiet
on /tf and the arm appears "frozen at zero" in RViz. The deploy
manifest enforces this by listing `piper_ctl` before
`piper_description` under `primitive:`.

Independent of `orbbec_camera`; the camera stream is consumed later by
llm_detect/yolo_grasp, while this package only publishes the arm TF
subtree.

## Capability surface

**Empty on purpose**. We register a provider on atlas (so rbnx boot
proceeds) but declare no routed contracts.

| Provider                                   | Capabilities |
| ------------------------------------------ | ------------ |
| `piper_description` @ `robonix/primitive/tf` | (none)       |

TF is a global ROS 2 side-channel — every tf2-aware node joins
`/tf` + `/tf_static` automatically — so atlas-routing it would only
add indirection. With `capabilities: []` rbnx boot's
`spawn_and_init` walks `wait_for_registration` → finds
`driver_contract=None` → marks the package ACTIVE without trying
to drive INIT/ACTIVATE (`deploy.rs:1247-1253`).

## Frame names — NOT prefixed with `arm/`

The vendored upstream URDF uses bare names (`base_link`, `link1`...
`link6`). MIGRATION_PLAN.md aspirationally talked about
`arm/base_link → arm/link6`, but we kept the upstream names
verbatim — that matches what `piper_ctl_rbnx` actually publishes
in `JointState.name = ['joint1', 'joint2', ..., 'joint6', 'gripper']`.

> ⚠️ **Future moving-platform conflict**: if the Piper ever sits
> on a Ranger Mini / mobile chassis whose own URDF also defines
> `base_link`, the two TF roots collide. Resolution then will be
> to fork the URDF and add a `tf_prefix` (e.g. `arm/`) to all
> link/joint names, AND update piper_ctl_rbnx's
> `JointState.name` accordingly. For the current fixed-arm
> piper_grasp deploy, no chassis is involved, so collision is
> impossible.

## Layout

```
piper_description_rbnx/
├── package_manifest.yaml        # capabilities: []
├── launch/
│   └── piper_urdf.launch.py     # robot_state_publisher + remap
├── scripts/
│   ├── build.sh                 # rbnx codegen + colcon build
│   ├── start.sh                 # source ROS, exec atlas_register_and_launch.py
│   └── atlas_register_and_launch.py
└── src/
    └── piper_description/       # vendored upstream (URDF + meshes only;
                                 # launch/, rviz/, mujoco_model/ stripped)
        ├── package.xml
        ├── CMakeLists.txt
        ├── meshes/              # *.STL (~9 MB)
        └── urdf/                # piper_description.urdf (with-gripper variant, default)
                                 # piper_no_gripper_description.urdf (override via env)
```

## Build

`scripts/build.sh` does **two** things, both fast:

1. `rbnx codegen` — generates `atlas_pb2 / atlas_pb2_grpc` Python
   stubs for `scripts/atlas_register_and_launch.py`.
2. `colcon build --packages-select piper_description` — installs
   URDF + meshes under `rbnx-build/ws/install/piper_description/share/`
   so the URDF's `package://piper_description/meshes/...` references
   resolve at RViz / MoveIt time. RSP itself doesn't need this for
   TF computation, but downstream visualisers do.

## Run standalone

```bash
bash scripts/build.sh
ROBONIX_ATLAS=127.0.0.1:50051 \
    bash scripts/start.sh
```

The script exits if atlas is unreachable (5s RPC timeout). Once
the launch is running, `/tf` should carry `base_link → link6` IFF
piper_ctl_rbnx is also up and publishing JointState.

## Choosing the URDF variant

| Variant                                | Joints                | When to use |
|----------------------------------------|-----------------------|-------------|
| `piper_description.urdf` (default)     | joint1..joint8        | `gripper_exist: true` in piper_ctl config (Stage 6 pick uses gripper) |
| `piper_no_gripper_description.urdf`    | joint1..joint6        | `gripper_exist: false` (rare; manual EE swap) |

Override by setting `PIPER_URDF_PATH` in the deploy manifest's
`env:` block, OR by modifying
`atlas_register_and_launch.py::_resolve_urdf_path` directly.

## Verification (Stage 3A deliverable)

After `rbnx boot` from `piper_grasp_deploy/` (with piper_ctl already up):

```bash
rbnx caps | grep piper_description
# Expected: piper_description provider, namespace robonix/primitive/tf, no caps

ros2 run tf2_ros tf2_echo base_link link6
# Should report a transform that moves when you ros2 topic pub to
# /arm/pos_cmd or wiggle the arm by hand.

# Camera hand-eye is checked through the 2D homography used by
# yolo_grasp, not through an easy_handeye2 TF edge in this deploy.
```

## Vendor / upstream

`src/piper_description/` is a verbatim subset of
[agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros)
`src/piper_description/`, with the following stripped (none of them
needed for TF publishing in a robonix deploy):

- `launch/` — upstream launches add `joint_state_publisher_gui` +
  RViz, both inappropriate for a headless deploy.
- `rviz/` — display config; users who want RViz should write their
  own outside this package.
- `mujoco_model/` — sim-only; not in pipeline.

## License

This package: MulanPSL-2.0 (matches robonix). Vendored URDF + meshes
inherit upstream piper_ros's licensing (TODO: declared as "TODO" in
upstream package.xml — the practical assumption is the
non-commercial Solidworks export defaults; consult AgileX before
redistribution).
