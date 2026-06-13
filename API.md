# robot_tools API Reference

API reference for all packages installed by this distribution. See
[README.md](README.md) for installation and quickstart examples.

- [robot_tools](#robot_tools) — kinematics, dynamics, collision checking
- [robot_visualization](#robot_visualization) — PyVista 3D visualization
- [robot_video_tools](#robot_video_tools) — video generation and overlays
- [urdfpy](#urdfpy) — URDF parsing (vendored fork)

---

## robot_tools

### RobotModel

```python
RobotModel(urdf_file, p0=np.zeros(3), R0=np.eye(3), Tgp=np.eye(4))
```

Kinematics and dynamics of a robot loaded from a URDF file (Pinocchio backend).

| Parameter | Type | Description |
|---|---|---|
| `urdf_file` | `str` | Path to the URDF file |
| `p0` | `(3,) ndarray` | Base position in the world frame |
| `R0` | `(3,3) ndarray` | Base orientation in the world frame |
| `Tgp` | `(4,4) ndarray` | Transform from a robot frame to the grasp frame |

**Attributes:** `pin_model` / `pin_data` (Pinocchio objects), `nq` (DOF),
`T0` (`pinocchio.SE3` base transform), `Tgp` (`pinocchio.SE3` grasp offset).

**Methods**

- `update_kinematics(frame_name, q, dq) -> (T, J)` — grasp-frame pose
  (`pinocchio.SE3`, world frame; access `T.translation` / `T.rotation`) and 6xN
  world-aligned Jacobian (linear rows 0–2, angular rows 3–5).
- `update_frame_kinematics(frame_name, q, dq, ddq) -> (T, v, a)` — pose, spatial
  velocity, and spatial acceleration of a robot frame (world-aligned).
- `update_dynamics(q, dq) -> (M, c, g)` — symmetric mass matrix,
  Coriolis/centrifugal torques, and gravity torques for
  `tau = M(q) @ ddq + c(q, dq) + g(q)`.
  Note: gravity is assumed along `[0, 0, -9.81]` in the *base* frame; it is not
  rotated by `R0`.

### Collision checking

All primitives take an obstacle dictionary with a `"type"` key, a 4x4 transform
`"T"` (world placement), and shape parameters, and expose:

- `is_in_collision(point) -> bool` — whether a world-frame point lies inside or
  on the primitive.
- `to_local(point) -> ndarray` — transform a world point to the primitive frame.

| Class | `"type"` | Shape parameters |
|---|---|---|
| `EllipsoidCollision` | `"ellipsoid"` | `"xradius"`, `"yradius"`, `"zradius"` |
| `CylinderCollision` | `"cylinder"` | `"radius"`, `"height"` (axis = local z) |
| `BoxCollision` | `"box"` | `"xsize"`, `"ysize"`, `"zsize"` |

- `create_collision_objects(obstacles_list) -> list[CollisionObject]` — factory
  building one object per dictionary; raises `ValueError` on unknown types.
- `CollisionObject` — abstract base class for custom primitives.
- `register_collision_type(name, cls)` — register a `CollisionObject` subclass so
  `create_collision_objects` accepts its `"type"` name.

---

## robot_visualization

### Robot

```python
Robot(urdf_file, plotter=None, **kwargs)
```

PyVista visualization of a URDF robot. Multiple instances of the same robot can be
shown via mesh `id`s (e.g. start and goal configurations).

| Keyword | Default | Description |
|---|---|---|
| `color` | `'lightgray'` | Mesh color |
| `opacity` | `1.0` | Mesh opacity |
| `p0` | `np.zeros(3)` | Base position |
| `R0` | `np.eye(3)` | Base orientation |
| `decimate` | `None` | Optional mesh decimation factor (0..1) for large meshes |

**Methods**

- `set_robot_mesh(id=0, color=None, opacity=None)` — add a robot mesh instance to
  the plotter.
- `update(q, id=0, color=None, opacity=None)` — move mesh instance `id` to joint
  configuration `q` (creates it if missing).
- `fk(q, ee_link_name) -> (4,4) ndarray` — world-frame link pose.
- `toggle_mesh_type()` — switch between visual and collision meshes (call
  `update()` afterwards).
- `plot_ee(q, Tgp=np.eye(4), ee_link_name=..., color="red", size=0.01, type="sphere")`
  — end-effector marker (`"sphere"`, `"cube"`, or `"cross"`).
- `plot_ee_frame(q, Tgp=np.eye(4), ee_link_name=..., color=None, scale=0.1)` —
  coordinate frame at the end-effector.
- `plot_ee_path(path, actor=None, Tgp=np.eye(4), ee_link_name=..., color=None,
  opacity=None, line_width=4) -> actor` — polyline through the end-effector
  positions of a joint-space path; pass the returned `actor` again to update it
  in place.

### Primitives

- `AxesVisualizer(plotter, origin=None, scale=1.0, color=None, line_width=2)` —
  RGB coordinate frame; `update(position, rotation)` accepts a Rodrigues vector or
  rotation matrix; `plot_path(p1, p2, color, line_width)` draws a line.
- `ArrowVisualizer(plotter, origin=None, direction=None, scale=1.0, color='white')`
  — single arrow; `update(origin, direction)`.
- `BoxVisualizer(plotter, x_size, y_size, z_size, color, opacity)` — pose-tracked
  boxes by `id`; `update(id, T, color=None, opacity=None)` with a 4x4 transform.

### video_utils

Shared helpers used by `robot_video_tools`: `calculate_fps_from_timestamps`,
`add_timestamp_to_frame`, `create_video_writer`, `interpolate_frame_count`,
`display_frame`, `close_all_windows`.

---

## robot_video_tools

### Video generation

- `generate_video_from_images(image_folder, output_video)` — build an MP4 from PNG
  frames named `*_<seconds>.png`; playback speed is derived from the timestamps
  and gaps are padded. Also available as the `robot-gen-video` CLI.
- `gif_to_mp4(input_gif, output_mp4, desired_duration)` — convert a GIF to MP4,
  resampling frames to the target duration in seconds.

### Camera-calibrated overlays

- `CalibrationPaths(transform_file, intrinsic_file, dist_coeffs_file)` — paths to
  hand-eye calibration outputs; `CalibrationPaths.from_directory(dir)` expects
  `O_T_C.txt`, `camera_matrix.txt`, `dist_coeffs.txt`.
- `calibrate_camera(image_path, calibration, off_screen=True) -> pv.Plotter` —
  plotter whose virtual camera matches the physical camera.
- `ImageOverlay(image_folder, calibration, n_layers=1)` — composites PyVista
  scenes onto a timestamped image sequence:
  - `plotter` / `layer(i)` — calibrated off-screen plotters to add scene objects to.
  - `get_frame_index(t) -> int | None` — background image index for time `t`.
  - `render(t) -> (frame, k, timestamp) | None` — composited BGR frame.
  - `add_time_stamp(frame, timestamp, ...)` — draw a timestamp onto a frame.

### Animation

- `AnimationState` — mutable per-run state (`idx_path`, `idx_query`,
  `path_actors`); `reset()` restarts.
- `animate_step(j, jp, t_end, time, data, mpdata, colors, robots, robots_des,
  state, ee_link_name=..., plot_robot=True)` — advance one frame of a logged
  robot + motion-planner animation; returns 2D pixel coordinates
  `(query_point, recv_point, bif_point)` for annotation overlays.
- `world_to_display(plotter, point3d) -> (2,) ndarray` — project a 3D world point
  to top-left-origin pixel coordinates.

---

## urdfpy

Vendored fork of [urdfpy](https://github.com/mmatl/urdfpy) (installed automatically;
do **not** install urdfpy from PyPI alongside it). Most relevant entry points:

```python
from urdfpy import URDF

robot = URDF.load("robot_assets/urdf/iiwa7.urdf")
robot.links, robot.joints           # kinematic structure
fk = robot.link_fk(q, "link_name")  # 4x4 link pose
fk = robot.visual_trimesh_fk(q)     # {trimesh: pose} for all visual meshes
```

See the [urdfpy documentation](https://urdfpy.readthedocs.io/) for the full API.
