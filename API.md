# robot_tools API Documentation

Complete API reference for the robot_tools package.

## Table of Contents

1. [robot_tools Module](#robot_tools-module)
   - [RobotModel](#robotmodel)
   - [Collision Detection](#collision-detection)
2. [robot_visualization Module](#robot_visualization-module)
   - [Robot](#robot)
   - [AxesVisualizer](#axesvisualizer)
   - [ArrowVisualizer](#arrowvisualizer)
3. [urdfpy Module](#urdfpy-module)
   - [URDF](#urdf)

---

## robot_tools Module

The main module for robot modeling, kinematics, dynamics, and collision detection.

### RobotModel

```python
class RobotModel(urdf_file, p0=np.zeros(3), R0=np.eye(3), Tgp=np.eye(4))
```

Main class for computing robot kinematics and dynamics using Pinocchio.

#### Parameters

- **urdf_file** (*str*): Path to the URDF file, relative to `robot_assets/urdf/` directory
- **p0** (*np.ndarray*, optional): Base position in world frame [x, y, z]. Default: `[0, 0, 0]`
- **R0** (*np.ndarray*, optional): Base orientation (3x3 rotation matrix). Default: `np.eye(3)`
- **Tgp** (*np.ndarray*, optional): Grasp point to end-effector transformation (4x4 homogeneous matrix). Default: `np.eye(4)`

#### Attributes

- **pin_model** (*pinocchio.Model*): Pinocchio model object
- **pin_data** (*pinocchio.Data*): Pinocchio data object
- **nq** (*int*): Number of joints (degrees of freedom)
- **p0** (*np.ndarray*): Base position
- **R0** (*np.ndarray*): Base rotation matrix
- **T0** (*pinocchio.SE3*): Base transformation
- **Tgp** (*pinocchio.SE3*): Grasp point transformation
- **Ggp** (*np.ndarray*): Grasp point adjoint matrix (6x6)

#### Methods

##### update_kinematics

```python
def update_kinematics(frame_name, q, dq) -> tuple
```

Compute forward kinematics and Jacobian for a specified frame.

**Parameters:**
- **frame_name** (*str*): Name of the frame (e.g., "end_effector_link")
- **q** (*np.ndarray*): Joint positions (nq,)
- **dq** (*np.ndarray*): Joint velocities (nq,)

**Returns:**
- **T** (*pinocchio.SE3*): Frame pose in world coordinates
  - Access position: `T.translation`
  - Access rotation: `T.rotation`
- **J** (*np.ndarray*): Geometric Jacobian (6 x nq) in world frame
  - Rows 0-2: Linear velocity Jacobian
  - Rows 3-5: Angular velocity Jacobian

**Example:**
```python
robot = RobotModel("iiwa7.urdf")
q = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
dq = np.zeros(7)
T, J = robot.update_kinematics("lbr1_gripper_link_ee", q, dq)

# Extract position and orientation
position = T.translation
rotation = T.rotation
```

##### update_dynamics

```python
def update_dynamics(q, dq) -> tuple
```

Compute robot dynamics matrices.

**Parameters:**
- **q** (*np.ndarray*): Joint positions (nq,)
- **dq** (*np.ndarray*): Joint velocities (nq,)

**Returns:**
- **M** (*np.ndarray*): Joint-space mass matrix (nq x nq), symmetric positive definite
- **c** (*np.ndarray*): Coriolis and centrifugal forces vector (nq,)
- **g** (*np.ndarray*): Gravity forces vector (nq,)

**Dynamics Equation:**
```
τ = M(q)q̈ + c(q,q̇) + g(q)
```

**Example:**
```python
robot = RobotModel("iiwa7.urdf")
q = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
dq = np.zeros(7)
M, c, g = robot.update_dynamics(q, dq)

# Required torque for desired acceleration
ddq_desired = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
tau = M @ ddq_desired + c + g
```

---

### Collision Detection

#### EllipsoidCollision

```python
class EllipsoidCollision(obstacle)
```

Ellipsoid collision primitive for fast collision checking.

**Parameters:**
- **obstacle** (*dict*): Dictionary with keys:
  - `"type"` (*str*): Must be `"ellipsoid"`
  - `"T"` (*array_like*): 4x4 transformation matrix
  - `"xradius"` (*float*): Radius along x-axis
  - `"yradius"` (*float*): Radius along y-axis
  - `"zradius"` (*float*): Radius along z-axis

**Attributes:**
- **T** (*np.ndarray*): Transformation matrix (4x4)
- **radii** (*np.ndarray*): Radii [rx, ry, rz]
- **rotation_inv** (*np.ndarray*): Inverse of rotation matrix

**Methods:**

##### is_in_collision

```python
def is_in_collision(point) -> bool
```

Check if a point is inside or on the ellipsoid surface.

**Parameters:**
- **point** (*np.ndarray*): 3D point coordinates [x, y, z]

**Returns:**
- **bool**: True if point is inside or on surface, False otherwise

**Example:**
```python
obstacle = {
    "type": "ellipsoid",
    "T": np.eye(4),
    "xradius": 0.5,
    "yradius": 0.5,
    "zradius": 1.0
}
ellipsoid = EllipsoidCollision(obstacle)
point = np.array([0.0, 0.0, 0.5])
in_collision = ellipsoid.is_in_collision(point)
```

#### CylinderCollision

```python
class CylinderCollision(obstacle)
```

Cylinder collision primitive for fast collision checking.

**Parameters:**
- **obstacle** (*dict*): Dictionary with keys:
  - `"type"` (*str*): Must be `"cylinder"`
  - `"T"` (*array_like*): 4x4 transformation matrix (z-axis aligned)
  - `"radius"` (*float*): Cylinder radius
  - `"height"` (*float*): Cylinder height

**Attributes:**
- **T** (*np.ndarray*): Transformation matrix (4x4)
- **radius** (*float*): Cylinder radius
- **height** (*float*): Cylinder height
- **rotation_inv** (*np.ndarray*): Inverse of rotation matrix

**Methods:**

##### is_in_collision

```python
def is_in_collision(point) -> bool
```

Check if a point is inside the cylinder.

**Parameters:**
- **point** (*np.ndarray*): 3D point coordinates [x, y, z]

**Returns:**
- **bool**: True if point is inside, False otherwise

**Example:**
```python
obstacle = {
    "type": "cylinder",
    "T": np.eye(4),
    "radius": 0.3,
    "height": 1.0
}
cylinder = CylinderCollision(obstacle)
point = np.array([0.1, 0.1, 0.3])
in_collision = cylinder.is_in_collision(point)
```

#### BoxCollision

```python
class BoxCollision(obstacle)
```

Box collision primitive for fast collision checking.

**Parameters:**
- **obstacle** (*dict*): Dictionary with keys:
  - `"type"` (*str*): Must be `"box"`
  - `"T"` (*array_like*): 4x4 transformation matrix
  - `"xsize"` (*float*): Box size along x-axis
  - `"ysize"` (*float*): Box size along y-axis
  - `"zsize"` (*float*): Box size along z-axis

**Attributes:**
- **T** (*np.ndarray*): Transformation matrix (4x4)
- **size** (*np.ndarray*): Box dimensions [sx, sy, sz]
- **rotation_inv** (*np.ndarray*): Inverse of rotation matrix

**Methods:**

##### is_in_collision

```python
def is_in_collision(point) -> bool
```

Check if a point is inside the box.

**Parameters:**
- **point** (*np.ndarray*): 3D point coordinates [x, y, z]

**Returns:**
- **bool**: True if point is inside, False otherwise

**Example:**
```python
obstacle = {
    "type": "box",
    "T": np.eye(4),
    "xsize": 0.5,
    "ysize": 0.5,
    "zsize": 1.0
}
box = BoxCollision(obstacle)
point = np.array([0.1, 0.1, 0.3])
in_collision = box.is_in_collision(point)
```

#### create_collision_objects

```python
def create_collision_objects(obstacles_list) -> list
```

Factory function to create collision objects from a list of obstacle definitions.

**Parameters:**
- **obstacles_list** (*list*): List of obstacle dictionaries

**Returns:**
- **list**: List of collision objects (EllipsoidCollision, CylinderCollision, or BoxCollision)

**Raises:**
- **ValueError**: If an unknown obstacle type is provided

**Example:**
```python
obstacles = [
    {"type": "ellipsoid", "T": np.eye(4), "xradius": 0.5, "yradius": 0.5, "zradius": 1.0},
    {"type": "cylinder", "T": np.eye(4), "radius": 0.3, "height": 1.0},
    {"type": "box", "T": np.eye(4), "xsize": 0.5, "ysize": 0.5, "zsize": 1.0}
]
collision_objects = create_collision_objects(obstacles)

# Check collision for a trajectory
for point in trajectory:
    for obj in collision_objects:
        if obj.is_in_collision(point):
            print("Collision detected!")
```

---

## robot_visualization Module

Module for interactive 3D robot visualization using PyVista.

### Robot

```python
class Robot(urdf_file, plotter=None, p0=np.zeros(3), R0=np.eye(3), color='lightgray', opacity=1.0)
```

Main class for robot visualization and forward kinematics.

#### Parameters

- **urdf_file** (*str*): Path to URDF file, relative to `robot_assets/urdf/`
- **plotter** (*pv.Plotter*, optional): PyVista plotter instance. If None, visualization disabled
- **p0** (*np.ndarray*, optional): Base position [x, y, z]. Default: `[0, 0, 0]`
- **R0** (*np.ndarray*, optional): Base rotation matrix (3x3). Default: `np.eye(3)`
- **color** (*str*, optional): Robot mesh color. Default: `'lightgray'`
- **opacity** (*float*, optional): Robot mesh opacity (0-1). Default: `1.0`

#### Attributes

- **urdf** (*URDF*): URDF object
- **plotter** (*pv.Plotter*): PyVista plotter
- **color** (*str*): Mesh color
- **opacity** (*float*): Mesh opacity
- **p0** (*np.ndarray*): Base position
- **R0** (*np.ndarray*): Base rotation
- **robot_actors** (*dict*): Dictionary of robot mesh actors

#### Methods

##### set_robot_mesh

```python
def set_robot_mesh(id=0)
```

Initialize and add robot mesh to the plotter.

**Parameters:**
- **id** (*int*, optional): Robot instance ID for multiple robots. Default: `0`

**Example:**
```python
robot = Robot("iiwa7.urdf", plotter=plotter)
robot.set_robot_mesh(id=0)  # First robot
robot.set_robot_mesh(id=1)  # Second robot (different configuration)
```

##### update

```python
def update(q, id=0)
```

Update robot configuration to new joint angles.

**Parameters:**
- **q** (*np.ndarray*): Joint positions (nq,)
- **id** (*int*, optional): Robot instance ID. Default: `0`

**Example:**
```python
q_new = np.array([0.5, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
robot.update(q_new, id=0)
```

##### fk

```python
def fk(q, ee_link_name) -> np.ndarray
```

Compute forward kinematics for end-effector.

**Parameters:**
- **q** (*np.ndarray*): Joint positions (nq,)
- **ee_link_name** (*str*): End-effector link name

**Returns:**
- **np.ndarray**: 4x4 homogeneous transformation matrix

**Example:**
```python
q = np.array([0.5, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
T_ee = robot.fk(q, "lbr1_gripper_link_ee")
position = T_ee[:3, 3]
orientation = T_ee[:3, :3]
```

##### plot_ee

```python
def plot_ee(q, ee_link_name, color='red', size=0.02, type='sphere')
```

Plot end-effector position marker.

**Parameters:**
- **q** (*np.ndarray*): Joint positions (nq,)
- **ee_link_name** (*str*): End-effector link name
- **color** (*str*, optional): Marker color. Default: `'red'`
- **size** (*float*, optional): Marker size. Default: `0.02`
- **type** (*str*, optional): Marker type ('sphere', 'cube', 'cross'). Default: `'sphere'`

**Example:**
```python
robot.plot_ee(q, "end_effector", color='blue', size=0.03, type='sphere')
```

##### plot_ee_frame

```python
def plot_ee_frame(q, ee_link_name)
```

Plot end-effector coordinate frame (x-red, y-green, z-blue).

**Parameters:**
- **q** (*np.ndarray*): Joint positions (nq,)
- **ee_link_name** (*str*): End-effector link name

**Example:**
```python
q = np.array([0.5, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
robot.plot_ee_frame(q, "lbr1_gripper_link_ee")
```

##### plot_ee_path

```python
def plot_ee_path(q_path, ee_link_name, color='blue', opacity=1.0, line_width=4)
```

Plot end-effector trajectory path.

**Parameters:**
- **q_path** (*np.ndarray*): Array of joint configurations (n_steps, nq)
- **ee_link_name** (*str*): End-effector link name
- **color** (*str*, optional): Path color. Default: `'blue'`
- **opacity** (*float*, optional): Path opacity (0-1). Default: `1.0`
- **line_width** (*float*, optional): Line width. Default: `4`

**Example:**
```python
q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_goal = np.array([1.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
q_path = np.linspace(q_start, q_goal, 50)

robot.plot_ee_path(q_path, "end_effector", color='green', line_width=5)
```

---

### AxesVisualizer

```python
class AxesVisualizer(plotter, origin=None, scale=1.0)
```

Visualize 3D coordinate frames (axes).

#### Parameters

- **plotter** (*pv.Plotter*): PyVista plotter instance
- **origin** (*array_like*, optional): Frame origin [x, y, z]. Default: `[0, 0, 0]`
- **scale** (*float*, optional): Axes scale factor. Default: `1.0`

#### Attributes

- **plotter** (*pv.Plotter*): PyVista plotter
- **scale** (*float*): Axes scale
- **axes** (*dict*): Dictionary of axes actors

#### Methods

##### update

```python
def update(position, rotation)
```

Update coordinate frame position and orientation.

**Parameters:**
- **position** (*np.ndarray*): New frame origin [x, y, z]
- **rotation** (*np.ndarray*): Rotation as Euler angles [rx, ry, rz] in radians

**Example:**
```python
axes = AxesVisualizer(plotter, origin=[0, 0, 0], scale=0.5)
axes.update(position=[1.0, 0.5, 0.2], rotation=[0.0, 0.0, np.pi/4])
```

##### plot_path

```python
def plot_path(p1, p2, color='gray', line_width=2)
```

Draw a line between two points.

**Parameters:**
- **p1** (*np.ndarray*): Start point [x, y, z]
- **p2** (*np.ndarray*): End point [x, y, z]
- **color** (*str*, optional): Line color. Default: `'gray'`
- **line_width** (*float*, optional): Line width. Default: `2`

**Example:**
```python
axes = AxesVisualizer(plotter)
axes.plot_path([0, 0, 0], [1, 1, 1], color='blue', line_width=3)
```

---

### ArrowVisualizer

```python
class ArrowVisualizer(plotter, origin=None, direction=None, scale=1.0, color='white')
```

Visualize 3D arrows for directions/vectors.

#### Parameters

- **plotter** (*pv.Plotter*): PyVista plotter instance
- **origin** (*array_like*, optional): Arrow origin [x, y, z]. Default: `[0, 0, 0]`
- **direction** (*array_like*, optional): Arrow direction [dx, dy, dz]. Default: `[1, 0, 0]`
- **scale** (*float*, optional): Arrow scale factor. Default: `1.0`
- **color** (*str*, optional): Arrow color. Default: `'white'`

#### Attributes

- **plotter** (*pv.Plotter*): PyVista plotter
- **scale** (*float*): Arrow scale
- **color** (*str*): Arrow color
- **arrow_actor** (*pv.Actor*): PyVista arrow actor

#### Methods

##### update

```python
def update(origin, direction)
```

Update arrow position and direction.

**Parameters:**
- **origin** (*np.ndarray*): New arrow origin [x, y, z]
- **direction** (*np.ndarray*): New arrow direction [dx, dy, dz]

**Example:**
```python
arrow = ArrowVisualizer(plotter, origin=[0, 0, 0], direction=[1, 0, 0], scale=0.5, color='red')
arrow.update(origin=[0.5, 0.5, 0.5], direction=[0, 1, 0])
```

---

## urdfpy Module

URDF parsing and manipulation module.

### URDF

```python
class URDF
```

URDF model representation and parser.

#### Class Methods

##### load

```python
@classmethod
def load(cls, file_path) -> URDF
```

Load URDF from file.

**Parameters:**
- **file_path** (*str*): Path to URDF file

**Returns:**
- **URDF**: URDF object

**Example:**
```python
from urdfpy import URDF
robot_urdf = URDF.load("path/to/robot.urdf")
```

##### from_xml_string

```python
@classmethod
def from_xml_string(cls, xml_string) -> URDF
```

Load URDF from XML string.

**Parameters:**
- **xml_string** (*str*): URDF XML content

**Returns:**
- **URDF**: URDF object

**Example:**
```python
xml_content = "<robot name='test'>...</robot>"
robot_urdf = URDF.from_xml_string(xml_content)
```

#### Attributes

- **name** (*str*): Robot name
- **links** (*list*): List of Link objects
- **joints** (*list*): List of Joint objects
- **transmissions** (*list*): List of Transmission objects
- **materials** (*list*): List of Material objects

#### Methods

See the urdfpy documentation for complete API details.

---

## Utility Functions

### skew

```python
def skew(v) -> np.ndarray
```

Compute skew-symmetric matrix from 3D vector.

**Parameters:**
- **v** (*np.ndarray*): 3D vector [x, y, z]

**Returns:**
- **np.ndarray**: 3x3 skew-symmetric matrix

**Mathematical Definition:**
```
skew([x, y, z]) = [ 0  -z   y]
                  [ z   0  -x]
                  [-y   x   0]
```

**Example:**
```python
from robot_tools.robot_model import skew
v = np.array([1.0, 2.0, 3.0])
v_skew = skew(v)
# Used for: cross product as matrix multiplication
# v × w = skew(v) @ w
```

---

## Constants and Configuration

### MODEL_DIR

```python
MODEL_DIR = "robot_assets/urdf/"
```

Default directory for URDF files. Can be modified to use custom robot models.

**Example:**
```python
from robot_tools import robot_model
robot_model.MODEL_DIR = "/custom/path/to/urdf/"
```

---

## Type Hints

Common type annotations used throughout the package:

```python
import numpy as np
from typing import Tuple, List, Dict

# Common types
Point3D = np.ndarray  # shape (3,)
Rotation3D = np.ndarray  # shape (3, 3)
Transform4D = np.ndarray  # shape (4, 4)
JointConfig = np.ndarray  # shape (nq,)
Jacobian = np.ndarray  # shape (6, nq)
```

---

## Error Handling

### Common Exceptions

- **ValueError**: Invalid obstacle type in collision detection
- **FileNotFoundError**: URDF file not found
- **ImportError**: Missing dependencies (pinocchio, pyvista, etc.)
- **KeyError**: Missing required keys in obstacle dictionary

### Example Error Handling

```python
try:
    robot = RobotModel("nonexistent.urdf")
except FileNotFoundError:
    print("URDF file not found!")

try:
    obstacle = {"type": "unknown", "T": np.eye(4)}
    obj = create_collision_objects([obstacle])
except ValueError as e:
    print(f"Invalid obstacle type: {e}")
```

---

## Performance Considerations

### Kinematics Computation

- Forward kinematics is computed using Pinocchio's efficient algorithms (O(n) complexity)
- Jacobian computation is analytical, not numerical differentiation
- Frame updates are cached internally by Pinocchio

### Collision Detection

- Primitive collision checks are O(1) operations
- For many obstacles, consider spatial hashing or BVH for optimization
- Transform inverse is pre-computed and cached

### Visualization

- PyVista uses VTK for efficient 3D rendering
- Mesh updates use in-place modifications when possible
- For animations, use timer events instead of continuous redraws

---

## Version History

### Version 0.1.0 (Current)

- Initial release
- Robot kinematics and dynamics
- Collision detection primitives
- Interactive 3D visualization
- URDF support
- Animation capabilities

---

## Related Documentation

- [Pinocchio Documentation](https://stack-of-tasks.github.io/pinocchio/)
- [PyVista Documentation](https://docs.pyvista.org/)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
