# robot_tools

A comprehensive Python package for robot path planning, visualization, and modeling. This package provides tools for robot kinematics, dynamics, collision detection, and interactive 3D visualization using URDF files.

## Features

### Robot Modeling (`robot_tools`)
- **Kinematics & Dynamics**: Forward kinematics, Jacobian computation, and dynamics calculations using Pinocchio
- **Collision Detection**: Efficient collision checking with ellipsoid, cylinder, and box primitives
- **URDF Support**: Load and work with robot models defined in URDF format
- **Flexible Base Transformations**: Support for arbitrary robot base positions and orientations

### Robot Visualization (`robot_visualization`)
- **Interactive 3D Visualization**: Real-time robot visualization using PyVista
- **End-Effector Tracking**: Visualize end-effector positions, orientations, and trajectories
- **Animation Support**: Create animations and export them as GIF files
- **Visualization Primitives**: Built-in support for coordinate frames, arrows, and paths
- **Graph Visualization**: Display motion planning graphs and roadmaps

### URDF Parsing (`urdfpy`)
- **Complete URDF Support**: Parse and manipulate URDF files
- **Mesh Loading**: Support for STL, DAE, and OBJ mesh formats
- **Kinematic Tree**: Access robot kinematic structure and joint information

## Installation

### Standard Installation

```bash
cd robot_tools
pip install -e .
```

### Development Installation

For development with testing tools:

```bash
pip install -e ".[dev]"
```

### Dependencies

This package automatically installs the following dependencies:
- numpy (>=1.19.0)
- pinocchio
- pyvista (>=0.37.0)
- scipy (>=1.7.0)
- lxml (>=4.6.0)
- networkx (>=3.0)
- pillow (>=8.0.0)
- pycollada (==0.6)
- pyrender (>=0.1.20)
- trimesh (>=3.9.0)

## Quick Start

### 1. Robot Modeling and Kinematics

```python
from robot_tools import RobotModel
import numpy as np

# Load a robot model from URDF
robot = RobotModel("iiwa7.urdf",
                   p0=np.array([0.0, 0.0, 0.0]),  # base position
                   R0=np.eye(3))                  # base orientation

# Define joint configuration
q = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
dq = np.zeros(7)  # joint velocities

# Compute forward kinematics and Jacobian
T, J = robot.update_kinematics("end_effector_link", q, dq)

print("End-effector pose:", T)
print("Jacobian:", J)

# Compute dynamics
M, c, g = robot.update_dynamics(q, dq)
print("Mass matrix:", M)
print("Coriolis/centrifugal:", c)
print("Gravity:", g)
```

### 2. Collision Detection

```python
from robot_tools import EllipsoidCollision, CylinderCollision, BoxCollision, create_collision_objects
import numpy as np

# Define collision objects
obstacles = [
    {
        "type": "ellipsoid",
        "T": np.eye(4),  # transformation matrix
        "xradius": 0.5,
        "yradius": 0.5,
        "zradius": 1.0
    },
    {
        "type": "cylinder",
        "T": np.eye(4),
        "radius": 0.3,
        "height": 1.0
    },
    {
        "type": "box",
        "T": np.eye(4),
        "xsize": 0.5,
        "ysize": 0.5,
        "zsize": 1.0
    }
]

# Create collision checkers
collision_objects = create_collision_objects(obstacles)

# Check for collisions
point = np.array([0.0, 0.0, 0.5])
for obj in collision_objects:
    if obj.is_in_collision(point):
        print(f"Point {point} is in collision!")
```

### 3. Robot Visualization

```python
from robot_visualization import Robot
import pyvista as pv
import numpy as np

# Create a 3D plotter
plotter = pv.Plotter()
plotter.add_axes()

# Load and visualize a robot
robot = Robot("iiwa7.urdf",
              plotter=plotter,
              p0=np.array([0.0, 0.0, 0.0]),
              R0=np.eye(3),
              color='lightblue',
              opacity=1.0)

# Set initial robot mesh
robot.set_robot_mesh(id=0)

# Update robot configuration
q = np.array([0.5, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
robot.update(q, id=0)

# Visualize end-effector
robot.plot_ee_frame(q, ee_link_name="end_effector_link")

# Display the visualization
plotter.show()
```

### 4. End-Effector Path Visualization

```python
from robot_visualization import Robot
import pyvista as pv
import numpy as np

plotter = pv.Plotter()
robot = Robot("iiwa7.urdf", plotter=plotter)

# Define start and goal configurations
q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_goal = np.array([1.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])

# Interpolate between configurations
num_steps = 20
q_path = np.linspace(q_start, q_goal, num_steps)

# Visualize the path
robot.set_robot_mesh(id=0)
robot.update(q_start, id=0)

for q in q_path:
    robot.plot_ee_frame(q, ee_link_name="end_effector_link")

robot.plot_ee_path(q_path, ee_link_name="end_effector_link", color="blue", line_width=4)

plotter.show()
```

### 5. Robot Animation

```python
from robot_visualization import Robot
import pyvista as pv
import numpy as np

plotter = pv.Plotter()
robot = Robot("iiwa7.urdf", plotter=plotter, color='lightblue', opacity=1.0)

# Setup animation
q_start = np.array([1.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0])
q_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

num_steps = 50
q_path = np.linspace(q_start, q_goal, num_steps)

robot.set_robot_mesh(id=0)

# Create animation callback
def update_robot(step):
    robot.update(q_path[step], id=0)
    robot.plot_ee_frame(q_path[step], ee_link_name="end_effector_link")

# Run animation
plotter.add_timer_event(max_steps=num_steps, duration=100, callback=update_robot)
plotter.show()
```

### 6. Visualization Primitives

```python
from robot_visualization import AxesVisualizer, ArrowVisualizer
import pyvista as pv
import numpy as np

plotter = pv.Plotter()
plotter.add_axes()

# Create coordinate frame
axes = AxesVisualizer(plotter, origin=[0, 0, 0], scale=1.0)
axes.update(position=[1, 1, 1], rotation=np.array([0.1, 0.2, 0.3]))

# Create arrow
arrow = ArrowVisualizer(plotter, origin=[0, 0, 0], direction=[1, 0, 0], scale=0.5, color='red')
arrow.update(origin=[0.5, 0.5, 0.5], direction=[0, 1, 0])

plotter.show()
```

## Package Structure

```
robot_tools/
├── robot_tools/              # Main package
│   ├── __init__.py
│   ├── robot_model.py        # Robot kinematics and dynamics
│   └── collisions.py         # Collision detection primitives
├── robot_visualization/      # Visualization subpackage
│   └── robot_visualization/
│       ├── __init__.py
│       ├── robot.py          # Robot visualization class
│       ├── primitives.py     # Visualization primitives
│       └── urdfpy/           # URDF parsing library
├── robot_assets/             # Robot URDF files and meshes
│   └── urdf/
│       ├── iiwa7.urdf
│       ├── simple_robot.urdf
│       └── meshes/
├── tests/                    # Example scripts and tests
│   ├── test_robot_model.py
│   ├── test_collisions.py
│   ├── test_robot_visualization.py
│   └── test_robot_animation.py
├── setup.py
└── README.md
```

## API Reference

### RobotModel

Main class for robot kinematics and dynamics computations.

**Constructor:**
```python
RobotModel(urdf_file, p0=np.zeros(3), R0=np.eye(3), Tgp=np.eye(4))
```

**Parameters:**
- `urdf_file` (str): Path to URDF file (relative to robot_assets/urdf/)
- `p0` (np.ndarray): Base position [x, y, z]
- `R0` (np.ndarray): Base rotation matrix (3x3)
- `Tgp` (np.ndarray): Grasp to end-effector transformation (4x4)

**Methods:**
- `update_kinematics(frame_name, q, dq)`: Compute forward kinematics and Jacobian
  - Returns: `(T, J)` where T is SE3 pose and J is 6xN Jacobian
- `update_dynamics(q, dq)`: Compute mass matrix, Coriolis, and gravity terms
  - Returns: `(M, c, g)` mass matrix, Coriolis/centrifugal, gravity

### Collision Detection Classes

#### EllipsoidCollision
```python
EllipsoidCollision(obstacle)
```
- `obstacle` (dict): Dictionary with keys `"type"`, `"T"`, `"xradius"`, `"yradius"`, `"zradius"`
- `is_in_collision(point)`: Check if point is inside ellipsoid

#### CylinderCollision
```python
CylinderCollision(obstacle)
```
- `obstacle` (dict): Dictionary with keys `"type"`, `"T"`, `"radius"`, `"height"`
- `is_in_collision(point)`: Check if point is inside cylinder

#### BoxCollision
```python
BoxCollision(obstacle)
```
- `obstacle` (dict): Dictionary with keys `"type"`, `"T"`, `"xsize"`, `"ysize"`, `"zsize"`
- `is_in_collision(point)`: Check if point is inside box

#### create_collision_objects
```python
create_collision_objects(obstacles_list)
```
Factory function to create collision objects from a list of obstacle definitions.

### Robot (Visualization)

Main class for robot visualization.

**Constructor:**
```python
Robot(urdf_file, plotter=None, p0=np.zeros(3), R0=np.eye(3), color='lightgray', opacity=1.0)
```

**Parameters:**
- `urdf_file` (str): Path to URDF file (relative to robot_assets/urdf/)
- `plotter` (pv.Plotter): PyVista plotter instance
- `p0` (np.ndarray): Base position
- `R0` (np.ndarray): Base rotation matrix
- `color` (str): Robot mesh color
- `opacity` (float): Robot mesh opacity

**Methods:**
- `set_robot_mesh(id=0)`: Initialize robot mesh
- `update(q, id=0)`: Update robot configuration
- `plot_ee(q, ee_link_name, color, size, type)`: Plot end-effector marker
- `plot_ee_frame(q, ee_link_name)`: Plot end-effector coordinate frame
- `plot_ee_path(q_path, ee_link_name, color, opacity, line_width)`: Plot end-effector trajectory
- `fk(q, ee_link_name)`: Compute forward kinematics

### AxesVisualizer

Visualize 3D coordinate frames.

**Constructor:**
```python
AxesVisualizer(plotter, origin=None, scale=1.0)
```

**Methods:**
- `update(position, rotation)`: Update frame pose
- `plot_path(p1, p2, color, line_width)`: Draw line between points

### ArrowVisualizer

Visualize 3D arrows.

**Constructor:**
```python
ArrowVisualizer(plotter, origin=None, direction=None, scale=1.0, color='white')
```

**Methods:**
- `update(origin, direction)`: Update arrow position and direction

## Examples

See the `tests/` directory for complete examples:

- [test_robot_model.py](tests/test_robot_model.py) - Robot kinematics and dynamics
- [test_collisions.py](tests/test_collisions.py) - Collision detection
- [test_robot_visualization.py](tests/test_robot_visualization.py) - Robot visualization with motion planning graphs
- [test_robot_animation.py](tests/test_robot_animation.py) - Animated robot motion

## Robot Assets

The package includes several robot URDF files in `robot_assets/urdf/`:

- **iiwa7.urdf**: KUKA iiwa 7-DOF collaborative robot
- **simple_robot.urdf**: Simple 3-DOF robot for testing
- Additional robots: GoFa5, IRB1100, CRB15000, and more

All mesh files (STL, DAE) are included in the `robot_assets/urdf/meshes/` directory.

## Development

### Running Tests

The test files are example scripts that demonstrate package functionality:

```bash
# Test collision detection
python tests/test_collisions.py

# Test robot model kinematics
python tests/test_robot_model.py

# Test robot visualization
python tests/test_robot_visualization.py

# Test robot animation
python tests/test_robot_animation.py

# Generate GIF animation
python tests/test_robot_animation.py --generate_gif
```

### Package Development

```bash
# Install in development mode
pip install -e ".[dev]"

# Run linting
flake8 robot_tools robot_visualization

# Run with coverage
pytest --cov=robot_tools --cov=robot_visualization tests/
```

## Troubleshooting

### Import Errors

If you encounter import errors, make sure the package is installed:
```bash
pip install -e .
```

### URDF Loading Issues

URDF files should be placed in `robot_assets/urdf/` or provide absolute paths. The package looks for robot models relative to the installation directory.

### Visualization Not Showing

Make sure you call `plotter.show()` after setting up your visualization. For headless environments, you may need to configure PyVista's rendering backend.

### Missing Meshes

Ensure all mesh files referenced in the URDF are present in `robot_assets/urdf/meshes/`. The package includes mesh files for the provided robots.

## Citation

If you use this package in your research, please cite:

```bibtex
@software{robot_tools,
  author = {Maximilian Dio},
  title = {robot_tools: Visualization and Modeling Tools for Robot Path Planning},
  year = {2025},
  version = {0.1.0}
}
```

## License

[Specify your license here]

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Contact

For questions and support, contact:
- Maximilian Dio
- Email: maximilian.dio@fau.de

## Acknowledgments

- Built on [Pinocchio](https://github.com/stack-of-tasks/pinocchio) for robot dynamics
- Uses [PyVista](https://pyvista.org/) for 3D visualization
- URDF parsing provided by [urdfpy](https://github.com/mmatl/urdfpy)
