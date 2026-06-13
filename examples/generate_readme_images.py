"""Generate the images embedded in the README via off-screen rendering.

Run from anywhere (no display required):

    python examples/generate_readme_images.py

Outputs are written to docs/images/.
"""

from pathlib import Path

import networkx as nx
import numpy as np
import pyvista as pv

from robot_tools import create_collision_objects
from robot_visualization import Robot

REPO_DIR = Path(__file__).resolve().parents[1]
URDF_DIR = REPO_DIR / "robot_assets" / "urdf"
IMAGE_DIR = REPO_DIR / "docs" / "images"

WINDOW_SIZE = [1100, 750]


def _new_plotter(off_screen: bool = True) -> pv.Plotter:
    plotter = pv.Plotter(off_screen=off_screen, window_size=WINDOW_SIZE)
    plotter.set_background("white")
    plotter.add_axes()
    return plotter


def iiwa7_path_image() -> None:
    """iiwa7 at start/goal configuration with EE frames, EE path, and roadmap graph."""
    plotter = _new_plotter()
    plotter.camera_position = "yz"
    plotter.set_focus([0, -0.55, 0.62])
    plotter.set_position([3.6, -0.55, 0.62])

    robot = Robot(str(URDF_DIR / "iiwa7.urdf"), plotter,
                  p0=np.array([0.0, -1.0, 0.0]), R0=np.eye(3),
                  color="lightblue", opacity=1.0)
    robot.set_robot_mesh(id=0)
    robot.set_robot_mesh(id=1)

    q0 = np.array([1.0, 0.5, 0, -1.0, 0, 1.0, 0])
    q1 = np.zeros(7)
    robot.update(q0, id=0)
    robot.update(q1, id=1, opacity=0.4)

    q = np.linspace(q0, q1, 20)
    for qi in q:
        robot.plot_ee_frame(qi, ee_link_name="lbr1_gripper_link_ee", scale=0.05)
    robot.plot_ee_path(q, ee_link_name="lbr1_gripper_link_ee", color="blue", opacity=1.0)

    # Motion-planning roadmap in end-effector space
    graph = nx.read_graphml(REPO_DIR / "tests" / "robot_graph.graphml")
    poses = {}
    for key, data in graph.nodes(data=True):
        qn = np.fromstring(data["coords"], sep=",")
        poses[key] = robot.fk(qn, ee_link_name="lbr1_gripper_link_ee")[:3, 3]
    for u, v in graph.edges():
        plotter.add_mesh(pv.Line(poses[u], poses[v]), color="gray", line_width=2, opacity=0.5)

    plotter.screenshot(IMAGE_DIR / "iiwa7_path.png")
    plotter.close()
    print("wrote iiwa7_path.png")


def simple_robot_image() -> None:
    """Simple 3-DOF robot at two configurations with interpolated EE path."""
    plotter = _new_plotter()
    plotter.camera_position = "yz"
    plotter.set_focus([0, -2.0, 0])
    plotter.set_position([4.0, -2.0, 0])

    robot = Robot(str(URDF_DIR / "simple_robot.urdf"), plotter,
                  p0=np.array([0.0, -2.0, 0.0]), R0=np.eye(3),
                  color="lightblue", opacity=1.0)
    robot.set_robot_mesh(id=0)
    robot.set_robot_mesh(id=1)

    q0 = np.array([0.0, np.pi / 2, 0])
    q1 = np.array([0.0, -np.pi / 2, 0])
    robot.update(q0, id=0)
    robot.update(q1, id=1, opacity=0.4)

    q = np.linspace(q0, q1, 20)
    for qi in q:
        robot.plot_ee_frame(qi, ee_link_name="end_effector", scale=0.15)
    robot.plot_ee_path(q, ee_link_name="end_effector", color="blue", opacity=1.0)

    plotter.screenshot(IMAGE_DIR / "simple_robot.png")
    plotter.close()
    print("wrote simple_robot.png")


def collision_primitives_image() -> None:
    """Collision primitives with sample points colored by collision state."""
    obstacles = [
        {"type": "ellipsoid", "T": np.eye(4), "xradius": 0.5, "yradius": 0.3, "zradius": 0.4},
        {"type": "cylinder",
         "T": np.array([[1, 0, 0, 1.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
         "radius": 0.3, "height": 0.8},
        {"type": "box",
         "T": np.array([[1, 0, 0, -1.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
         "xsize": 0.6, "ysize": 0.6, "zsize": 0.6},
    ]
    collision_objects = create_collision_objects(obstacles)

    plotter = _new_plotter()
    plotter.add_mesh(pv.ParametricEllipsoid(0.5, 0.3, 0.4), color="lightblue", opacity=0.4)
    plotter.add_mesh(pv.Cylinder(center=(1.4, 0, 0), direction=(0, 0, 1), radius=0.3, height=0.8),
                     color="lightblue", opacity=0.4)
    plotter.add_mesh(pv.Cube(center=(-1.4, 0, 0), x_length=0.6, y_length=0.6, z_length=0.6),
                     color="lightblue", opacity=0.4)

    rng = np.random.default_rng(42)
    points = rng.uniform([-2.2, -0.8, -0.8], [2.2, 0.8, 0.8], size=(600, 3))
    in_collision = np.array(
        [any(obj.is_in_collision(p) for obj in collision_objects) for p in points]
    )
    plotter.add_points(points[in_collision], color="red", point_size=8,
                       render_points_as_spheres=True)
    plotter.add_points(points[~in_collision], color="green", point_size=4,
                       render_points_as_spheres=True, opacity=0.35)

    plotter.camera_position = "xz"
    plotter.camera.azimuth = 25
    plotter.camera.elevation = 20
    plotter.screenshot(IMAGE_DIR / "collision_primitives.png")
    plotter.close()
    print("wrote collision_primitives.png")


def iiwa7_motion_gif() -> None:
    """Animated iiwa7 moving between two configurations."""
    plotter = _new_plotter()
    plotter.camera_position = "yz"
    plotter.set_focus([0, -0.55, 0.62])
    plotter.set_position([3.6, -0.55, 0.62])

    robot = Robot(str(URDF_DIR / "iiwa7.urdf"), plotter,
                  p0=np.array([0.0, -1.0, 0.0]), R0=np.eye(3),
                  color="lightblue", opacity=1.0)
    robot.set_robot_mesh(id=0)

    q0 = np.array([1.0, 0.5, 0, -1.0, 0, 1.0, 0])
    q1 = np.zeros(7)
    q = np.linspace(q0, q1, 40)

    plotter.open_gif(str(IMAGE_DIR / "iiwa7_motion.gif"), fps=15)
    for i, qi in enumerate(q):
        robot.update(qi, id=0)
        if i > 0:
            robot.plot_ee_path(q[i - 1:i + 1], ee_link_name="lbr1_gripper_link_ee",
                               color="blue", opacity=1.0)
        plotter.write_frame()
    plotter.close()
    print("wrote iiwa7_motion.gif")


if __name__ == "__main__":
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    iiwa7_path_image()
    simple_robot_image()
    collision_primitives_image()
    iiwa7_motion_gif()
