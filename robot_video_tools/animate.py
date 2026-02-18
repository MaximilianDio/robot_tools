"""
Animation utilities for robot visualization overlays.

Provides AnimationState to track per-run state across animation frames,
and animate_step to advance motion-planning visualization by one frame.
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


def world_to_display(plotter, point3d: np.ndarray) -> np.ndarray:
    """Convert a 3D world coordinate to 2D pixel coordinates in PyVista.

    Args:
        plotter:  PyVista Plotter instance.
        point3d:  3-element array [x, y, z] in world coordinates.

    Returns:
        2-element array [px, py] in top-left-origin pixel coordinates.
    """
    ren = plotter.renderer
    ren.SetWorldPoint(point3d[0], point3d[1], point3d[2], 1.0)
    ren.WorldToDisplay()
    x, y, _ = ren.GetDisplayPoint()
    # PyVista's (0,0) is bottom-left; flip Y for top-left origin (PIL/OpenCV)
    height = plotter.window_size[1]
    return np.array([x, height - y])


@dataclass
class AnimationState:
    """
    Mutable state that persists across animate_step calls within one animation run.

    Create one instance before the frame loop and pass it to every animate_step call.
    Call reset() (or discard and create a new instance) to restart from the beginning.

    Attributes:
        idx_path:    Index of the next motion-planning path segment to reveal.
        idx_query:   Index of the next query/bifurcation event to reveal.
        path_actors: PyVista actors for the currently visible path segments
                     (faded when the next path is revealed).
    """

    idx_path: int = 0
    idx_query: int = 1
    path_actors: List = field(default_factory=list)

    def reset(self) -> None:
        """Reset counters and clear actor references for a new animation run."""
        self.idx_path = 0
        self.idx_query = 1
        self.path_actors = []


def animate_step(
    j: int,
    jp: int,
    t_end: float,
    time: np.ndarray,
    data: list,
    mpdata: Optional[dict],
    colors: list,
    robots: list,
    robots_des: list,
    state: AnimationState,
    ee_link_name: str = "fer_hand_tcp",
    plot_robot: bool = True,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Advance one frame of the robot + motion-planning animation.

    Args:
        j:            Current time-step index into data arrays.
        jp:           Previous time-step index into data arrays.
        t_end:        Timestamp of the current frame (same units as ``time``).
        time:         Full time array for the logged trajectory.
        data:         List of per-robot dicts, each with keys ``'q'`` and ``'q_des'``.
        mpdata:       Motion-planner data dict with keys ``'q_paths'``,
                      ``'q_queries'``, ``'q_bifs'``, ``'t_queries'``,
                      ``'t_path_recv'``; or ``None`` if unavailable.
        colors:       Interleaved color list [actual_color, desired_color, ...],
                      one pair per robot.
        robots:       List of Robot visualization instances (actual pose).
        robots_des:   List of Robot visualization instances (desired pose).
        state:        AnimationState instance shared across calls in this run.
        ee_link_name: Name of the end-effector link for FK queries.
        plot_robot:   Whether to update the robot mesh and plot the EE trail.

    Returns:
        Tuple (query_point, recv_point, bif_point) — each a 2-D pixel
        coordinate array or None if not yet revealed at this frame.
    """
    query_point: Optional[np.ndarray] = None
    recv_point: Optional[np.ndarray] = None
    bif_point: Optional[np.ndarray] = None

    for _colori, _colord, robot, robot_des, data_robot in zip(
        colors[::2], colors[1::2], robots, robots_des, data
    ):
        qi = data_robot["q"][j] if j < len(data_robot["q"]) else data_robot["q"][-1]
        if jp < len(data_robot["q"]):
            qj = data_robot["q"][jp]
            qj_des = data_robot["q_des"][jp]
        else:
            qj = data_robot["q"][-1]
            qj_des = data_robot["q_des"][-1]

        if plot_robot:
            robot.update(qj)
            robot_des.update(qj_des)
            robot.plot_ee_path(
                np.vstack((qi, qj)),
                ee_link_name=ee_link_name,
                color="gray",
                opacity=1.0,
            )

        if mpdata is None:
            continue

        num_goals = len(mpdata["q_paths"])
        geometric_step = 5

        if num_goals == 2:
            colors_path = ["red", "blue"]
        elif num_goals == 7:
            colors_path = [
                "#ff0000", "#d82e75", "#d64ba1",
                "#9c33cc", "#4233cc", "#1b86dd", "#13bde7",
            ]
        else:
            colors_path = plt.cm.jet(np.linspace(0, 1, num_goals))

        # Reveal query / bifurcation point when its timestamp is reached
        clamped_query_idx = min(state.idx_query, len(mpdata["q_queries"]) - 1)
        if not (
            mpdata["t_queries"][clamped_query_idx] > t_end
            or state.idx_query >= num_goals
        ):
            mask_q = time <= mpdata["t_queries"][state.idx_path]
            if np.any(mask_q):
                idx_q = int(np.where(mask_q)[0][-1])
                ee_pos_query = robot.robot.link_fk(
                    data_robot["q_des"][idx_q], ee_link_name
                )[:3, 3]
                query_point = world_to_display(robot.plotter, ee_pos_query)

            ee_pos_bif = robot.robot.link_fk(
                mpdata["q_bifs"][state.idx_query], ee_link_name
            )[:3, 3]
            bif_point = world_to_display(robot.plotter, ee_pos_bif)
            state.idx_query += 1

        # Reveal next planned path segment when its receive timestamp is reached
        clamped_path_idx = min(state.idx_path, len(mpdata["q_paths"]) - 1)
        if not (
            mpdata["t_path_recv"][clamped_path_idx] > t_end
            or state.idx_path >= num_goals
        ):
            # Fade previously drawn path actors before drawing the new one
            for actor in state.path_actors:
                actor.GetProperty().SetOpacity(0.8)
            state.path_actors = []

            print(
                f"plotting path {state.idx_path + 1}/{num_goals} "
                f"planned at {mpdata['t_path_recv'][state.idx_path]:.3f} s"
            )

            mask_r = time <= mpdata["t_path_recv"][state.idx_path]
            if np.any(mask_r):
                idx_r = int(np.where(mask_r)[0][-1])
                ee_pos_recv = robot.robot.link_fk(
                    data_robot["q_des"][idx_r], ee_link_name
                )[:3, 3]
                recv_point = world_to_display(robot.plotter, ee_pos_recv)

            path_configs = mpdata["q_paths"][state.idx_path]
            # step_idx: renamed from 'j' to avoid shadowing the outer parameter j
            for step_idx in range(0, len(path_configs), geometric_step):
                qi_path = path_configs[step_idx]
                end_idx = min(step_idx + geometric_step, len(path_configs) - 1)
                qj_path = path_configs[end_idx]

                state.path_actors.append(
                    robot.plot_ee_path(
                        np.vstack((qi_path, qj_path)),
                        ee_link_name=ee_link_name,  # was hardcoded "fer_hand_tcp"
                        color=colors_path[state.idx_path],
                        opacity=0.99,
                        line_width=7,
                    )
                )

            state.idx_path += 1

    return query_point, recv_point, bif_point
