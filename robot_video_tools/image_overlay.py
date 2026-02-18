"""
Image overlay generation for robot visualization pipelines.

Composites 3D robot renderings and motion-planning annotations onto
real camera footage by matching the virtual camera to the physical one
via hand-eye calibration data.
"""

import os
from dataclasses import dataclass
from natsort import natsorted
from typing import Optional

import numpy as np
import pickle as pkl
import cv2
import pyvista
pyvista.global_theme.transparent_background = True
import vtk
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

from robot_visualization import Robot
from robot_video_tools.animate import animate_step, AnimationState


@dataclass
class CalibrationPaths:
    """
    Paths to the three hand-eye calibration output files.

    Use ``from_directory`` to build from a directory that follows the
    standard naming convention (``O_T_C.txt``, ``camera_matrix.txt``,
    ``dist_coeffs.txt``).
    """

    transform_file: str    # 4x4 SE(3) matrix: O_T_C.txt
    intrinsic_file: str    # 3x3 camera intrinsic matrix: camera_matrix.txt
    dist_coeffs_file: str  # distortion coefficients: dist_coeffs.txt

    @classmethod
    def from_directory(cls, directory: str) -> "CalibrationPaths":
        """
        Construct CalibrationPaths from a directory containing the three
        standard calibration output files.

        Args:
            directory: Path to the calibration output directory.

        Returns:
            CalibrationPaths instance with resolved file paths.
        """
        return cls(
            transform_file=os.path.join(directory, "O_T_C.txt"),
            intrinsic_file=os.path.join(directory, "camera_matrix.txt"),
            dist_coeffs_file=os.path.join(directory, "dist_coeffs.txt"),
        )


def trans_to_matrix(trans: np.ndarray) -> vtk.vtkMatrix4x4:
    """Convert a 4x4 numpy array to a vtkMatrix4x4."""
    matrix = vtk.vtkMatrix4x4()
    for i in range(trans.shape[0]):
        for j in range(trans.shape[1]):
            matrix.SetElement(i, j, trans[i, j])
    return matrix


def calibrate_camera(
    image_path: str,
    calibration: CalibrationPaths,
    data_robot=None,
    interactive: bool = False,
    off_screen: bool = True,
) -> pyvista.Plotter:
    """
    Create a PyVista Plotter whose camera matches a physical camera.

    Intrinsics (focal length, principal point) and extrinsics (robot-to-camera
    transform) are loaded from the calibration files and applied to the virtual
    camera so that 3D-to-2D projections align with real imagery.

    Args:
        image_path:   Path to a camera image (used for resolution).
        calibration:  CalibrationPaths with paths to the calibration files.
        data_robot:   Robot data dict (required when ``interactive=True``).
        interactive:  If ``True``, opens an interactive plotter window for
                      manual camera verification.
        off_screen:   If ``True``, renders off-screen (no display window).

    Returns:
        Configured pyvista.Plotter instance.

    Raises:
        ValueError: If ``data_robot`` is None when ``interactive=True``, or if
                    no logged timestamps precede the image timestamp.
    """
    image = cv2.imread(image_path)

    image_filename = os.path.basename(image_path)
    image_timestamp = float(os.path.splitext(image_filename)[0].split("_")[-1])
    print(f"Processing frame at {image_timestamp}", end="\r")

    O_T_C = np.loadtxt(calibration.transform_file)
    intrinsic = np.loadtxt(calibration.intrinsic_file)
    dist = np.loadtxt(calibration.dist_coeffs_file)

    image = cv2.undistort(image, intrinsic, dist)
    h, w = image.shape[:2]

    plotter = pyvista.Plotter(off_screen=off_screen, window_size=[w, h])

    # Intrinsics: principal point -> VTK window-centre normalised coordinates
    cx = intrinsic[0, 2]
    cy = intrinsic[1, 2]
    f = intrinsic[0, 0]

    wcx = -2 * (cx - float(w) / 2) / w
    wcy = 2 * (cy - float(h) / 2) / h
    plotter.camera.SetWindowCenter(wcx, wcy)

    # Focal length -> vertical view angle
    view_angle = 180 / np.pi * (2.0 * np.arctan2(h / 2.0, f))
    plotter.camera.SetViewAngle(view_angle)

    # Extrinsics: apply world->camera transform to scene; camera stays at origin
    plotter.camera.SetModelTransformMatrix(trans_to_matrix(np.linalg.inv(O_T_C)))
    plotter.camera.SetPosition(0, 0, 0)
    plotter.camera.SetFocalPoint(0.01, 0, 1)
    plotter.camera.SetViewUp(0, -1, 0)

    # Headlight matches the physical camera's illumination direction
    plotter.remove_all_lights()
    light = pyvista.Light(light_type="headlight")
    light.position = plotter.camera.position
    light.focal_point = plotter.camera.focal_point
    light.intensity = 1.0
    plotter.add_light(light)

    if interactive:
        if data_robot is None:
            raise ValueError("data_robot must be provided when interactive=True")

        robot = Robot(
            urdf_file="robot_tools/robot_assets/urdf/fer_franka_hand.urdf",
            color="white",
            opacity=0.5,
            plotter=plotter,
        )
        ee_link_name = "fer_hand_tcp"
        plotter.add_background_image(image_path)

        def callback(x):
            print(x)
            print(f"camera position: {plotter.camera.position}")
            print(
                f"camera az,rol,elev: "
                f"{plotter.camera.azimuth},{plotter.camera.roll},"
                f"{plotter.camera.elevation}"
            )
            print(
                f"camera view angle, focal point: "
                f"{plotter.camera.view_angle, plotter.camera.focal_point}"
            )

        t_end = image_timestamp
        idx = np.array(data_robot["fc_log"]["time_des"]) <= t_end
        if np.any(idx):
            idx = int(np.where(idx)[0][-1])
        else:
            raise ValueError("No timestamps found less than or equal to t_end.")

        robot.update(data_robot["fc_log"]["q"][idx])
        plotter.track_click_position(callback)
        plotter.show_axes()
        plotter.show()
        print("Camera calibration done. Exiting...")

    return plotter


def image_overlay(
    path: str,
    data,
    file: str,
    calibration: CalibrationPaths,
    urdf_file: str = "robot_tools/robot_assets/urdf/fer_franka_hand.urdf",
    ee_link_name: str = "fer_hand_tcp",
) -> None:
    """
    Generate composite images by overlaying robot 3D renders onto camera footage.

    For each camera frame, the robot mesh, end-effector trajectory, and
    motion-planning annotations (paths, query points, bifurcation points) are
    rendered with a calibrated virtual camera and composited onto the real image
    using alpha blending. Outputs are written to ``{image_dir}/overlay/``.

    Args:
        path:         Directory containing the robot data and camera images.
        data:         Robot state dict with keys ``'time'``, ``'q_des'``,
                      ``'dq_des'``.
        file:         Filename prefix (e.g. ``'rhtoppra'``); used to locate
                      ``{file}_camera_images/``.
        calibration:  CalibrationPaths pointing to the hand-eye calibration files.
        urdf_file:    Path to the robot URDF file.
        ee_link_name: Name of the end-effector link for FK queries.
    """
    print("Starting image overlay generation...")

    image_dir = os.path.join(path, f"{file}_camera_images")

    try:
        mpdata = pkl.load(open(f"{path}/motion_planner_data.pkl", "rb"))
    except Exception:
        mpdata = None

    images = natsorted(
        [img for img in os.listdir(image_dir) if img.endswith(".png")]
    )

    plotter = calibrate_camera(
        os.path.join(image_dir, images[0]), calibration, data
    )
    plotter_2 = calibrate_camera(
        os.path.join(image_dir, images[0]), calibration, data
    )

    robot = Robot(
        urdf_file=urdf_file,
        color="white",
        opacity=0.5,
        plotter=plotter,
    )

    os.makedirs(os.path.join(image_dir, "overlay"), exist_ok=True)
    # Clear stale overlay files from a previous run ('entry' avoids shadowing 'file')
    for entry in os.scandir(os.path.join(image_dir, "overlay")):
        if entry.is_file():
            os.unlink(entry.path)

    # AnimationState replaces the old static function-attribute pattern;
    # no manual cleanup needed after the loop.
    state = AnimationState()

    k0 = 0
    log_time_step = 10
    ds_max = np.max(np.linalg.norm(data["dq_des"], axis=1))

    # Persist last known marker positions across frames
    bif_point: Optional[np.ndarray] = None
    query_point: Optional[np.ndarray] = None
    recv_point: Optional[np.ndarray] = None

    for frame_idx in range(len(images)):
        image_path = os.path.join(image_dir, images[frame_idx])

        image_filename = os.path.basename(image_path)
        image_timestamp = float(
            os.path.splitext(image_filename)[0].split("_")[-1]
        )

        print(
            f"Processing frame {frame_idx + 1}/{len(images)} at {image_timestamp}",
            end="\r",
        )

        background_image = cv2.imread(image_path)
        background_image = cv2.cvtColor(background_image, cv2.COLOR_BGR2RGB)

        t_end = image_timestamp

        idx_mask = data["time"] < t_end
        idx = int(np.where(idx_mask)[0][-1]) if np.any(idx_mask) else 0

        # Accumulate EE trajectory path up to this frame
        for j in range(k0, len(data["q_des"]), log_time_step):
            if data["time"][j] > t_end:
                break
            qi = data["q_des"][max(0, j - log_time_step)]
            qj = data["q_des"][j]
            color = plt.cm.viridis(np.linalg.norm(data["dq_des"][j]) / ds_max)
            robot.plot_ee_path(
                np.vstack((qi, qj)),
                ee_link_name=ee_link_name,
                color=color,
                opacity=1,
                line_width=10,
                plotter=plotter_2,
            )
            k0 = j

        trajectory_image = plotter_2.screenshot(
            return_img=True, transparent_background=True
        )

        query_point_, recv_point_, bif_point_ = animate_step(
            max(0, idx),
            max(0, idx + 1),
            t_end,
            data["time"],
            [data],
            mpdata,
            ["red", "blue"],
            [robot],
            [robot],
            state=state,
            ee_link_name=ee_link_name,
            plot_robot=False,
        )

        rendered_image = plotter.screenshot(
            return_img=True, transparent_background=True
        )

        # Alpha-composite: background -> robot overlay -> trajectory overlay
        res_image = Image.fromarray(background_image).convert("RGBA")
        res_image = Image.alpha_composite(
            res_image, Image.fromarray(rendered_image).convert("RGBA")
        )
        res_image = Image.alpha_composite(
            res_image, Image.fromarray(trajectory_image).convert("RGBA")
        )

        size = 20
        line_width = 5

        # Bifurcation marker: circle
        if bif_point_ is not None:
            bif_point = bif_point_
        try:
            if bif_point is not None:
                overlay_bif = Image.new("RGBA", res_image.size, (0, 0, 0, 0))
                draw = ImageDraw.Draw(overlay_bif)
                draw.ellipse(
                    (
                        bif_point[0] - size, bif_point[1] - size,
                        bif_point[0] + size, bif_point[1] + size,
                    ),
                    outline="white",
                    width=line_width,
                )
                res_image = Image.alpha_composite(res_image, overlay_bif)
        except Exception as e:
            print(f"\nWarning: could not draw bif_point: {e}")

        # Query marker: cross (+)
        if query_point_ is not None:
            query_point = query_point_
        try:
            if query_point is not None:
                overlay_query = Image.new("RGBA", res_image.size, (0, 0, 0, 0))
                draw = ImageDraw.Draw(overlay_query)
                draw.line(
                    (query_point[0] - size, query_point[1],
                     query_point[0] + size, query_point[1]),
                    fill="white", width=line_width,
                )
                draw.line(
                    (query_point[0], query_point[1] - size,
                     query_point[0], query_point[1] + size),
                    fill="white", width=line_width,
                )
                res_image = Image.alpha_composite(res_image, overlay_query)
        except Exception as e:
            print(f"\nWarning: could not draw query_point: {e}")

        # Receive marker: cross (x)
        if recv_point_ is not None:
            recv_point = recv_point_
        try:
            if recv_point is not None:
                d = size / np.sqrt(2)
                overlay_recv = Image.new("RGBA", res_image.size, (0, 0, 0, 0))
                draw = ImageDraw.Draw(overlay_recv)
                draw.line(
                    (recv_point[0] - d, recv_point[1] - d,
                     recv_point[0] + d, recv_point[1] + d),
                    fill="white", width=line_width,
                )
                draw.line(
                    (recv_point[0] + d, recv_point[1] - d,
                     recv_point[0] - d, recv_point[1] + d),
                    fill="white", width=line_width,
                )
                res_image = Image.alpha_composite(res_image, overlay_recv)
        except Exception as e:
            print(f"\nWarning: could not draw recv_point: {e}")

        res_image = cv2.cvtColor(
            np.array(res_image.convert("RGB")), cv2.COLOR_RGB2BGR
        )

        cv2.imshow("Overlay", res_image)
        cv2.waitKey(1)

        overlay_image_filename = os.path.join(
            image_dir, "overlay", image_filename
        )
        cv2.imwrite(overlay_image_filename, res_image)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    path = "out/test_fer/res2"
    file = "rhtoppra"

    image_dir = os.path.join(path, f"{file}_camera_images")
    data_robot = pkl.load(open(f"{path}/{file}_robot_data.pkl", "rb"))

    images = natsorted(
        [img for img in os.listdir(image_dir) if img.endswith(".png")]
    )

    cal = CalibrationPaths.from_directory(
        "franka-control/hand-eye-calibration/output"
    )
    plotter = calibrate_camera(
        os.path.join(image_dir, images[80]),
        cal,
        data_robot,
        interactive=True,
    )
