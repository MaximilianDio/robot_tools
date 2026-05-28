"""
Image overlay generation for robot visualization pipelines.

Composites 3D pyvista scenes onto real camera footage by matching
the virtual camera to the physical one via hand-eye calibration data.

"""

import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
import cv2
from pyrender import light
import pyvista
pyvista.global_theme.transparent_background = True
from natsort import natsorted
from PIL import Image, ImageDraw, ImageFont

from robot_video_tools.animate import world_to_display  # re-export convenience


@dataclass
class CalibrationPaths:
    """
    Paths to the three hand-eye calibration output files.

    Use ``from_directory`` to build from a directory that follows the
    standard naming convention (``O_T_C.txt``, ``camera_matrix.txt``,
    ``dist_coeffs.txt``).
    """

    transform_file: str    # 4x4 SE(3) matrix  O_T_C.txt
    intrinsic_file: str    # 3x3 camera intrinsic matrix  camera_matrix.txt
    dist_coeffs_file: str  # distortion coefficients  dist_coeffs.txt

    @classmethod
    def from_directory(cls, directory: str) -> "CalibrationPaths":
        return cls(
            transform_file=os.path.join(directory, "O_T_C.txt"),
            intrinsic_file=os.path.join(directory, "camera_matrix.txt"),
            dist_coeffs_file=os.path.join(directory, "dist_coeffs.txt"),
        )


def _apply_calibration(
    plotter: pyvista.Plotter,
    intrinsic: np.ndarray,
    O_T_C: np.ndarray,
    image: np.ndarray,
) -> None:
    """Configure plotter camera to match physical camera intrinsics and extrinsics."""
    h, w = image.shape[:2]

    # Extrinsics: T_WC = inv(O_T_C) gives camera pose in world frame
    T_WC = O_T_C
    plotter.camera.position    = T_WC[:3, 3].tolist()
    plotter.camera.focal_point = (T_WC[:3, 3] + T_WC[:3, 2]).tolist()  # +Z = optical axis
    plotter.camera.up          = (-T_WC[:3, 1]).tolist()                 # OpenCV Y-down → flip

    # Intrinsics: vertical FOV from fy; principal-point offset via window centre
    fy = float(intrinsic[1, 1])
    plotter.camera.view_angle = float(np.degrees(2.0 * np.arctan(h / (2.0 * fy))))

    cx, cy = intrinsic[0, 2], intrinsic[1, 2]
    plotter.camera.SetWindowCenter(
        -2.0 * (cx - w / 2.0) / w,
         2.0 * (cy - h / 2.0) / h,
    )

    # Recalculate near/far planes now that camera is positioned
    plotter.renderer.ResetCameraClippingRange()

    plotter.remove_all_lights()
    
    for pos in [(2, 1, 2), (2, -1, 2)]:
        light = pyvista.Light(position=pos, color='white', intensity=0.8)
        light.positional = True
        plotter.add_light(light)


    pyvista.Property().ambient = 1.0


def calibrate_camera(
    image_path: str,
    calibration: CalibrationPaths,
    off_screen: bool = True,
) -> pyvista.Plotter:
    """
    Return a PyVista Plotter whose camera matches a physical camera.

    The plotter window is sized to the image resolution; the camera position,
    orientation, and field-of-view are derived from the calibration files.
    Add your scene objects to the returned plotter normally.

    Args:
        image_path:  Path to a camera image (used for resolution and undistortion).
        calibration: CalibrationPaths pointing to O_T_C, intrinsics, and dist coeffs.
        off_screen:  If ``True``, renders off-screen (no display window).

    Returns:
        Configured pyvista.Plotter instance.
    """
    O_T_C     = np.loadtxt(calibration.transform_file)
    intrinsic  = np.loadtxt(calibration.intrinsic_file)
    dist       = np.loadtxt(calibration.dist_coeffs_file)

    image = cv2.undistort(cv2.imread(image_path), intrinsic, dist)
    h, w  = image.shape[:2]

    plotter = pyvista.Plotter(off_screen=off_screen, window_size=[w, h])
    _apply_calibration(plotter, intrinsic, O_T_C, image)
    return plotter


def _parse_timestamp(filename: str) -> float:
    """Extract timestamp (seconds) from image filename.

    Expected tail format: ``…_s.ms.jpg``  e.g. ``frame_001_1234.567.jpg``
    """
    return float(os.path.splitext(os.path.basename(filename))[0].split("_")[-1])


class ImageOverlay:
    """
    Camera-calibrated compositor: overlays a pyvista scene onto a real image sequence.

    Set up the scene on ``overlay.plotter`` exactly as you would any PyVista plotter.
    Then call ``render(t)`` each frame — it picks the matching background image,
    screenshots the current plotter state, and returns the composited BGR frame.

    Example::

        overlay = ImageOverlay("path/to/images/", calibration)
        robot   = Robot(urdf_file, plotter=overlay.plotter)
        robot.plot_ee_path(path, color="blue")

        for t in timeline:
            robot.update(q_at_t)
            frame = overlay.render(t)
            if frame is not None:
                cv2.imwrite(f"out/{i:04d}.jpg", frame)

    Image filename format: ``xxx_k_s.ms.jpg``
    (``s`` = seconds, ``ms`` = milliseconds, last ``_``-delimited field is ``s.ms``).
    Image ``k`` is used when ``timestamps[k] < t <= timestamps[k+1]``.
    """

    def __init__(self, image_folder: str, calibration: CalibrationPaths) -> None:
        self._folder      = image_folder
        self._calibration = calibration

        names = natsorted(
            f for f in os.listdir(image_folder)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))
        )
        if not names:
            raise ValueError(f"No images found in {image_folder!r}")

        self._images     = names
        self._timestamps = np.array([_parse_timestamp(n) for n in names])

        O_T_C      = np.loadtxt(calibration.transform_file)
        intrinsic   = np.loadtxt(calibration.intrinsic_file)
        self._dist  = np.loadtxt(calibration.dist_coeffs_file)
        self._intrinsic = intrinsic

        first_image = cv2.undistort(
            cv2.imread(os.path.join(image_folder, names[0])), intrinsic, self._dist
        )
        h, w = first_image.shape[:2]

        self._plotter = pyvista.Plotter(off_screen=True, window_size=[w, h])
        _apply_calibration(self._plotter, intrinsic, O_T_C, first_image)

    @property
    def plotter(self) -> pyvista.Plotter:
        """Calibrated off-screen plotter. Add scene objects to it normally."""
        return self._plotter

    def get_frame_index(self, t: float) -> Optional[int]:
        """Return the index of the image frame corresponding to time t, or None if out of range."""
        k = int(np.searchsorted(self._timestamps, t, side="left"))
        if k < 0 or k >= len(self._images):
            return None
        return k

    def render(self, t: float) -> Optional[np.ndarray]:
        """
        Composite the current plotter scene onto the background image at time ``t``.

        Selects image ``k`` where ``timestamps[k] < t <= timestamps[k+1]``.

        Args:
            t: Query timestamp in the same units as the image filenames (seconds).

        Returns:
            Composited BGR ``np.ndarray``, or ``None`` if ``t`` is outside the
            image sequence range.
        """
        k = self.get_frame_index(t)
        if k is None:
            return None

        path   = os.path.join(self._folder, self._images[k])
        bg     = cv2.undistort(cv2.imread(path), self._intrinsic, self._dist)
        bg_rgb = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)

        overlay_rgba = self._plotter.screenshot(
            return_img=True, transparent_background=True
        )

        result = Image.alpha_composite(
            Image.fromarray(bg_rgb).convert("RGBA"),
            Image.fromarray(overlay_rgba).convert("RGBA"),
        )
        return cv2.cvtColor(np.array(result.convert("RGB")), cv2.COLOR_RGB2BGR), k, self._timestamps[k]

    _FONT_CANDIDATES = [
        "/usr/share/fonts/truetype/freefont/FreeSerif.ttf",          # Times-compatible, closest to LaTeX
        "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf",
    ]

    def add_time_stamp(self, frame: np.ndarray, timestamp: float, position=(10, 10), font_scale=1.0, color=(255, 255, 255), thickness=1) -> np.ndarray:
        """Utility to add a timestamp overlay to a frame. color is BGR (OpenCV convention)."""
        text = f"Time: {timestamp:.3f} s"
        font_size = max(12, int(30 * font_scale))

        font = None
        for path in self._FONT_CANDIDATES:
            if os.path.exists(path):
                font = ImageFont.truetype(path, font_size)
                break
        if font is None:
            font = ImageFont.load_default()

        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img)
        rgb_color = (color[2], color[1], color[0])  # BGR → RGB
        draw.text(position, text, font=font, fill=rgb_color,
                  stroke_width=max(0, thickness - 1), stroke_fill=(0, 0, 0))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)