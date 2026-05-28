"""
robot_video_tools - Video generation and image overlay utilities
for robot visualization pipelines.

Provides:
  - AnimationState: per-run animation state tracker
  - animate_step:   advance one frame of robot + motion-planning animation
  - world_to_display: 3D -> 2D pixel coordinate conversion
  - CalibrationPaths: hand-eye calibration file path container
  - calibrate_camera: build a PyVista Plotter matched to a real camera
  - ImageOverlay:     camera-calibrated compositor for arbitrary pyvista scenes
  - generate_video_from_images: build MP4 from timestamped PNG folder
  - gif_to_mp4:       convert GIF to MP4 with duration resampling
"""

from robot_video_tools.animate import world_to_display, AnimationState, animate_step
from robot_video_tools.gen_video import generate_video_from_images, gif_to_mp4
from robot_video_tools.image_overlay import CalibrationPaths, calibrate_camera, ImageOverlay

__version__ = "0.1.0"

__all__ = [
    "world_to_display",
    "AnimationState",
    "animate_step",
    "generate_video_from_images",
    "gif_to_mp4",
    "CalibrationPaths",
    "calibrate_camera",
    "ImageOverlay",
]
