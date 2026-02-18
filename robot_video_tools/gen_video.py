"""
Video generation utilities for robot visualization pipelines.

Converts folders of timestamped PNG frames to MP4 videos, delegating
frame-timing and overlay utilities to robot_visualization.video_utils.
"""

import os
import cv2
import numpy as np
from natsort import natsorted
import imageio

from robot_visualization.video_utils import (
    calculate_fps_from_timestamps,
    add_timestamp_to_frame,
    create_video_writer,
    interpolate_frame_count,
)


def generate_video_from_images(image_folder: str, output_video: str) -> None:
    """
    Build an MP4 video from a folder of timestamped PNG files.

    Each PNG filename must end with ``_<timestamp>.png`` where ``<timestamp>``
    is a floating-point number of seconds (e.g. ``frame_1.234.png``).
    Timestamps are extracted, converted to milliseconds, and used to derive
    the actual playback FPS and to pad gaps between non-uniform frames.

    Note: The fps parameter was removed from the original signature because it
    was always overwritten by the timestamp-derived value. To force a specific
    FPS, resample the output file with ffmpeg after generation.

    Args:
        image_folder: Directory containing ``.png`` files named with timestamps.
        output_video: Destination path for the output ``.mp4`` file.
    """
    images = natsorted(
        [img for img in os.listdir(image_folder) if img.endswith(".png")]
    )

    if not images:
        print("No .png files found in the folder.")
        return

    # Extract timestamps from filenames; filenames encode seconds, convert to ms
    images_timestamps = [
        float(image.split("_")[-1].replace(".png", "")) * 1e3
        for image in images
    ]

    # Some capture pipelines write duplicate entries at t=0 and t=t_end;
    # trim leading/trailing duplicates so we start and end at unique timestamps.
    idx_first = next(i for i, v in enumerate(images_timestamps) if v != 0) - 1
    idx_last = next(
        i for i, v in enumerate(images_timestamps) if v == images_timestamps[-1]
    )
    images_timestamps = images_timestamps[idx_first : idx_last + 1]
    images = images[idx_first : idx_last + 1]

    timestamps_arr = np.array(images_timestamps)
    fps_mean = 1000.0 / np.mean(np.diff(timestamps_arr))
    fps = calculate_fps_from_timestamps(timestamps_arr)

    print(
        f"Generating video at {fps:.2f} FPS (mean inter-frame FPS: {fps_mean:.2f}) "
        f"from {len(images)} images..."
    )

    first_frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width = first_frame.shape[:2]

    video = create_video_writer(output_video, fps, (width, height))

    prev_frame = None
    prev_time = None  # milliseconds

    for image, t_ist in zip(images, images_timestamps):
        frame = cv2.imread(os.path.join(image_folder, image))
        frame = add_timestamp_to_frame(frame, t_ist)

        if prev_time is None:
            video.write(frame)
            prev_frame = frame
            prev_time = t_ist
            continue

        # Repeat the previous frame to fill any timestamp gap
        repeat_count = interpolate_frame_count(t_ist, prev_time, fps_mean)
        for _ in range(repeat_count):
            video.write(prev_frame)

        prev_frame = frame
        prev_time = t_ist

    # Write the final frame once
    video.write(prev_frame)
    video.release()
    print(f"Video saved as {output_video}")


def gif_to_mp4(input_gif: str, output_mp4: str, desired_duration: float) -> None:
    """
    Convert a GIF to MP4, resampling frames to match a desired total duration.

    Args:
        input_gif:        Path to input GIF file.
        output_mp4:       Path for the output ``.mp4`` file.
        desired_duration: Target duration in seconds.
    """
    print(f"Converting {input_gif} to {output_mp4} with duration {desired_duration}s")

    gif = imageio.mimread(input_gif, memtest=False)
    gif = [cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR) for frame in gif]
    original_frame_count = len(gif)

    reader = imageio.get_reader(input_gif)
    meta = reader.get_meta_data()
    original_duration = meta.get("duration", 100) * original_frame_count / 1000

    fps = meta.get("fps", 30)
    if original_duration == 0:
        original_duration = original_frame_count / fps

    new_frame_count = int(fps * desired_duration)
    indices = np.linspace(0, original_frame_count - 1, new_frame_count).astype(int)
    new_frames = [gif[i] for i in indices]

    height, width, _ = new_frames[0].shape
    out = create_video_writer(output_mp4, fps, (width, height))
    for frame in new_frames:
        out.write(frame)
    out.release()


if __name__ == "__main__":
    image_folder = "out/rhtoppra_camera_images"
    output_video = "out/rhtoppra_camera_images/video.mp4"
    generate_video_from_images(image_folder, output_video)
