# Programa en pruebas para ver si se podría automatizar para que publique tantos topics como cámaras haya conectadas
# TODO: Falta testear con varias camaras.

import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def detect_cameras():
    """Detect all connected video devices."""
    try:
        # Use v4l2-ctl to list all video devices
        result = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        lines = result.stdout.splitlines()
        cameras = []

        for i, line in enumerate(lines):
            if "/dev/video" in line:
                device_path = line.strip()
                cameras.append(device_path)
        return cameras
    except Exception as e:
        print(f"Error detecting cameras: {e}")
        return []

def generate_launch_description():
    # Detect cameras
    cameras = detect_cameras()
    if not cameras:
        print("No cameras detected.")
        return LaunchDescription([])

    # Create nodes dynamically for each detected camera
    camera_nodes = []
    for i, camera_path in enumerate(cameras):
        node = Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=f"camera_{i}",
            parameters=[
                {"video_device": camera_path},
                {"image_size": [1280, 720]},  # Adjust resolution as needed
                {"frame_rate": 10},          # Set FPS
                {"pixel_format": "MJPEG"}    # Pixel format
            ],
            remappings=[("/image_raw/compressed", f"/camera_{i}/image_raw/compressed")]
        )
        camera_nodes.append(node)

    return LaunchDescription(camera_nodes)
