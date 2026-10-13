"""Deterministic, clearly synthetic messages and imagery for perception smoke tests."""

from __future__ import annotations

import numpy as np


def synthetic_blue_block_bgr() -> np.ndarray:
    """Return a 640x640 BGR drawing used only for software integration tests."""
    import cv2

    image = np.full((640, 640, 3), 235, dtype=np.uint8)
    cv2.rectangle(image, (0, 430), (639, 639), (180, 180, 180), -1)
    cv2.fillPoly(image, [np.array([[205, 250], [360, 225], [438, 325], [283, 354]], np.int32)], (220, 50, 20))
    cv2.fillPoly(image, [np.array([[360, 225], [438, 325], [438, 472], [360, 370]], np.int32)], (190, 38, 18))
    cv2.fillPoly(image, [np.array([[283, 354], [438, 325], [438, 472], [283, 505]], np.int32)], (160, 45, 15))
    cv2.line(image, (205, 250), (283, 354), (250, 100, 80), 4)
    cv2.line(image, (283, 354), (438, 325), (110, 30, 10), 4)
    cv2.line(image, (438, 325), (438, 472), (110, 30, 10), 4)
    return image


def organized_model_cloud(model_path: str, width: int = 640, height: int = 480):
    """Project the packaged PCD model into a synthetic organized camera cloud.

    This is generated geometry, not sensor data and not a pose-accuracy test.
    Returns (xyz image, fx, fy, cx, cy, bbox, expected_center, point_count).
    """
    import open3d as o3d

    cloud = o3d.io.read_point_cloud(model_path)
    if cloud.is_empty():
        raise ValueError(f"empty PCD model: {model_path}")
    points = np.asarray(cloud.points, dtype=np.float32)
    points -= points.mean(axis=0, keepdims=True)
    extent = points.max(axis=0) - points.min(axis=0)
    if float(extent.max()) > 2.0:
        points *= 0.001

    fx = fy = 600.0
    cx, cy = width / 2.0, height / 2.0
    center = np.array([0.0, 0.0, 0.65], dtype=np.float32)
    camera_points = points + center
    z = camera_points[:, 2]
    valid = np.isfinite(camera_points).all(axis=1) & (z > 0.001)
    camera_points = camera_points[valid]
    u = np.rint(fx * camera_points[:, 0] / camera_points[:, 2] + cx).astype(np.int32)
    v = np.rint(fy * camera_points[:, 1] / camera_points[:, 2] + cy).astype(np.int32)
    valid_pixels = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, camera_points = u[valid_pixels], v[valid_pixels], camera_points[valid_pixels]

    xyz = np.full((height, width, 3), np.nan, dtype=np.float32)
    # Deterministic z-buffer: for duplicate projected points keep the nearest.
    order = np.argsort(camera_points[:, 2])[::-1]
    xyz[v[order], u[order]] = camera_points[order]
    occupied = np.isfinite(xyz[:, :, 2])
    ys, xs = np.nonzero(occupied)
    if len(xs) < 250:
        raise ValueError(f"synthetic projection has only {len(xs)} unique points")
    bbox = (int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1))
    return xyz, fx, fy, cx, cy, bbox, center, int(len(xs))
