"""Pluggable detection backends.

A backend turns a raw LiDAR point cloud into 3D boxes. Two exist:

* :class:`LivoxCenterPointBackend` - the ported livox_detection network.
* :class:`ClusteringBackend` - a dependency-free geometric fallback, so the
  RViz pipeline can be exercised without a GPU or model weights.

Both return the same tuple so :mod:`detection_node` does not care which is in use.
"""

from __future__ import annotations

import abc

import numpy as np

# livox_detection's class ordering, taken from the 3-channel heatmap head.
CLASS_NAMES = ("car", "pedestrian", "cyclist")

# Rendering colours (RGB) matching the reference implementation's convention.
CLASS_COLORS = {
    "car": (0.0, 1.0, 1.0),
    "pedestrian": (1.0, 1.0, 0.0),
    "cyclist": (0.0, 1.0, 0.0),
}


class DetectionBackend(abc.ABC):
    """Interface for anything that turns points into boxes."""

    @abc.abstractmethod
    def load(self) -> None:
        """Prepare the backend. Called once before the first :meth:`infer`."""

    @abc.abstractmethod
    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Run detection on an ``(N, 4)`` array of ``x, y, z, intensity``.

        Returns:
            boxes: ``(M, 7)`` of ``x, y, z, dx, dy, dz, yaw`` in the sensor frame.
            scores: ``(M,)`` confidences in ``[0, 1]``.
            labels: ``(M,)`` indices into :data:`CLASS_NAMES`.
        """

    @property
    def name(self) -> str:
        return type(self).__name__


def _empty() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return (
        np.zeros((0, 7), dtype=np.float32),
        np.zeros((0,), dtype=np.float32),
        np.zeros((0,), dtype=np.int64),
    )


class ClusteringBackend(DetectionBackend):
    """Voxel-grid connected-component clustering with a person-shaped filter.

    Deliberately simple: it exists so the topic graph, message types and RViz
    config can be validated end-to-end without the neural network. It finds
    upright, person-sized blobs and is not expected to be accurate.
    """

    def __init__(
        self,
        voxel_size: float = 0.2,
        min_points: int = 12,
        z_range: tuple[float, float] = (-1.6, 1.2),
        max_radius: float = 25.0,
    ):
        self.voxel_size = voxel_size
        self.min_points = min_points
        self.z_range = z_range
        self.max_radius = max_radius

    def load(self) -> None:  # nothing to do
        pass

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if points.shape[0] == 0:
            return _empty()

        xyz = points[:, :3]
        radial = np.linalg.norm(xyz[:, :2], axis=1)
        keep = (
            (xyz[:, 2] > self.z_range[0])
            & (xyz[:, 2] < self.z_range[1])
            & (radial < self.max_radius)
            & (radial > 0.5)  # drop returns off the robot itself
        )
        xyz = xyz[keep]
        if xyz.shape[0] < self.min_points:
            return _empty()

        # Cluster in the BEV plane only: pedestrians are well separated from
        # above even when their point columns overlap in 3D.
        grid = np.floor(xyz[:, :2] / self.voxel_size).astype(np.int64)
        _, inverse, counts = np.unique(grid, axis=0, return_inverse=True, return_counts=True)

        boxes, scores, labels = [], [], []
        for cell in np.flatnonzero(counts >= self.min_points):
            member = xyz[inverse == cell]
            lo, hi = member.min(axis=0), member.max(axis=0)
            extent = hi - lo
            height = float(extent[2])
            footprint = float(max(extent[0], extent[1]))

            # Person-shaped: tall, narrow, and taller than it is wide.
            if not (0.8 < height < 2.2 and footprint < 1.2 and height > footprint):
                continue

            centre = (lo + hi) / 2.0
            boxes.append([centre[0], centre[1], centre[2], max(extent[0], 0.3), max(extent[1], 0.3), height, 0.0])
            # Confidence is a placeholder - this backend has no real notion of one.
            scores.append(0.5)
            labels.append(CLASS_NAMES.index("pedestrian"))

        if not boxes:
            return _empty()

        return (
            np.asarray(boxes, dtype=np.float32),
            np.asarray(scores, dtype=np.float32),
            np.asarray(labels, dtype=np.int64),
        )
