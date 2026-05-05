"""Algorithmic unit tests for scene_processor pipeline functions."""

import numpy as np

from gp4_perception.scene_processor import (
    _euclidean_clusters,
    _pca_bbox,
    _ransac_plane,
    _roi_crop,
    _voxel_downsample,
)


class TestRoiCrop:
    def test_crops_outside(self):
        pts = np.array(
            [
                [0.30, 0.0, 0.1],  # inside
                [0.6, 0.0, 0.1],  # outside x max
                [0.3, -0.4, 0.1],  # outside y min
            ],
            dtype=np.float32,
        )
        bbox = {"x": [0.2, 0.55], "y": [-0.30, 0.30], "z": [0.00, 0.40]}
        cropped = _roi_crop(pts, bbox)
        assert len(cropped) == 1
        np.testing.assert_array_equal(cropped[0], pts[0])


class TestVoxelDownsample:
    def test_reduces_points(self):
        pts = np.random.rand(1000, 3).astype(np.float32) * 0.1
        down = _voxel_downsample(pts, voxel_size=0.02)
        assert len(down) < len(pts)

    def test_empty_input(self):
        pts = np.empty((0, 3), dtype=np.float32)
        down = _voxel_downsample(pts, voxel_size=0.01)
        assert len(down) == 0


class TestRansacPlane:
    def test_removes_largest_plane(self):
        # Large plane at z=0
        plane = np.random.rand(500, 3).astype(np.float32)
        plane[:, 2] = 0.0
        # Small cluster above
        cluster = np.random.rand(50, 3).astype(np.float32)
        cluster[:, 2] = 0.1
        pts = np.vstack([plane, cluster])
        inliers, outliers = _ransac_plane(pts, threshold=0.005)
        assert len(inliers) > len(outliers)
        assert len(outliers) > 0

    def test_empty_input(self):
        pts = np.empty((0, 3), dtype=np.float32)
        inliers, outliers = _ransac_plane(pts, threshold=0.005)
        assert len(inliers) == 0
        assert len(outliers) == 0


class TestEuclideanClusters:
    def test_finds_two_clusters(self):
        c1 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.3, 0.0, 0.1]
        )
        c2 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.4, 0.0, 0.1]
        )
        pts = np.vstack([c1, c2])
        clusters = _euclidean_clusters(pts, tolerance=0.03, min_size=20, max_size=500)
        assert len(clusters) == 2

    def test_noise_only(self):
        pts = np.random.rand(20, 3).astype(np.float32) * 0.1
        clusters = _euclidean_clusters(pts, tolerance=0.01, min_size=10, max_size=500)
        assert len(clusters) == 0


class TestPcaBbox:
    def test_box_on_axis_aligned_cube(self):
        pts = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.0, 0.05, 0.0],
                [0.0, 0.0, 0.02],
            ],
            dtype=np.float32,
        )
        pose, dims = _pca_bbox(pts)
        assert dims.x > 0
        assert dims.y > 0
        assert dims.z > 0
        assert abs(pose.position.x - 0.025) < 0.01
