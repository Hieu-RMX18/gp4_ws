"""RGB-D detection: color voting, RANSAC normal gate,
index-tracking downsample/cluster, multi-factor confidence."""

import numpy as np

from gp4_perception.scene_geometry import (
    _confidence_score,
    _dominant_color_voting,
    _euclidean_cluster_indices,
    _euclidean_clusters,
    _has_chromatic_border,
    _ransac_plane,
    _ransac_plane_fit,
    _semantic_class_id,
    _voxel_downsample,
    _voxel_downsample_indices,
)


def _solid_patch(rgb: tuple[int, int, int], n: int) -> np.ndarray:
    return np.tile(np.array(rgb, dtype=np.uint8), (n, 1))


class TestDominantColorVoting:
    def test_solid_red_votes_red(self):
        name, conf = _dominant_color_voting(_solid_patch((220, 20, 20), 100))
        assert name == "red"
        assert conf > 0.9

    def test_red_with_minority_text_still_red(self):
        # Red box with yellow/white/black logo text — mean would be polluted,
        # voting should still pick red.
        red = _solid_patch((220, 20, 20), 80)
        text = np.vstack(
            [
                _solid_patch((250, 250, 250), 8),  # white
                _solid_patch((240, 220, 20), 8),  # yellow
                _solid_patch((10, 10, 10), 4),  # black
            ]
        )
        name, _ = _dominant_color_voting(np.vstack([red, text]))
        assert name == "red"

    def test_white_with_blue_border_prefers_blue(self):
        # White rectangle with a thin blue border: achromatic majority but a
        # meaningful chromatic minority should win.
        white = _solid_patch((245, 245, 245), 80)
        blue = _solid_patch((20, 40, 220), 20)
        name, conf = _dominant_color_voting(np.vstack([white, blue]))
        assert name == "blue"
        assert conf > 0.0

    def test_empty_is_unknown(self):
        name, conf = _dominant_color_voting(np.empty((0, 3), dtype=np.uint8))
        assert name == "unknown"
        assert conf == 0.0

    def test_none_is_unknown(self):
        name, conf = _dominant_color_voting(None)
        assert name == "unknown"
        assert conf == 0.0


class TestRansacPlaneFit:
    def test_horizontal_plane_removed_with_vertical_normal_gate(self):
        plane = np.random.rand(500, 3).astype(np.float32)
        plane[:, 2] = 0.0  # normal = z-axis, normal_z = 1
        cluster = np.random.rand(50, 3).astype(np.float32)
        cluster[:, 2] = 0.1
        pts = np.vstack([plane, cluster])
        in_idx, out_idx, normal = _ransac_plane_fit(
            pts, threshold=0.005, normal_z_min=0.85
        )
        assert len(in_idx) > len(out_idx)
        assert abs(normal[2]) >= 0.85

    def test_vertical_plane_kept_with_vertical_normal_gate(self):
        # Plane at x=const → normal along x, normal_z ≈ 0. Must NOT be removed.
        plane = np.random.rand(500, 3).astype(np.float32)
        plane[:, 0] = 0.0
        pts = plane
        in_idx, out_idx, _ = _ransac_plane_fit(
            pts, threshold=0.005, normal_z_min=0.85
        )
        assert len(in_idx) == 0
        assert len(out_idx) == len(pts)

    def test_backcompat_wrapper_returns_points(self):
        plane = np.random.rand(500, 3).astype(np.float32)
        plane[:, 2] = 0.0
        cluster = np.random.rand(50, 3).astype(np.float32)
        cluster[:, 2] = 0.1
        pts = np.vstack([plane, cluster])
        inliers, outliers = _ransac_plane(pts, threshold=0.005)
        assert inliers.shape[1] == 3
        assert len(inliers) > len(outliers)


class TestVoxelDownsampleIndices:
    def test_indices_reconstruct_points(self):
        pts = (np.random.rand(1000, 3).astype(np.float32) * 0.1)
        idx = _voxel_downsample_indices(pts, voxel_size=0.02)
        np.testing.assert_array_equal(pts[idx], _voxel_downsample(pts, 0.02))

    def test_empty(self):
        pts = np.empty((0, 3), dtype=np.float32)
        idx = _voxel_downsample_indices(pts, voxel_size=0.01)
        assert len(idx) == 0


class TestEuclideanClusterIndices:
    def test_indices_match_point_clusters(self):
        c1 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.3, 0.0, 0.1]
        )
        c2 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.4, 0.0, 0.1]
        )
        pts = np.vstack([c1, c2])
        idx_list = _euclidean_cluster_indices(
            pts, tolerance=0.03, min_size=20, max_size=500
        )
        pt_clusters = _euclidean_clusters(pts, 0.03, 20, 500)
        assert len(idx_list) == len(pt_clusters) == 2
        for idx, pc in zip(idx_list, pt_clusters):
            np.testing.assert_array_equal(pts[idx], pc)


class TestConfidenceScore:
    def test_renormalizes_without_temporal(self):
        # All factors perfect → score 1.0 even though temporal is absent.
        s = _confidence_score(1.0, 1.0, 1.0, temporal_conf=None)
        assert abs(s - 1.0) < 1e-6

    def test_with_temporal(self):
        s = _confidence_score(1.0, 1.0, 1.0, temporal_conf=1.0)
        assert abs(s - 1.0) < 1e-6

    def test_low_color_lowers_score(self):
        hi = _confidence_score(1.0, 1.0, 1.0)
        lo = _confidence_score(0.0, 1.0, 1.0)
        assert lo < hi

    def test_bounded_unit_interval(self):
        assert 0.0 <= _confidence_score(0.0, 0.0, 0.0) <= 1.0
        assert 0.0 <= _confidence_score(0.5, 0.7, 0.3, 0.6) <= 1.0


class TestDualColorBorderAnalysis:
    """Blue-bordered white objects should be identified as blue_rectangle."""

    def test_white_interior_blue_border_maps_to_blue_rectangle(self):
        # Simulate a white box with blue border: 80% white, 20% blue.
        # Arrange points so blue pixels are on the edge (outermost).
        n_white = 80
        n_blue = 20
        cluster = np.zeros((n_white + n_blue, 3), dtype=np.float32)
        # Interior white points: near center
        cluster[:n_white] = np.random.rand(n_white, 3).astype(np.float32) * 0.01
        # Border blue points: further from center
        cluster[n_white:] = (
            np.random.rand(n_blue, 3).astype(np.float32) * 0.005 + 0.03
        )
        rgb = np.vstack([
            _solid_patch((245, 245, 245), n_white),
            _solid_patch((20, 40, 220), n_blue),
        ])

        # Create a mock dims object.
        class Dims:
            x = 0.05
            y = 0.03
            z = 0.01

        class_id = _semantic_class_id("white", "box", Dims(), rgb_pixels=rgb, cluster=cluster)
        assert class_id == "blue_rectangle"

    def test_pure_white_stays_white(self):
        rgb = _solid_patch((245, 245, 245), 100)
        cluster = np.random.rand(100, 3).astype(np.float32) * 0.03

        class Dims:
            x = 0.05
            y = 0.03
            z = 0.01

        class_id = _semantic_class_id("white", "box", Dims(), rgb_pixels=rgb, cluster=cluster)
        assert class_id == "white_box"

    def test_has_chromatic_border_returns_none_for_uniform_white(self):
        rgb = _solid_patch((245, 245, 245), 100)
        cluster = np.random.rand(100, 3).astype(np.float32) * 0.03
        color, ratio = _has_chromatic_border(rgb, cluster)
        assert color is None
        assert ratio == 0.0

    def test_has_chromatic_border_detects_blue(self):
        n_white = 80
        n_blue = 20
        cluster = np.zeros((n_white + n_blue, 3), dtype=np.float32)
        cluster[:n_white] = np.random.rand(n_white, 3).astype(np.float32) * 0.01
        cluster[n_white:] = (
            np.random.rand(n_blue, 3).astype(np.float32) * 0.005 + 0.03
        )
        rgb = np.vstack([
            _solid_patch((245, 245, 245), n_white),
            _solid_patch((20, 40, 220), n_blue),
        ])
        color, ratio = _has_chromatic_border(rgb, cluster)
        assert color == "blue"
        assert ratio > 0.0
