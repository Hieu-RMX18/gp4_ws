"""Packed RGB/RGBA decode from PointCloud2 + per-cluster color diagnostics."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from gp4_perception.scene_geometry import (
    _color_diagnostics,
    _read_xyz_rgb,
    _unpack_packed_rgb,
)


def _packed_cloud_array(colors_rgb, field="rgb", dtype="f"):
    """Structured array like sensor_msgs_py.read_points output with a packed
    rgb field. *colors_rgb* is a list of (r,g,b)."""
    u32 = np.array(
        [(r << 16) | (g << 8) | b for (r, g, b) in colors_rgb], dtype=np.uint32
    )
    if dtype == "f":
        packed = u32.view(np.float32)
        np_dtype = np.float32
    else:
        packed = u32
        np_dtype = np.uint32
    arr = np.zeros(
        len(colors_rgb),
        dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), (field, np_dtype)],
    )
    arr["x"] = np.arange(len(colors_rgb), dtype=np.float32) * 0.01
    arr["y"] = 0.0
    arr["z"] = 0.1
    arr[field] = packed
    return arr


class TestUnpackPackedRgb:
    def test_float32_rgb_unpacks_to_uint8(self):
        arr = _packed_cloud_array([(255, 0, 0), (0, 0, 255), (10, 200, 30)])
        xyz, rgb = _unpack_packed_rgb(arr, "rgb")
        assert xyz.shape == (3, 3)
        assert xyz.dtype == np.float32
        assert rgb.dtype == np.uint8
        assert rgb[0].tolist() == [255, 0, 0]
        assert rgb[1].tolist() == [0, 0, 255]
        assert rgb[2].tolist() == [10, 200, 30]

    def test_uint32_rgb_unpacks(self):
        arr = _packed_cloud_array([(12, 34, 56)], dtype="u")
        _, rgb = _unpack_packed_rgb(arr, "rgb")
        assert rgb[0].tolist() == [12, 34, 56]

    def test_rgba_field_ignores_alpha(self):
        # 0xAARRGGBB → RGB extraction drops alpha.
        u32 = np.array([(0xFF << 24) | (200 << 16) | (100 << 8) | 50], dtype=np.uint32)
        arr = np.zeros(
            1,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgba", np.uint32)],
        )
        arr["rgba"] = u32
        _, rgb = _unpack_packed_rgb(arr, "rgba")
        assert rgb[0].tolist() == [200, 100, 50]


class TestReadPointCloud2Rgb:
    def test_reads_packed_rgb_pointcloud2(self):
        # Match RealSense/PCL layout: x,y,z float32 + rgb float32 bit-packed.
        colors = [(255, 0, 0), (0, 0, 255)]
        packed = np.array(
            [(r << 16) | (g << 8) | b for (r, g, b) in colors], dtype=np.uint32
        ).view(np.float32)
        data = np.zeros(
            2,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)],
        )
        data["x"] = [0.1, 0.2]
        data["z"] = [0.3, 0.4]
        data["rgb"] = packed

        cloud = PointCloud2()
        cloud.height = 1
        cloud.width = 2
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = data.dtype.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name="x", offset=data.dtype.fields["x"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=data.dtype.fields["y"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=data.dtype.fields["z"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=data.dtype.fields["rgb"][1], datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = data.tobytes()

        xyz, rgb = _read_xyz_rgb(cloud)
        np.testing.assert_allclose(xyz, [[0.1, 0.0, 0.3], [0.2, 0.0, 0.4]])
        assert rgb.tolist() == [[255, 0, 0], [0, 0, 255]]


class TestColorDiagnostics:
    def test_red_cluster_votes_red_in_rgb(self):
        rgb = np.tile(np.array([220, 20, 20], dtype=np.uint8), (50, 1))
        d = _color_diagnostics(rgb)
        assert d["n"] == 50
        assert d["rgb_mean"][0] > d["rgb_mean"][2]  # R > B
        assert d["vote_rgb"][0] == "red"

    def test_bgr_swap_changes_vote(self):
        # Data is actually blue but if interpreted as BGR (swapped) reads red.
        rgb = np.tile(np.array([20, 20, 220], dtype=np.uint8), (50, 1))
        d = _color_diagnostics(rgb)
        assert d["vote_rgb"][0] == "blue"
        assert d["vote_bgr"][0] == "red"

    def test_empty(self):
        d = _color_diagnostics(np.empty((0, 3), dtype=np.uint8))
        assert d["n"] == 0
