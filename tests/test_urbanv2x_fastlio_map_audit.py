import importlib.util
import pathlib
import tempfile
import unittest

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_fastlio_map_audit.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_fastlio_map_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_test_pcd(path, points, trailing=b""):
    points = np.asarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n"
    ) % (len(points), len(points))
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(points.tobytes())
        handle.write(trailing)


class PcdTest(unittest.TestCase):
    def test_binary_header_and_payload_are_validated(self):
        points = np.array([
            [0.0, 1.0, 2.0, 10.0],
            [3.0, 4.0, 5.0, 20.0],
            [6.0, 7.0, 8.0, 30.0],
        ])
        with tempfile.TemporaryDirectory() as directory:
            valid_path = pathlib.Path(directory) / "valid.pcd"
            invalid_path = pathlib.Path(directory) / "invalid.pcd"
            write_test_pcd(valid_path, points)
            write_test_pcd(invalid_path, points, trailing=b"x")
            valid = MODULE.parse_pcd_header(valid_path)
            invalid = MODULE.parse_pcd_header(invalid_path)
            inspected, xyz, intensity = MODULE.inspect_chunk(valid_path, 2)

        self.assertTrue(valid["payload_exact"])
        self.assertFalse(invalid["payload_exact"])
        self.assertEqual(valid["point_step"], 16)
        self.assertEqual(inspected["points"], 3)
        self.assertEqual(len(xyz), 2)
        self.assertEqual(len(intensity), 2)

    def test_voxel_downsample_averages_points(self):
        xyz = np.array([
            [0.01, 0.01, 0.01],
            [0.09, 0.09, 0.09],
            [0.21, 0.01, 0.01],
        ], dtype=np.float32)
        result = MODULE.voxel_downsample_xyz(xyz, 0.2)
        self.assertEqual(result.shape, (2, 3))
        np.testing.assert_allclose(result[0], [0.05, 0.05, 0.05], atol=1e-7)


class OverlapTest(unittest.TestCase):
    def test_identical_geometry_with_small_shift_has_small_world_residual(self):
        x, y = np.meshgrid(np.arange(0.0, 2.0, 0.1), np.arange(0.0, 2.0, 0.1))
        left = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
        right = left + np.array([0.03, 0.0, 0.0])
        result = MODULE.adjacent_overlap(left, right, voxel=0.05, radius=0.2)
        self.assertGreater(result["mutual_matches"], 300)
        self.assertLess(result["mutual_distance_median_m"], 0.031)
        self.assertLess(result["mutual_distance_p95_m"], 0.031)


if __name__ == "__main__":
    unittest.main()
