import importlib.util
import math
import pathlib
import unittest

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_map_consistency.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_map_consistency", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RotationTest(unittest.TestCase):
    def test_level_north_flu_axes_map_to_enu(self):
        vectors = np.eye(3)
        zeros = np.zeros(3)
        heading_north = np.zeros(3)
        transformed = MODULE.imu_flu_to_enu(
            vectors, zeros, zeros, heading_north
        )
        expected = np.array([
            [0.0, 1.0, 0.0],   # forward -> north
            [-1.0, 0.0, 0.0],  # left -> west
            [0.0, 0.0, 1.0],   # up -> up
        ])
        np.testing.assert_allclose(transformed, expected, atol=1e-12)

    def test_east_heading_maps_forward_to_east(self):
        transformed = MODULE.imu_flu_to_enu(
            np.array([[1.0, 0.0, 0.0]]),
            np.array([0.0]), np.array([0.0]), np.array([math.pi / 2.0]),
        )
        np.testing.assert_allclose(transformed, [[1.0, 0.0, 0.0]], atol=1e-12)


class VoxelTest(unittest.TestCase):
    def test_voxel_downsample_averages_position_and_intensity(self):
        xyz = np.array([[0.01, 0.01, 0.01], [0.09, 0.09, 0.09], [1.0, 0.0, 0.0]])
        intensity = np.array([10.0, 20.0, 30.0])
        down, down_i, _, counts = MODULE.voxel_downsample(xyz, intensity, 0.1)
        self.assertEqual(len(down), 2)
        self.assertEqual(sorted(counts.tolist()), [1, 2])
        self.assertIn(15.0, down_i.tolist())


class RankTest(unittest.TestCase):
    def test_consistent_metric_winner_is_preferred(self):
        results = []
        for name, scale in (("a", 1.0), ("b", 1.2), ("c", 1.3)):
            results.append({
                "candidate": name,
                "map_voxels_per_1000_points": 100.0 * scale,
                "ground_z_iqr_median_m": 0.1 * scale,
                "planar_residual_median_m": 0.05 * scale,
                "overlap_nearest_median_m": 0.2 * scale,
            })
        preferred, decision, margin, metrics = MODULE.rank_candidates(results)
        self.assertEqual(preferred, "a")
        self.assertEqual(decision, "preferred")
        self.assertGreater(margin, 0.01)
        self.assertEqual(len(metrics), 4)


if __name__ == "__main__":
    unittest.main()
