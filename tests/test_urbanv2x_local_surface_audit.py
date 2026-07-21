import math
import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_local_surface_audit import (  # noqa: E402
    azel_to_enu,
    fit_surface_patch,
    select_representative_tiles,
    trace_local_ray,
    voxelise_surface,
)


class LocalSurfaceAuditTests(unittest.TestCase):
    def test_azel_to_enu_cardinal_directions(self):
        np.testing.assert_allclose(azel_to_enu(0, 0), [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(azel_to_enu(90, 0), [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(azel_to_enu(0, 90), [0, 0, 1], atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(azel_to_enu(237, 31)), 1.0)

    def test_representative_tile_selection_is_deterministic(self):
        values = list(range(2, 42, 2))
        selected = select_representative_tiles(values, 5)
        self.assertEqual(selected, [2, 12, 22, 30, 40])
        self.assertEqual(select_representative_tiles([3, 1, 2], 0), [1, 2, 3])

    def test_voxelise_surface_density_normalises_points(self):
        xyz = np.array([
            [0.01, 0.01, 0.01], [0.09, 0.09, 0.09], [1.01, 0, 0]
        ])
        intensity = np.array([10.0, 30.0, 50.0])
        centres, values = voxelise_surface(xyz, intensity, 0.1)
        self.assertEqual(len(centres), 2)
        np.testing.assert_allclose(centres[0], [0.05, 0.05, 0.05])
        self.assertAlmostEqual(float(values[0]), 20.0)

    def test_planar_patch_normal_and_intensity(self):
        grid = np.array([
            [5.0, y, z]
            for y in np.linspace(-0.4, 0.4, 5)
            for z in np.linspace(-0.4, 0.4, 5)
        ])
        intensity = np.arange(1, 26, dtype=float)
        patch = fit_surface_patch(
            grid, intensity, np.arange(len(grid)), np.array([5.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]), 0.05, 100.0, 8,
        )
        self.assertIsNotNone(patch)
        self.assertAlmostEqual(abs(patch["normal_x"]), 1.0, places=7)
        self.assertAlmostEqual(patch["incidence_deg"], 0.0, places=7)
        self.assertEqual(patch["patch_diffuse_points"], 25)
        self.assertAlmostEqual(patch["patch_intensity_median"], 13.0)

    def test_trace_supported_hit(self):
        class BruteTree:
            def __init__(self, points):
                self.points = np.asarray(points)

            def query(self, queries, k=1, distance_upper_bound=math.inf):
                queries = np.atleast_2d(queries)
                distances = np.linalg.norm(
                    queries[:, None, :] - self.points[None, :, :], axis=2
                )
                indices = np.argmin(distances, axis=1)
                minimum = distances[np.arange(len(queries)), indices]
                invalid = minimum > distance_upper_bound
                minimum[invalid] = math.inf
                indices[invalid] = len(self.points)
                return minimum, indices

            def query_ball_point(self, query, r):
                distances = np.linalg.norm(self.points - query, axis=1)
                return np.flatnonzero(distances <= r).tolist()

        xyz = np.array([
            [5.0, -0.1, 0.0], [5.0, 0.0, 0.0], [5.0, 0.1, 0.0],
            [8.0, 0.0, 0.0],
        ])
        result = trace_local_ray(
            BruteTree(xyz), xyz, np.zeros(3), np.array([1.0, 0.0, 0.0]),
            1.0, 10.0, 0.1, 0.25, 3,
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result["axial_distance_m"], 5.0)
        self.assertGreaterEqual(result["support"], 3)


if __name__ == "__main__":
    unittest.main()
