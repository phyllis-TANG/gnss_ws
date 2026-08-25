import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_satellite_facade_exposure import (  # noqa: E402
    azel_to_enu,
    exposure_weight,
    facade_tier,
    intersect_ray_plane,
    interval_gap,
)


def plane(x=10.0):
    return {
        "centroid_e_m": str(x), "centroid_n_m": "0", "centroid_u_m": "2",
        "normal_e": "1", "normal_n": "0", "normal_u": "0",
        "horizontal_min_m": "-3", "horizontal_max_m": "3",
        "vertical_min_m": "0", "vertical_max_m": "4",
        "member_patches": "3", "cluster_geometry_consistent": "1",
    }


class FacadeExposureTests(unittest.TestCase):
    def test_azel_enu(self):
        np.testing.assert_allclose(azel_to_enu(90, 0), [1, 0, 0], atol=1e-12)

    def test_interval_gap(self):
        self.assertEqual(interval_gap(2, 0, 4), 0.0)
        self.assertEqual(interval_gap(7, 0, 4), 3.0)

    def test_ray_plane_intersection_and_vertical_extrapolation(self):
        result = intersect_ray_plane(
            np.zeros(3), azel_to_enu(90, 45), plane(), 3, 100, 0.05
        )
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result["intersection"], [10, 0, 10], atol=1e-10)
        self.assertAlmostEqual(result["intersection_distance_m"], np.sqrt(200))
        self.assertAlmostEqual(result["horizontal_gap_m"], 0.0)
        self.assertAlmostEqual(result["vertical_gap_m"], 6.0)
        self.assertEqual(result["within_observed_rectangle"], 0)

    def test_parallel_and_behind_intersections_are_rejected(self):
        self.assertIsNone(intersect_ray_plane(
            np.zeros(3), azel_to_enu(0, 0), plane(), 3, 100, 0.05
        ))
        self.assertIsNone(intersect_ray_plane(
            np.zeros(3), azel_to_enu(90, 0), plane(-10), 3, 100, 0.05
        ))

    def test_exposure_weight_is_monotonic(self):
        exact = exposure_weight(0.8, 0, 0, 2.5, 10)
        horizontal = exposure_weight(0.8, 2, 0, 2.5, 10)
        vertical = exposure_weight(0.8, 0, 10, 2.5, 10)
        self.assertGreater(exact, horizontal)
        self.assertGreater(exact, vertical)

    def test_facade_tiers(self):
        physical = {
            "physical_geometry_consistent": "1",
            "repeated_across_tiles": "1",
            "cross_tile_reflectivity_repeatable": "1",
        }
        self.assertEqual(facade_tier(plane(), physical), "A_REPEATED")
        physical["repeated_across_tiles"] = "0"
        self.assertEqual(facade_tier(plane(), physical), "A_MULTI_PATCH")
        single = plane()
        single["member_patches"] = "1"
        self.assertEqual(facade_tier(single, physical), "B_SINGLE_PATCH")


if __name__ == "__main__":
    unittest.main()
