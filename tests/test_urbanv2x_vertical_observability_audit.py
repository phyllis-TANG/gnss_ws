import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_vertical_observability_audit import (  # noqa: E402
    circular_difference_degrees,
    elevation_bin,
    point_angles,
    sector_statistics,
)


class VerticalObservabilityTests(unittest.TestCase):
    def test_circular_difference_wraps(self):
        output = circular_difference_degrees([359, 1, 180], 0)
        np.testing.assert_allclose(output, [-1, 1, -180])

    def test_point_angles_use_enu_azimuth(self):
        points = np.array([
            [0, 10, 0], [10, 0, 10], [0, -10, -10], [0.1, 0.1, 0],
        ])
        azimuth, elevation = point_angles(points, [0, 0, 0], 1, 20)
        np.testing.assert_allclose(azimuth, [0, 90, 180], atol=1e-10)
        np.testing.assert_allclose(elevation, [0, 45, -45], atol=1e-10)

    def test_sector_statistics_wraparound(self):
        azimuth = np.array([358, 1, 20, 180], dtype=float)
        elevation = np.array([5, 15, 50, -10], dtype=float)
        result = sector_statistics(azimuth, elevation, 0, 3, 2)
        self.assertEqual(result["sector_points"], 2)
        self.assertEqual(result["sector_sufficient"], 1)
        self.assertAlmostEqual(result["sector_elevation_max_deg"], 15.0)

    def test_elevation_bins(self):
        self.assertEqual(elevation_bin(10), "0-15")
        self.assertEqual(elevation_bin(15), "15-30")
        self.assertEqual(elevation_bin(44.9), "30-45")
        self.assertEqual(elevation_bin(75), "60-90")


if __name__ == "__main__":
    unittest.main()
