import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_facade_patch_inventory import (  # noqa: E402
    evaluate_repeatability,
    facade_quality_reasons,
    fit_patch_geometry,
    intensity_statistics,
)


class Arguments:
    normal_repeat_max = 15.0
    plane_offset_repeat_max = 0.15
    intensity_repeat_absolute = 5.0
    intensity_repeat_relative = 0.30
    vertical_normal_z_max = 0.35
    planarity_min = 0.30
    plane_residual_median_max = 0.10
    plane_residual_p95_max = 0.20
    horizontal_span_min = 0.80
    vertical_span_min = 0.80
    diffuse_points_min = 12


def vertical_plane(x_value=4.0, noise=0.0):
    rng = np.random.default_rng(4)
    points = np.array([
        [x_value, y, z]
        for y in np.linspace(-1.0, 1.0, 9)
        for z in np.linspace(0.0, 2.0, 9)
    ], dtype=float)
    points[:, 0] += rng.normal(0.0, noise, len(points))
    return points


class FacadePatchInventoryTests(unittest.TestCase):
    def test_vertical_plane_geometry(self):
        geometry = fit_patch_geometry(vertical_plane(noise=0.01))
        self.assertLess(abs(geometry["normal"][2]), 0.02)
        self.assertGreater(geometry["planarity"], 0.9)
        self.assertGreater(geometry["horizontal_span_m"], 1.5)
        self.assertGreater(geometry["vertical_span_m"], 1.5)

    def test_horizontal_plane_is_rejected(self):
        points = np.array([
            [x, y, 1.0]
            for x in np.linspace(-1, 1, 8)
            for y in np.linspace(-1, 1, 8)
        ])
        geometry = fit_patch_geometry(points)
        intensity = intensity_statistics(np.full(len(points), 20.0), 100.0)
        self.assertIn("not_vertical", facade_quality_reasons(
            geometry, intensity, Arguments()
        ))

    def test_intensity_statistics_exclude_retro(self):
        result = intensity_statistics([0, 10, 20, 30, 150], 100.0)
        self.assertEqual(result["diffuse_points"], 3)
        self.assertEqual(result["retro_points"], 1)
        self.assertAlmostEqual(result["intensity_median"], 20.0)
        self.assertAlmostEqual(result["retro_fraction"], 0.25)

    def test_repeatability_separates_geometry_and_intensity(self):
        early_geometry = fit_patch_geometry(vertical_plane(4.00, noise=0.005))
        late_geometry = fit_patch_geometry(vertical_plane(4.04, noise=0.005))
        early_intensity = intensity_statistics(np.full(30, 20.0), 100.0)
        late_intensity = intensity_statistics(np.full(30, 22.0), 100.0)
        result = evaluate_repeatability(
            early_geometry, late_geometry, early_intensity, late_intensity,
            Arguments(),
        )
        self.assertEqual(result["geometry_repeatable"], 1)
        self.assertEqual(result["reflectivity_repeatable"], 1)
        self.assertAlmostEqual(result["plane_offset_repeat_m"], 0.04, places=2)
        self.assertAlmostEqual(result["intensity_repeat_abs"], 2.0)


if __name__ == "__main__":
    unittest.main()
