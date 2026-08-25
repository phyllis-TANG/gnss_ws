import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_local_surface_consensus import (  # noqa: E402
    confidence_band,
    max_pairwise_normal_angle,
    quality_reasons,
    unsigned_normal_angle,
)


class Arguments:
    diffuse_points_min = 12
    planarity_min = 0.30
    plane_residual_max = 0.15
    incidence_max = 80.0
    retro_fraction_max = 0.20


class LocalSurfaceConsensusTests(unittest.TestCase):
    def test_confidence_bands(self):
        self.assertEqual(confidence_band("111"), "CORE_STRICT")
        self.assertEqual(confidence_band("011"), "MID_TOLERANCE")
        self.assertEqual(confidence_band("001"), "LOOSE_ONLY")
        self.assertEqual(confidence_band("000"), "LOCAL_MAP_UNOBSERVED")
        self.assertEqual(confidence_band("101"), "NON_MONOTONIC")

    def test_unsigned_normal_angle(self):
        self.assertAlmostEqual(unsigned_normal_angle([1, 0, 0], [-1, 0, 0]), 0.0)
        self.assertAlmostEqual(unsigned_normal_angle([1, 0, 0], [0, 1, 0]), 90.0)

    def test_max_pairwise_normal_angle(self):
        rows = [
            {"normal_x": "1", "normal_y": "0", "normal_z": "0"},
            {"normal_x": "0.98480775", "normal_y": "0.17364818", "normal_z": "0"},
            {"normal_x": "-1", "normal_y": "0", "normal_z": "0"},
        ]
        self.assertAlmostEqual(max_pairwise_normal_angle(rows), 10.0, places=5)

    def test_quality_reasons(self):
        good = {
            "patch_diffuse_points": "20", "planarity": "0.5",
            "plane_residual_median_m": "0.05", "incidence_deg": "40",
            "patch_retro_fraction": "0",
        }
        self.assertEqual(quality_reasons(good, Arguments()), [])
        bad = dict(good)
        bad.update({"planarity": "0.1", "incidence_deg": "85"})
        self.assertEqual(
            quality_reasons(bad, Arguments()),
            ["low_planarity", "grazing_incidence"],
        )


if __name__ == "__main__":
    unittest.main()
