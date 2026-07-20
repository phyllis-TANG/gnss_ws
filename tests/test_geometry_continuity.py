import importlib.util
import math
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "urbanv2x_geometry_continuity.py"
)
SPEC = importlib.util.spec_from_file_location("geometry_continuity", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(epoch, azimuth, elevation, satellite=(2.0e7, 1.0e7, 1.5e7), toe=0.0):
    return {
        "epoch_utc": epoch,
        "sat_id": "G01",
        "sys": "G",
        "azimuth_deg": azimuth,
        "elevation_deg": elevation,
        "sat_ecef": satellite,
        "rx_ecef": (1.0, 2.0, 3.0),
        "geometric_range_m": 2.2e7,
        "ephemeris_toe_gpst": toe,
    }


class AngleTest(unittest.TestCase):
    def test_azimuth_wrap_is_small(self):
        difference = MODULE.circular_difference_degrees(1.0, 359.0)
        self.assertEqual(difference, 2.0)
        separation = MODULE.angular_separation_degrees(
            MODULE.los_unit(359.0, 0.0), MODULE.los_unit(1.0, 0.0)
        )
        self.assertAlmostEqual(separation, 2.0, places=10)

    def test_near_zenith_uses_3d_los_not_raw_azimuth(self):
        separation = MODULE.angular_separation_degrees(
            MODULE.los_unit(0.0, 89.99), MODULE.los_unit(180.0, 89.99)
        )
        self.assertLess(separation, 0.03)


class PairTest(unittest.TestCase):
    def setUp(self):
        self.thresholds = {
            "max_pair_gap": 2.0,
            "max_los_rate": 0.1,
            "max_sat_speed": 6000.0,
            "max_range_rate": 2000.0,
        }

    def test_smooth_pair_passes(self):
        first = row(10.0, 359.99, 30.0)
        second = row(
            11.0,
            0.01,
            30.005,
            satellite=(2.0003e7, 1.0002e7, 1.5001e7),
        )
        result = MODULE.evaluate_pair(first, second, self.thresholds)
        self.assertEqual(result["flags"], [])
        self.assertLess(result["los_rate_deg_s"], 0.1)
        self.assertTrue(math.isfinite(result["satellite_speed_m_s"]))

    def test_angle_jump_is_flagged(self):
        first = row(10.0, 10.0, 30.0)
        second = row(11.0, 40.0, 30.0)
        result = MODULE.evaluate_pair(first, second, self.thresholds)
        self.assertIn("los_rate", result["flags"])

    def test_long_gap_is_not_treated_as_continuity_pair(self):
        result = MODULE.evaluate_pair(
            row(10.0, 10.0, 30.0), row(20.0, 10.1, 30.1), self.thresholds
        )
        self.assertTrue(result["time_gap"])
        self.assertEqual(result["flags"], [])
        self.assertNotIn("los_rate_deg_s", result)


if __name__ == "__main__":
    unittest.main()
