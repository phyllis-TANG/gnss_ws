import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_primary_ray_manifest import build_manifest  # noqa: E402


class PrimaryRayManifestTests(unittest.TestCase):
    def test_reference_receiver_maps_to_enu_origin(self):
        gt = [{
            "time": 100.0, "latitude": 22.0, "longitude": 114.0,
            "ecef": (1.0, 2.0, 3.0),
        }]
        rays = [{
            "epoch_utc": 101.25, "sat_id": "G01", "sys": "G",
            "signal_records": 2, "signals": "S1C;S2X",
            "tile_window_index": 7, "azimuth_deg": 90.0,
            "elevation_deg": 30.0, "rx_ecef": np.array([1.0, 2.0, 3.0]),
        }]
        rows = build_manifest(rays, gt)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["epoch_utc"], "101.250000")
        self.assertEqual(rows[0]["signal_records"], 2)
        self.assertEqual(rows[0]["direct_surface_audit_available"], 0)
        np.testing.assert_allclose([
            rows[0]["receiver_e_m"], rows[0]["receiver_n_m"],
            rows[0]["receiver_u_m"],
        ], [0, 0, 0], atol=1e-12)

    def test_empty_ground_truth_is_rejected(self):
        with self.assertRaises(ValueError):
            build_manifest([], [])


if __name__ == "__main__":
    unittest.main()
