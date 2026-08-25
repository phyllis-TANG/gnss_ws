import os
import sys
import unittest
import importlib.util

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from urbanv2x_point_time_convention_audit import (  # noqa: E402
    aggregate_scores,
    decide,
    mutual_planar_pair,
    point_epochs,
    select_target_motion_windows,
)


class EpochConventionTest(unittest.TestCase):
    def test_all_four_time_conventions(self):
        frame = {
            "header_time": 10.0,
            "relative_time": np.array([0.0, 0.05, 0.1]),
        }
        np.testing.assert_allclose(point_epochs(frame, "frame_header"), [10, 10, 10])
        np.testing.assert_allclose(
            point_epochs(frame, "header_plus_time"), [10, 10.05, 10.1]
        )
        np.testing.assert_allclose(
            point_epochs(frame, "header_minus_time"), [10, 9.95, 9.9]
        )
        np.testing.assert_allclose(
            point_epochs(frame, "header_plus_time_minus_span"), [9.9, 9.95, 10]
        )


class MotionSelectionTest(unittest.TestCase):
    def test_selects_high_turn_and_fast_straight_without_overlap(self):
        windows = []
        for index in range(12):
            windows.append({
                "start_utc": float(index * 8),
                "end_utc": float(index * 8 + 8),
                "median_speed_m_s": float(index),
                "yaw_rate_p75_deg_s": 40.0 if index in (3, 7) else float(index % 3),
                "moving_ratio": 1.0,
                "gt_quality_mode": 2,
                "gt_quality_mode_ratio": 1.0,
            })
        selected = select_target_motion_windows(windows, 2, 2)
        self.assertEqual(len(selected), 4)
        self.assertEqual(
            sum(row["selection_role"] == "high_turn" for row in selected), 2
        )
        self.assertEqual(
            sum(row["selection_role"] == "fast_straight" for row in selected), 2
        )
        self.assertEqual(len({row["start_utc"] for row in selected}), 4)


class PlanarPairTest(unittest.TestCase):
    @unittest.skipUnless(
        importlib.util.find_spec("scipy") is not None,
        "scipy is supplied by the ROS analysis container",
    )
    def test_parallel_planes_recover_known_point_to_plane_offset(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
        left = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
        right = left.copy()
        right[:, 2] += 0.02
        result = mutual_planar_pair(
            left, right, voxel=0.05, mutual_radius=0.1,
            plane_neighbours=8, plane_radius=0.6,
            planarity_ratio=0.12, maximum_samples=1000,
        )
        self.assertGreater(result["mutual_matches"], 80)
        self.assertGreater(len(result["plane_residuals"]), 50)
        self.assertAlmostEqual(
            float(np.median(result["plane_residuals"])), 0.02, places=5
        )


class RankingTest(unittest.TestCase):
    def test_clear_lower_and_higher_metric_winner_is_preferred(self):
        rows = []
        for segment in range(4):
            for convention, factor in (
                ("frame_header", 1.25),
                ("header_plus_time", 1.00),
                ("header_minus_time", 1.10),
                ("header_plus_time_minus_span", 1.15),
            ):
                rows.append({
                    "segment_index": segment,
                    "convention": convention,
                    "static_plane_residual_median_m": 0.02 * factor,
                    "mutual_distance_median_m": 0.1 * factor,
                    "mutual_ratio_median": 0.5 / factor,
                    "planar_matches": 1000.0 / factor,
                })
        summary, usable = aggregate_scores(rows)
        decision, preferred, margin = decide(summary, 0.01)
        self.assertEqual(len(usable), 16)
        self.assertEqual(decision, "PREFERRED")
        self.assertEqual(preferred, "header_plus_time")
        self.assertGreater(margin, 0.01)


if __name__ == "__main__":
    unittest.main()
