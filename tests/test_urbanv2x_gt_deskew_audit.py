import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from urbanv2x_gt_deskew_audit import (  # noqa: E402
    aggregate_candidate_scores,
    decide,
    select_representative_windows,
    transform_frame_candidate,
)


class LinearPose:
    def interpolate(self, epochs):
        epochs = np.asarray(epochs, dtype=np.float64)
        position = np.column_stack([epochs, np.zeros_like(epochs), np.zeros_like(epochs)])
        zeros = np.zeros_like(epochs)
        # Heading 90 degrees makes FLU forward coincide with ENU east.
        heading = np.full_like(epochs, np.pi / 2.0)
        return position, zeros, zeros, heading


class TransformTest(unittest.TestCase):
    def test_pointwise_control_uses_individual_point_epochs(self):
        frame = {
            "header_time": 10.0,
            "xyz": np.zeros((3, 3), dtype=np.float64),
            "intensity": np.ones(3, dtype=np.float32),
            "relative_time": np.array([0.0, 0.05, 0.1]),
        }
        matrix = np.eye(4)
        pointwise = transform_frame_candidate(
            frame, LinearPose(), matrix, 0.0, True
        )
        rigid = transform_frame_candidate(
            frame, LinearPose(), matrix, 0.0, False
        )
        np.testing.assert_allclose(pointwise["xyz"][:, 0], [10.0, 10.05, 10.1])
        np.testing.assert_allclose(rigid["xyz"][:, 0], [10.0, 10.0, 10.0])


class SelectionTest(unittest.TestCase):
    def test_representative_selection_keeps_motion_extremes(self):
        windows = []
        for index in range(10):
            windows.append({
                "start_utc": float(index * 8),
                "end_utc": float(index * 8 + 8),
                "median_speed_m_s": float(index),
                "yaw_rate_p75_deg_s": 50.0 if index == 4 else float(index % 3),
                "moving_ratio": 1.0,
                "gt_quality_mode": 1,
                "gt_quality_mode_ratio": 1.0,
            })
        selected = select_representative_windows(windows, 6)
        roles = {row["selection_role"] for row in selected}
        starts = {row["start_utc"] for row in selected}
        self.assertEqual(len(selected), 6)
        self.assertIn("high_turn", roles)
        self.assertIn("fast_straight", roles)
        self.assertIn("low_motion", roles)
        self.assertIn(32.0, starts)
        self.assertIn(0.0, starts)


class RankingTest(unittest.TestCase):
    def test_pointwise_candidate_beats_frame_and_lag_can_be_indistinguishable(self):
        rows = []
        values = {
            "point_minus_lag": 1.00,
            "point_raw": 1.005,
            "point_plus_lag": 1.006,
            "frame_raw": 1.20,
        }
        for segment in range(3):
            for candidate, value in values.items():
                row = {"segment_index": segment, "candidate": candidate}
                for metric in (
                    "map_voxels_per_1000_points",
                    "ground_z_iqr_median_m",
                    "planar_residual_median_m",
                    "overlap_nearest_median_m",
                ):
                    row[metric] = value
                rows.append(row)
        summary, usable = aggregate_candidate_scores(rows)
        result = decide(summary, 0.01)
        self.assertEqual(len(usable), 12)
        self.assertEqual(result["deskew_decision"], "POINTWISE_SUPPORTED")
        self.assertEqual(result["best_point_candidate"], "point_minus_lag")
        self.assertEqual(result["lag_decision"], "INDISTINGUISHABLE")


if __name__ == "__main__":
    unittest.main()
