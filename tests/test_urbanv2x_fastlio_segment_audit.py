import importlib.util
import math
import pathlib
import unittest

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_fastlio_segment_audit.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_fastlio_segment_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def rotation_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])


THRESHOLDS = {
    "ate_pass": 0.5,
    "translation_pass": 0.2,
    "rotation_pass": 1.0,
    "ate_review": 1.0,
    "translation_review": 0.5,
    "rotation_review": 2.0,
}


class WindowTest(unittest.TestCase):
    def test_exact_rigidly_transformed_trajectory_passes_every_window(self):
        times = np.arange(0.0, 60.1, 0.1)
        gt_positions = np.column_stack([
            times,
            4.0 * np.sin(times / 8.0),
            0.2 * np.sin(times / 5.0),
        ])
        gt_rotations = np.asarray([rotation_z(0.01 * time) for time in times])
        world_rotation = rotation_z(math.radians(31.0))
        translation = np.array([20.0, -8.0, 3.0])
        body_offset = rotation_z(math.pi / 2.0)
        odom_positions = (
            (world_rotation.T @ (gt_positions - translation).T).T
        )
        odom_rotations = np.asarray([
            world_rotation.T @ rotation @ body_offset.T
            for rotation in gt_rotations
        ])

        rows = MODULE.build_windows(
            times, odom_positions, odom_rotations,
            gt_positions, gt_rotations,
            durations=[10.0, 20.0, 30.0], step=10.0,
            rpe_horizon=1.0, thresholds=THRESHOLDS, minimum_poses=30,
        )
        summaries, recommended = MODULE.summarize_windows(
            rows, [10.0, 20.0, 30.0]
        )

        self.assertTrue(rows)
        self.assertTrue(all(row["decision"] == "PASS" for row in rows))
        self.assertEqual(recommended, 30.0)
        self.assertTrue(all(row["pass_ratio"] == 1.0 for row in summaries))
        self.assertLess(max(row["ate_p95_m"] for row in rows), 1e-10)

    def test_summary_does_not_recommend_failure_heavy_window(self):
        rows = []
        for duration, decisions in (
            (10.0, ["PASS"] * 9 + ["REVIEW"]),
            (20.0, ["PASS"] * 8 + ["REVIEW"] * 2),
            (30.0, ["PASS"] * 7 + ["FAIL"] * 3),
        ):
            for decision in decisions:
                rows.append({
                    "window_s": duration,
                    "decision": decision,
                    "ate_p95_m": 0.2,
                    "rpe_translation_p95_m": 0.1,
                    "rpe_rotation_p95_deg": 0.5,
                })
        _, recommended = MODULE.summarize_windows(rows, [10.0, 20.0, 30.0])
        self.assertEqual(recommended, 20.0)


if __name__ == "__main__":
    unittest.main()
