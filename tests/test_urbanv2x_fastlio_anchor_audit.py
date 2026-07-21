import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from urbanv2x_fastlio_anchor_audit import (  # noqa: E402
    build_window_assignments,
    ordered_scan_files,
    transform_points,
)
from urbanv2x_fastlio_gt_eval import rigid_alignment  # noqa: E402


def _source_row(start, end, decision="PASS"):
    return {
        "requested_start_utc": float(start),
        "requested_end_utc": float(end),
        "source_decision": decision,
        "source_reason": "test",
        "source_ate_p95_m": 0.01,
        "source_rpe_translation_p95_m": 0.02,
        "source_rpe_rotation_p95_deg": 0.1,
    }


class FileSequenceTest(unittest.TestCase):
    def test_requires_contiguous_one_based_sequence(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            for number in (3, 1, 2):
                open(os.path.join(directory, "scans_%d.pcd" % number), "wb").close()

            paths = ordered_scan_files(directory)
            self.assertEqual(
                [os.path.basename(path) for path in paths],
                ["scans_1.pcd", "scans_2.pcd", "scans_3.pcd"],
            )

            os.unlink(os.path.join(directory, "scans_2.pcd"))
            with self.assertRaisesRegex(ValueError, "contiguous"):
                ordered_scan_files(directory)


class WindowAssignmentTest(unittest.TestCase):
    def test_covers_fixed_windows_and_short_tail_once(self):
        times = np.arange(0.0, 12.1, 1.0)
        rows = [_source_row(0.0, 5.0), _source_row(5.0, 10.0, "REVIEW")]

        assignments = build_window_assignments(times, rows)

        self.assertEqual(len(assignments), 3)
        self.assertEqual(
            [(row["start_index"], row["end_index"]) for row in assignments],
            [(0, 5), (5, 10), (10, 13)],
        )
        self.assertTrue(assignments[-1]["is_tail"])
        self.assertEqual(assignments[-1]["source_decision"], "TAIL_REVIEW")

        covered = []
        for row in assignments:
            covered.extend(range(row["start_index"], row["end_index"]))
        self.assertEqual(covered, list(range(len(times))))

    def test_rejects_gaps(self):
        times = np.arange(0.0, 12.1, 1.0)
        rows = [_source_row(0.0, 5.0), _source_row(6.0, 10.0)]

        with self.assertRaisesRegex(ValueError, "missing=1"):
            build_window_assignments(times, rows)


class RigidAnchorTest(unittest.TestCase):
    def test_recovers_target_without_scale_change(self):
        source = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [1.0, 1.0, 3.0],
            ]
        )
        angle = np.deg2rad(32.0)
        rotation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        translation = np.array([12.0, -4.0, 2.5])
        target = transform_points(source, rotation, translation)

        estimated_rotation, estimated_translation, _ = rigid_alignment(source, target)
        anchored = transform_points(source, estimated_rotation, estimated_translation)

        np.testing.assert_allclose(anchored, target, atol=1e-12)
        np.testing.assert_allclose(
            np.linalg.norm(anchored[1:] - anchored[0], axis=1),
            np.linalg.norm(source[1:] - source[0], axis=1),
            atol=1e-12,
        )
        self.assertAlmostEqual(np.linalg.det(estimated_rotation), 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
