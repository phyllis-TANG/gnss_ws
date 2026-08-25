import importlib.util
import math
import pathlib
import unittest

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_fastlio_gt_eval.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_fastlio_gt_eval", SCRIPT)
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


class AlignmentTest(unittest.TestCase):
    def test_rigid_alignment_recovers_rotation_and_translation_without_scale(self):
        source = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [1.0, 1.0, 1.0],
            [-2.0, 0.5, 0.2],
        ])
        expected_rotation = rotation_z(math.radians(37.0))
        expected_translation = np.array([10.0, -4.0, 2.5])
        target = (expected_rotation @ source.T).T + expected_translation

        rotation, translation, singular_values = MODULE.rigid_alignment(source, target)
        aligned = (rotation @ source.T).T + translation

        np.testing.assert_allclose(rotation, expected_rotation, atol=1e-12)
        np.testing.assert_allclose(translation, expected_translation, atol=1e-12)
        np.testing.assert_allclose(aligned, target, atol=1e-12)
        self.assertEqual(len(singular_values), 3)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.0)

    def test_rpe_is_zero_after_consistent_global_transform(self):
        times = np.arange(0.0, 6.0, 1.0)
        source_positions = np.column_stack([
            times,
            0.1 * times ** 2,
            np.zeros_like(times),
        ])
        source_rotations = np.asarray([
            rotation_z(0.05 * epoch) for epoch in times
        ])
        global_rotation = rotation_z(math.radians(-22.0))
        translation = np.array([3.0, 7.0, -1.0])
        target_positions = (
            (global_rotation @ source_positions.T).T + translation
        )
        target_rotations = np.asarray([
            global_rotation @ rotation for rotation in source_rotations
        ])

        result = MODULE.compute_rpe(
            times,
            target_positions,
            target_rotations,
            target_positions,
            target_rotations,
            horizon=1.0,
            tolerance=0.01,
        )
        self.assertEqual(result["pairs"], 5)
        self.assertLess(result["translation_error_m"]["max"], 1e-12)
        self.assertLess(result["rotation_error_deg"]["max"], 1e-9)

    def test_world_translation_rpe_does_not_confuse_constant_body_axis_offset(self):
        times = np.arange(0.0, 4.0, 1.0)
        positions = np.column_stack([times, np.zeros_like(times), np.zeros_like(times)])
        estimated_rotations = np.asarray([np.eye(3) for _ in times])
        body_axis_offset = rotation_z(math.pi / 2.0)
        gt_rotations = np.asarray([body_axis_offset for _ in times])

        result = MODULE.compute_rpe(
            times, positions, estimated_rotations,
            positions, gt_rotations,
            horizon=1.0, tolerance=0.01,
        )

        self.assertLess(result["translation_error_m"]["max"], 1e-12)
        self.assertGreater(result["body_frame_translation_error_m"]["median"], 1.0)

    def test_constant_body_axis_rotation_is_recovered(self):
        base = np.asarray([
            rotation_z(0.1 * index) for index in range(6)
        ])
        expected = rotation_z(math.radians(90.0))
        target = np.asarray([rotation @ expected for rotation in base])
        estimated = MODULE.estimate_body_axis_rotation(base, target)
        np.testing.assert_allclose(estimated, expected, atol=1e-12)


class ConventionTest(unittest.TestCase):
    def test_navigation_heading_rotates_flu_forward_into_enu(self):
        forward = np.array([1.0, 0.0, 0.0])
        north_heading = MODULE.navigation_rotation(0.0, 0.0, 0.0)
        east_heading = MODULE.navigation_rotation(0.0, 0.0, math.pi / 2.0)
        np.testing.assert_allclose(north_heading @ forward, [0.0, 1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(east_heading @ forward, [1.0, 0.0, 0.0], atol=1e-12)

    def test_quaternion_rotation_is_normalized(self):
        rotation = MODULE.quaternion_to_rotation(0.0, 0.0, 2.0, 2.0)
        expected = rotation_z(math.pi / 2.0)
        np.testing.assert_allclose(rotation, expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
