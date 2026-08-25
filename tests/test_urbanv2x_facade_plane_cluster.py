import os
import sys
import unittest

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from urbanv2x_facade_plane_cluster import (  # noqa: E402
    can_join_patches,
    can_match_planes,
    interval_gap,
    is_true,
    unsigned_normal_angle,
    weighted_median,
)


class Arguments:
    within_cell_gap = 1
    within_normal_angle = 10.0
    within_plane_offset = 0.20
    cross_centroid_max = 30.0
    cross_normal_angle = 10.0
    cross_plane_offset = 0.30
    cross_horizontal_gap = 2.5
    cross_vertical_gap = 2.5


def patch(patch_id, tile, cell, centre, normal=(1, 0, 0)):
    return {
        "patch_id": patch_id,
        "tile_window_index": str(tile),
        "cell_x": str(cell[0]), "cell_y": str(cell[1]), "cell_z": str(cell[2]),
        "centroid_e_m": str(centre[0]), "centroid_n_m": str(centre[1]),
        "centroid_u_m": str(centre[2]),
        "normal_e": str(normal[0]), "normal_n": str(normal[1]),
        "normal_u": str(normal[2]),
    }


def plane(plane_id, tile, centre, normal=(1, 0, 0), width=4, height=3):
    return {
        "plane_observation_id": plane_id,
        "tile_window_index": str(tile),
        "centroid_e_m": str(centre[0]), "centroid_n_m": str(centre[1]),
        "centroid_u_m": str(centre[2]),
        "normal_e": str(normal[0]), "normal_n": str(normal[1]),
        "normal_u": str(normal[2]),
        "horizontal_span_m": str(width), "vertical_span_m": str(height),
        "vertical_min_m": str(centre[2] - height / 2),
        "vertical_max_m": str(centre[2] + height / 2),
    }


class FacadePlaneClusterTests(unittest.TestCase):
    def test_boolean_fields_work_before_and_after_csv_roundtrip(self):
        self.assertTrue(is_true({"flag": 1}, "flag"))
        self.assertTrue(is_true({"flag": "1"}, "flag"))
        self.assertFalse(is_true({"flag": 0}, "flag"))

    def test_interval_gap(self):
        self.assertEqual(interval_gap(0, 2, 1, 3), 0.0)
        self.assertEqual(interval_gap(0, 1, 2.5, 4), 1.5)

    def test_unsigned_normal_is_sign_invariant(self):
        self.assertAlmostEqual(unsigned_normal_angle([1, 0, 0], [-1, 0, 0]), 0.0)

    def test_weighted_median(self):
        self.assertEqual(weighted_median([1, 10, 20], [1, 5, 1]), 10.0)

    def test_adjacent_coplanar_patches_join(self):
        left = patch("a", 1, (0, 0, 0), (4.0, 0.0, 1.0))
        right = patch("b", 1, (0, 1, 0), (4.05, 2.0, 1.0))
        self.assertTrue(can_join_patches(left, right, Arguments()))
        corner = patch("c", 1, (0, 1, 0), (4.0, 2.0, 1.0), (0, 1, 0))
        self.assertFalse(can_join_patches(left, corner, Arguments()))

    def test_cross_tile_overlap_matches(self):
        left = plane("p1", 1, (4.0, 0.0, 2.0))
        right = plane("p2", 2, (4.1, 3.0, 2.2))
        self.assertTrue(can_match_planes(left, right, Arguments()))
        parallel_far = plane("p3", 2, (5.0, 3.0, 2.2))
        self.assertFalse(can_match_planes(left, parallel_far, Arguments()))
        same_tile = plane("p4", 1, (4.1, 3.0, 2.2))
        self.assertFalse(can_match_planes(left, same_tile, Arguments()))


if __name__ == "__main__":
    unittest.main()
