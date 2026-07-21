import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from urbanv2x_epoch_tile_filter import (  # noqa: E402
    add_continuous_boundaries,
    aggregate_epochs,
    append_tile_fields,
    find_tile,
)


def tile(index, start, end, decision="PASS", tail=False):
    return {
        "window_index": index,
        "start_elapsed_s": start,
        "end_elapsed_s": end,
        "anchor_decision": decision,
        "source_decision": decision,
        "geometry_decision": "PASS",
        "is_tail": tail,
        "first_pcd": "scans_%d.pcd" % (index * 10 + 1),
        "last_pcd": "scans_%d.pcd" % ((index + 1) * 10),
    }


def signal(epoch=100.0, geometry="ok", west="40", east=""):
    return {
        "epoch_utc": str(epoch),
        "sat_id": "G01",
        "sys": "G",
        "signal": "S1C",
        "geometry_status": geometry,
        "west_cn0_dbhz": west,
        "east_cn0_dbhz": east,
    }


class BoundaryTest(unittest.TestCase):
    def test_midpoint_boundaries_are_continuous(self):
        windows = add_continuous_boundaries([
            tile(0, 0.0, 4.9), tile(1, 5.0, 9.9), tile(2, 10.0, 14.9)
        ])
        self.assertAlmostEqual(windows[0]["right_boundary_elapsed_s"], 4.95)
        self.assertAlmostEqual(windows[1]["left_boundary_elapsed_s"], 4.95)
        self.assertEqual(find_tile(windows, 4.94)["window_index"], 0)
        self.assertEqual(find_tile(windows, 4.95)["window_index"], 1)


class ClassificationTest(unittest.TestCase):
    def setUp(self):
        self.windows = add_continuous_boundaries([
            tile(0, 0.0, 4.9),
            tile(1, 5.0, 9.9, "REVIEW"),
            tile(2, 10.0, 14.9, "FAIL"),
            tile(3, 15.0, 19.0, "REVIEW", tail=True),
        ])

    def classify(self, elapsed, index, **updates):
        row = signal()
        row.update(updates)
        return append_tile_fields(row, self.windows[index], elapsed, 0.5)

    def test_primary_review_and_exclusion_rules(self):
        primary = self.classify(2.0, 0)
        review = self.classify(7.0, 1)
        failed = self.classify(12.0, 2)
        tail = self.classify(17.0, 3)
        boundary = self.classify(5.1, 1)
        no_reference = self.classify(2.0, 0, west_cn0_dbhz="")
        bad_geometry = self.classify(2.0, 0, geometry_status="no_gt")
        self.assertEqual(primary["analysis_group"], "PRIMARY")
        self.assertEqual(review["analysis_group"], "REVIEW_SENSITIVITY")
        self.assertEqual(failed["analysis_group"], "EXCLUDED")
        self.assertEqual(tail["analysis_group"], "EXCLUDED")
        self.assertEqual(boundary["analysis_group"], "EXCLUDED")
        self.assertEqual(no_reference["analysis_group"], "EXCLUDED")
        self.assertEqual(bad_geometry["analysis_group"], "EXCLUDED")


class EpochAggregationTest(unittest.TestCase):
    def test_epoch_uses_best_available_signal_group(self):
        rows = [
            {
                **signal(100.0), "analysis_group": "EXCLUDED",
                "eligibility_reasons": "no_reference_cn0",
                "odom_elapsed_s": 1.0, "tile_window_index": 0,
                "tile_anchor_decision": "PASS", "tile_boundary_distance_s": 1.0,
                "west_reference_available": 0, "east_reference_available": 0,
                "reference_availability": "NONE",
            },
            {
                **signal(100.0), "analysis_group": "PRIMARY",
                "eligibility_reasons": "eligible",
                "odom_elapsed_s": 1.0, "tile_window_index": 0,
                "tile_anchor_decision": "PASS", "tile_boundary_distance_s": 1.0,
                "west_reference_available": 1, "east_reference_available": 0,
                "reference_availability": "WEST",
            },
        ]
        epochs = aggregate_epochs(rows)
        self.assertEqual(len(epochs), 1)
        self.assertEqual(epochs[0]["epoch_analysis_group"], "PRIMARY")
        self.assertEqual(epochs[0]["signals_primary"], 1)
        self.assertEqual(epochs[0]["signals_excluded"], 1)


if __name__ == "__main__":
    unittest.main()
