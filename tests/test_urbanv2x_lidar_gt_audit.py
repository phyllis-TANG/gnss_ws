import importlib.util
import math
import pathlib
import struct
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_lidar_gt_audit.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_lidar_gt_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


CALIBRATION = """Initialization result:
Time Lag IMU to LiDAR (second)     = 0.100000
Homogeneous Transformation Matrix from LiDAR to IMU:
 1 0 0 1
 0 1 0 2
 0 0 1 3
 0 0 0 1

Refinement result:
Time Lag IMU to LiDAR (second)     = -0.012587
Homogeneous Transformation Matrix from LiDAR to IMU:
 0.999394 -0.034800  0.000902 -0.001595
 0.034802  0.999391 -0.002629 -0.022252
-0.000810  0.002659  0.999996  0.157731
 0.000000  0.000000  0.000000  1.000000
"""


class CalibrationTest(unittest.TestCase):
    def test_refined_transform_is_selected_and_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "calibration.txt"
            path.write_text(CALIBRATION, encoding="utf-8")
            parsed = MODULE.parse_calibration(path)
        self.assertEqual(parsed["source_section"], "refinement")
        self.assertEqual(parsed["direction"], "lidar_to_imu")
        self.assertAlmostEqual(parsed["time_lag_imu_to_lidar_s"], -0.012587)
        self.assertAlmostEqual(parsed["matrix"][2][3], 0.157731)
        self.assertAlmostEqual(parsed["metrics"]["determinant"], 1.0, places=5)
        self.assertLess(parsed["metrics"]["orthogonality_max_error"], 2e-6)


class TimeTest(unittest.TestCase):
    def test_nearest_time_and_angle_wrap(self):
        index, residual = MODULE.nearest_time([1.0, 2.0, 3.0], 2.2)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(residual, -0.2)
        self.assertAlmostEqual(
            MODULE.signed_angle_difference_degrees(1.0, 359.0), 2.0
        )


class MockField:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class MockCloud:
    width = 4
    height = 1
    point_step = 4
    row_step = 16
    is_bigendian = False
    fields = [MockField("time", 0)]
    data = struct.pack("<ffff", 0.0, 0.03, 0.07, 0.1)


class PointFieldTest(unittest.TestCase):
    def test_point_time_binary_sampling(self):
        values = MODULE.sample_point_field(MockCloud(), "time", 4)
        self.assertEqual(len(values), 4)
        self.assertAlmostEqual(min(values), 0.0)
        self.assertAlmostEqual(max(values), 0.1, places=6)
        self.assertEqual(MODULE.sample_point_field(MockCloud(), "missing", 4), [])


class GroundTruthTest(unittest.TestCase):
    def test_gt_parser_uses_documented_columns(self):
        numeric = (
            "1751963506.000 2374 203524 22.0 114.0 5.0 "
            "-2419319 5379741 2417969 1 2 3 4 5 6 0 0 0 0 0 0 "
            "0.3 1.0 155.0 6 0 0 0\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "gt.txt"
            path.write_text("header\nunits\n" + numeric, encoding="ascii")
            samples = MODULE.read_ground_truth(path)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["velocity_enu"], (4.0, 5.0, 6.0))
        self.assertEqual(samples[0]["quality"], 6)
        self.assertTrue(math.isclose(samples[0]["heading"], 155.0))


if __name__ == "__main__":
    unittest.main()
