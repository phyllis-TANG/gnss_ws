import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "urbanv2x_lio_preflight.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_lio_preflight", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TimingTest(unittest.TestCase):
    def test_nearest_residual_preserves_sign(self):
        self.assertAlmostEqual(MODULE.nearest_residual([1.0, 2.0], 1.2), -0.2)
        self.assertAlmostEqual(MODULE.nearest_residual([1.0, 2.0], 1.8), 0.2)


class PackageTest(unittest.TestCase):
    def test_lio_package_name_detection(self):
        self.assertTrue(MODULE.is_lio_candidate("fast_lio", "/tmp/FAST_LIO"))
        self.assertTrue(MODULE.is_lio_candidate("mapping", "/ws/src/point_lio"))
        self.assertFalse(MODULE.is_lio_candidate("rosbag", "/opt/ros/noetic"))


class DecisionTest(unittest.TestCase):
    def fixtures(self):
        lidar = {"nonpositive_intervals": 0}
        imu = {
            "nonpositive_intervals": 0,
            "nonfinite_messages": 0,
            "rate_hz": 400.0,
            "acceleration_norm_m_s2": {"median": 9.81},
            "quaternion_norm": {"median": 1.0},
        }
        timing = {
            "imu_covers_all_lidar_headers": True,
            "lidar_header_to_nearest_imu_abs_s": {"p95": 0.001},
        }
        dependencies = {"python_modules": {"scipy": True}}
        return lidar, imu, timing, dependencies

    def test_ready_inputs_without_package(self):
        lidar, imu, timing, dependencies = self.fixtures()
        decision, blockers, warnings = MODULE.decide(
            lidar, imu, timing, [], dependencies
        )
        self.assertEqual(decision, "INPUTS_READY_NO_LIO_PACKAGE")
        self.assertEqual(blockers, [])
        self.assertEqual(warnings, [])

    def test_package_with_launch_is_ready_for_config(self):
        lidar, imu, timing, dependencies = self.fixtures()
        packages = [{"launch_files": ["run.launch"]}]
        decision, _, _ = MODULE.decide(
            lidar, imu, timing, packages, dependencies
        )
        self.assertEqual(decision, "READY_FOR_CONFIG_AUDIT")


if __name__ == "__main__":
    unittest.main()
