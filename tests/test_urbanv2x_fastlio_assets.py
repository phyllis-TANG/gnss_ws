import ast
import pathlib
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).parents[1]
CONFIG = ROOT / "config" / "urbanv2x_fastlio_single_corrected.yaml"
LAUNCH = ROOT / "launch" / "urbanv2x_fastlio_single_short.launch"


def yaml_inline_list(text, key):
    match = re.search(
        r"^\s*%s:\s*(\[[^\]]+\])" % re.escape(key),
        text,
        flags=re.MULTILINE,
    )
    if not match:
        raise AssertionError("Missing YAML key %s" % key)
    return ast.literal_eval(match.group(1).replace("\n", " "))


class FastlioConfigTest(unittest.TestCase):
    def test_config_uses_refined_lidar_to_imu_transform_directly(self):
        text = CONFIG.read_text(encoding="utf-8")
        translation = yaml_inline_list(text, "extrinsic_T")
        rotation = yaml_inline_list(text, "extrinsic_R")
        self.assertEqual(translation, [-0.001595, -0.022252, 0.157731])
        self.assertEqual(
            rotation,
            [
                0.999394, -0.034800, 0.000902,
                0.034802, 0.999391, -0.002629,
                -0.000810, 0.002659, 0.999996,
            ],
        )
        self.assertIn("multi_lidar: false", text)
        self.assertIn('lid_topic: "/velodyne_points"', text)
        self.assertIn('imu_topic: "/imu/data"', text)
        self.assertIn("timestamp_unit: 0", text)
        self.assertIn("pcd_save_en: true", text)
        self.assertIn("interval: 50", text)


class FastlioLaunchTest(unittest.TestCase):
    def test_launch_is_bounded_and_stops_on_playback_exit(self):
        root = ET.parse(LAUNCH).getroot()
        arguments = {item.attrib.get("name"): item for item in root.findall("arg")}
        self.assertEqual(arguments["pcd_interval"].attrib.get("default"), "50")
        parameters = {item.attrib.get("name"): item for item in root.findall("param")}
        interval = parameters["/pcd_save/interval"]
        self.assertEqual(interval.attrib.get("value"), "$(arg pcd_interval)")
        self.assertEqual(interval.attrib.get("type"), "int")
        nodes = {node.attrib.get("name"): node for node in root.findall("node")}
        player = nodes["play_urbanv2x_short"]
        self.assertEqual(player.attrib.get("required"), "true")
        self.assertIn("--duration $(arg duration)", player.attrib["args"])
        self.assertIn("/velodyne_points /imu/data", player.attrib["args"])
        mapper = nodes["laserMapping_multi"]
        self.assertEqual(mapper.attrib["pkg"], "fast_lio_multi")
        self.assertEqual(mapper.attrib["type"], "fast_lio_multi_node")
        recorder = nodes["record_fastlio_outputs"]
        self.assertIn("/Odometry /path", recorder.attrib["args"])


if __name__ == "__main__":
    unittest.main()
