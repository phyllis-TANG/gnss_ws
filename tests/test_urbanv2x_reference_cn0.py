import datetime as dt
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "urbanv2x_reference_cn0.py"
SPEC = importlib.util.spec_from_file_location("urbanv2x_reference_cn0", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def header_line(content, label):
    return "%-60s%-20s\n" % (content, label)


def observation_line(sat_id, values):
    fields = []
    for value in values:
        fields.append("%14.3f  " % value if value is not None else " " * 16)
    return sat_id + "".join(fields) + "\n"


class SignalMappingTest(unittest.TestCase):
    def test_satellite_numbering(self):
        self.assertEqual(MODULE.satellite_id(5), "G05")
        self.assertEqual(MODULE.satellite_id(39), "R07")
        self.assertEqual(MODULE.satellite_id(63), "E04")
        self.assertEqual(MODULE.satellite_id(103), "C06")

    def test_code_first_mapping_separates_gps_l2_and_galileo_e5b(self):
        self.assertEqual(MODULE.SIGNAL_BY_SYSTEM_CODE[("G", 17)], "S2X")
        self.assertEqual(MODULE.SIGNAL_BY_SYSTEM_CODE[("E", 28)], "S7X")
        self.assertEqual(
            MODULE.PHYSICAL_FREQUENCY_MHZ[("E", "S7X")], 1207.140
        )


class RinexParsingTest(unittest.TestCase):
    def test_parse_signal_strength_and_nearest_match(self):
        gps_types = "C1C L1C D1C S1C C2X L2X D2X S2X"
        gal_types = "C1X L1X D1X S1X C7X L7X D7X S7X"
        content = [
            header_line("     3.04           OBSERVATION DATA    M: Mixed", "RINEX VERSION / TYPE"),
            header_line("G    8 " + gps_types, "SYS / # / OBS TYPES"),
            header_line("E    8 " + gal_types, "SYS / # / OBS TYPES"),
            header_line("  2025    07    08    08    33   46.0050000     GPS", "TIME OF FIRST OBS"),
            header_line("", "END OF HEADER"),
            "> 2025 07 08 08 33 46.0050000  0  2\n",
            observation_line(
                "G01", [23000000, 1, 0, 40, 23000010, 1, 0, 34]
            ),
            observation_line(
                "E04", [24000000, 1, 0, 36, 24000010, 1, 0, 35]
            ),
        ]

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.obs"
            path.write_text("".join(content), encoding="ascii")
            records, metadata = MODULE.parse_rinex_cn0(str(path), 18.0)

        self.assertEqual(metadata["time_system"], "GPS")
        self.assertEqual(metadata["records"], 4)
        by_key = {
            (record["sat_id"], record["signal"]): record for record in records
        }
        self.assertEqual(by_key[("G01", "S1C")]["cn0"], 40.0)
        self.assertEqual(by_key[("E04", "S7X")]["cn0"], 35.0)

        expected = (
            dt.datetime(2025, 7, 8, 8, 33, 46, 5000, tzinfo=dt.timezone.utc)
            .timestamp()
            - 18.0
        )
        self.assertAlmostEqual(records[0]["epoch_utc"], expected, places=6)

        index = MODULE.ReferenceIndex(records)
        cn0, delta = index.nearest(expected + 0.002, "E04", "S7X", 0.2)
        self.assertEqual(cn0, 35.0)
        self.assertAlmostEqual(delta, -0.002, places=6)


class DifferenceTest(unittest.TestCase):
    def test_raw_and_channel_centered_difference(self):
        rows = [
            {
                "sys": "E",
                "signal": "S7X",
                "epoch_utc": 10.0,
                "sat_id": "E04",
                "vehicle_cn0_dbhz": 30.0,
            },
            {
                "sys": "E",
                "signal": "S7X",
                "epoch_utc": 11.0,
                "sat_id": "E04",
                "vehicle_cn0_dbhz": 25.0,
            },
        ]
        reference = MODULE.ReferenceIndex(
            [
                {"epoch_utc": 10.0, "sat_id": "E04", "signal": "S7X", "cn0": 35.0},
                {"epoch_utc": 11.0, "sat_id": "E04", "signal": "S7X", "cn0": 35.0},
            ]
        )
        MODULE.add_reference_matches(rows, "west", reference, 0.2)
        medians = MODULE.add_channel_centered_differences(rows, "west")

        self.assertEqual(rows[0]["west_drop_raw_db"], 5.0)
        self.assertEqual(rows[1]["west_drop_raw_db"], 10.0)
        self.assertEqual(medians[("E", "S7X")], 7.5)
        self.assertEqual(rows[0]["west_drop_channel_centered_db"], -2.5)
        self.assertEqual(rows[1]["west_drop_channel_centered_db"], 2.5)


if __name__ == "__main__":
    unittest.main()
