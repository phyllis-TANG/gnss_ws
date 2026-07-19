import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gnss = load_module("gnss_broadcast")


def header_line(content, label):
    return "%-60s%-20s\n" % (content, label)


def nav_first_line(sat_id, values):
    prefix = "%s 2025 07 08 08 00 00" % sat_id
    assert len(prefix) == 23
    return prefix + "".join("%19.12E" % value for value in values) + "\n"


def nav_continuation(values):
    return "    " + "".join("%19.12E" % value for value in values) + "\n"


class CoordinateTest(unittest.TestCase):
    def test_geodetic_ecef_round_trip(self):
        expected = (22.4248517973, 114.2138893890, 4.816)
        ecef = gnss.geodetic_to_ecef(*expected)
        actual = gnss.ecef_to_geodetic(ecef)
        self.assertAlmostEqual(actual[0], expected[0], places=9)
        self.assertAlmostEqual(actual[1], expected[1], places=9)
        self.assertAlmostEqual(actual[2], expected[2], places=4)

    def test_local_azimuth_and_elevation(self):
        receiver = gnss.geodetic_to_ecef(22.0, 114.0, 0.0)
        latitude = math.radians(22.0)
        longitude = math.radians(114.0)
        east = (-math.sin(longitude), math.cos(longitude), 0.0)
        up = (
            math.cos(latitude) * math.cos(longitude),
            math.cos(latitude) * math.sin(longitude),
            math.sin(latitude),
        )
        satellite = tuple(
            receiver[i] + 2.0e7 * (east[i] + up[i]) / math.sqrt(2.0)
            for i in range(3)
        )
        elevation, azimuth = gnss.elevation_azimuth(receiver, satellite)
        self.assertAlmostEqual(elevation, 45.0, places=8)
        self.assertAlmostEqual(azimuth, 90.0, places=8)


class RinexNavigationTest(unittest.TestCase):
    def test_parse_and_propagate_gps_and_glonass(self):
        gps = [0.0] * 31
        gps[3] = 10
        gps[5] = 4.5e-9
        gps[6] = 0.5
        gps[8] = 0.01
        gps[10] = 5153.7955
        gps[11] = 201600.0
        gps[13] = 1.0
        gps[15] = 0.94
        gps[17] = 0.3
        gps[18] = -8.0e-9
        gps[21] = 2374
        gps[24] = 0
        gps[26] = 10

        glo = [0.0] * 15
        glo[3], glo[7], glo[11] = 19000.0, 12000.0, 13000.0
        glo[4], glo[8], glo[12] = -1.0, 2.0, 1.5
        glo[6] = 0
        glo[10] = -4

        lines = [
            header_line("     3.04           NAVIGATION DATA     M: Mixed", "RINEX VERSION / TYPE"),
            header_line("", "END OF HEADER"),
            nav_first_line("G01", gps[:3]),
        ]
        lines.extend(nav_continuation(gps[3 + 4 * i : 7 + 4 * i]) for i in range(7))
        lines.append(nav_first_line("R07", glo[:3]))
        lines.extend(nav_continuation(glo[3 + 4 * i : 7 + 4 * i]) for i in range(3))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.rnx"
            path.write_text("".join(lines), encoding="ascii")
            records, audit = gnss.read_rinex3_navigation(str(path))

        self.assertEqual(audit.raw_records, 2)
        self.assertEqual(set(records), {"G01", "R07"})
        self.assertEqual(records["G01"][0].iode, 10)
        self.assertEqual(records["R07"][0].frequency_channel, -4)

        gps_record = records["G01"][0]
        gps_position = gnss.satellite_position(gps_record, gps_record.toe_gpst + 30.0)
        self.assertTrue(all(math.isfinite(value) for value in gps_position))
        self.assertTrue(2.0e7 < gnss.norm3(gps_position) < 3.0e7)

        glo_record = records["R07"][0]
        glo_position = gnss.satellite_position(glo_record, glo_record.toe_gpst + 60.0)
        self.assertTrue(all(math.isfinite(value) for value in glo_position))
        self.assertTrue(2.0e7 < gnss.norm3(glo_position) < 3.0e7)


class SelectionTest(unittest.TestCase):
    def test_stale_ephemeris_is_rejected(self):
        record = gnss.KeplerEphemeris(
            sat_id="G01",
            sys="G",
            prn=1,
            toc_gpst=1000.0,
            toe_gpst=1000.0,
            toes=1000.0,
            week=0,
            iode=1,
            iodc=1,
            code=0,
            health=0,
            f0=0.0,
            f1=0.0,
            f2=0.0,
            sqrt_a=5153.7955,
            eccentricity=0.01,
            i0=0.94,
            omega0=1.0,
            omega=0.3,
            m0=0.5,
            delta_n=0.0,
            omega_dot=0.0,
            idot=0.0,
            crc=0.0,
            crs=0.0,
            cuc=0.0,
            cus=0.0,
            cic=0.0,
            cis=0.0,
        )
        selected, age, limit = gnss.select_ephemeris([record], 9000.0)
        self.assertIsNone(selected)
        self.assertEqual(age, 8000.0)
        self.assertEqual(limit, 7200.0)


if __name__ == "__main__":
    unittest.main()
