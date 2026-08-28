import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class DummyI2C:
    I2C1 = 1
    STANDARD_MODE = 0

    def __init__(self, *_args):
        pass


machine = types.ModuleType("machine")
machine.I2C = DummyI2C
sys.modules.setdefault("machine", machine)
utime = types.ModuleType("utime")
utime.sleep_ms = lambda _value: None
utime.sleep = lambda _value: None
sys.modules.setdefault("utime", utime)

from collector_sensors import SGM58031  # noqa: E402


class SensorTests(unittest.TestCase):
    def test_sgm58031_channel_scaling_matches_existing_hardware(self):
        sensor = SGM58031.__new__(SGM58031)
        sensor.battery_selected = True
        sensor._read = lambda _register: 16384
        selected = []
        sensor._select_channel = lambda battery: selected.append(battery)

        battery, power = sensor.measure()
        self.assertAlmostEqual(battery, 22.528, places=3)
        self.assertIsNone(power)
        self.assertEqual(selected, [False])

        battery, power = sensor.measure()
        self.assertIsNone(battery)
        self.assertAlmostEqual(power, 43.008, places=3)
        self.assertEqual(selected, [False, True])


if __name__ == "__main__":
    unittest.main()
