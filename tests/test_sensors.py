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

from collector_config import (  # noqa: E402
    CELL_LOCATOR_HOST,
    CELL_LOCATOR_LOCATION_TYPE,
    CELL_LOCATOR_PORT,
    CELL_LOCATOR_TIMEOUT,
    CELL_LOCATOR_TOKEN,
)
from collector_sensors import SGM58031, SensorService  # noqa: E402


class FakeLogger:
    def warn(self, *_args, **_kwargs):
        return True


class FakeCloud:
    def __init__(self):
        self.connected = True
        self.published = []

    def publish_properties(self, values, qos=0):
        self.published.append((values, qos))
        return True


class SensorTests(unittest.TestCase):
    def test_cell_locator_uses_original_application_parameters(self):
        self.assertEqual(CELL_LOCATOR_HOST, "www.queclocator.com")
        self.assertEqual(CELL_LOCATOR_PORT, 80)
        self.assertEqual(CELL_LOCATOR_TOKEN, "qa6qTK91597826z6")
        self.assertEqual(CELL_LOCATOR_LOCATION_TYPE, 8)
        self.assertEqual(CELL_LOCATOR_TIMEOUT, 1)

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

    def test_startup_cycle_reports_sensors_voltages_and_sim_with_qos_one(self):
        cloud = FakeCloud()
        service = SensorService(None, cloud, FakeLogger())
        service.aht10 = types.SimpleNamespace(measure=lambda: (29.73, 35.2))
        readings = iter(((11.796, None), (None, 0.071)))
        service.adc = types.SimpleNamespace(measure=lambda: next(readings))

        fake_sim = types.ModuleType("sim")
        fake_sim.getImsi = lambda: "460068072791011"
        fake_sim.getIccid = lambda: "89860624720056240112"
        old_sim = sys.modules.get("sim")
        sys.modules["sim"] = fake_sim
        try:
            self.assertTrue(service._publish_cycle(include_sim=True))
        finally:
            sys.modules.pop("sim", None)
            if old_sim is not None:
                sys.modules["sim"] = old_sim

        self.assertEqual(len(cloud.published), 1)
        values, qos = cloud.published[0]
        self.assertEqual(qos, 1)
        self.assertEqual(
            values,
            {
                "Temperature": 29.73,
                "Humidity": 35.2,
                "BatteryVoltage": 11.796,
                "PowerVoltage": 0.071,
                "product_information:IMSI": "460068072791011",
                "product_information:ICCID": "89860624720056240112",
            },
        )

    def test_ota_pause_suppresses_sensor_collection_and_publish(self):
        cloud = FakeCloud()
        service = SensorService(None, cloud, FakeLogger())
        service.aht10 = types.SimpleNamespace(
            measure=lambda: self.fail("sensor must remain paused")
        )
        self.assertTrue(service.pause_for_ota())
        self.assertFalse(service._publish_cycle(include_sim=True))
        self.assertEqual(cloud.published, [])
        self.assertTrue(service.resume_after_ota())


if __name__ == "__main__":
    unittest.main()
