import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

if "utime" not in sys.modules:
    fake_utime = types.ModuleType("utime")
    fake_utime.time = time.time
    fake_utime.sleep = time.sleep
    fake_utime.sleep_ms = lambda value: time.sleep(float(value) / 1000.0)
    fake_utime.ticks_ms = lambda: int(time.monotonic() * 1000)
    fake_utime.ticks_diff = lambda now, then: now - then
    sys.modules["utime"] = fake_utime

saved_sensor_module = sys.modules.get("collector_sensors")
saved_uart_module = sys.modules.get("collector_uart")
sensor_stub = types.ModuleType("collector_sensors")
sensor_stub.SensorService = object
uart_stub = types.ModuleType("collector_uart")
uart_stub.UartPipeline = object
sys.modules["collector_sensors"] = sensor_stub
sys.modules["collector_uart"] = uart_stub
try:
    import collector_app as app_module  # noqa: E402
    from collector_app import CollectorApplication  # noqa: E402
finally:
    if saved_sensor_module is None:
        sys.modules.pop("collector_sensors", None)
    else:
        sys.modules["collector_sensors"] = saved_sensor_module
    if saved_uart_module is None:
        sys.modules.pop("collector_uart", None)
    else:
        sys.modules["collector_uart"] = saved_uart_module


class FakeLogger:
    def __init__(self):
        self.info_calls = []
        self.warn_calls = []

    def info(self, *args, **kwargs):
        self.info_calls.append((args, kwargs))
        return True

    def warn(self, *args, **kwargs):
        self.warn_calls.append((args, kwargs))
        return True


def sample_stats():
    return {
        "rx_sample_id": 1,
        "frame_sample_id": 1,
        "rx_sample_size": 206,
        "rx_sample": bytes(range(100)),
        "frame_sample_kind": "B0",
        "frame_sample_size": 206,
        "frame_sample": bytes(range(100)),
    }


class AppTests(unittest.TestCase):
    def make_app(self, enabled):
        app = CollectorApplication.__new__(CollectorApplication)
        app.cloud = types.SimpleNamespace(uart_sample_log_enabled=enabled)
        app.logger = FakeLogger()
        app.last_uart_rx_sample_id = 0
        app.last_uart_frame_sample_id = 0
        return app

    def test_disabled_uart_sampling_skips_hex_conversion_and_advances_ids(self):
        app = self.make_app(False)
        with patch.object(app_module, "hex_preview") as preview:
            self.assertFalse(app._log_uart_sample(sample_stats()))
        preview.assert_not_called()
        self.assertEqual(app.last_uart_rx_sample_id, 1)
        self.assertEqual(app.last_uart_frame_sample_id, 1)
        self.assertEqual(app.logger.info_calls, [])

    def test_enabled_uart_sampling_logs_bounded_preview(self):
        app = self.make_app(True)
        self.assertTrue(app._log_uart_sample(sample_stats()))
        self.assertEqual(len(app.logger.info_calls), 1)
        extra = app.logger.info_calls[0][1]["extra"]
        self.assertIn("chunk_len=206", extra)
        self.assertIn("frame_type=B0", extra)
        self.assertLess(len(extra), 400)

    def test_bridge_low_occupancy_gate_supports_legacy_delivery_store(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.cloud = types.SimpleNamespace(connected=True)
        app.quiet_gate = types.SimpleNamespace(quiet_since_ms=None)
        app.ram_queue = types.SimpleNamespace(capacity=60, depth=lambda: 10)
        app.journal = types.SimpleNamespace(capacity=20)
        app.delivery = types.SimpleNamespace(flash_depth=lambda: 10)
        self.assertTrue(app._ota_ready(True))
        app.ram_queue.depth = lambda: 11
        self.assertFalse(app._ota_ready(True))

    def test_bridge_finalizer_uses_double_check_without_new_queue_api(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.ram_queue = types.SimpleNamespace(depth=lambda: 0)
        app.delivery = types.SimpleNamespace(
            accepted=7,
            flash_depth=lambda: 0,
        )
        app.cloud = types.SimpleNamespace(stats=lambda: {"inflight": 0})
        with patch.object(app_module.utime, "sleep_ms", lambda _value: None):
            self.assertTrue(app._prepare_migration_reboot())


if __name__ == "__main__":
    unittest.main()
