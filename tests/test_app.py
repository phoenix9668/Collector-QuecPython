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
        self.error_calls = []
        self.runtime_calls = []

    def info(self, *args, **kwargs):
        self.info_calls.append((args, kwargs))
        return True

    def warn(self, *args, **kwargs):
        self.warn_calls.append((args, kwargs))
        return True

    def error(self, *args, **kwargs):
        self.error_calls.append((args, kwargs))
        return True

    def runtime(self, *args, **kwargs):
        self.runtime_calls.append((args, kwargs))
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

    def test_ota_ready_no_longer_depends_on_queue_occupancy(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.cloud = types.SimpleNamespace(connected=True)
        app.ram_queue = types.SimpleNamespace(capacity=60, depth=lambda: 10)
        app.journal = types.SimpleNamespace(capacity=20)
        app.delivery = types.SimpleNamespace(flash_depth=lambda: 10)
        self.assertTrue(app._ota_ready(True))
        app.ram_queue.depth = lambda: 11
        self.assertTrue(app._ota_ready(False))

    def test_ota_exclusive_mode_stops_business_and_discards_data(self):
        calls = []
        app = CollectorApplication.__new__(CollectorApplication)
        app.ota_exclusive = False
        app._ota_legacy_cloud_methods = None
        app._ota_legacy_accept = None
        app.migration = types.SimpleNamespace(stop=False)
        app.uart = types.SimpleNamespace(
            pause_for_ota=lambda: calls.append("uart_pause") or True,
            resume_after_ota=lambda: calls.append("uart_resume") or True,
        )
        app.sensors = types.SimpleNamespace(
            pause_for_ota=lambda: calls.append("sensor_pause") or True,
            resume_after_ota=lambda: calls.append("sensor_resume") or True,
        )
        app.cloud = types.SimpleNamespace(
            enter_ota_mode=lambda: calls.append("cloud_pause") or 2,
            exit_ota_mode=lambda: calls.append("cloud_resume") or True,
        )
        app.delivery = types.SimpleNamespace(
            discard_all=lambda: calls.append("discard") or 7,
            resume_accepting=lambda: calls.append("accept_resume") or True,
        )
        app.logger = FakeLogger()
        app.logger.paused = False
        self.assertTrue(app._enter_ota_mode({"version": "4.1.1"}))
        self.assertTrue(app.ota_exclusive)
        self.assertTrue(app.migration.stop)
        self.assertTrue(app.logger.paused)
        self.assertEqual(
            calls, ["cloud_pause", "discard", "uart_pause", "sensor_pause"]
        )
        self.assertTrue(app._exit_ota_mode())
        self.assertFalse(app.ota_exclusive)
        self.assertFalse(app.migration.stop)
        self.assertFalse(app.logger.paused)
        self.assertEqual(
            calls[-4:],
            ["accept_resume", "cloud_resume", "uart_resume", "sensor_resume"],
        )

    def test_migration_mode_keeps_logs_and_manager_active(self):
        calls = []
        app = CollectorApplication.__new__(CollectorApplication)
        app.ota_exclusive = False
        app._ota_legacy_cloud_methods = None
        app._ota_legacy_accept = None
        app.migration = types.SimpleNamespace(stop=False)
        app.uart = types.SimpleNamespace(
            pause_for_ota=lambda: calls.append("uart_pause") or True,
        )
        app.sensors = types.SimpleNamespace(
            pause_for_ota=lambda: calls.append("sensor_pause") or True,
        )
        app.cloud = types.SimpleNamespace(
            enter_ota_mode=lambda: calls.append("cloud_pause") or 0,
        )
        app.delivery = types.SimpleNamespace(
            discard_all=lambda: calls.append("discard") or 0,
        )
        app.logger = FakeLogger()
        app.logger.paused = False
        self.assertTrue(
            app._pause_business(
                "migration", stop_migration=False, pause_logs=False
            )
        )
        self.assertFalse(app.migration.stop)
        self.assertFalse(app.logger.paused)
        self.assertEqual(
            calls, ["cloud_pause", "discard", "uart_pause", "sensor_pause"]
        )

    def test_maintenance_keeps_external_watchdog_heartbeat_alive(self):
        writes = []
        app = CollectorApplication.__new__(CollectorApplication)
        app.cloud = types.SimpleNamespace(connected=False)
        app.uart = types.SimpleNamespace(write=lambda data: writes.append(data) or 9)
        app.logger = FakeLogger()
        app.ota_exclusive = True
        self.assertTrue(app._send_heartbeat())
        self.assertEqual(writes, [b"Heartbeat"])

        app.ota_exclusive = False
        self.assertFalse(app._send_heartbeat())
        self.assertEqual(writes, [b"Heartbeat"])

        app.cloud.connected = True
        self.assertTrue(app._send_heartbeat())
        self.assertEqual(writes, [b"Heartbeat", b"Heartbeat"])

    def test_pending_ota_reserve_hold_is_info_not_error(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = {"version": "4.1.1"}
        app.logger = FakeLogger()
        self.assertTrue(app._report_reserve_unavailable(startup=True))
        self.assertEqual(len(app.logger.info_calls), 1)
        self.assertEqual(
            app.logger.info_calls[0][0][1], "OTA_RESERVE_DEFERRED"
        )
        self.assertEqual(app.logger.error_calls, [])

    def test_non_ota_reserve_failure_remains_an_error(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = None
        app.logger = FakeLogger()
        self.assertTrue(app._report_reserve_unavailable())
        self.assertEqual(app.logger.info_calls, [])
        self.assertEqual(len(app.logger.error_calls), 1)
        self.assertEqual(app.logger.error_calls[0][0][1], "OTA_RESERVE")

    def test_migration_storage_hold_is_informational(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = None
        app.migration_mode = True
        app.logger = FakeLogger()
        self.assertTrue(app._report_reserve_unavailable(startup=True))
        self.assertEqual(app.logger.error_calls, [])
        self.assertEqual(
            app.logger.info_calls[0][0][1], "MIGRATION_STORAGE_DEFERRED"
        )

    def test_health_confirmation_restores_storage_before_clearing_hold(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = {"version": "4.1.1"}
        app.logger = FakeLogger()
        restored = []
        app._restore_runtime_storage = lambda: restored.append(True) or True
        with (
            patch.object(app_module.utime, "sleep", lambda _value: None),
            patch.object(app_module.OtaBootGuard, "mark_healthy", return_value=True),
        ):
            app._health_worker()
        self.assertEqual(restored, [True])
        self.assertIsNone(app.pending_ota)
        self.assertEqual(app.logger.error_calls, [])
        self.assertEqual(len(app.logger.runtime_calls), 1)

    def test_health_confirmation_reports_real_restore_failure(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = {"version": "4.1.1"}
        app.logger = FakeLogger()
        app._restore_runtime_storage = lambda: False
        with (
            patch.object(app_module.utime, "sleep", lambda _value: None),
            patch.object(app_module.OtaBootGuard, "mark_healthy", return_value=True),
        ):
            app._health_worker()
        self.assertIsNone(app.pending_ota)
        self.assertEqual(len(app.logger.error_calls), 1)
        self.assertEqual(
            app.logger.error_calls[0][0][1], "OTA_RESERVE_RESTORE"
        )

    def test_runtime_storage_accepts_ram_only_after_reserve_restore(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.storage = types.SimpleNamespace(
            restore_reserve=lambda: True,
            spool_budget=lambda: 0,
        )
        app.delivery = types.SimpleNamespace(journal=None)
        app.journal = None
        app.logger = FakeLogger()
        self.assertTrue(app._restore_runtime_storage())
        self.assertIsNone(app.journal)
        self.assertEqual(app.logger.error_calls, [])
        self.assertEqual(app.logger.warn_calls, [])
        self.assertEqual(app.logger.info_calls[0][0][1], "SPOOL_DISABLED")

    def test_health_confirmation_defers_storage_during_migration(self):
        app = CollectorApplication.__new__(CollectorApplication)
        app.pending_ota = {"version": "4.1.2"}
        app.migration_mode = True
        app.logger = FakeLogger()
        app._restore_runtime_storage = lambda: self.fail(
            "storage must remain deferred during migration"
        )
        started = []
        app._start_migration = lambda: started.append(True) or True
        with (
            patch.object(app_module.utime, "sleep", lambda _value: None),
            patch.object(app_module.OtaBootGuard, "mark_healthy", return_value=True),
        ):
            app._health_worker()
        self.assertEqual(started, [True])
        self.assertEqual(app.logger.error_calls, [])
        self.assertEqual(
            app.logger.info_calls[0][0][1], "MIGRATION_STORAGE_DEFERRED"
        )


if __name__ == "__main__":
    unittest.main()
