import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_config import LOCAL_LOG_MAX_BYTES  # noqa: E402
from collector_log import StructuredLogger, redact  # noqa: E402


class LogTests(unittest.TestCase):
    def test_signsdata_name_is_not_mistaken_for_a_secret(self):
        self.assertEqual(redact("SignsData ACK timeout"), "SignsData ACK timeout")

    def test_event_fields_match_model_and_secrets_are_redacted(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = StructuredLogger(local_dir=directory)
            published = []
            logger.attach_cloud(
                lambda identifier, payload: published.append((identifier, payload))
                or True,
                lambda: {"mqtt_state": "connected", "net_state": "stage=3"},
            )
            with contextlib.redirect_stdout(io.StringIO()):
                logger.warn(
                    "mqtt",
                    "TEST",
                    "password=do-not-upload",
                    extra="deviceSecret=also-secret",
                )
                logger.runtime(
                    "boot", "collector", "device", "start", "device", "ready"
                )
            self.assertTrue(logger.flush_one())
            self.assertTrue(logger.flush_one())
            debug = published[0][1]
            self.assertEqual(
                set(debug),
                {
                    "level",
                    "module",
                    "code",
                    "msg",
                    "seq",
                    "version",
                    "boot_ms",
                    "mqtt_state",
                    "net_state",
                    "extra",
                },
            )
            self.assertNotIn("do-not-upload", debug["msg"])
            self.assertNotIn("also-secret", debug["extra"])
            runtime = published[1][1]
            self.assertEqual(
                set(runtime),
                {
                    "category",
                    "targetType",
                    "target",
                    "action",
                    "source",
                    "desc",
                    "temperature",
                    "humidity",
                    "metricValue",
                    "seq",
                    "version",
                    "extra",
                },
            )
            self.assertNotIn("value", runtime)
            self.assertIn("mqtt_state=connected", runtime["extra"])
            self.assertIn("net_state=stage=3", runtime["extra"])

    def test_critical_log_rotation_and_ota_cleanup_stay_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = StructuredLogger(local_dir=directory)
            for sequence in range(100):
                logger._persist(
                    {"event": "debugLog", "seq": sequence, "text": "x" * 2048}
                )
            paths = [
                Path(directory) / "critical.log",
                Path(directory) / "critical.log.1",
            ]
            sizes = [path.stat().st_size for path in paths if path.exists()]
            self.assertTrue(sizes)
            self.assertTrue(all(size <= LOCAL_LOG_MAX_BYTES for size in sizes))
            self.assertLessEqual(sum(sizes), LOCAL_LOG_MAX_BYTES * 2)
            logger.clear_persistent()
            self.assertFalse(any(path.exists() for path in paths))

    def test_local_error_is_persisted_but_never_queued_for_cloud(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = StructuredLogger(local_dir=directory)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(
                    logger.local_error("mqtt", "EVENT_REPLY", "rejected")
                )
            self.assertEqual(logger.stats()["queued"], 0)
            self.assertTrue((Path(directory) / "critical.log").exists())


if __name__ == "__main__":
    unittest.main()
