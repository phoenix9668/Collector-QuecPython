import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_config import (  # noqa: E402
    COLLECTOR_SETTINGS_FILE,
    DEVICE_CONFIG_FILE,
    DEVICE_SECRET_CACHE_FILE,
    DeviceConfig,
    clear_cached_device_secret,
    load_cached_device_secret,
    load_uart_sample_log,
    save_cached_device_secret,
    save_uart_sample_log,
)


class ConfigTests(unittest.TestCase):
    def make_config(self):
        return DeviceConfig(
            {
                "productKey": "pk",
                "deviceName": "dn",
                "mqttServer": "example.mqtt.iothub.aliyuncs.com",
                "productSecret": "product-secret",
            }
        )

    def test_device_secret_cache_and_power_loss_temp_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            config = self.make_config()
            save_cached_device_secret(config, "device-secret", path)
            self.assertEqual(
                load_cached_device_secret(config, path), "device-secret"
            )
            Path(path).unlink()
            Path(path + ".tmp").write_text(
                json.dumps(
                    {
                        "productKey": "pk",
                        "deviceName": "dn",
                        "deviceSecret": "recovered-secret",
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                load_cached_device_secret(config, path), "recovered-secret"
            )
            self.assertTrue(clear_cached_device_secret(path))
            self.assertEqual(load_cached_device_secret(config, path), "")

    def test_device_files_live_directly_under_usr(self):
        self.assertEqual(DEVICE_CONFIG_FILE, "/usr/device.json")
        self.assertEqual(DEVICE_SECRET_CACHE_FILE, "/usr/device_secret.json")
        self.assertEqual(COLLECTOR_SETTINGS_FILE, "/usr/collector_settings.json")
        example = json.loads(
            (ROOT / "src" / "device.example.json").read_text(encoding="utf-8")
        )
        self.assertNotIn("cellLocatorToken", example)

    def test_uart_sample_switch_defaults_off_and_recovers_temp_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "collector_settings.json")
            self.assertFalse(load_uart_sample_log(path))
            save_uart_sample_log(True, path)
            self.assertTrue(load_uart_sample_log(path))
            Path(path).unlink()
            Path(path + ".tmp").write_text(
                json.dumps({"uartSampleLog": False}), encoding="utf-8"
            )
            self.assertFalse(load_uart_sample_log(path))
            Path(path + ".tmp").write_text("not-json", encoding="utf-8")
            self.assertFalse(load_uart_sample_log(path))

    def test_identity_rejects_topic_wildcards(self):
        with self.assertRaises(ValueError):
            DeviceConfig(
                {
                    "productKey": "bad+key",
                    "deviceName": "dn",
                    "mqttServer": "host",
                    "deviceSecret": "secret",
                }
            )


if __name__ == "__main__":
    unittest.main()
