import json
import sys
import tempfile
import unittest
import threading
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import collector_config as config_module
from collector_config import (  # noqa: E402
    COLLECTOR_SETTINGS_FILE,
    DEVICE_CONFIG_FILE,
    DEVICE_SECRET_CACHE_FILE,
    DeviceConfig,
    CredentialStoreError,
    CredentialStoreFull,
    check_credential_capacity,
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
            Path(path + ".bak").unlink()
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
            save_cached_device_secret(config, "recovered-secret", path)
            self.assertTrue(Path(path + ".bak").is_file())

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

    def test_missing_differs_from_corrupt_or_unreadable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            config = self.make_config()
            self.assertEqual(load_cached_device_secret(config, path), "")
            Path(path).write_text("[]", encoding="utf-8")
            with self.assertRaisesRegex(CredentialStoreError, "corrupt"):
                load_cached_device_secret(config, path)
            with patch("collector_config.open", side_effect=OSError(5)):
                with self.assertRaisesRegex(CredentialStoreError, "read failed"):
                    load_cached_device_secret(config, path)

    def test_corrupt_primary_recovers_temp_or_backup_and_repairs(self):
        for survivor in (".bak", ".tmp"):
            with self.subTest(survivor=survivor), tempfile.TemporaryDirectory() as directory:
                path = str(Path(directory) / "secret.json")
                config = self.make_config()
                save_cached_device_secret(config, "secret", path)
                if survivor == ".tmp":
                    Path(path + ".bak").rename(path + ".tmp")
                Path(path).write_text("[]", encoding="utf-8")
                self.assertEqual(load_cached_device_secret(config, path), "secret")
                save_cached_device_secret(config, "secret", path)
                self.assertEqual(Path(path).read_bytes(), Path(path + ".bak").read_bytes())
                self.assertFalse(Path(path + ".tmp").exists())

    def test_all_identity_and_explicit_rotation_history_survives(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            a = self.make_config()
            b = DeviceConfig(dict(a.as_dict(), deviceName="second"))
            save_cached_device_secret(a, "a1", path)
            save_cached_device_secret(b, "b1", path)
            save_cached_device_secret(a, "a2", path)
            self.assertEqual(load_cached_device_secret(a, path), "a2")
            self.assertEqual(load_cached_device_secret(b, path), "b1")
            payload = json.loads(Path(path).read_text())
            self.assertEqual(payload["deviceName"], a.device_name)
            self.assertEqual(payload["deviceSecret"], "a2")  # old reader compatibility
            self.assertIn(["pk", "dn", "a1"], json.loads(payload["body"])[2])
            before = Path(path).read_bytes()
            with patch("collector_config._write_credential_file") as write:
                save_cached_device_secret(a, "a2", path)
                write.assert_not_called()
            self.assertEqual(Path(path).read_bytes(), before)

    def test_checksum_and_compatibility_fields_must_agree(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            config = self.make_config()
            save_cached_device_secret(config, "secret", path)
            data = json.loads(Path(path).read_text())
            data["deviceSecret"] = "changed"
            Path(path).write_text(json.dumps(data))
            self.assertEqual(load_cached_device_secret(config, path), "secret")
            Path(path + ".bak").unlink()
            with self.assertRaises(CredentialStoreError):
                load_cached_device_secret(config, path)

    def test_legacy_directory_is_imported_without_losing_other_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            legacy = str(Path(directory) / "old-secret.json")
            a = self.make_config()
            b = DeviceConfig(dict(a.as_dict(), deviceName="second"))
            Path(legacy + ".tmp").write_text(json.dumps({
                "productKey": "pk", "deviceName": "dn", "deviceSecret": "legacy"
            }))
            with patch.multiple(config_module, DEVICE_SECRET_CACHE_FILE=path,
                                LEGACY_DEVICE_SECRET_CACHE_FILE=legacy):
                self.assertEqual(load_cached_device_secret(a), "legacy")
                save_cached_device_secret(b, "second-secret")
                Path(legacy + ".tmp").unlink()
                self.assertEqual(load_cached_device_secret(a), "legacy")
                self.assertEqual(load_cached_device_secret(b), "second-secret")

    def test_full_archive_refuses_new_identity_without_eviction(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            saved = []
            for number in range(100):
                config = DeviceConfig(dict(self.make_config().as_dict(), deviceName="dn-" + str(number)))
                before = Path(path).read_bytes() if Path(path).exists() else b""
                try:
                    check_credential_capacity(config, path)
                except CredentialStoreFull:
                    self.assertEqual(Path(path).read_bytes(), before)
                    break
                save_cached_device_secret(config, "\\" * 128, path)
                saved.append(config)
            self.assertGreater(len(saved), 1)
            self.assertLess(len(saved), 100)
            with self.assertRaises(CredentialStoreFull):
                save_cached_device_secret(config, "\\" * 128, path)
            for previous in saved:
                self.assertEqual(load_cached_device_secret(previous, path), "\\" * 128)
            self.assertLessEqual(Path(path).stat().st_size, 4096)

    def test_power_loss_at_each_write_preserves_every_old_credential(self):
        class PowerLoss(BaseException):
            pass

        # Include recovery where the only complete copy is .tmp, and split
        # legacy files where truncating the wrong copy would lose an identity.
        for initial in ("normal", "temp_only", "split"):
            for failed_write in (1, 2, 3):
                for partial in (False, True):
                    with self.subTest(initial=initial, step=failed_write, partial=partial):
                        with tempfile.TemporaryDirectory() as directory:
                            path = str(Path(directory) / "secret.json")
                            a = self.make_config()
                            b = DeviceConfig(dict(a.as_dict(), deviceName="b"))
                            c = DeviceConfig(dict(a.as_dict(), deviceName="c"))
                            save_cached_device_secret(a, "a-secret", path)
                            save_cached_device_secret(b, "b-secret", path)
                            if initial == "temp_only":
                                Path(path).rename(path + ".tmp")
                                Path(path + ".bak").unlink()
                            elif initial == "split":
                                Path(path + ".bak").unlink()
                                for name, cfg, key in ((path, a, "a-secret"),
                                                       (path + ".tmp", b, "b-secret")):
                                    Path(name).write_text(json.dumps({
                                        "productKey": cfg.product_key,
                                        "deviceName": cfg.device_name, "deviceSecret": key,
                                    }))
                            real_write = config_module._write_credential_file
                            calls = []

                            def fail(name, raw):
                                calls.append(name)
                                if len(calls) == failed_write:
                                    if partial:
                                        Path(name).write_text(raw[:len(raw) // 2])
                                    raise PowerLoss()
                                real_write(name, raw)

                            with patch.object(config_module, "_write_credential_file", side_effect=fail):
                                try:
                                    save_cached_device_secret(c, "c-secret", path)
                                except PowerLoss:
                                    pass
                            for cfg, key in ((a, "a-secret"), (b, "b-secret")):
                                self.assertEqual(load_cached_device_secret(cfg, path), key)
                            save_cached_device_secret(c, "c-secret", path)
                            self.assertEqual(load_cached_device_secret(a, path), "a-secret")
                            self.assertEqual(load_cached_device_secret(b, path), "b-secret")

    def test_short_writes_and_flush_failure_do_not_replace_good_copies(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            config = self.make_config()
            save_cached_device_secret(config, "original", path)
            real_open = open

            class BrokenWriter:
                short = True
                def __enter__(self):
                    return self
                def __exit__(self, *_args):
                    pass
                def write(self, raw):
                    return len(raw) - 1 if self.short else len(raw)
                def flush(self):
                    raise OSError(28)

            def broken_open(name, mode="r"):
                return BrokenWriter() if mode == "w" else real_open(name, mode)

            for short in (False, True):
                BrokenWriter.short = short
                with patch("collector_config.open", side_effect=broken_open):
                    with self.assertRaises(CredentialStoreError):
                        save_cached_device_secret(config, "new", path)
            self.assertEqual(load_cached_device_secret(config, path), "original")

    def test_concurrent_saves_merge_without_lost_updates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            configs = [DeviceConfig(dict(self.make_config().as_dict(), deviceName=str(i))) for i in range(8)]
            errors = []
            def save(config):
                try:
                    save_cached_device_secret(config, "secret-" + config.device_name, path)
                except Exception as error:
                    errors.append(error)
            threads = [threading.Thread(target=save, args=(cfg,)) for cfg in configs]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(5)
                self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            for cfg in configs:
                self.assertEqual(load_cached_device_secret(cfg, path), "secret-" + cfg.device_name)

    def test_filesystem_capacity_is_checked_before_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            with patch.object(config_module.os, "statvfs", return_value=(4096, 4096, 144, 2), create=True):
                with self.assertRaises(CredentialStoreFull):
                    check_credential_capacity(self.make_config(), path)
            self.assertFalse(Path(path).exists())

    def test_device_config_recovers_valid_temp_after_invalid_root(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device.json"
            path.write_text("[]")
            Path(str(path) + ".tmp").write_text(json.dumps(self.make_config().as_dict()))
            self.assertEqual(config_module.load_device_config(str(path)).as_dict(), self.make_config().as_dict())

    def test_invalid_config_secrets_are_not_stringified(self):
        for key in ("deviceSecret", "productSecret"):
            for value in (123, [], {}, True, "\n", "with space"):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    DeviceConfig(dict(self.make_config().as_dict(), **{key: value}))

    def test_reduced_file_api_without_flush_uses_close_and_readback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            real_open = open
            class NoFlush:
                def __init__(self, name):
                    self.stream = real_open(name, "w")
                def __enter__(self):
                    return self
                def __exit__(self, *_args):
                    self.stream.close()
                def write(self, raw):
                    return self.stream.write(raw)
            def reduced_open(name, mode="r"):
                return NoFlush(name) if mode == "w" else real_open(name, mode)
            with patch("collector_config.open", side_effect=reduced_open):
                save_cached_device_secret(self.make_config(), "secret", path)
            self.assertEqual(load_cached_device_secret(self.make_config(), path), "secret")

    def test_unreadable_newer_copy_never_means_identity_missing_or_safe_to_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            a = self.make_config()
            b = DeviceConfig(dict(a.as_dict(), deviceName="b"))
            save_cached_device_secret(a, "a-key", path)
            old = Path(path).read_bytes()
            save_cached_device_secret(b, "b-key", path)
            newest = Path(path).read_bytes()
            Path(path + ".bak").write_bytes(old)
            real_open = open
            def unreadable(name, mode="r"):
                if name == path:
                    raise OSError(5)
                return real_open(name, mode)
            with patch("collector_config.open", side_effect=unreadable), patch.object(
                config_module, "_write_credential_file"
            ) as write:
                self.assertEqual(load_cached_device_secret(a, path), "a-key")
                with self.assertRaises(CredentialStoreError):
                    load_cached_device_secret(b, path)
                with self.assertRaises(CredentialStoreError):
                    check_credential_capacity(b, path)
                with self.assertRaises(CredentialStoreError):
                    save_cached_device_secret(a, "a-key", path)
                write.assert_not_called()
            self.assertEqual(Path(path).read_bytes(), newest)
            self.assertEqual(load_cached_device_secret(b, path), "b-key")

    def test_corrupt_copy_with_other_identity_backup_blocks_new_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "secret.json")
            a = self.make_config()
            b = DeviceConfig(dict(a.as_dict(), deviceName="b"))
            save_cached_device_secret(a, "a-key", path)
            Path(path).write_text("[]")
            self.assertEqual(load_cached_device_secret(a, path), "a-key")
            with self.assertRaises(CredentialStoreError):
                load_cached_device_secret(b, path)
            with self.assertRaises(CredentialStoreError):
                check_credential_capacity(b, path)


if __name__ == "__main__":
    unittest.main()
