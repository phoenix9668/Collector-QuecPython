import contextlib
import hashlib
import io
import json
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import collector_ota as ota_module  # noqa: E402
from collector_config import DeviceConfig, FILESYSTEM_SAFETY_BYTES  # noqa: E402
from collector_ota import (  # noqa: E402
    OtaBootGuard,
    OtaManager,
    aligned_size,
    crc16_ibm,
    is_newer_version,
    normalize_http_url,
    parse_file_reply,
    safe_target_path,
    validate_python_file,
)


class FakeLogger:
    paused = False

    def __getattr__(self, _name):
        return lambda *args, **kwargs: True


class FakeStorage:
    def __init__(self, free=10 * 1024 * 1024):
        self.free = free
        self.released = False
        self.restored = False

    def ota_free_bytes(self):
        return self.free

    def release_reserve(self):
        self.released = True
        return True

    def restore_reserve(self):
        self.restored = True
        return True


class OtaTests(unittest.TestCase):
    def make_manager(self, storage=None, **kwargs):
        config = DeviceConfig(
            {
                "productKey": "pk",
                "deviceName": "dn",
                "mqttServer": "example.mqtt.iothub.aliyuncs.com",
                "deviceSecret": "secret",
                "otaAllowedHosts": [".aliyuncs.com"],
            }
        )
        return OtaManager(
            config,
            storage or FakeStorage(),
            lambda *_args: True,
            FakeLogger(),
            lambda: True,
            lambda: None,
            **kwargs,
        )

    def test_version_and_alignment(self):
        self.assertTrue(is_newer_version("4.0.1", "4.0.0"))
        self.assertFalse(is_newer_version("3.9.9", "4.0.0"))
        self.assertEqual(aligned_size(1), 4096)
        self.assertEqual(aligned_size(4097), 8192)

    def test_progress_uses_aliyun_string_step(self):
        published = []
        manager = self.make_manager()
        manager.publish = lambda topic, payload, qos: published.append(
            (topic, json.loads(payload), qos)
        ) or True
        output = io.StringIO()
        with (
            patch.object(ota_module, "ticks_ms", return_value=10000),
            contextlib.redirect_stdout(output),
        ):
            self.assertTrue(manager.progress(-1, "failed"))
        self.assertEqual(published[0][1]["params"]["step"], "-1")
        self.assertIsInstance(published[0][1]["params"]["step"], str)
        self.assertIn("progress=-1% failed", output.getvalue())

    def test_enqueue_prints_received_stage_without_task_urls(self):
        manager = self.make_manager()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertTrue(
                manager.enqueue(
                    {
                        "version": "4.0.13",
                        "files": [{"fileUrl": "https://secret.example/signed"}],
                    }
                )
            )
        text = output.getvalue()
        self.assertIn("version=4.0.13 progress=0%", text)
        self.assertNotIn("secret.example", text)

    def test_target_path_allowlist(self):
        self.assertTrue(safe_target_path("/usr/collector_app.py"))
        self.assertFalse(safe_target_path("/usr/main.py"))
        self.assertFalse(safe_target_path("/usr/unrelated.py"))
        self.assertFalse(safe_target_path("/usr/collector_config.json"))
        self.assertFalse(safe_target_path("/usr/device.json"))
        self.assertFalse(safe_target_path("/usr/device_secret.json"))
        self.assertFalse(safe_target_path("/usr/data/signs.queue"))
        self.assertFalse(safe_target_path("/usr/.updater/collector_app.py"))
        self.assertFalse(safe_target_path("/usr/../etc/passwd"))

    def test_ec600m_url_normalization_and_host_check(self):
        source = "https://bucket.oss-cn-shanghai.aliyuncs.com/app.bin?x=1"
        result = normalize_http_url(source, [".aliyuncs.com"])
        self.assertEqual(
            result, "http://bucket.oss-cn-shanghai.aliyuncs.com/app.bin?x=1"
        )
        with self.assertRaises(ValueError):
            normalize_http_url("http://evil.example/app.bin", [".aliyuncs.com"])

    def test_mqtt_block_crc(self):
        meta = {"id": "7", "code": 200, "data": {"bOffset": 0, "bSize": 4}}
        encoded = json.dumps(meta).encode()
        block = b"data"
        payload = (
            len(encoded).to_bytes(2, "big")
            + encoded
            + block
            + struct.pack("<H", crc16_ibm(block))
        )
        parsed = parse_file_reply(payload)
        self.assertEqual(parsed["id"], "7")
        self.assertEqual(parsed["block"], block)
        corrupted = payload[:-3] + bytes([payload[-3] ^ 1]) + payload[-2:]
        self.assertIsNone(parse_file_reply(corrupted))

    def test_mqtt_download_validates_token_total_offset_and_size(self):
        with tempfile.TemporaryDirectory() as directory:
            target = str(Path(directory) / "main.py.ota")
            task = {
                "size": 4,
                "stream_id": "stream",
                "file_id": "file",
            }
            manager = self.make_manager()
            manager._request_block = lambda *_args: "1"
            manager._wait_reply = lambda _request_id: {
                "id": "1",
                "meta": {
                    "code": 200,
                    "data": {
                        "fileToken": "wrong",
                        "fileLength": 4,
                        "bOffset": 0,
                        "bSize": 4,
                    },
                },
                "block": b"data",
            }
            with patch.object(ota_module, "MQTT_TEMP_FILE", target):
                self.assertFalse(manager._download_single(task))

            manager._wait_reply = lambda _request_id: {
                "id": "1",
                "meta": {
                    "code": 200,
                    "data": {
                        "fileToken": ota_module.MQTT_FILE_TOKEN,
                        "fileLength": 4,
                        "bOffset": 0,
                        "bSize": 4,
                    },
                },
                "block": b"data",
            }
            with patch.object(ota_module, "MQTT_TEMP_FILE", target):
                self.assertTrue(manager._download_single(task))
            self.assertEqual(Path(target).read_bytes(), b"data")

    def test_legacy_bootstrap_must_compile(self):
        with tempfile.TemporaryDirectory() as directory:
            valid = Path(directory) / "valid.py"
            invalid = Path(directory) / "invalid.py"
            valid.write_text("value = 1\n", encoding="utf-8")
            invalid.write_text("def broken(:\n", encoding="utf-8")
            self.assertTrue(validate_python_file(str(valid)))
            self.assertFalse(validate_python_file(str(invalid)))

    def test_multi_file_manifest_counts_directory_and_requires_md5(self):
        manager = self.make_manager()
        data = {
            "signMethod": "MD5",
            "files": [
                {
                    "fileName": "collector_app.py.bin",
                    "fileSize": 1,
                    "fileMd5": "a" * 32,
                    "fileUrl": "https://bucket.aliyuncs.com/app.bin",
                }
            ],
            "extData": {
                "_package_udi": {
                    "files": {"collector_app.py.bin": "/usr/collector_app.py"}
                }
            },
        }
        files, total = manager._build_multi_files(data)
        self.assertEqual(len(files), 1)
        self.assertEqual(total, 4096 + 8192)
        data["signMethod"] = "SHA256"
        with self.assertRaises(ValueError):
            manager._build_multi_files(data)
        data["signMethod"] = "MD5"
        data["files"][0]["fileMd5"] = "z" * 32
        with self.assertRaises(ValueError):
            manager._build_multi_files(data)

    def test_package_mapping_accepts_aliyun_and_double_wrapped_custom_info(self):
        manager = self.make_manager()
        mapping = {"collector_app.py.bin": "/usr/collector_app.py"}
        package = {"files": mapping}

        standard = {
            "extData": {"_package_udi": json.dumps(package)}
        }
        self.assertEqual(manager._package_mapping(standard), mapping)

        string_ext_data = {
            "extData": json.dumps(
                {"_package_udi": json.dumps(package)}
            )
        }
        self.assertEqual(manager._package_mapping(string_ext_data), mapping)

        double_wrapped = {
            "extData": {
                "_package_udi": json.dumps(
                    {"_package_udi": json.dumps(package)}
                )
            }
        }
        self.assertEqual(manager._package_mapping(double_wrapped), mapping)

    def test_multi_and_legacy_preflight_include_backup_copy(self):
        data = {
            "signMethod": "MD5",
            "files": [
                {
                    "fileName": "collector_app.py.bin",
                    "fileSize": 4096,
                    "fileMd5": "a" * 32,
                    "fileUrl": "http://bucket.aliyuncs.com/app.bin",
                }
            ],
            "extData": {
                "_package_udi": {
                    "files": {"collector_app.py.bin": "/usr/collector_app.py"}
                }
            },
        }
        total = 4096 + 8192
        multi_required = total + 8192 + 4096 + FILESYSTEM_SAFETY_BYTES + 20 * 1024
        multi_storage = FakeStorage(
            multi_required - 1
        )
        with self.assertRaises(OSError):
            self.make_manager(multi_storage)._process_multi(data, "4.0.1")
        self.assertFalse(multi_storage.released)

        single_required = 4096 * 3 + 8192 + FILESYSTEM_SAFETY_BYTES
        single_storage = FakeStorage(single_required - 1)
        legacy = {
            "dProtocol": "mqtt",
            "size": 100,
            "sign": "b" * 32,
            "signMethod": "MD5",
            "streamId": "stream",
            "streamFileId": "file",
        }
        with self.assertRaises(OSError):
            self.make_manager(single_storage)._process_single(legacy, "4.0.1")
        self.assertFalse(single_storage.released)

    def test_rollback_restores_backup_and_cleans_only_after_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rollback = root / "rollback"
            rollback.mkdir()
            backup = rollback / "0.bak"
            target = root / "application.py"
            pending_path = root / "pending.json"
            health_path = root / "healthy"
            backup.write_text("old\n", encoding="utf-8")
            target.write_text("new\n", encoding="utf-8")
            newly_created = root / "new_module.py"
            newly_created.write_text("new module\n", encoding="utf-8")
            pending = {
                "attempts": 1,
                "backups": [
                    {"target": str(target), "backup": str(backup)},
                    {"target": str(newly_created), "created": True},
                ],
            }
            pending_path.write_text(json.dumps(pending), encoding="utf-8")
            health_path.write_text("4.0.1", encoding="utf-8")
            with (
                patch.object(ota_module, "OTA_HEALTH_PENDING_FILE", str(pending_path)),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", str(health_path)),
                patch.object(ota_module, "ROLLBACK_DIR", str(rollback)),
            ):
                self.assertTrue(OtaBootGuard.restore())
            self.assertEqual(target.read_text(encoding="utf-8"), "old\n")
            self.assertFalse(newly_created.exists())
            self.assertFalse(pending_path.exists())
            self.assertFalse(rollback.exists())

    def test_failed_rollback_keeps_pending_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pending_path = root / "pending.json"
            pending = {
                "attempts": 1,
                "backups": [
                    {
                        "target": str(root / "application.py"),
                        "backup": str(root / "missing.bak"),
                    }
                ],
            }
            pending_path.write_text(json.dumps(pending), encoding="utf-8")
            with (
                patch.object(ota_module, "OTA_HEALTH_PENDING_FILE", str(pending_path)),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", str(root / "healthy")),
                patch.object(ota_module, "ROLLBACK_DIR", str(root / "rollback")),
            ):
                self.assertFalse(OtaBootGuard.restore())
            self.assertTrue(pending_path.exists())

    def test_rollback_recovers_completed_temporary_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pending_path = root / "pending.json"
            target = root / "application.py"
            backup = root / "application.bak"
            target.write_text("new\n", encoding="utf-8")
            backup.write_text("old\n", encoding="utf-8")
            Path(str(pending_path) + ".tmp").write_text(
                json.dumps(
                    {
                        "attempts": 1,
                        "backups": [
                            {"target": str(target), "backup": str(backup)}
                        ],
                    }
                ),
                encoding="utf-8",
            )
            with (
                patch.object(ota_module, "OTA_HEALTH_PENDING_FILE", str(pending_path)),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", str(root / "healthy")),
                patch.object(ota_module, "ROLLBACK_DIR", str(root / "rollback")),
            ):
                self.assertTrue(OtaBootGuard.restore())
            self.assertEqual(target.read_text(encoding="utf-8"), "old\n")
            self.assertFalse(Path(str(pending_path) + ".tmp").exists())

    def test_multi_file_download_verifies_md5_before_update_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            updater_dir = str(root / "updater")
            rollback_dir = str(root / "rollback")
            pending_path = str(root / "pending.json")
            health_path = str(root / "healthy")
            target = "/virtual/collector_app.py"
            content = b"value = 2\n"
            expected_md5 = hashlib.md5(content).hexdigest()
            item = {
                "url": "http://bucket.aliyuncs.com/app.bin",
                "file_name": target,
                "size": len(content),
                "md5": expected_md5,
            }
            calls = {"flag": 0, "reboot": 0}
            sequence = []
            usb_steps = []

            class FakeUpdater:
                def bulk_download(self, _download_list):
                    staged = Path(updater_dir + target)
                    staged.parent.mkdir(parents=True, exist_ok=True)
                    staged.write_bytes(content)
                    return []

                def set_update_flag(self, use_rename=False):
                    self.use_rename = use_rename
                    calls["flag"] += 1
                    sequence.append("flag")
                    return 0

            app_fota = types.ModuleType("app_fota")
            app_fota.new = lambda: FakeUpdater()
            storage = FakeStorage()
            manager = self.make_manager(storage)
            manager.reboot = lambda: calls.__setitem__("reboot", 1)
            manager._build_multi_files = lambda _data: ([item], 12288)
            manager.publish = lambda _topic, payload, _qos: sequence.append(
                int(json.loads(payload)["params"]["step"])
            ) or True
            manager._usb_progress = lambda step, _description, version="": (
                usb_steps.append(int(step))
            )
            with (
                patch.object(ota_module, "OTA_UPDATER_DIR", updater_dir),
                patch.object(ota_module, "ROLLBACK_DIR", rollback_dir),
                patch.object(ota_module, "OTA_HEALTH_PENDING_FILE", pending_path),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", health_path),
                patch.dict(sys.modules, {"app_fota": app_fota}),
                patch.object(ota_module, "sleep_ms", lambda _value: None),
            ):
                manager._process_multi({}, "4.0.1")
                self.assertEqual(calls, {"flag": 1, "reboot": 1})
                self.assertIn(5, sequence)
                self.assertIn(10, sequence)
                self.assertIn(85, sequence)
                self.assertIn(95, usb_steps)
                self.assertLess(sequence.index("flag"), sequence.index(99))
                self.assertLess(sequence.index(99), sequence.index(100))
                self.assertTrue(storage.released)
                self.assertTrue(Path(pending_path).exists())
                self.assertTrue(OtaBootGuard.restore())

    def test_multi_file_md5_failure_restores_reserve_and_never_sets_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            updater_dir = str(root / "updater")
            target = "/virtual/collector_app.py"
            item = {
                "url": "http://bucket.aliyuncs.com/app.bin",
                "file_name": target,
                "size": 3,
                "md5": hashlib.md5(b"new").hexdigest(),
            }
            calls = {"flag": 0}

            class FakeUpdater:
                def bulk_download(self, _download_list):
                    staged = Path(updater_dir + target)
                    staged.parent.mkdir(parents=True, exist_ok=True)
                    staged.write_bytes(b"bad")
                    return []

                def set_update_flag(self, **_kwargs):
                    calls["flag"] += 1
                    return 0

            app_fota = types.ModuleType("app_fota")
            app_fota.new = lambda: FakeUpdater()
            storage = FakeStorage()
            manager = self.make_manager(storage)
            manager._build_multi_files = lambda _data: ([item], 12288)
            with (
                patch.object(ota_module, "OTA_UPDATER_DIR", updater_dir),
                patch.object(ota_module, "ROLLBACK_DIR", str(root / "rollback")),
                patch.object(
                    ota_module, "OTA_HEALTH_PENDING_FILE", str(root / "pending.json")
                ),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", str(root / "healthy")),
                patch.dict(sys.modules, {"app_fota": app_fota}),
                patch.object(ota_module, "sleep_ms", lambda _value: None),
            ):
                with self.assertRaises(ValueError):
                    manager._process_multi({}, "4.0.1")
            self.assertEqual(calls["flag"], 0)
            self.assertTrue(storage.restored)

    def test_uart_activity_during_download_aborts_before_update_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            updater_dir = str(root / "updater")
            target = "/virtual/collector_app.py"
            content = b"value = 3\n"
            item = {
                "url": "http://bucket.aliyuncs.com/app.bin",
                "file_name": target,
                "size": len(content),
                "md5": hashlib.md5(content).hexdigest(),
            }
            calls = {"activity": 0, "prepare": 0, "restore": 0, "flag": 0}

            class FakeUpdater:
                def bulk_download(self, _download_list):
                    staged = Path(updater_dir + target)
                    staged.parent.mkdir(parents=True, exist_ok=True)
                    staged.write_bytes(content)
                    calls["activity"] += 1
                    return []

                def set_update_flag(self, **_kwargs):
                    calls["flag"] += 1
                    return 0

            def prepare():
                calls["prepare"] += 1
                return True

            def restore():
                calls["restore"] += 1
                return True

            app_fota = types.ModuleType("app_fota")
            app_fota.new = lambda: FakeUpdater()
            manager = self.make_manager(
                storage_prepare=prepare,
                storage_restore=restore,
                activity_provider=lambda: calls["activity"],
            )
            manager._build_multi_files = lambda _data: ([item], 12288)
            with (
                patch.object(ota_module, "OTA_UPDATER_DIR", updater_dir),
                patch.object(ota_module, "ROLLBACK_DIR", str(root / "rollback")),
                patch.object(
                    ota_module, "OTA_HEALTH_PENDING_FILE", str(root / "pending.json")
                ),
                patch.object(
                    ota_module, "OTA_HEALTH_OK_FILE", str(root / "healthy")
                ),
                patch.dict(sys.modules, {"app_fota": app_fota}),
                patch.object(ota_module, "sleep_ms", lambda _value: None),
            ):
                with self.assertRaisesRegex(OSError, "UART activity resumed"):
                    manager._process_multi({}, "4.0.4")
            self.assertEqual(calls["prepare"], 1)
            self.assertGreaterEqual(calls["restore"], 1)
            self.assertEqual(calls["flag"], 0)

    def test_multi_file_verification_supports_private_staging_layouts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            updater_dir = root / "updater"
            private_dir = updater_dir / "firmware-layout-v2" / "objects"
            private_dir.mkdir(parents=True)
            first = b"first application file\n"
            second = b"second application file\n"
            (private_dir / "0001.bin").write_bytes(first)
            (private_dir / "0002.bin").write_bytes(second)
            files = [
                {
                    "file_name": "/usr/collector_app.py",
                    "size": len(first),
                    "md5": hashlib.md5(first).hexdigest(),
                },
                {
                    "file_name": "/usr/collector_cloud.py",
                    "size": len(second),
                    "md5": hashlib.md5(second).hexdigest(),
                },
            ]
            manager = self.make_manager()
            with patch.object(ota_module, "OTA_UPDATER_DIR", str(updater_dir)):
                self.assertEqual(manager._verify_staged(files), (True, ""))
                files[1]["md5"] = hashlib.md5(b"wrong").hexdigest()
                valid, reason = manager._verify_staged(files)
            self.assertFalse(valid)
            self.assertIn("missing or invalid", reason)

    def test_legacy_install_can_restore_previous_main(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "main.py"
            temporary = root / "main.py.ota"
            backup = root / "main.py.bak"
            pending = root / "pending.json"
            target.write_text("value = 1\n", encoding="utf-8")
            temporary.write_text("value = 2\n", encoding="utf-8")
            task = {
                "version": "4.0.1",
                "size": temporary.stat().st_size,
                "md5": hashlib.md5(temporary.read_bytes()).hexdigest(),
            }
            manager = self.make_manager()
            with (
                patch.object(ota_module, "MQTT_TARGET_FILE", str(target)),
                patch.object(ota_module, "MQTT_TEMP_FILE", str(temporary)),
                patch.object(ota_module, "MQTT_BACKUP_FILE", str(backup)),
                patch.object(ota_module, "OTA_HEALTH_PENDING_FILE", str(pending)),
                patch.object(ota_module, "OTA_HEALTH_OK_FILE", str(root / "healthy")),
                patch.object(ota_module, "ROLLBACK_DIR", str(root / "rollback")),
            ):
                self.assertTrue(manager._install_single(task))
                self.assertEqual(target.read_text(encoding="utf-8"), "value = 2\n")
                self.assertTrue(OtaBootGuard.restore())
            self.assertEqual(target.read_text(encoding="utf-8"), "value = 1\n")
            self.assertFalse(backup.exists())


if __name__ == "__main__":
    unittest.main()
