import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import collector_migration as migration  # noqa: E402
from collector_config import DeviceConfig  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.runtime_calls = []
        self.error_calls = []

    def runtime(self, *args, **kwargs):
        self.runtime_calls.append((args, kwargs))
        return True

    def error(self, *args, **kwargs):
        self.error_calls.append((args, kwargs))
        return True


class FakeSequence:
    def __init__(self):
        self.value = 100

    def next(self):
        value = self.value
        self.value += 1
        return value


class FakeDelivery:
    def __init__(self):
        self.sequence = FakeSequence()


class FakeCloud:
    def __init__(self):
        self.connected = True
        self.stop = False
        self.connection_generation = 1
        self.result = None
        self.disconnected = False

    def stats(self):
        return {"inflight": 0}

    def _mark_disconnected(self):
        self.connected = False
        self.disconnected = True

    def publish_event_tracked(self, _identifier, _params):
        self.result = 200
        return "123"

    def event_result(self, _message_id):
        result = self.result
        self.result = None
        return result


def source_config():
    return DeviceConfig(
        {
            "productKey": "oldPk",
            "deviceName": "oldDn",
            "mqttServer": "old.iot-as-mqtt.cn-shanghai.aliyuncs.com",
            "deviceSecret": "old-secret",
        }
    )


def command():
    return {
        "schema": 1,
        "migrationId": "migration-001",
        "source": {"productKey": "oldPk", "deviceName": "oldDn"},
        "target": {
            "productKey": "newPk",
            "deviceName": "newDn",
            "mqttServer": "new.iot-as-mqtt.cn-shanghai.aliyuncs.com",
            "mqttPort": 1883,
            "deviceSecret": "",
            "productSecret": "new-product-secret",
            "otaAllowedHosts": [".aliyuncs.com"],
        },
        "confirmSeconds": 120,
        "rollbackSeconds": 180,
    }


class MigrationTests(unittest.TestCase):
    def path_patches(self, root):
        return patch.multiple(
            migration,
            DEVICE_CONFIG_FILE=str(root / "device.json"),
            DEVICE_SECRET_CACHE_FILE=str(root / "device_secret.json"),
            DEVICE_MIGRATION_FILE=str(root / "device_migration.json"),
            MIGRATION_STATE_0=str(root / "state.0"),
            MIGRATION_STATE_1=str(root / "state.1"),
            MIGRATION_DONE_FILE=str(root / "done.json"),
        )

    def test_dual_crc_state_falls_back_to_previous_generation(self):
        with tempfile.TemporaryDirectory() as temp:
            paths = (str(Path(temp) / "state.0"), str(Path(temp) / "state.1"))
            migration.save_migration_state({"stage": "one"}, paths)
            migration.save_migration_state({"stage": "two"}, paths)
            self.assertEqual(migration.load_migration_state(paths)["stage"], "two")
            newest = paths[0]
            Path(newest).write_text("broken", encoding="utf-8")
            self.assertEqual(migration.load_migration_state(paths)["stage"], "one")

    def test_migration_id_validator_uses_quecpython_compatible_ascii_rules(self):
        for value in ("1", "migration-001", "DEVICE_01.release"):
            self.assertEqual(migration._valid_migration_id(value), value)
        for value in ("", "has space", "path/name", "迁移-001"):
            self.assertEqual(migration._valid_migration_id(value), "")
        source = (ROOT / "src" / "collector_migration.py").read_text(encoding="utf-8")
        self.assertNotIn(".isalnum(", source)

    def test_pending_command_or_state_requires_startup_exclusive_mode(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root):
                manager = migration.MigrationManager(
                    source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None
                )
                self.assertFalse(manager.has_pending())
                (root / "device_migration.json").write_text(
                    json.dumps(command()), encoding="utf-8"
                )
                self.assertTrue(manager.has_pending())
                (root / "device_migration.json").unlink()
                manager.state = migration.save_migration_state(
                    {"migrationId": "migration-001", "stage": "preflight"}
                )
                self.assertTrue(manager.has_pending())

    def test_validation_requires_exact_source_and_product_secret(self):
        normalized = migration.normalize_migration(command(), source_config())
        self.assertEqual(normalized["target"]["productKey"], "newPk")
        self.assertEqual(normalized["target"]["deviceSecret"], "")
        invalid = command()
        invalid["source"]["deviceName"] = "wrong"
        with self.assertRaises(ValueError):
            migration.normalize_migration(invalid, source_config())
        invalid = command()
        invalid["target"]["deviceSecret"] = "not-allowed"
        with self.assertRaises(ValueError):
            migration.normalize_migration(invalid, source_config())
        invalid = command()
        invalid["target"]["mqttServer"] = "attacker.example"
        with self.assertRaises(ValueError):
            migration.normalize_migration(invalid, source_config())
        invalid = command()
        invalid["target"]["otaAllowedHosts"] = ["attacker.example"]
        with self.assertRaises(ValueError):
            migration.normalize_migration(invalid, source_config())

    def test_boot_recovery_chooses_source_before_commit_and_target_after_commit(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            state = {
                "migrationId": "migration-001",
                "stage": "prepared",
                "sourceConfig": source_config().as_dict(),
                "sourceSecret": None,
                "targetConfig": command()["target"],
                "targetSecret": {
                    "productKey": "newPk",
                    "deviceName": "newDn",
                    "deviceSecret": "new-device-secret",
                },
            }
            with self.path_patches(root):
                migration.save_migration_state(state)
                migration.recover_identity_before_load()
                active = json.loads((root / "device.json").read_text(encoding="utf-8"))
                self.assertEqual(active["productKey"], "oldPk")
                state["stage"] = "committed"
                migration.save_migration_state(state)
                migration.recover_identity_before_load()
                active = json.loads((root / "device.json").read_text(encoding="utf-8"))
                self.assertEqual(active["productKey"], "newPk")
    def test_preflight_commits_target_without_queue_or_watermark(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            cloud = FakeCloud()
            logger = FakeLogger()
            rebooted = []
            with self.path_patches(root), patch.object(
                migration, "dynamic_register_secret", return_value="new-device-secret"
            ), patch.object(migration, "probe_identity", return_value=True):
                manager = migration.MigrationManager(
                    source_config(), delivery, cloud, logger, lambda: rebooted.append(True)
                )
                manager._start_new(command())
            active = json.loads((root / "device.json").read_text(encoding="utf-8"))
            secret = json.loads(
                (root / "device_secret.json").read_text(encoding="utf-8")
            )
            self.assertEqual(active["productKey"], "newPk")
            self.assertEqual(secret["deviceSecret"], "new-device-secret")
            self.assertTrue(cloud.disconnected)
            self.assertEqual(rebooted, [True])

    def test_target_confirmation_reboots_only_after_event_ack(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            cloud = FakeCloud()
            logger = FakeLogger()
            rebooted = []
            state = {
                "migrationId": "migration-001",
                "stage": "committed",
                "confirmSeconds": 0,
                "rollbackSeconds": 180,
                "sourceConfig": source_config().as_dict(),
                "sourceSecret": None,
                "targetConfig": command()["target"],
                "targetSecret": {
                    "productKey": "newPk",
                    "deviceName": "newDn",
                    "deviceSecret": "new-device-secret",
                },
            }
            with self.path_patches(root), patch.object(
                migration, "sleep_ms", lambda _value: None
            ):
                manager = migration.MigrationManager(
                    DeviceConfig(command()["target"]),
                    delivery,
                    cloud,
                    logger,
                    lambda: rebooted.append(True),
                )
                manager.state = migration.save_migration_state(state)
                manager._confirm_target()
            done = json.loads((root / "done.json").read_text(encoding="utf-8"))
            self.assertEqual(done["status"], "success")
            self.assertEqual(rebooted, [True])
            self.assertFalse((root / "device_migration.json").exists())

    def test_confirmation_does_not_depend_on_queue_capacity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            cloud = FakeCloud()
            rebooted = []
            state = {
                "migrationId": "migration-001",
                "stage": "committed",
                "confirmSeconds": 0,
                "rollbackSeconds": 180,
                "sourceConfig": source_config().as_dict(),
                "sourceSecret": None,
                "targetConfig": command()["target"],
                "targetSecret": {
                    "productKey": "newPk",
                    "deviceName": "newDn",
                    "deviceSecret": "new-device-secret",
                },
            }
            with self.path_patches(root), patch.object(
                migration, "sleep_ms", lambda _value: None
            ):
                manager = migration.MigrationManager(
                    DeviceConfig(command()["target"]),
                    delivery,
                    cloud,
                    FakeLogger(),
                    lambda: rebooted.append(True),
                )
                manager.state = migration.save_migration_state(state)
                manager._confirm_target()
            done = json.loads((root / "done.json").read_text(encoding="utf-8"))
            self.assertEqual(done["status"], "success")
            self.assertEqual(rebooted, [True])

    def test_target_event_rejection_rolls_back(self):
        class RejectCloud(FakeCloud):
            def publish_event_tracked(self, _identifier, _params):
                self.result = 400
                return "123"

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            cloud = RejectCloud()
            rebooted = []
            state = {
                "migrationId": "migration-001",
                "stage": "committed",
                "confirmSeconds": 120,
                "rollbackSeconds": 180,
                "sourceConfig": source_config().as_dict(),
                "sourceSecret": None,
                "targetConfig": command()["target"],
                "targetSecret": {
                    "productKey": "newPk",
                    "deviceName": "newDn",
                    "deviceSecret": "new-device-secret",
                },
            }
            with self.path_patches(root), patch.object(
                migration, "sleep_ms", lambda _value: None
            ):
                manager = migration.MigrationManager(
                    DeviceConfig(command()["target"]),
                    delivery,
                    cloud,
                    FakeLogger(),
                    lambda: rebooted.append(True),
                )
                manager.state = migration.save_migration_state(state)
                manager._confirm_target()
            active = json.loads((root / "device.json").read_text(encoding="utf-8"))
            self.assertEqual(active["productKey"], "oldPk")
            self.assertEqual(rebooted, [True])


if __name__ == "__main__":
    unittest.main()
