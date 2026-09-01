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
        self.ceiling = None
        self.cleared = False
        self.percent = 0
        self.direct_persist = False
        self.one_rate = 0.0
        self.five_rate = 0.0
        self.available = 512
        self.headroom = 512

    def occupancy_percent(self):
        return self.percent

    def acceptance_rates(self):
        return self.one_rate, self.five_rate

    def available_frames(self):
        return self.available

    def migration_flash_headroom(self, _abort_percent=75):
        return self.headroom

    def migration_occupancy_percent(self):
        return self.percent

    def highest_accepted_sequence(self):
        return 42

    def set_send_ceiling(self, value):
        self.ceiling = value

    def arm_persistent_watermark(self):
        self.ceiling = self.highest_accepted_sequence()
        self.direct_persist = True
        return self.ceiling

    def resume_persistent_watermark(self, value):
        self.ceiling = value
        self.direct_persist = True
        return True

    def release_persistent_watermark(self):
        self.ceiling = None
        self.direct_persist = False
        self.cleared = True

    def clear_send_ceiling(self):
        self.ceiling = None
        self.cleared = True

    def pending_through(self, _value):
        return False


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

    def test_capacity_estimate_requires_180_seconds_at_150_percent(self):
        delivery = FakeDelivery()
        delivery.one_rate = 1.0
        delivery.headroom = 269
        manager = migration.MigrationManager(
            source_config(), delivery, FakeCloud(), FakeLogger(), lambda: None
        )
        ready, detail = manager._capacity_ready()
        self.assertFalse(ready)
        self.assertIn("269/270", detail)
        delivery.headroom = 270
        ready, detail = manager._capacity_ready()
        self.assertTrue(ready)
        self.assertIn("required=270", detail)

    def test_boot_recovery_chooses_source_before_commit_and_target_after_commit(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            state = {
                "migrationId": "migration-001",
                "stage": "armed",
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
    def test_preflight_arms_watermark_and_atomically_commits_target(self):
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
            self.assertEqual(delivery.ceiling, 42)
            self.assertEqual(active["productKey"], "newPk")
            self.assertEqual(secret["deviceSecret"], "new-device-secret")
            self.assertTrue(cloud.disconnected)
            self.assertEqual(rebooted, [True])

    def test_target_confirmation_releases_held_records_only_after_event_ack(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            cloud = FakeCloud()
            logger = FakeLogger()
            state = {
                "migrationId": "migration-001",
                "stage": "committed",
                "cutoverSeq": 42,
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
                migration, "MIGRATION_CONFIRM_SECONDS", 0
            ):
                manager = migration.MigrationManager(
                    DeviceConfig(command()["target"]),
                    delivery,
                    cloud,
                    logger,
                    lambda: None,
                )
                manager.state = migration.save_migration_state(state)
                manager._confirm_target()
            done = json.loads((root / "done.json").read_text(encoding="utf-8"))
            self.assertEqual(done["status"], "success")
            self.assertTrue(delivery.cleared)
            self.assertFalse((root / "device_migration.json").exists())

    def test_confirmation_capacity_threshold_restores_old_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            delivery = FakeDelivery()
            delivery.percent = 75
            cloud = FakeCloud()
            rebooted = []
            state = {
                "migrationId": "migration-001",
                "stage": "committed",
                "cutoverSeq": 42,
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
            done = json.loads((root / "done.json").read_text(encoding="utf-8"))
            self.assertEqual(active["productKey"], "oldPk")
            self.assertEqual(done["status"], "rolled_back")
            # Keep the durable border until reboot so late frames cannot fall
            # back into volatile RAM during the identity switch.
            self.assertEqual(delivery.ceiling, 42)
            self.assertTrue(delivery.direct_persist)
            self.assertEqual(rebooted, [True])

    def test_target_event_rejection_rolls_back_without_releasing_border(self):
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
                "cutoverSeq": 42,
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
            self.assertEqual(delivery.ceiling, 42)
            self.assertEqual(rebooted, [True])


if __name__ == "__main__":
    unittest.main()
