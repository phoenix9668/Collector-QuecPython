import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import collector_migration as migration  # noqa: E402
from collector_config import (DeviceConfig, CredentialStoreError,
                              load_cached_device_secret, save_cached_device_secret)  # noqa: E402


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

    def make_state(self, stage="preflight"):
        return {
            "migrationId": "migration-001", "stage": stage,
            "sourceConfig": source_config().as_dict(), "sourceSecret": None,
            "targetConfig": command()["target"],
            "targetSecret": {"productKey": "newPk", "deviceName": "newDn",
                             "deviceSecret": "new-device-secret"},
        }

    def test_valid_temp_state_survives_crc_bad_primary(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root):
                state = migration.save_migration_state(self.make_state())
                path = root / "state.1"
                (root / "state.1.tmp").write_bytes(path.read_bytes())
                broken = json.loads(path.read_text())
                broken["crc32"] ^= 1
                path.write_text(json.dumps(broken))
                self.assertEqual(migration.load_migration_state()["_generation"], state["_generation"])
                (root / "state.1.tmp").unlink()
                with self.assertRaises(CredentialStoreError):
                    migration.recover_identity_before_load()

    def test_probe_failure_retains_target_and_retry_reuses_it(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root), patch.object(migration, "sleep_ms"), patch.object(
                migration, "dynamic_register_secret", return_value="new-device-secret"
            ) as register, patch.object(migration, "probe_identity", return_value=False) as probe:
                manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None)
                (root / "device_migration.json").write_text(json.dumps(command()))
                with self.assertRaises(OSError):
                    manager._start_new(command())
                self.assertEqual(probe.call_count, 3)
                self.assertTrue(manager._abort_precommit("PROBE", "no reply"))
                path = str(root / "device_secret.json")
                target = DeviceConfig(command()["target"])
                self.assertEqual(load_cached_device_secret(target, path), "new-device-secret")
                self.assertEqual(load_cached_device_secret(source_config(), path), "old-secret")
                retry = command()
                retry["migrationId"] = "retry-002"
                probe.return_value = True
                manager._start_new(retry)
                register.assert_called_once()
                self.assertEqual(manager.state["stage"], "committed")

    def test_failed_restore_keeps_state_command_and_stops_boot(self):
        for operation in ("abort", "rollback", "boot"):
            with self.subTest(operation=operation), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                with self.path_patches(root), patch.object(migration, "sleep_ms"):
                    manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: self.fail("must not reboot"))
                    manager.state = migration.save_migration_state(self.make_state())
                    (root / "device_migration.json").write_text(json.dumps(command()))
                    real_write = migration.atomic_json_write
                    def fail_identity(path, data):
                        if path == str(root / "device.json"):
                            raise OSError(28)
                        return real_write(path, data)
                    with patch.object(migration, "atomic_json_write", side_effect=fail_identity):
                        if operation == "boot":
                            with self.assertRaises(OSError):
                                migration.recover_identity_before_load()
                        elif operation == "abort":
                            self.assertFalse(manager._abort_precommit("TEST", "failure"))
                        else:
                            self.assertFalse(manager._rollback("TEST", "failure"))
                    self.assertTrue(migration.load_migration_state())
                    self.assertTrue((root / "device_migration.json").is_file())
                    self.assertFalse((root / "done.json").exists())
                    # Successful retry recovers exactly the original identity.
                    manager.reboot = lambda: None
                    self.assertTrue(manager._abort_precommit("TEST", "failure"))
                    self.assertIsNone(migration.load_migration_state())
                    active = json.loads((root / "device.json").read_text())
                    self.assertEqual(active["deviceName"], "oldDn")

    def test_pending_target_save_retries_without_registration_or_probe(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            rebooted = []
            with self.path_patches(root), patch.object(migration, "sleep_ms"), patch.object(
                migration, "dynamic_register_secret", return_value="new-device-secret"
            ) as register, patch.object(migration, "probe_identity") as probe:
                manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: rebooted.append(True))
                real_save = migration.save_cached_device_secret
                def reject_target(config, secret, path):
                    if config.device_name == "newDn":
                        raise CredentialStoreError("disk unavailable")
                    return real_save(config, secret, path)
                with patch.object(migration, "save_cached_device_secret", side_effect=reject_target):
                    with self.assertRaises(CredentialStoreError):
                        manager._start_new(command())
                    for _ in range(3):
                        self.assertFalse(manager._abort_precommit("SAVE", "failed"))
                    self.assertEqual(manager.state["targetSecret"]["deviceSecret"], "new-device-secret")
                    self.assertTrue(migration.load_migration_state())
                    self.assertEqual(rebooted, [])
                self.assertTrue(manager._abort_precommit("SAVE", "failed"))
                probe.assert_not_called()
                register.assert_called_once()
                self.assertEqual(load_cached_device_secret(DeviceConfig(command()["target"]), str(root / "device_secret.json")), "new-device-secret")

    def test_a_to_b_to_a_reuses_both_dynamic_credentials(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root), patch.object(migration, "sleep_ms"), patch.object(
                migration, "dynamic_register_secret", return_value="b-secret"
            ) as register, patch.object(migration, "probe_identity", return_value=True):
                a = DeviceConfig(dict(source_config().as_dict(), deviceSecret="", productSecret="a-product-secret"))
                path = str(root / "device_secret.json")
                save_cached_device_secret(a, "a-secret", path)
                manager = migration.MigrationManager(a, FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None)
                manager._start_new(command())
                migration.recover_identity_before_load()
                manager._done("success")
                b = DeviceConfig(json.loads((root / "device.json").read_text()))
                reverse = command()
                reverse["migrationId"] = "back-to-a"
                reverse["source"] = {"productKey": b.product_key, "deviceName": b.device_name}
                reverse["target"] = a.as_dict()
                manager = migration.MigrationManager(b, FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None)
                manager._start_new(reverse)
                manager._done("success")
                register.assert_called_once()
                self.assertEqual(json.loads((root / "device.json").read_text())["deviceName"], a.device_name)
                self.assertEqual(load_cached_device_secret(a, path), "a-secret")
                self.assertEqual(load_cached_device_secret(b, path), "b-secret")
                self.assertEqual(json.loads(Path(path).read_text())["deviceSecret"], "a-secret")

    def test_power_loss_during_identity_commit_recovers_source_and_both_keys(self):
        class PowerLoss(BaseException):
            pass
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root), patch.object(migration, "sleep_ms"):
                manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None)
                manager.state = migration.save_migration_state(self.make_state("prepared"))
                real_write = migration.atomic_json_write
                def power_loss(path, data):
                    real_write(path, data)
                    if path == str(root / "device.json"):
                        raise PowerLoss()
                with patch.object(migration, "atomic_json_write", side_effect=power_loss):
                    with self.assertRaises(PowerLoss):
                        manager._commit()
                self.assertEqual(migration.load_migration_state()["stage"], "committing")
                migration.recover_identity_before_load()
                self.assertEqual(json.loads((root / "device.json").read_text())["deviceName"], "oldDn")
                path = str(root / "device_secret.json")
                self.assertEqual(load_cached_device_secret(source_config(), path), "old-secret")
                self.assertEqual(load_cached_device_secret(DeviceConfig(command()["target"]), path), "new-device-secret")

    def test_identity_mismatch_never_writes_active_config(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root):
                state = self.make_state()
                state["targetSecret"]["deviceName"] = "different-device"
                migration.save_migration_state(state)
                with self.assertRaises(CredentialStoreError):
                    migration.recover_identity_before_load()
                self.assertFalse((root / "device.json").exists())
                self.assertTrue(migration.load_migration_state())

    def test_worker_retries_target_persistence_without_repeating_registration(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            calls, rebooted = [], []
            with self.path_patches(root), patch.object(migration, "sleep_ms"), patch.object(
                migration, "dynamic_register_secret", return_value="new-device-secret"
            ) as register, patch.object(migration, "probe_identity") as probe:
                (root / "device_migration.json").write_text(json.dumps(command()))
                manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: rebooted.append(True))
                real_save = migration.save_cached_device_secret
                def intermittent(config, secret, path):
                    if config.device_name == "newDn":
                        calls.append(True)
                        if len(calls) < 4:
                            raise CredentialStoreError("write failed")
                    return real_save(config, secret, path)
                with patch.object(migration, "save_cached_device_secret", side_effect=intermittent):
                    manager.worker()
                register.assert_called_once()
                probe.assert_not_called()
                self.assertEqual(rebooted, [True])
                self.assertEqual(load_cached_device_secret(DeviceConfig(command()["target"]), str(root / "device_secret.json")), "new-device-secret")

    def test_confirmation_timeout_preserves_both_keys(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root), patch.object(migration, "sleep_ms"), patch.object(
                migration, "ticks_ms", side_effect=(0, 180001)
            ):
                manager = migration.MigrationManager(DeviceConfig(command()["target"]), FakeDelivery(), FakeCloud(), FakeLogger(), lambda: None)
                manager.state = migration.save_migration_state(self.make_state("committed"))
                manager._confirm_target()
                path = str(root / "device_secret.json")
                self.assertEqual(load_cached_device_secret(source_config(), path), "old-secret")
                self.assertEqual(load_cached_device_secret(DeviceConfig(command()["target"]), path), "new-device-secret")
                self.assertEqual(json.loads((root / "done.json").read_text())["status"], "rolled_back")

    def test_failure_logs_and_done_marker_redact_both_identities(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.path_patches(root), patch.object(migration, "sleep_ms"):
                logger = FakeLogger()
                manager = migration.MigrationManager(source_config(), FakeDelivery(), FakeCloud(), logger, lambda: None)
                manager.state = migration.save_migration_state(self.make_state())
                raw = "old-secret new-product-secret new-device-secret"
                with patch("builtins.print") as output:
                    manager._abort_precommit("TEST", raw)
                logs = str(logger.error_calls) + str(output.call_args_list) + (root / "done.json").read_text()
                for secret in raw.split():
                    self.assertNotIn(secret, logs)


if __name__ == "__main__":
    unittest.main()
