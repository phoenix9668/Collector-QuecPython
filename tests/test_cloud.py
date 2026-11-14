import sys
import json
import hashlib
import hmac
import tempfile
import types
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_cloud import (  # noqa: E402
    CloudClient,
    _load_mqtt_client,
    hmac_sha256_hex,
    extract_device_secret,
    ticks_ms,
)
import collector_cloud as cloud_module
import collector_config as config_module
from collector_config import DeviceConfig, CredentialStoreError, save_cached_device_secret  # noqa: E402
from collector_queue import DeliveryStore, RawFrameQueue, SequenceCounter  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.warn_calls = []
        self.error_calls = []

    def warn(self, *args, **kwargs):
        self.warn_calls.append((args, kwargs))
        return True

    def error(self, *args, **kwargs):
        self.error_calls.append((args, kwargs))
        return True

    def __getattr__(self, _name):
        return lambda *args, **kwargs: True


def signs_frame(value):
    frame = bytearray(206)
    frame[0] = 0x0B
    frame[4] = 0xB0
    frame[5:] = bytes([value & 0xFF]) * 201
    return bytes(frame)


class CloudAckTests(unittest.TestCase):
    def test_deliberate_disconnect_does_not_report_listener_ebadf(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, _published = self.make_cloud(temp)

            class ClosingClient:
                def wait_msg(inner_self):
                    cloud._mark_disconnected()
                    raise OSError(9)

                def close(inner_self):
                    return None

            client = ClosingClient()
            cloud.client = client
            cloud.connected = True
            cloud.connection_generation = 1
            cloud._listen_worker(1)
            self.assertEqual(cloud.logger.warn_calls, [])

    def test_umqtt_loader_patches_reduced_log_and_discards_partial_import(self):
        with tempfile.TemporaryDirectory() as temp:
            module_path = Path(temp) / "umqtt.py"
            module_path.write_text(
                "import log\n"
                "log.basicConfig(level=log.INFO)\n"
                "mqtt_log = log.getLogger('MQTT')\n"
                "mqtt_log.info('ready')\n"
                "mqtt_log.warning('warning')\n"
                "class MQTTClient:\n"
                "    pass\n",
                encoding="utf-8",
            )
            fake_log = types.ModuleType("log")
            old_log = sys.modules.get("log")
            old_umqtt = sys.modules.get("umqtt")
            sys.modules["log"] = fake_log
            sys.modules["umqtt"] = types.ModuleType("umqtt")
            sys.path.insert(0, temp)
            try:
                mqtt_client = _load_mqtt_client()
                self.assertEqual(mqtt_client.__name__, "MQTTClient")
                self.assertTrue(hasattr(fake_log, "basicConfig"))
                self.assertEqual(fake_log.INFO, 20)
                self.assertTrue(hasattr(fake_log, "getLogger"))
            finally:
                sys.path.remove(temp)
                sys.modules.pop("umqtt", None)
                sys.modules.pop("log", None)
                if old_log is not None:
                    sys.modules["log"] = old_log
                if old_umqtt is not None:
                    sys.modules["umqtt"] = old_umqtt

    def test_hmac_sha256_matches_standard_library(self):
        expected = hmac.new(b"secret", b"content", hashlib.sha256).hexdigest()
        self.assertEqual(hmac_sha256_hex("secret", "content"), expected)

    def test_hmac_does_not_require_mutable_bytearray(self):
        original = hmac_sha256_hex.__globals__.get("bytearray")

        class RejectedBytearray:
            def __init__(self, *_args, **_kwargs):
                raise TypeError("bytearray mutation is unavailable")

        hmac_sha256_hex.__globals__["bytearray"] = RejectedBytearray
        try:
            expected = hmac.new(b"secret", b"content", hashlib.sha256).hexdigest()
            self.assertEqual(hmac_sha256_hex("secret", "content"), expected)
        finally:
            if original is None:
                hmac_sha256_hex.__globals__.pop("bytearray", None)
            else:
                hmac_sha256_hex.__globals__["bytearray"] = original

    def make_cloud(self, temp):
        config = DeviceConfig(
            {
                "productKey": "pk",
                "deviceName": "dn",
                "mqttServer": "example.mqtt.iothub.aliyuncs.com",
                "deviceSecret": "secret",
            }
        )
        ram = RawFrameQueue(8)
        sequence = SequenceCounter(path=str(Path(temp) / "seq.json"), save_every=100)
        delivery = DeliveryStore(ram, sequence)
        cloud = CloudClient(config, delivery, FakeLogger())
        published = []
        cloud.publish_raw = lambda topic, payload, qos=0: published.append(
            (topic, payload, qos)
        ) or True
        return cloud, delivery, published

    def test_ack_reply_commits_in_order(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, _published = self.make_cloud(temp)
            self.assertTrue(delivery.accept(signs_frame(1), 1000))
            self.assertTrue(delivery.accept(signs_frame(2), 1001))
            first = delivery.next_record({})
            second = delivery.next_record({})
            self.assertTrue(cloud._send_record(first))
            self.assertTrue(cloud._send_record(second))
            cloud._handle_property_reply({"id": "1", "code": 200})
            self.assertEqual(delivery.ram.depth(), 2)
            cloud._handle_property_reply({"id": "0", "code": 200})
            self.assertEqual(delivery.ram.depth(), 0)
            self.assertEqual(cloud.stats()["inflight"], 0)
            self.assertEqual(cloud.stats()["post_reply_success"], 2)

    def test_ota_mode_clears_business_work_and_keeps_raw_mqtt_available(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, published = self.make_cloud(temp)
            self.assertTrue(delivery.accept(signs_frame(1), 1000))
            record = delivery.next_record({})
            self.assertTrue(cloud._send_record(record))
            cloud.defer_properties({"Temperature": 20})
            dropped = cloud.enter_ota_mode()
            self.assertEqual(dropped, 2)
            self.assertEqual(cloud.stats()["inflight"], 0)
            self.assertEqual(cloud.stats()["deferred"], 0)
            self.assertEqual(cloud.stats()["ota_exclusive"], 1)
            self.assertFalse(cloud.publish_properties({"Temperature": 21}))
            self.assertFalse(cloud.defer_properties({"Temperature": 22}))
            cloud._delivery_cycle()
            self.assertEqual(len(published), 1)
            self.assertTrue(cloud.exit_ota_mode())
            self.assertEqual(cloud.stats()["ota_exclusive"], 0)

    def test_timeout_reuses_the_exact_message_id_and_payload(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, published = self.make_cloud(temp)
            self.assertTrue(delivery.accept(signs_frame(3), 1000))
            record = delivery.next_record({})
            self.assertTrue(cloud._send_record(record))
            first_payload = published[-1][1]
            cloud.inflight["0"]["sent_ms"] = ticks_ms() - 16000
            cloud._check_timeouts()
            self.assertEqual(published[-1][1], first_payload)
            self.assertEqual(cloud.inflight["0"]["retries"], 1)

    def test_failed_and_duplicate_replies_do_not_drop_data(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, _published = self.make_cloud(temp)
            self.assertTrue(delivery.accept(signs_frame(4), 1000))
            record = delivery.next_record({})
            self.assertTrue(cloud._send_record(record))
            cloud._handle_property_reply({"id": "0", "code": 500})
            self.assertEqual(delivery.ram.depth(), 1)
            self.assertEqual(cloud.stats()["inflight"], 1)
            cloud._handle_property_reply(
                {
                    "id": "0",
                    "code": 200,
                    "message": "success",
                    "data": {"SignsData": "6311: tsl parse failed"},
                }
            )
            self.assertEqual(delivery.ram.depth(), 1)
            self.assertEqual(cloud.stats()["inflight"], 1)
            self.assertEqual(cloud.stats()["post_reply_rejected"], 2)
            cloud._handle_property_reply({"id": "0", "code": 200})
            self.assertEqual(delivery.ram.depth(), 0)
            cloud._handle_property_reply({"id": "0", "code": 200})
            self.assertEqual(delivery.ram.depth(), 0)

    def test_disconnect_releases_unconfirmed_ram_records(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, _published = self.make_cloud(temp)
            cloud.connected = True
            self.assertTrue(delivery.accept(signs_frame(5), 1000))
            record = delivery.next_record({})
            self.assertTrue(cloud._send_record(record))
            cloud._mark_disconnected()
            self.assertEqual(cloud.stats()["inflight"], 0)
            record = delivery.next_record({})
            self.assertEqual(record["seq"], 0)

    def test_custom_event_uses_direct_alink_params_ack_and_numeric_id(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            cloud.connected = True
            self.assertTrue(cloud.publish_event("runtimeLog", {"seq": 7}))
            payload = json.loads(published[-1][1])
            self.assertTrue(payload["id"].isdigit())
            self.assertEqual(payload["params"], {"seq": 7})
            self.assertNotIn("value", payload["params"])
            self.assertEqual(payload["sys"], {"ack": 1})
            self.assertEqual(payload["method"], "thing.event.runtimeLog.post")

    def test_custom_event_reply_and_timeout_retry_are_tracked(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            cloud.connected = True
            self.assertTrue(cloud.publish_event("debugLog", {"seq": 8}))
            payload = published[-1][1]
            message_id = json.loads(payload)["id"]
            cloud.event_inflight[message_id]["sent_ms"] = ticks_ms() - 16000
            cloud._check_event_timeouts()
            self.assertEqual(published[-1][1], payload)
            self.assertEqual(cloud.stats()["event_retry_count"], 1)
            cloud._handle_event_reply("debugLog", {"id": message_id, "code": 200})
            self.assertEqual(cloud.stats()["event_post_success"], 1)
            self.assertEqual(cloud.stats()["event_inflight"], 0)

            cloud.last_event_publish_ms = None
            self.assertTrue(cloud.publish_event("runtimeLog", {"seq": 9}))
            rejected_id = json.loads(published[-1][1])["id"]
            cloud._handle_event_reply(
                "runtimeLog", {"id": rejected_id, "code": 6300, "message": "bad"}
            )
            self.assertEqual(cloud.stats()["event_post_rejected"], 1)
            cloud._handle_event_reply("runtimeLog", {"id": "missing", "code": 200})
            self.assertEqual(cloud.stats()["event_post_unmatched"], 1)

    def test_uart_sample_property_set_persists_replies_and_reports_state(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            with patch("collector_cloud.save_uart_sample_log") as save:
                cloud._handle_property_set(
                    {
                        "id": "17",
                        "params": {"product_information:UartSampleLog": 1},
                    }
                )
            save.assert_called_once_with(True)
            self.assertTrue(cloud.uart_sample_log_enabled)
            reply = json.loads(published[0][1])
            report = json.loads(published[1][1])
            self.assertEqual(reply["code"], 200)
            self.assertEqual(
                report["params"]["product_information:UartSampleLog"]["value"], 1
            )

            published.clear()
            cloud._handle_property_set(
                {"id": "18", "params": {"UartSampleLog": "invalid"}}
            )
            self.assertEqual(json.loads(published[0][1])["code"], 400)
            self.assertTrue(cloud.uart_sample_log_enabled)

    def test_auxiliary_property_reply_is_not_counted_as_unmatched_signs(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            self.assertTrue(cloud.publish_properties({"BatteryVoltage": 12.3}))
            message_id = json.loads(published[-1][1])["id"]
            cloud._handle_property_reply(
                {"id": message_id, "code": 200, "data": {}}
            )
            self.assertEqual(cloud.stats()["post_reply_unmatched"], 0)
            cloud._handle_property_reply(
                {"id": "4294967295", "code": 200, "data": {}}
            )
            self.assertEqual(cloud.stats()["post_reply_unmatched"], 1)

    def test_delivery_cycle_limits_unconfirmed_messages_to_sixteen(self):
        with tempfile.TemporaryDirectory() as temp:
            config = DeviceConfig(
                {
                    "productKey": "pk",
                    "deviceName": "dn",
                    "mqttServer": "example.mqtt.iothub.aliyuncs.com",
                    "deviceSecret": "secret",
                }
            )
            ram = RawFrameQueue(32)
            sequence = SequenceCounter(
                path=str(Path(temp) / "seq.json"), save_every=100
            )
            delivery = DeliveryStore(ram, sequence)
            cloud = CloudClient(config, delivery, FakeLogger())
            cloud.publish_raw = lambda *_args: True
            cloud.connected = True
            for value in range(20):
                self.assertTrue(delivery.accept(signs_frame(value), 1000 + value))
            for _ in range(16):
                cloud.last_signs_publish_ms = None
                cloud._delivery_cycle()
            self.assertEqual(cloud.stats()["inflight"], 16)
            self.assertEqual(delivery.ram.depth(), 20)

    def test_signs_use_alink_reply_ack_with_nonblocking_mqtt_transport(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, published = self.make_cloud(temp)
            self.assertTrue(delivery.accept(signs_frame(6), 1000))
            record = delivery.next_record({})
            self.assertTrue(cloud._send_record(record))
            self.assertEqual(published[-1][2], 0)
            self.assertEqual(delivery.ram.depth(), 1)
            cloud._handle_property_reply({"id": "0", "code": 200, "data": {}})
            self.assertEqual(delivery.ram.depth(), 0)

    def test_delivery_cycle_obeys_signs_publish_interval(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, delivery, published = self.make_cloud(temp)
            cloud.connected = True
            self.assertTrue(delivery.accept(signs_frame(7), 1000))
            cloud.last_signs_publish_ms = ticks_ms()
            cloud._delivery_cycle()
            self.assertEqual(published, [])
            self.assertEqual(cloud.stats()["inflight"], 0)
            cloud.last_signs_publish_ms = None
            cloud._delivery_cycle()
            self.assertEqual(len(published), 1)

    def test_uart_auxiliary_properties_are_published_by_delivery_cycle(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            cloud.connected = True
            self.assertTrue(cloud.defer_properties({"BatteryVoltage": 12.3}))
            self.assertEqual(published, [])
            cloud._delivery_cycle()
            self.assertEqual(cloud.stats()["deferred"], 0)
            payload = json.loads(published[-1][1])
            self.assertEqual(payload["params"]["BatteryVoltage"]["value"], 12.3)

    def test_publish_raw_passes_qos_without_enabling_retain(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, _published = self.make_cloud(temp)
            calls = []

            class FakeMqttClient:
                def publish(self, topic, payload, retain=False, qos=0):
                    calls.append((topic, payload, retain, qos))

            cloud.publish_raw = CloudClient.publish_raw.__get__(cloud, CloudClient)
            cloud.client = FakeMqttClient()
            cloud.connected = True
            self.assertTrue(cloud.publish_raw("topic", "payload", qos=1))
            self.assertEqual(calls, [(b"topic", b"payload", False, 1)])


class CredentialConnectionTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        directory = self.stack.enter_context(tempfile.TemporaryDirectory())
        self.path = str(Path(directory) / "device_secret.json")
        self.stack.enter_context(patch.multiple(
            config_module, DEVICE_SECRET_CACHE_FILE=self.path,
            LEGACY_DEVICE_SECRET_CACHE_FILE=str(Path(directory) / "legacy.json")))
        self.config = DeviceConfig({
            "productKey": "pk", "deviceName": "dn",
            "mqttServer": "example.aliyuncs.com", "productSecret": "product-secret",
        })
        self.cloud = CloudClient(self.config, types.SimpleNamespace(release_all=lambda: None), FakeLogger())
        self.cloud.publish_properties = lambda *_args, **_kwargs: True
        self.stack.enter_context(patch.object(cloud_module, "_thread", None))

    def test_repeated_timeouts_auth4_and_subscribe_failures_never_reregister(self):
        save_cached_device_secret(self.config, "retained-secret")
        original = Path(self.path).read_bytes()
        attempts, passwords = [], []
        class Mqtt:
            def __init__(inner, *args, **kwargs):
                passwords.append(args[4])
            def set_callback(inner, cb):
                pass
            def connect(inner, **kwargs):
                attempts.append(True)
                if len(attempts) <= 6:
                    raise OSError("MQTT Connect Error='4'" if len(attempts) % 2 else "timed out")
            def subscribe(inner, *args):
                if len(attempts) < 10:
                    raise OSError("subscribe timed out")
            def close(inner):
                pass
        readiness = []
        delays = []
        def network_ready():
            readiness.append(True)
            return len(readiness) > 2
        def sleep(ms):
            delays.append(ms)
            if self.cloud.connected:
                self.cloud.stop = True
            self.assertLess(len(delays), 30)
        with patch.object(self.cloud, "_network_ready", side_effect=network_ready), patch.object(
            cloud_module, "_load_mqtt_client", return_value=Mqtt
        ), patch.object(cloud_module, "dynamic_register_secret") as register, patch.object(
            cloud_module, "sleep_ms", side_effect=sleep
        ):
            self.cloud._connection_worker()
        self.assertEqual(len(attempts), 10)
        register.assert_not_called()
        self.assertEqual(Path(self.path).read_bytes(), original)
        self.assertTrue(self.cloud.connected)
        self.assertEqual(self.cloud.connect_failures, 0)
        errors = str(self.cloud.logger.error_calls)
        self.assertIn("mqtt_login", errors)
        self.assertIn("subscribe", errors)
        self.assertNotIn("retained-secret", errors)
        self.assertNotIn("product-secret", errors)
        self.assertTrue(all(password not in errors for password in passwords))
        self.assertIn(300000, delays)

    def test_read_failure_or_corruption_blocks_registration(self):
        for mode in ("io", "corrupt"):
            with self.subTest(mode=mode):
                Path(self.path).write_text("[]")
                with ExitStack() as stack:
                    if mode == "io":
                        stack.enter_context(patch("collector_config.open", side_effect=OSError(5)))
                    register = stack.enter_context(patch.object(cloud_module, "dynamic_register_secret"))
                    with self.assertRaises(CredentialStoreError):
                        self.cloud._get_device_secret()
                    register.assert_not_called()
                    self.assertEqual(self.cloud.connect_stage, "credential_read")

    def test_pending_registration_retries_persistence_before_any_login(self):
        real_write = config_module._write_credential_file
        with patch.object(cloud_module, "dynamic_register_secret", return_value="new-device-key") as register:
            with patch.object(config_module, "_write_credential_file", side_effect=CredentialStoreError("disk full")), patch.object(
                cloud_module, "_load_mqtt_client"
            ) as mqtt:
                for _ in range(5):
                    with self.assertRaises(CredentialStoreError):
                        self.cloud._connect_once()
                self.assertEqual(self.cloud.pending_secret, "new-device-key")
                self.assertEqual(self.cloud.connect_stage, "credential_write")
                mqtt.assert_not_called()
            with patch.object(config_module, "_write_credential_file", wraps=real_write):
                self.assertEqual(self.cloud._get_device_secret(), "new-device-key")
            register.assert_called_once()
        self.assertEqual(self.cloud.pending_secret, "")
        self.assertEqual(config_module.load_cached_device_secret(self.config), "new-device-key")
        # A newly constructed client after reboot reads disk and never registers.
        with patch.object(cloud_module, "dynamic_register_secret") as register:
            fresh = CloudClient(self.config, self.cloud.delivery, FakeLogger())
            self.assertEqual(fresh._get_device_secret(), "new-device-key")
            register.assert_not_called()

    def test_archive_full_does_not_disconnect_existing_config_identity(self):
        self.config.device_secret = "known-key"
        with patch.object(cloud_module, "save_cached_device_secret", side_effect=config_module.CredentialStoreFull("full")), patch.object(
            cloud_module, "dynamic_register_secret"
        ) as register:
            self.assertEqual(self.cloud._get_device_secret(), "known-key")
            register.assert_not_called()
            self.assertTrue(self.cloud.logger.warn_calls)

    def test_full_archive_prevents_new_registration(self):
        with patch.object(cloud_module, "check_credential_capacity", side_effect=config_module.CredentialStoreFull("full")), patch.object(
            cloud_module, "dynamic_register_secret"
        ) as register:
            with self.assertRaises(config_module.CredentialStoreFull):
                self.cloud._get_device_secret()
            register.assert_not_called()

    def test_registration_reply_rejects_non_string_or_empty_values(self):
        for invalid in (None, 42, {}, [], True, "", " ", "secret\n", "a" * 129):
            for nested in (False, True):
                reply = {"deviceSecret": invalid}
                if nested:
                    reply = {"data": reply}
                with self.subTest(invalid=invalid, nested=nested):
                    self.assertEqual(extract_device_secret(json.dumps(reply)), "")
        for root in ([], None, "text", 42):
            self.assertEqual(extract_device_secret(json.dumps(root)), "")
        self.assertEqual(extract_device_secret('{"deviceSecret":"good-key"}'), "good-key")
        self.assertEqual(extract_device_secret('{"data":{"deviceSecret":"good-key"}}'), "good-key")

    def test_connection_error_redacts_all_auth_material(self):
        self.config.device_secret = "direct-key"
        self.cloud.pending_secret = "pending-key"
        self.cloud.auth_secret = "cached-key"
        self.cloud.auth_password = "signed-password"
        raw = "product-secret direct-key pending-key cached-key signed-password"
        self.assertEqual(self.cloud._connect_error_text(OSError(raw)), " ".join(["[redacted]"] * 5))


if __name__ == "__main__":
    unittest.main()
