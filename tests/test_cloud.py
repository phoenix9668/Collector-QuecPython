import sys
import json
import hashlib
import hmac
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_cloud import (  # noqa: E402
    CloudClient,
    _load_mqtt_client,
    hmac_sha256_hex,
    ticks_ms,
)
from collector_config import DeviceConfig  # noqa: E402
from collector_queue import DeliveryStore, RawFrameQueue, SequenceCounter  # noqa: E402


class FakeLogger:
    def __getattr__(self, _name):
        return lambda *args, **kwargs: True


def signs_frame(value):
    frame = bytearray(206)
    frame[0] = 0x0B
    frame[4] = 0xB0
    frame[5:] = bytes([value & 0xFF]) * 201
    return bytes(frame)


class CloudAckTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
