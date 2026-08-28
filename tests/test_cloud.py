import sys
import json
import hashlib
import hmac
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_cloud import CloudClient, hmac_sha256_hex, ticks_ms  # noqa: E402
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
    def test_hmac_sha256_matches_standard_library(self):
        expected = hmac.new(b"secret", b"content", hashlib.sha256).hexdigest()
        self.assertEqual(hmac_sha256_hex("secret", "content"), expected)

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

    def test_custom_event_uses_alink_value_envelope_and_numeric_id(self):
        with tempfile.TemporaryDirectory() as temp:
            cloud, _delivery, published = self.make_cloud(temp)
            self.assertTrue(cloud.publish_event("runtimeLog", {"seq": 7}))
            payload = json.loads(published[-1][1])
            self.assertTrue(payload["id"].isdigit())
            self.assertEqual(payload["params"]["value"], {"seq": 7})
            self.assertIsInstance(payload["params"]["time"], int)

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
            cloud._delivery_cycle()
            self.assertEqual(cloud.stats()["inflight"], 16)
            self.assertEqual(delivery.ram.depth(), 20)

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


if __name__ == "__main__":
    unittest.main()
