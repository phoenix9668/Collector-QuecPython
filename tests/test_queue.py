import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import collector_queue as queue_module  # noqa: E402
from collector_config import (  # noqa: E402
    FILESYSTEM_SAFETY_BYTES,
    FLASH_SPOOL_MAX_BYTES,
    RUNTIME_STORAGE_ALLOWANCE_BYTES,
)
from collector_queue import (  # noqa: E402
    DeliveryStore,
    FlashJournal,
    RECORD_SIZE,
    RawFrameQueue,
    SequenceCounter,
    StorageBudget,
)


def frame(value):
    return bytes([value & 0xFF]) * 206


class QueueTests(unittest.TestCase):
    def test_ram_ack_commits_only_contiguous_head(self):
        queue = RawFrameQueue(8)
        for sequence in range(4):
            self.assertTrue(queue.push(sequence, 1000 + sequence, frame(sequence)))
        first = queue.next_queued()
        self.assertEqual(first["seq"], 0)
        self.assertTrue(queue.mark_inflight(0))
        self.assertTrue(queue.mark_inflight(1))
        self.assertTrue(queue.ack(1))
        self.assertEqual(queue.depth(), 4)
        self.assertTrue(queue.ack(0))
        self.assertEqual(queue.depth(), 2)

    def test_flash_spill_survives_reopen_and_preserves_order(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 32,
            )
            records = [
                {"seq": seq, "timestamp_ms": 5000 + seq, "frame": frame(seq)}
                for seq in range(12)
            ]
            self.assertTrue(journal.append_batch(records))
            self.assertEqual(journal.depth(), 12)

            reopened = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 32,
            )
            self.assertEqual(reopened.depth(), 12)
            for sequence in range(12):
                record = reopened.peek()
                self.assertEqual(record["seq"], sequence)
                self.assertEqual(record["frame"], frame(sequence))
                self.assertTrue(reopened.ack(sequence))
            self.assertEqual(reopened.depth(), 0)

    def test_flash_supports_large_persistent_sequences(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 4,
            )
            sequence = (1 << 40) + 7
            self.assertTrue(
                journal.append_batch(
                    [{"seq": sequence, "timestamp_ms": 99, "frame": frame(7)}]
                )
            )
            self.assertEqual(journal.peek()["seq"], sequence)

    def test_sequence_reservation_prevents_id_reuse_after_power_loss(self):
        with tempfile.TemporaryDirectory() as temp:
            path = str(Path(temp) / "seq.json")
            first = SequenceCounter(path=path, save_every=8)
            self.assertEqual([first.next() for _ in range(3)], [0, 1, 2])
            restarted = SequenceCounter(path=path, save_every=8)
            self.assertEqual(restarted.next(), 8)

    def test_sequence_recovers_completed_temporary_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "seq.json"
            (Path(str(path) + ".tmp")).write_text(
                '{"next": 64}', encoding="utf-8"
            )
            counter = SequenceCounter(path=str(path), save_every=8)
            self.assertEqual(counter.next(), 64)

    def test_failed_sequence_reservation_never_uses_unpersisted_ids(self):
        with tempfile.TemporaryDirectory() as temp:
            counter = SequenceCounter(
                path=str(Path(temp) / "seq.json"), save_every=8
            )
            self.assertEqual([counter.next() for _ in range(8)], list(range(8)))
            with patch.object(
                queue_module,
                "atomic_json_write",
                side_effect=OSError("disk full"),
            ):
                with self.assertRaises(OSError):
                    counter.next()
            self.assertEqual(counter.value, 8)
            self.assertEqual(counter.reserved_until, 8)
            self.assertEqual(counter.next(), 8)

    def test_flash_out_of_order_ack_is_retained(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 8,
            )
            records = [
                {"seq": seq, "timestamp_ms": seq, "frame": frame(seq)}
                for seq in range(3)
            ]
            self.assertTrue(journal.append_batch(records))
            self.assertTrue(journal.ack(1))
            self.assertEqual(journal.depth(), 3)
            self.assertTrue(journal.ack(0))
            self.assertEqual(journal.depth(), 1)
            self.assertEqual(journal.peek()["seq"], 2)

    def test_delivery_store_spills_without_dropping(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            ram = RawFrameQueue(8)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 32,
            )
            sequence = SequenceCounter(path=str(root / "seq.json"), save_every=100)
            delivery = DeliveryStore(ram, sequence, journal)
            for value in range(8):
                self.assertTrue(delivery.accept(frame(value), value))
            self.assertEqual(delivery.spill(force=True), 8)
            self.assertEqual(delivery.stats()["ram_depth"], 0)
            self.assertEqual(delivery.stats()["flash_depth"], 8)
            for value in range(8):
                record = delivery.next_record({})
                self.assertEqual(record["seq"], value)
                self.assertTrue(delivery.acknowledge(value, "flash"))
            self.assertEqual(delivery.stats()["flash_depth"], 0)

    def test_final_flash_batch_uses_remaining_capacity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            ram = RawFrameQueue(8)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 5,
            )
            sequence = SequenceCounter(path=str(root / "seq.json"), save_every=16)
            delivery = DeliveryStore(ram, sequence, journal)
            for value in range(8):
                self.assertTrue(delivery.accept(frame(value), value))
            self.assertEqual(delivery.spill(force=True), 5)
            self.assertEqual(journal.depth(), 5)
            self.assertEqual(ram.depth(), 3)
            self.assertEqual(journal.peek()["seq"], 3)

    def test_interrupted_flash_batch_is_reconciled(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            paths = (str(root / "meta.0"), str(root / "meta.1"))
            journal = FlashJournal(
                str(root / "queue.bin"), paths[0], paths[1], RECORD_SIZE * 8
            )
            record = {"seq": 12, "timestamp_ms": 34, "frame": frame(5)}
            with open(journal.data_path, "r+b") as stream:
                journal._write_slot(stream, journal.tail, record)
                stream.flush()
            reopened = FlashJournal(
                journal.data_path, paths[0], paths[1], RECORD_SIZE * 8
            )
            self.assertEqual(reopened.depth(), 1)
            self.assertEqual(reopened.peek()["seq"], 12)

    def test_corrupt_metadata_scans_records_and_bad_crc_is_not_delivered(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            data_path = str(root / "queue.bin")
            meta_0 = str(root / "meta.0")
            meta_1 = str(root / "meta.1")
            journal = FlashJournal(data_path, meta_0, meta_1, RECORD_SIZE * 4)
            self.assertTrue(
                journal.append_batch(
                    [
                        {"seq": 1, "timestamp_ms": 1, "frame": frame(1)},
                        {"seq": 2, "timestamp_ms": 2, "frame": frame(2)},
                    ]
                )
            )
            Path(meta_0).write_text("broken", encoding="utf-8")
            Path(meta_1).write_text("broken", encoding="utf-8")
            recovered = FlashJournal(data_path, meta_0, meta_1, RECORD_SIZE * 4)
            self.assertEqual(recovered.depth(), 2)
            with open(data_path, "r+b") as stream:
                stream.seek(RECORD_SIZE + 30)
                original = stream.read(1)
                stream.seek(RECORD_SIZE + 30)
                stream.write(bytes([original[0] ^ 0xFF]))
            Path(meta_0).write_text("broken", encoding="utf-8")
            Path(meta_1).write_text("broken", encoding="utf-8")
            recovered = FlashJournal(data_path, meta_0, meta_1, RECORD_SIZE * 4)
            self.assertEqual(recovered.depth(), 1)
            self.assertEqual(recovered.peek()["seq"], 1)
            self.assertGreater(recovered.corrupt_records, 0)

    def test_interior_crc_hole_never_becomes_overwrite_capacity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            data_path = str(root / "queue.bin")
            meta_0 = str(root / "meta.0")
            meta_1 = str(root / "meta.1")
            journal = FlashJournal(data_path, meta_0, meta_1, RECORD_SIZE * 5)
            self.assertTrue(
                journal.append_batch(
                    [
                        {"seq": seq, "timestamp_ms": seq, "frame": frame(seq)}
                        for seq in (1, 2, 3)
                    ]
                )
            )
            with open(data_path, "r+b") as stream:
                stream.seek(RECORD_SIZE + 30)
                original = stream.read(1)
                stream.seek(RECORD_SIZE + 30)
                stream.write(bytes([original[0] ^ 0xFF]))
            Path(meta_0).write_text("broken", encoding="utf-8")
            Path(meta_1).write_text("broken", encoding="utf-8")

            recovered = FlashJournal(data_path, meta_0, meta_1, RECORD_SIZE * 5)
            self.assertEqual(recovered.depth(), 3)
            self.assertEqual(recovered.peek()["seq"], 1)
            self.assertTrue(
                recovered.append_batch(
                    [
                        {"seq": seq, "timestamp_ms": seq, "frame": frame(seq)}
                        for seq in (4, 5)
                    ]
                )
            )
            self.assertFalse(
                recovered.append_batch(
                    [{"seq": 6, "timestamp_ms": 6, "frame": frame(6)}]
                )
            )
            self.assertEqual(recovered.peek()["seq"], 1)

    def test_full_flash_queue_preserves_oldest_records(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 2,
            )
            first = [
                {"seq": value, "timestamp_ms": value, "frame": frame(value)}
                for value in range(2)
            ]
            self.assertTrue(journal.append_batch(first))
            self.assertFalse(
                journal.append_batch(
                    [{"seq": 2, "timestamp_ms": 2, "frame": frame(2)}]
                )
            )
            self.assertEqual(journal.depth(), 2)
            self.assertEqual(journal.peek()["seq"], 0)

    def test_flash_read_error_does_not_advance_or_send_newer_ram_data(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            journal = FlashJournal(
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
                RECORD_SIZE * 4,
            )
            self.assertTrue(
                journal.append_batch(
                    [{"seq": 0, "timestamp_ms": 0, "frame": frame(0)}]
                )
            )
            ram = RawFrameQueue(4)
            sequence = SequenceCounter(
                path=str(root / "seq.json"), initial=1, save_every=8
            )
            delivery = DeliveryStore(ram, sequence, journal)
            self.assertTrue(delivery.accept(frame(1), 1))
            with patch("builtins.open", side_effect=OSError("read failed")):
                self.assertIsNone(delivery.next_record({}))
            self.assertEqual(journal.depth(), 1)
            self.assertEqual(ram.depth(), 1)
            self.assertGreater(journal.io_errors, 0)

    def test_empty_flash_journal_can_be_reclaimed_and_recreated_for_ota(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            paths = (
                str(root / "queue.bin"),
                str(root / "meta.0"),
                str(root / "meta.1"),
            )
            journal = FlashJournal(*paths, RECORD_SIZE * 8)
            delivery = DeliveryStore(
                RawFrameQueue(8),
                SequenceCounter(path=str(root / "seq.json"), save_every=8),
                journal,
            )
            self.assertTrue(
                journal.append_batch(
                    [{"seq": 0, "timestamp_ms": 0, "frame": frame(0)}]
                )
            )
            self.assertFalse(delivery.detach_empty_journal())
            self.assertTrue(journal.ack(0))
            self.assertTrue(delivery.detach_empty_journal())
            self.assertIsNone(delivery.journal)
            self.assertFalse(any(Path(path).exists() for path in paths))

            recreated = FlashJournal(*paths, RECORD_SIZE * 8)
            self.assertTrue(delivery.attach_journal(recreated))
            self.assertIs(delivery.journal, recreated)

    def test_576_kib_partition_uses_dynamic_bounded_spool(self):
        budget = StorageBudget(
            reserve_path="unused.reserve",
            reserve_bytes=256 * 1024,
            safety_bytes=FILESYSTEM_SAFETY_BYTES,
            root="/usr",
        )
        budget.reserve_ready = True
        # Approximate free space after the 256 KiB reserve and /usr/log
        # directory have been created on the measured 576 KiB partition.
        free_after_reserve = 132 * 1024
        expected = (
            free_after_reserve
            - FILESYSTEM_SAFETY_BYTES
            - RUNTIME_STORAGE_ALLOWANCE_BYTES
        )
        expected = (expected // RECORD_SIZE) * RECORD_SIZE
        with patch.object(
            queue_module,
            "filesystem_free_bytes",
            return_value=free_after_reserve,
        ):
            self.assertEqual(budget.spool_budget(), expected)
            self.assertLessEqual(budget.spool_budget(), FLASH_SPOOL_MAX_BYTES)

    def test_ota_reserve_is_physically_allocated_and_restored(self):
        with tempfile.TemporaryDirectory() as temp:
            reserve = str(Path(temp) / "ota.reserve")
            budget = StorageBudget(
                reserve_path=reserve,
                reserve_bytes=64 * 1024,
                safety_bytes=4096,
                root=temp,
            )
            self.assertTrue(budget.ensure_reserve())
            self.assertEqual(Path(reserve).stat().st_size, 64 * 1024)
            self.assertGreater(budget.spool_budget(), 0)
            self.assertTrue(budget.release_reserve())
            self.assertFalse(Path(reserve).exists())
            self.assertTrue(budget.restore_reserve())
            self.assertEqual(Path(reserve).stat().st_size, 64 * 1024)


if __name__ == "__main__":
    unittest.main()
