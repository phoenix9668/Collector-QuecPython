"""Bounded RAM queue, power-loss-aware flash spool, and OTA space budget."""

try:
    import ujson as json
except ImportError:
    import json

try:
    import uos as os
except ImportError:
    import os

try:
    import ustruct as struct
except ImportError:
    import struct

try:
    import ubinascii as binascii
except ImportError:
    import binascii

try:
    import _thread

    def new_lock():
        return _thread.allocate_lock()

except ImportError:
    import threading

    def new_lock():
        return threading.Lock()

from collector_config import (
    FILESYSTEM_SAFETY_BYTES,
    FLASH_SPOOL_MAX_BYTES,
    OTA_RESERVE_FILE,
    OTA_RESERVED_BYTES,
    RUNTIME_STORAGE_ALLOWANCE_BYTES,
    SEQUENCE_FILE,
    SIGNS_FRAME_SIZE,
    SPILL_BATCH_FRAMES,
    atomic_json_write,
    ensure_dir,
)


RECORD_MAGIC = b"SQ"
RECORD_VERSION = 2
# Store the sequence as two uint32 values instead of relying on ustruct's
# optional uint64 format.  At full UART line rate this remains monotonic for
# far longer than the service life of the device.
RECORD_HEADER_FORMAT = ">2sBBIIIIH"
RECORD_HEADER_SIZE = struct.calcsize(RECORD_HEADER_FORMAT)
RECORD_SIZE = RECORD_HEADER_SIZE + SIGNS_FRAME_SIZE + 4
PREALLOCATE_BLOCK = 4096
READ_ERROR = object()


def crc32(data):
    try:
        return binascii.crc32(data) & 0xFFFFFFFF
    except Exception:
        value = 0xFFFFFFFF
        for byte in data:
            if not isinstance(byte, int):
                byte = ord(byte)
            value ^= byte
            for _ in range(8):
                mask = -(value & 1)
                value = (value >> 1) ^ (0xEDB88320 & mask)
        return (value ^ 0xFFFFFFFF) & 0xFFFFFFFF


def file_size(path):
    try:
        return int(os.stat(path)[6])
    except Exception:
        return -1


def remove_file(path):
    try:
        os.remove(path)
        return True
    except Exception:
        return False


def filesystem_free_bytes(path="/usr"):
    try:
        values = os.statvfs(path)
        # QuecPython documents the available filesystem capacity as
        # f_bsize * f_bavail (tuple indexes 0 and 3 respectively).
        block_size = int(values[0])
        available = int(values[3])
        return block_size * available
    except Exception:
        # CPython on Windows has no os.statvfs; this branch is only used by
        # host-side validation and does not add a QuecPython dependency.
        try:
            import shutil

            return int(shutil.disk_usage(path).free)
        except Exception:
            return -1


def preallocate(path, size):
    """Allocate real blocks without creating a large temporary bytes object."""
    parent = path.replace("\\", "/").rsplit("/", 1)[0]
    ensure_dir(parent)
    current = file_size(path)
    if current == size:
        return True
    mode = "ab" if current >= 0 and current < size else "wb"
    written = current if mode == "ab" else 0
    block = bytes(bytearray(PREALLOCATE_BLOCK))
    try:
        with open(path, mode) as stream:
            while written < size:
                count = size - written
                if count > PREALLOCATE_BLOCK:
                    count = PREALLOCATE_BLOCK
                stream.write(block[:count])
                written += count
            try:
                stream.flush()
            except Exception:
                pass
        return file_size(path) == size
    except Exception:
        return False


class StorageBudget:
    """Physically reserves OTA space before the signs spool is sized."""

    def __init__(
        self,
        reserve_path=OTA_RESERVE_FILE,
        reserve_bytes=OTA_RESERVED_BYTES,
        safety_bytes=FILESYSTEM_SAFETY_BYTES,
        root="/usr",
    ):
        self.reserve_path = reserve_path
        self.reserve_bytes = int(reserve_bytes)
        self.safety_bytes = int(safety_bytes)
        self.root = root
        self.reserve_ready = False

    def ensure_reserve(self):
        self.reserve_ready = preallocate(self.reserve_path, self.reserve_bytes)
        return self.reserve_ready

    def release_reserve(self):
        if file_size(self.reserve_path) < 0:
            self.reserve_ready = False
            return True
        result = remove_file(self.reserve_path)
        if result:
            self.reserve_ready = False
        return result

    def restore_reserve(self):
        return self.ensure_reserve()

    def spool_budget(self):
        if not self.reserve_ready:
            return 0
        free = filesystem_free_bytes(self.root)
        if free < 0:
            return 0
        # Keep the advertised safety margin intact after critical logs,
        # sequence metadata, queue metadata, and credential cache grow.
        usable = free - self.safety_bytes - RUNTIME_STORAGE_ALLOWANCE_BYTES
        if usable <= RECORD_SIZE:
            return 0
        if usable > FLASH_SPOOL_MAX_BYTES:
            usable = FLASH_SPOOL_MAX_BYTES
        return (usable // RECORD_SIZE) * RECORD_SIZE

    def ota_free_bytes(self):
        reserved = file_size(self.reserve_path)
        free = filesystem_free_bytes(self.root)
        if free < 0:
            return -1
        if reserved > 0:
            free += reserved
        return free


class SequenceCounter:
    """Power-loss-safe monotonic counter using ahead-of-use reservations."""

    def __init__(self, path=SEQUENCE_FILE, initial=0, save_every=256):
        self.path = path
        self.value = int(initial)
        self.reserve_block = max(1, int(save_every))
        self.reserved_until = self.value
        self.lock = new_lock()
        # If power failed between removing the old metadata and renaming the
        # completed temporary file, the temporary reservation is authoritative.
        for candidate in (path, path + ".tmp"):
            try:
                with open(candidate, "r") as stream:
                    data = json.loads(stream.read())
                persisted = int(data.get("next", 0))
                if persisted > self.value:
                    self.value = persisted
            except Exception:
                pass
        self.reserved_until = self.value
        # Reserve the first range during application initialization, before
        # UART parsing starts, so the normal parser path performs no file I/O.
        self._reserve_locked()

    def _reserve_locked(self):
        new_limit = self.value + self.reserve_block
        atomic_json_write(self.path, {"next": new_limit})
        self.reserved_until = new_limit

    def next(self):
        self.lock.acquire()
        try:
            if self.value >= self.reserved_until:
                self._reserve_locked()
            value = self.value
            self.value += 1
            return value
        finally:
            self.lock.release()

    def reserve_if_low(self, margin=64):
        self.lock.acquire()
        try:
            if self.reserved_until - self.value > int(margin):
                return False
            self._reserve_locked()
            return True
        finally:
            self.lock.release()

    def save(self):
        self.lock.acquire()
        try:
            atomic_json_write(self.path, {"next": self.reserved_until})
        finally:
            self.lock.release()


class RawFrameQueue:
    """Preallocated 206-byte FIFO. Records are removed only after ACK/spill."""

    QUEUED = 0
    INFLIGHT = 1
    ACKED = 2
    SPILLING = 3

    def __init__(self, capacity):
        self.capacity = int(capacity)
        self.frames = bytearray(self.capacity * SIGNS_FRAME_SIZE)
        self.sequences = [0] * self.capacity
        self.timestamps = [0] * self.capacity
        self.states = [self.QUEUED] * self.capacity
        self.head = 0
        self.tail = 0
        self.count = 0
        self.lock = new_lock()

    def depth(self):
        return self.count

    def percent(self):
        if not self.capacity:
            return 100
        return (self.count * 100) // self.capacity

    def _copy_frame(self, index):
        start = index * SIGNS_FRAME_SIZE
        return bytes(self.frames[start : start + SIGNS_FRAME_SIZE])

    def push(self, sequence, timestamp_ms, frame):
        if len(frame) != SIGNS_FRAME_SIZE:
            raise ValueError("invalid signs frame size")
        self.lock.acquire()
        try:
            if self.count >= self.capacity:
                return False
            index = self.tail
            start = index * SIGNS_FRAME_SIZE
            self.frames[start : start + SIGNS_FRAME_SIZE] = frame
            self.sequences[index] = int(sequence)
            self.timestamps[index] = int(timestamp_ms)
            self.states[index] = self.QUEUED
            self.tail = (self.tail + 1) % self.capacity
            self.count += 1
            return True
        finally:
            self.lock.release()

    def next_queued(self):
        self.lock.acquire()
        try:
            for offset in range(self.count):
                index = (self.head + offset) % self.capacity
                if self.states[index] == self.QUEUED:
                    return {
                        "source": "ram",
                        "seq": self.sequences[index],
                        "timestamp_ms": self.timestamps[index],
                        "frame": self._copy_frame(index),
                    }
            return None
        finally:
            self.lock.release()

    def mark_inflight(self, sequence):
        self.lock.acquire()
        try:
            for offset in range(self.count):
                index = (self.head + offset) % self.capacity
                if self.sequences[index] == sequence:
                    if self.states[index] != self.QUEUED:
                        return False
                    self.states[index] = self.INFLIGHT
                    return True
            return False
        finally:
            self.lock.release()

    def release(self, sequence):
        self.lock.acquire()
        try:
            for offset in range(self.count):
                index = (self.head + offset) % self.capacity
                if self.sequences[index] == sequence:
                    if self.states[index] == self.INFLIGHT:
                        self.states[index] = self.QUEUED
                    return True
            return False
        finally:
            self.lock.release()

    def release_all(self):
        self.lock.acquire()
        try:
            for offset in range(self.count):
                index = (self.head + offset) % self.capacity
                if self.states[index] == self.INFLIGHT:
                    self.states[index] = self.QUEUED
        finally:
            self.lock.release()

    def ack(self, sequence):
        self.lock.acquire()
        try:
            found = False
            for offset in range(self.count):
                index = (self.head + offset) % self.capacity
                if self.sequences[index] == sequence:
                    self.states[index] = self.ACKED
                    found = True
                    break
            while self.count and self.states[self.head] == self.ACKED:
                self.states[self.head] = self.QUEUED
                self.head = (self.head + 1) % self.capacity
                self.count -= 1
            return found
        finally:
            self.lock.release()

    def tail_batch(self, maximum=SPILL_BATCH_FRAMES):
        self.lock.acquire()
        try:
            return self._tail_batch_locked(maximum)
        finally:
            self.lock.release()

    def _tail_batch_locked(self, maximum):
        result = []
        offset = 1
        while offset <= self.count and len(result) < maximum:
            index = (self.tail - offset) % self.capacity
            if self.states[index] != self.QUEUED:
                break
            result.append(
                {
                    "source": "ram",
                    "seq": self.sequences[index],
                    "timestamp_ms": self.timestamps[index],
                    "frame": self._copy_frame(index),
                }
            )
            self.states[index] = self.SPILLING
            offset += 1
        result.reverse()
        return result

    def drop_tail_batch(self, records):
        self.lock.acquire()
        try:
            return self._drop_tail_batch_locked(records)
        finally:
            self.lock.release()

    def _drop_tail_batch_locked(self, records):
        if len(records) > self.count:
            return False
        for record in reversed(records):
            index = (self.tail - 1) % self.capacity
            if self.states[index] != self.SPILLING:
                return False
            if self.sequences[index] != record["seq"]:
                return False
            self.tail = index
            self.count -= 1
        return True

    def _release_spill_batch_locked(self, records):
        sequences = {}
        for record in records:
            sequences[record["seq"]] = True
        for offset in range(self.count):
            index = (self.head + offset) % self.capacity
            if (
                self.states[index] == self.SPILLING
                and self.sequences[index] in sequences
            ):
                self.states[index] = self.QUEUED

    def release_spill_batch(self, records):
        self.lock.acquire()
        try:
            self._release_spill_batch_locked(records)
        finally:
            self.lock.release()

    def oldest(self):
        self.lock.acquire()
        try:
            if not self.count:
                return None
            return {
                "source": "ram",
                "seq": self.sequences[self.head],
                "timestamp_ms": self.timestamps[self.head],
            }
        finally:
            self.lock.release()


class FlashJournal:
    """Fixed-record circular spool with alternating CRC-protected metadata."""

    def __init__(self, data_path, meta_0, meta_1, allocated_bytes):
        self.data_path = data_path
        self.meta_paths = (meta_0, meta_1)
        self.capacity = int(allocated_bytes) // RECORD_SIZE
        self.head = 0
        self.tail = 0
        self.count = 0
        self.generation = 0
        self.last_seq = -1
        self.acked = {}
        self.corrupt_records = 0
        self.io_errors = 0
        self.peek_failed = False
        self.lock = new_lock()
        if self.capacity > 0:
            self._open_or_create()

    def enabled(self):
        return self.capacity > 0

    def depth(self):
        return self.count

    def _metadata_crc(self, data):
        text = "{},{},{},{},{},{}".format(
            data.get("generation", 0),
            data.get("head", 0),
            data.get("tail", 0),
            data.get("count", 0),
            data.get("capacity", 0),
            data.get("last_seq", -1),
        )
        return crc32(text.encode("utf-8"))

    def _read_meta(self, path):
        try:
            with open(path, "r") as stream:
                data = json.loads(stream.read())
            if int(data.get("crc", -1)) != self._metadata_crc(data):
                return None
            if int(data.get("capacity", 0)) != self.capacity:
                return None
            return data
        except Exception:
            return None

    def _save_meta(self):
        self.generation += 1
        data = {
            "generation": self.generation,
            "head": self.head,
            "tail": self.tail,
            "count": self.count,
            "capacity": self.capacity,
            "last_seq": self.last_seq,
        }
        data["crc"] = self._metadata_crc(data)
        path = self.meta_paths[self.generation & 1]
        atomic_json_write(path, data)

    def _load_meta(self):
        first = self._read_meta(self.meta_paths[0])
        second = self._read_meta(self.meta_paths[1])
        best = first
        if second and (not best or second["generation"] > best["generation"]):
            best = second
        if not best:
            return False
        self.generation = int(best["generation"])
        self.head = int(best["head"])
        self.tail = int(best["tail"])
        self.count = int(best["count"])
        self.last_seq = int(best.get("last_seq", -1))
        return True

    def _open_or_create(self):
        expected = self.capacity * RECORD_SIZE
        ensure_dir(self.data_path.replace("\\", "/").rsplit("/", 1)[0])
        existing = file_size(self.data_path)
        if existing > 0 and existing % RECORD_SIZE == 0:
            self.capacity = existing // RECORD_SIZE
            expected = existing
        elif existing > 0 and existing != expected:
            # Never truncate an existing queue whose layout cannot be proven safe.
            self.capacity = 0
            return
        if existing != expected:
            if not preallocate(self.data_path, expected):
                self.capacity = 0
                return
            self.head = 0
            self.tail = 0
            self.count = 0
            self._save_meta()
            return
        if not self._load_meta():
            if not self._full_recovery_scan():
                self.capacity = 0
        elif not self._reconcile_after_power_loss():
            self.capacity = 0

    def _read_slot(self, index, stream=None):
        try:
            if stream is None:
                with open(self.data_path, "rb") as reader:
                    reader.seek(index * RECORD_SIZE)
                    data = reader.read(RECORD_SIZE)
            else:
                stream.seek(index * RECORD_SIZE)
                data = stream.read(RECORD_SIZE)
        except Exception:
            self.io_errors += 1
            return READ_ERROR
        return self._decode(data)

    def _write_slot(self, stream, index, record):
        stream.seek(index * RECORD_SIZE)
        stream.write(self._encode(record))

    def _clear_slot(self, index, stream=None):
        try:
            if stream is None:
                with open(self.data_path, "r+b") as writer:
                    writer.seek(index * RECORD_SIZE)
                    writer.write(b"\x00\x00")
                    try:
                        writer.flush()
                    except Exception:
                        pass
            else:
                stream.seek(index * RECORD_SIZE)
                stream.write(b"\x00\x00")
            return True
        except Exception:
            self.io_errors += 1
            return False

    def _encode(self, record):
        timestamp_ms = int(record["timestamp_ms"])
        sequence = int(record["seq"])
        sequence_hi = (sequence >> 32) & 0xFFFFFFFF
        sequence_lo = sequence & 0xFFFFFFFF
        time_hi = (timestamp_ms >> 32) & 0xFFFFFFFF
        time_lo = timestamp_ms & 0xFFFFFFFF
        frame = record["frame"]
        header = struct.pack(
            RECORD_HEADER_FORMAT,
            RECORD_MAGIC,
            RECORD_VERSION,
            0,
            sequence_hi,
            sequence_lo,
            time_hi,
            time_lo,
            len(frame),
        )
        body = header + frame
        return body + struct.pack(">I", crc32(body))

    def _decode(self, data):
        if len(data) != RECORD_SIZE or data[:2] != RECORD_MAGIC:
            return None
        body = data[:-4]
        expected = struct.unpack(">I", data[-4:])[0]
        if crc32(body) != expected:
            self.corrupt_records += 1
            return None
        unpacked = struct.unpack(RECORD_HEADER_FORMAT, body[:RECORD_HEADER_SIZE])
        if unpacked[1] != RECORD_VERSION or unpacked[7] != SIGNS_FRAME_SIZE:
            return None
        sequence = (int(unpacked[3]) << 32) | int(unpacked[4])
        timestamp_ms = (int(unpacked[5]) << 32) | int(unpacked[6])
        return {
            "source": "flash",
            "seq": sequence,
            "timestamp_ms": timestamp_ms,
            "frame": bytes(body[RECORD_HEADER_SIZE:]),
        }

    def _full_recovery_scan(self):
        minimum = None
        maximum = None
        count = 0
        try:
            with open(self.data_path, "rb") as stream:
                for index in range(self.capacity):
                    record = self._read_slot(index, stream)
                    if record is READ_ERROR:
                        return False
                    if not record:
                        continue
                    count += 1
                    if minimum is None or record["seq"] < minimum[0]:
                        minimum = (record["seq"], index)
                    if maximum is None or record["seq"] > maximum[0]:
                        maximum = (record["seq"], index)
        except Exception:
            self.io_errors += 1
            return False
        if not count:
            self.count = 0
            self.head = 0
            self.tail = 0
            self.last_seq = -1
        else:
            self.head = minimum[1]
            self.tail = (maximum[1] + 1) % self.capacity
            self.last_seq = maximum[0]
            # Count the occupied physical span, including an invalid slot
            # between valid records. Treating an interior CRC hole as free
            # would let the circular writer wrap early and overwrite the
            # still-valid oldest record.
            self.count = ((maximum[1] - minimum[1]) % self.capacity) + 1
        self._save_meta()
        return True

    def _reconcile_after_power_loss(self):
        changed = False
        try:
            with open(self.data_path, "rb") as stream:
                while self.count:
                    record = self._read_slot(self.head, stream)
                    if record is READ_ERROR:
                        return False
                    if record is not None:
                        break
                    self.head = (self.head + 1) % self.capacity
                    self.count -= 1
                    changed = True
                while self.count < self.capacity:
                    record = self._read_slot(self.tail, stream)
                    if record is READ_ERROR:
                        return False
                    if not record or record["seq"] <= self.last_seq:
                        break
                    self.tail = (self.tail + 1) % self.capacity
                    self.count += 1
                    self.last_seq = record["seq"]
                    changed = True
        except Exception:
            self.io_errors += 1
            return False
        if changed:
            self._save_meta()
        return True

    def append_batch(self, records):
        if not records:
            return True
        self.lock.acquire()
        try:
            if not self.enabled() or len(records) > self.capacity - self.count:
                return False
            try:
                with open(self.data_path, "r+b") as stream:
                    index = self.tail
                    for record in records:
                        self._write_slot(stream, index, record)
                        index = (index + 1) % self.capacity
                    try:
                        stream.flush()
                    except Exception:
                        pass
            except Exception:
                self.io_errors += 1
                return False
            self.tail = index
            self.count += len(records)
            self.last_seq = records[-1]["seq"]
            try:
                self._save_meta()
            except Exception:
                # The records and in-memory cursor are valid. A reboot scan can
                # reconcile them even if this metadata generation was not saved.
                self.io_errors += 1
            return True
        finally:
            self.lock.release()

    def peek(self, excluded=None):
        if excluded is None:
            excluded = {}
        self.lock.acquire()
        try:
            self.peek_failed = False
            index = self.head
            scanned = 0
            try:
                with open(self.data_path, "rb") as stream:
                    while scanned < self.count and scanned < self.capacity:
                        record = self._read_slot(index, stream)
                        if record is READ_ERROR:
                            self.peek_failed = True
                            return None
                        scanned += 1
                        index = (index + 1) % self.capacity
                        if not record:
                            continue
                        sequence = record["seq"]
                        if sequence in self.acked or sequence in excluded:
                            continue
                        return record
            except Exception:
                self.io_errors += 1
                self.peek_failed = True
            return None
        finally:
            self.lock.release()

    def ack(self, sequence):
        self.lock.acquire()
        try:
            self.acked[int(sequence)] = True
            changed = False
            try:
                with open(self.data_path, "r+b") as stream:
                    while self.count:
                        record = self._read_slot(self.head, stream)
                        if record is READ_ERROR:
                            return False
                        if record is None:
                            self.head = (self.head + 1) % self.capacity
                            self.count -= 1
                            changed = True
                            continue
                        if record["seq"] not in self.acked:
                            break
                        self._clear_slot(self.head, stream)
                        del self.acked[record["seq"]]
                        self.head = (self.head + 1) % self.capacity
                        self.count -= 1
                        changed = True
                    try:
                        stream.flush()
                    except Exception:
                        pass
            except Exception:
                self.io_errors += 1
                return False
            if changed:
                try:
                    self._save_meta()
                except Exception:
                    # Cleared/acknowledged records remain safe; dual metadata
                    # plus the startup scan will reconcile after a reboot.
                    self.io_errors += 1
            # An out-of-order ACK is valid even when the durable head cannot yet move.
            return True
        finally:
            self.lock.release()


class DeliveryStore:
    """Coordinates RAM and flash without changing the global sequence order."""

    def __init__(self, ram_queue, sequence, journal=None):
        self.ram = ram_queue
        self.sequence = sequence
        self.journal = journal
        self.accepted = 0
        self.acked = 0
        self.rejected = 0
        self.spilled = 0

    def accept(self, frame, timestamp_ms):
        sequence = self.sequence.next()
        if not self.ram.push(sequence, timestamp_ms, frame):
            self.rejected += 1
            return False
        self.accepted += 1
        return True

    def spill(self, force=False, connected=True, high_watermark=75):
        if not self.journal or not self.journal.enabled():
            return 0
        if not force and connected and self.ram.percent() < high_watermark:
            return 0
        # Keep selection, Flash append, and RAM tail removal atomic with
        # respect to producers. UART bytes continue accumulating in their
        # independent ring while the parser briefly waits for this lock.
        self.ram.lock.acquire()
        try:
            available = self.journal.capacity - self.journal.depth()
            if available <= 0:
                return 0
            maximum = min(SPILL_BATCH_FRAMES, available)
            records = self.ram._tail_batch_locked(maximum)
            if not records:
                return 0
            if not self.journal.append_batch(records):
                self.ram._release_spill_batch_locked(records)
                return 0
            if not self.ram._drop_tail_batch_locked(records):
                self.ram._release_spill_batch_locked(records)
                return 0
            self.spilled += len(records)
            return len(records)
        finally:
            self.ram.lock.release()

    def next_record(self, flash_excluded=None):
        ram_record = self.ram.next_queued()
        flash_record = None
        if self.journal:
            flash_record = self.journal.peek(flash_excluded)
            if self.journal.peek_failed:
                return None
        if ram_record and flash_record:
            record = (
                ram_record
                if ram_record["seq"] < flash_record["seq"]
                else flash_record
            )
        else:
            record = ram_record or flash_record
        if record and record["source"] == "ram":
            if not self.ram.mark_inflight(record["seq"]):
                return None
        return record

    def release(self, sequence, source):
        if source == "ram":
            self.ram.release(sequence)

    def release_all(self):
        self.ram.release_all()

    def acknowledge(self, sequence, source):
        result = False
        if source == "ram":
            result = self.ram.ack(sequence)
        elif self.journal:
            result = self.journal.ack(sequence)
        if result:
            self.acked += 1
        return result

    def flash_depth(self):
        return self.journal.depth() if self.journal else 0

    def oldest_timestamp_ms(self):
        ram_record = self.ram.oldest()
        flash_record = self.journal.peek() if self.journal else None
        if ram_record and flash_record:
            record = (
                ram_record
                if ram_record["seq"] < flash_record["seq"]
                else flash_record
            )
        else:
            record = ram_record or flash_record
        return record["timestamp_ms"] if record else 0

    def stats(self):
        return {
            "accepted": self.accepted,
            "acked": self.acked,
            "rejected": self.rejected,
            "spilled": self.spilled,
            "ram_depth": self.ram.depth(),
            "ram_capacity": self.ram.capacity,
            "flash_depth": self.flash_depth(),
            "flash_capacity": self.journal.capacity if self.journal else 0,
            "flash_corrupt": self.journal.corrupt_records if self.journal else 0,
            "flash_io_errors": self.journal.io_errors if self.journal else 0,
            "oldest_timestamp_ms": self.oldest_timestamp_ms(),
        }
