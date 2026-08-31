"""High-throughput UART framing and legacy SignsData encoding."""

try:
    import utime as time
except ImportError:
    import time

try:
    import _thread

    def new_lock():
        return _thread.allocate_lock()

except ImportError:
    import threading

    def new_lock():
        return threading.Lock()


SIGNS_FRAME_SIZE = 206
VOLTAGE_FRAME_SIZE = 9
FRAME_PREFIX = 0x0B
SIGNS_TYPE = 0xB0
VOLTAGE_TYPE = 0xB2
INFO_MARKER = b"##"
MAX_INFO_SIZE = 512
UART_SAMPLE_SIZE = 48


def now_ms():
    try:
        return int(time.time() * 1000)
    except Exception:
        try:
            return int(time.ticks_ms())
        except Exception:
            return 0


class ByteRing:
    """A fixed byte ring. It never overwrites unread UART data."""

    def __init__(self, capacity):
        self.capacity = int(capacity)
        # EC600M R06A06M08 can construct bytearray objects while rejecting
        # item and slice assignment. Probe before allocating the full 32 KiB
        # buffer; affected firmware uses a bounded, preallocated ring of
        # immutable UART segments instead.
        probe = bytearray(1)
        try:
            probe[0:1] = b"\x00"
            self.storage_mode = "bytearray"
        except Exception:
            self.storage_mode = "segments"

        if self.storage_mode == "bytearray":
            self.buffer = bytearray(self.capacity)
            self.segments = None
            self.segment_capacity = 0
        else:
            self.buffer = None
            # RX DMA normally reports 64-byte blocks and the timeout event a
            # smaller tail. One slot per 32 bytes provides fragmentation
            # headroom without an unbounded callback-time list allocation.
            self.segment_capacity = max(128, self.capacity // 32)
            self.segments = [None] * self.segment_capacity
        self.segment_head = 0
        self.segment_tail = 0
        self.segment_count = 0
        self.segment_offset = 0
        self.head = 0
        self.tail = 0
        self.count = 0

    def free(self):
        return self.capacity - self.count

    def write(self, data):
        length = len(data)
        accepted = length
        if accepted > self.free():
            accepted = self.free()
        if self.storage_mode == "segments":
            if accepted <= 0 or self.segment_count >= self.segment_capacity:
                return 0
            if isinstance(data, bytes):
                segment = data
            else:
                segment = bytes(data)
            if accepted < length:
                segment = segment[:accepted]
            self.segments[self.segment_tail] = segment
            self.segment_tail = (
                self.segment_tail + 1
            ) % self.segment_capacity
            self.segment_count += 1
            self.count += accepted
            return accepted
        first = accepted
        right = self.capacity - self.tail
        if first > right:
            first = right
        try:
            source = memoryview(data)
        except Exception:
            source = data
        if first:
            self.buffer[self.tail : self.tail + first] = source[:first]
        second = accepted - first
        if second:
            self.buffer[0:second] = source[first : first + second]
        self.tail = (self.tail + accepted) % self.capacity
        self.count += accepted
        return accepted

    def peek_byte(self, offset=0):
        if offset < 0 or offset >= self.count:
            raise IndexError("ring offset")
        if self.storage_mode == "segments":
            remaining = offset
            index = self.segment_head
            inner = self.segment_offset
            segments_left = self.segment_count
            while segments_left:
                segment = self.segments[index]
                available = len(segment) - inner
                if remaining < available:
                    value = segment[inner + remaining]
                    return value if isinstance(value, int) else ord(value)
                remaining -= available
                index = (index + 1) % self.segment_capacity
                inner = 0
                segments_left -= 1
            raise IndexError("segment ring offset")
        return self.buffer[(self.head + offset) % self.capacity]

    def read(self, length):
        if length > self.count:
            length = self.count
        if self.storage_mode == "segments":
            remaining = length
            parts = []
            while remaining:
                segment = self.segments[self.segment_head]
                available = len(segment) - self.segment_offset
                take = remaining if remaining < available else available
                start = self.segment_offset
                if start == 0 and take == len(segment):
                    parts.append(segment)
                else:
                    parts.append(segment[start : start + take])
                self._discard_segments(take)
                remaining -= take
            if not parts:
                return b""
            if len(parts) == 1:
                return bytes(parts[0])
            return b"".join(parts)
        output = bytearray(length)
        first = length
        right = self.capacity - self.head
        if first > right:
            first = right
        if first:
            output[:first] = self.buffer[self.head : self.head + first]
        second = length - first
        if second:
            output[first:] = self.buffer[:second]
        self.discard(length)
        return bytes(output)

    def discard(self, length):
        if length > self.count:
            length = self.count
        if self.storage_mode == "segments":
            discarded = length
            while length:
                segment = self.segments[self.segment_head]
                available = len(segment) - self.segment_offset
                take = length if length < available else available
                self._discard_segments(take)
                length -= take
            return discarded
        self.head = (self.head + length) % self.capacity
        self.count -= length
        return length

    def _discard_segments(self, length):
        segment = self.segments[self.segment_head]
        available = len(segment) - self.segment_offset
        self.count -= length
        if length < available:
            self.segment_offset += length
            return
        self.segments[self.segment_head] = None
        self.segment_head = (self.segment_head + 1) % self.segment_capacity
        self.segment_count -= 1
        self.segment_offset = 0

    def find_pair(self, first, second, start=0, limit=None):
        if limit is None or limit > self.count:
            limit = self.count
        index = start
        while index + 1 < limit:
            if self.peek_byte(index) == first and self.peek_byte(index + 1) == second:
                return index
            index += 1
        return -1


class FrameStream:
    """Separates callback-time byte copying from parser-time frame extraction."""

    def __init__(self, capacity=32 * 1024):
        self.ring = ByteRing(capacity)
        self.lock = new_lock()
        self.rx_bytes = 0
        self.rx_overflow_bytes = 0
        self.signs_frames = 0
        self.voltage_frames = 0
        self.info_frames = 0
        self.sync_errors = 0
        self.rx_sample_id = 0
        self.rx_sample_size = 0
        self.rx_sample = b""
        self.frame_sample_id = 0
        self.frame_sample_kind = ""
        self.frame_sample_size = 0
        self.frame_sample = b""

    def feed(self, data):
        if not data:
            return 0
        self.lock.acquire()
        try:
            self.rx_bytes += len(data)
            # Keep one small bounded snapshot for the 60-second diagnostic
            # worker. Hex conversion and printing stay outside this callback.
            self.rx_sample_id += 1
            self.rx_sample_size = len(data)
            self.rx_sample = bytes(data[:UART_SAMPLE_SIZE])
            accepted = self.ring.write(data)
            self.rx_overflow_bytes += len(data) - accepted
            return accepted
        finally:
            self.lock.release()

    def _extract_one_locked(self):
        while self.ring.count:
            first = self.ring.peek_byte(0)
            if first == INFO_MARKER[0]:
                if self.ring.count < 2:
                    return None
                if self.ring.peek_byte(1) == INFO_MARKER[1]:
                    end = self.ring.find_pair(
                        INFO_MARKER[0], INFO_MARKER[1], 2, MAX_INFO_SIZE
                    )
                    if end < 0:
                        if self.ring.count <= MAX_INFO_SIZE:
                            return None
                        self.ring.discard(1)
                        self.sync_errors += 1
                        continue
                    info = self.ring.read(end + 2)
                    self.info_frames += 1
                    self._record_frame_sample("info", info)
                    return ("info", info, now_ms())

            if first != FRAME_PREFIX:
                self.ring.discard(1)
                self.sync_errors += 1
                continue
            if self.ring.count < 5:
                return None

            frame_type = self.ring.peek_byte(4)
            if frame_type == SIGNS_TYPE:
                if self.ring.count < SIGNS_FRAME_SIZE:
                    break
                frame = self.ring.read(SIGNS_FRAME_SIZE)
                self.signs_frames += 1
                self._record_frame_sample("B0", frame)
                return ("signs", frame, now_ms())
            if frame_type == VOLTAGE_TYPE:
                if self.ring.count < VOLTAGE_FRAME_SIZE:
                    break
                frame = self.ring.read(VOLTAGE_FRAME_SIZE)
                self.voltage_frames += 1
                self._record_frame_sample("B2", frame)
                return ("voltage", frame, now_ms())

            self.ring.discard(1)
            self.sync_errors += 1
        return None

    def _record_frame_sample(self, kind, frame):
        self.frame_sample_id += 1
        self.frame_sample_kind = kind
        self.frame_sample_size = len(frame)
        self.frame_sample = bytes(frame[:UART_SAMPLE_SIZE])

    def drain(self, signs_callback, voltage_callback=None, info_callback=None, limit=0):
        processed = 0
        while not limit or processed < limit:
            self.lock.acquire()
            try:
                item = self._extract_one_locked()
            finally:
                self.lock.release()
            if item is None:
                break
            kind, frame, captured_ms = item
            processed += 1
            # Never invoke application code while holding the UART ring lock.
            if kind == "signs":
                signs_callback(frame, captured_ms)
            elif kind == "voltage" and voltage_callback:
                voltage_callback(frame, captured_ms)
            elif kind == "info" and info_callback:
                info_callback(frame, captured_ms)
        return processed

    def stats(self):
        self.lock.acquire()
        try:
            return {
                "rx_bytes": self.rx_bytes,
                "rx_overflow_bytes": self.rx_overflow_bytes,
                "signs_frames": self.signs_frames,
                "voltage_frames": self.voltage_frames,
                "info_frames": self.info_frames,
                "sync_errors": self.sync_errors,
                "ring_depth": self.ring.count,
                "ring_capacity": self.ring.capacity,
                "ring_mode": self.ring.storage_mode,
                "rx_sample_id": self.rx_sample_id,
                "rx_sample_size": self.rx_sample_size,
                "rx_sample": self.rx_sample,
                "frame_sample_id": self.frame_sample_id,
                "frame_sample_kind": self.frame_sample_kind,
                "frame_sample_size": self.frame_sample_size,
                "frame_sample": self.frame_sample,
            }
        finally:
            self.lock.release()


def legacy_hex(data):
    """Preserve the historical leading-space, lowercase, unpadded format."""
    result = ""
    for value in data:
        if not isinstance(value, int):
            value = ord(value)
        result += " " + hex(value)[2:]
    return result


def hex_preview(data):
    """Format a bounded diagnostic snapshot as unambiguous two-digit hex."""
    values = []
    for value in data:
        if not isinstance(value, int):
            value = ord(value)
        values.append("%02x" % value)
    return " ".join(values)


def battery_pct(raw):
    value = (2 * (float(raw) / 4096) * 3 - 3.0) / (4.2 - 3.0)
    if value >= 1.0:
        value = 1.0
    elif value < 0.0:
        value = 0.0
    return float("%.2f" % value)


def calc_rssi_dbm(raw):
    if raw >= 128:
        value = (raw - 256) / 2.0 - 74
    else:
        value = raw / 2.0 - 74
    return float("%.2f" % value)


def decode_signs_frame(frame, captured_ms):
    if len(frame) != SIGNS_FRAME_SIZE:
        raise ValueError("SignsData frame must be 206 bytes")
    if frame[0] != FRAME_PREFIX or frame[4] != SIGNS_TYPE:
        raise ValueError("invalid SignsData header")
    return {
        "collar_information:CollectorID": {"value": legacy_hex(frame[0:4])},
        "collar_information:RFID": {"value": legacy_hex(frame[5:11])},
        "collar_information:GUID": {"value": legacy_hex(frame[11:43])},
        "collar_information:restArray": {"value": legacy_hex(frame[43:67])},
        "collar_information:ingestionArray": {"value": legacy_hex(frame[67:91])},
        "collar_information:movementArray": {"value": legacy_hex(frame[91:115])},
        "collar_information:climbArray": {"value": legacy_hex(frame[115:139])},
        "collar_information:ruminateArray": {"value": legacy_hex(frame[139:163])},
        "collar_information:otherArray": {"value": legacy_hex(frame[163:187])},
        "collar_information:Stage": {"value": int(frame[187])},
        "collar_information:BatteryVoltage": {
            "value": battery_pct((frame[188] << 8) | frame[189])
        },
        "collar_information:ResetCnt": {
            "value": (frame[190] << 8) | frame[191]
        },
        "collar_information:SignalStrength": {
            "value": calc_rssi_dbm(frame[192])
        },
        "collar_information:UTCtime": {"value": str(int(captured_ms))},
    }


def decode_voltage_frame(frame):
    if len(frame) != VOLTAGE_FRAME_SIZE:
        raise ValueError("voltage frame must be 9 bytes")
    return {
        "BatteryVoltage": float(
            "%.3f" % (((frame[5] << 8) | frame[6]) / 32768.0 * 4.096 * 11)
        ),
        "PowerVoltage": float(
            "%.3f" % (((frame[7] << 8) | frame[8]) / 32768.0 * 4.096 * 21)
        ),
    }
