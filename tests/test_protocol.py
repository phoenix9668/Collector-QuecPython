import builtins
import random
import sys
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from collector_protocol import (  # noqa: E402
    ByteRing,
    FrameStream,
    decode_signs_frame,
    legacy_hex,
)


def make_frame(sequence):
    frame = bytearray(206)
    frame[0] = 0x0B
    frame[4] = 0xB0
    for index in range(5, 206):
        frame[index] = (sequence + index) & 0xFF
    frame[193:197] = int(sequence).to_bytes(4, "big")
    return bytes(frame)


class ProtocolTests(unittest.TestCase):
    def test_read_only_bytearray_firmware_uses_segment_ring(self):
        module_globals = ByteRing.__init__.__globals__
        original_bytearray = module_globals.get("bytearray")

        class ReadOnlyBytearray(builtins.bytearray):
            def __setitem__(self, _key, _value):
                raise TypeError("bytearray object doesn't support item assignment")

        module_globals["bytearray"] = ReadOnlyBytearray
        try:
            stream = FrameStream(32768)
        finally:
            if original_bytearray is None:
                module_globals.pop("bytearray", None)
            else:
                module_globals["bytearray"] = original_bytearray

        self.assertEqual(stream.ring.storage_mode, "segments")
        received = []
        source = make_frame(41) + make_frame(42)
        for offset in range(0, len(source), 17):
            chunk = source[offset : offset + 17]
            self.assertEqual(stream.feed(chunk), len(chunk))
        stream.drain(
            lambda frame, _timestamp: received.append(
                int.from_bytes(frame[193:197], "big")
            )
        )
        self.assertEqual(received, [41, 42])
        self.assertEqual(stream.stats()["ring_depth"], 0)
        self.assertEqual(stream.stats()["rx_overflow_bytes"], 0)

    def test_legacy_hex_format_is_unchanged(self):
        self.assertEqual(legacy_hex(bytes([0x0B, 0x31, 0, 0xAF])), " b 31 0 af")

    def test_random_chunking_noise_and_auxiliary_frames(self):
        stream = FrameStream(32768)
        received = []
        voltages = []
        infos = []
        rng = random.Random(20260827)
        source = bytearray(b"noise")
        for sequence in range(2000):
            source.extend(make_frame(sequence))
            if sequence % 137 == 0:
                source.extend(bytes([0x0B, 0, 0, 0, 0xB2, 0, 1, 0, 2]))
            if sequence % 211 == 0:
                source.extend(b"##Read Memory Complete##")

        offset = 0
        while offset < len(source):
            size = rng.randint(1, 997)
            chunk = source[offset : offset + size]
            offset += len(chunk)
            self.assertEqual(stream.feed(chunk), len(chunk))
            stream.drain(
                lambda frame, _ts: received.append(
                    int.from_bytes(frame[193:197], "big")
                ),
                lambda frame, _ts: voltages.append(frame),
                lambda frame, _ts: infos.append(frame),
            )
        stream.drain(
            lambda frame, _ts: received.append(int.from_bytes(frame[193:197], "big")),
            lambda frame, _ts: voltages.append(frame),
            lambda frame, _ts: infos.append(frame),
        )
        self.assertEqual(received, list(range(2000)))
        self.assertEqual(len(voltages), 15)
        self.assertEqual(len(infos), 10)
        self.assertEqual(stream.rx_overflow_bytes, 0)

    def test_one_hundred_thousand_back_to_back_frames(self):
        stream = FrameStream(32768)
        expected = [0]
        voltage_count = [0]
        info_count = [0]
        rng = random.Random(206115200)
        started = time.perf_counter()

        def accept(frame, _timestamp):
            sequence = int.from_bytes(frame[193:197], "big")
            self.assertEqual(sequence, expected[0])
            expected[0] += 1

        def feed_random_chunks(data):
            offset = 0
            view = memoryview(data)
            while offset < len(view):
                size = rng.randint(1, 997)
                chunk = view[offset : offset + size]
                offset += len(chunk)
                self.assertEqual(stream.feed(chunk), len(chunk))
                stream.drain(
                    accept,
                    lambda _frame, _ts: voltage_count.__setitem__(
                        0, voltage_count[0] + 1
                    ),
                    lambda _frame, _ts: info_count.__setitem__(
                        0, info_count[0] + 1
                    ),
                )

        batch = bytearray()
        for sequence in range(100000):
            if sequence % 5000 == 0:
                batch.extend(b"noise")
            batch.extend(make_frame(sequence))
            if sequence % 10000 == 0:
                batch.extend(bytes([0x0B, 0, 0, 0, 0xB2, 0, 1, 0, 2]))
            if sequence % 12500 == 0:
                batch.extend(b"##Read Memory Complete##")
            if len(batch) >= 24576:
                feed_random_chunks(batch)
                batch = bytearray()
        if batch:
            feed_random_chunks(batch)
        stream.drain(
            accept,
            lambda _frame, _ts: voltage_count.__setitem__(0, voltage_count[0] + 1),
            lambda _frame, _ts: info_count.__setitem__(0, info_count[0] + 1),
        )
        self.assertEqual(expected[0], 100000)
        self.assertEqual(voltage_count[0], 10)
        self.assertEqual(info_count[0], 8)
        self.assertEqual(stream.rx_overflow_bytes, 0)
        elapsed = time.perf_counter() - started
        throughput = (100000 * 206) / elapsed
        # 115200 8N1 carries at most 11,520 data bytes/s; require at least 2x.
        self.assertGreater(throughput, 23040)

    def test_signs_offsets_and_timestamp(self):
        frame = bytearray(make_frame(1))
        frame[187] = 7
        frame[188:190] = bytes([0x08, 0x00])
        frame[190:192] = bytes([0x12, 0x34])
        frame[192] = 128
        params = decode_signs_frame(bytes(frame), 123456789)
        self.assertEqual(params["collar_information:Stage"]["value"], 7)
        self.assertEqual(params["collar_information:ResetCnt"]["value"], 0x1234)
        self.assertEqual(
            params["collar_information:UTCtime"]["value"], "123456789"
        )


if __name__ == "__main__":
    unittest.main()
