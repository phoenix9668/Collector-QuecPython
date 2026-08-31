"""EC600M UART2 receive pipeline.

The hardware callback only drains UART bytes into a fixed ring. Parsing,
storage, JSON conversion, logging, and MQTT all run outside the callback.
"""

import _thread
import utime
from machine import UART

from collector_config import UART_BAUDRATE, UART_RX_RING_MINIMUM, UART_RX_RING_TARGET
from collector_protocol import FrameStream, decode_voltage_frame


class UartPipeline:
    def __init__(self, delivery, cloud, logger):
        self.delivery = delivery
        self.cloud = cloud
        self.logger = logger
        self.stream = self._allocate_stream()
        self.stop = False
        self.rejected_frames = 0
        self.storage_error_frames = 0
        self.driver_error_events = 0
        self.event_error_events = 0
        self.read_error_events = 0
        self.feed_error_events = 0
        self.empty_read_events = 0
        self.last_driver_error = ""
        self.write_lock = _thread.allocate_lock()
        self.uart = UART(UART.UART2, UART_BAUDRATE, 8, 0, 1, 0)
        self.uart.set_callback(self._callback)

    def _allocate_stream(self):
        try:
            return FrameStream(UART_RX_RING_TARGET)
        except MemoryError:
            try:
                import gc

                gc.collect()
            except Exception:
                pass
            return FrameStream(UART_RX_RING_MINIMUM)

    def _callback(self, event):
        if not event:
            return
        if event[0] != 0:
            # Keep the IRQ callback bounded: aggregate here and report from the
            # 60-second telemetry worker instead of logging in callback context.
            self.driver_error_events += 1
            self.event_error_events += 1
            self.last_driver_error = "event={}".format(event)
            return
        expected = int(event[2]) if len(event) > 2 else 0
        while True:
            try:
                available = int(self.uart.any())
            except Exception:
                available = expected
            if available <= 0 and expected > 0:
                available = expected
            if available <= 0:
                break
            try:
                data = self.uart.read(available)
            except Exception as error:
                self.driver_error_events += 1
                self.read_error_events += 1
                self.last_driver_error = "read={}".format(error)
                break
            if not data:
                self.driver_error_events += 1
                self.empty_read_events += 1
                self.last_driver_error = "empty_read={}".format(available)
                break
            try:
                self.stream.feed(data)
            except Exception as error:
                self.driver_error_events += 1
                self.feed_error_events += 1
                self.last_driver_error = "feed={}".format(error)
                break
            expected = 0

    def write(self, data):
        self.write_lock.acquire()
        try:
            return self.uart.write(data)
        finally:
            self.write_lock.release()

    def _on_signs(self, frame, captured_ms):
        try:
            accepted = self.delivery.accept(frame, captured_ms)
        except Exception as error:
            self.storage_error_frames += 1
            self.logger.error(
                "uart", "RAM_QUEUE", "Unable to store SignsData in RAM queue",
                extra=str(error), rate_limit_s=5
            )
            return
        if not accepted:
            self.rejected_frames += 1
            self.logger.error(
                "uart", "QUEUE_FULL", "SignsData queue capacity exhausted",
                extra="rejected={}".format(self.rejected_frames), rate_limit_s=5
            )

    def _on_voltage(self, frame, _captured_ms):
        try:
            values = decode_voltage_frame(frame)
            self.cloud.defer_properties(values)
        except Exception as error:
            self.logger.warn(
                "uart", "B2_PARSE", "Voltage frame error: {}".format(error),
                rate_limit_s=10
            )

    def _on_info(self, frame, _captured_ms):
        try:
            text = frame.decode("utf-8")
        except Exception:
            text = str(frame)
        self.cloud.defer_properties(
            {"product_information:StatusInfo": text[:1024]}
        )
        if "##Read Memory Complete##" in text and len(frame) >= 26:
            collector_id = frame[18:26]
            try:
                collector_id = collector_id.decode("utf-8")
            except Exception:
                collector_id = str(collector_id)
            self.cloud.defer_properties(
                {"product_information:CollectorID": collector_id}
            )

    def _parser_worker(self):
        while not self.stop:
            try:
                count = self.stream.drain(
                    self._on_signs,
                    voltage_callback=self._on_voltage,
                    info_callback=self._on_info,
                    limit=64,
                )
            except Exception as error:
                count = 0
                self.logger.error(
                    "uart", "PARSER", "UART parser worker recovered",
                    extra=str(error), rate_limit_s=10
                )
            if count == 0:
                utime.sleep_ms(2)

    def start(self):
        _thread.start_new_thread(self._parser_worker, ())
        self.logger.runtime(
            "uart", "collector", "UART2", "start", "device",
            "UART receive pipeline started",
            value=self.stream.ring.capacity,
            extra="baud={}".format(UART_BAUDRATE),
        )

    def stats(self):
        result = self.stream.stats()
        result["rejected_frames"] = self.rejected_frames
        result["storage_error_frames"] = self.storage_error_frames
        result["driver_error_events"] = self.driver_error_events
        result["event_error_events"] = self.event_error_events
        result["read_error_events"] = self.read_error_events
        result["feed_error_events"] = self.feed_error_events
        result["empty_read_events"] = self.empty_read_events
        result["last_driver_error"] = self.last_driver_error
        return result
