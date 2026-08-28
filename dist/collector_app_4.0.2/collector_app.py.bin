"""Collector EC600M application composition root."""

import _thread
import utime

from collector_cloud import CloudClient
from collector_config import (
    FRAME_QUEUE_FALLBACKS,
    FLASH_SPOOL_FILE,
    FLASH_SPOOL_META_0,
    FLASH_SPOOL_META_1,
    INFLIGHT_LIMIT,
    OTA_HEALTH_CONFIRM_SECONDS,
    OTA_QUIET_SECONDS,
    PROJECT_NAME,
    PROJECT_VERSION,
    RAM_SPILL_HIGH_WATERMARK,
    RAM_OTA_LOW_WATERMARK,
    SPILL_BATCH_FRAMES,
    load_device_config,
)
from collector_log import StructuredLogger
from collector_ota import OtaBootGuard, OtaManager
from collector_queue import (
    DeliveryStore,
    FlashJournal,
    RawFrameQueue,
    SequenceCounter,
    StorageBudget,
    file_size,
)
from collector_sensors import SensorService
from collector_uart import UartPipeline


class OtaQuietGate:
    def __init__(self, delivery):
        self.delivery = delivery
        self.quiet_since_ms = None

    def ready(self):
        quiet = (
            self.delivery.flash_depth() == 0
            and self.delivery.ram.percent() < RAM_OTA_LOW_WATERMARK
        )
        now = utime.ticks_ms()
        if not quiet:
            self.quiet_since_ms = None
            return False
        if self.quiet_since_ms is None:
            self.quiet_since_ms = now
            return False
        return utime.ticks_diff(now, self.quiet_since_ms) >= OTA_QUIET_SECONDS * 1000


class CollectorApplication:
    def __init__(self):
        self.storage = StorageBudget()
        reserve_ready = self.storage.ensure_reserve()
        self.logger = StructuredLogger()
        if not reserve_ready:
            self.logger.error(
                "storage", "OTA_RESERVE",
                "Unable to reserve 1.5 MiB; new Flash spool allocation is disabled"
            )

        self.ram_queue = self._allocate_ram_queue()
        existing_spool_bytes = file_size(FLASH_SPOOL_FILE)
        spool_bytes = (
            existing_spool_bytes
            if existing_spool_bytes > 0
            else self.storage.spool_budget()
        )
        self.journal = None
        if spool_bytes > 0:
            self.journal = FlashJournal(
                FLASH_SPOOL_FILE,
                FLASH_SPOOL_META_0,
                FLASH_SPOOL_META_1,
                spool_bytes,
            )
            if not self.journal.enabled():
                self.logger.error(
                    "storage", "SPOOL_INIT", "Flash SignsData spool initialization failed"
                )
        else:
            self.logger.warn(
                "storage", "SPOOL_DISABLED",
                "Flash SignsData spool is unavailable; only the RAM queue is active"
            )

        initial_sequence = 0
        if self.journal and self.journal.last_seq >= 0:
            initial_sequence = self.journal.last_seq + 1
        self.sequence = SequenceCounter(initial=initial_sequence)
        self.delivery = DeliveryStore(self.ram_queue, self.sequence, self.journal)
        self.config = load_device_config()
        self.cloud = CloudClient(self.config, self.delivery, self.logger)
        self.quiet_gate = OtaQuietGate(self.delivery)
        self.ota = OtaManager(
            self.config,
            self.storage,
            self.cloud.publish_raw,
            self.logger,
            self._ota_ready,
            self._reboot,
            self.cloud.next_message_id,
        )
        self.cloud.attach_ota(self.ota)
        self.logger.attach_cloud(self.cloud.publish_event, self.cloud.context)
        self.uart = UartPipeline(self.delivery, self.cloud, self.logger)
        self.sensors = SensorService(self.config, self.cloud, self.logger)

    def _allocate_ram_queue(self):
        last_error = None
        for capacity in FRAME_QUEUE_FALLBACKS:
            try:
                return RawFrameQueue(capacity)
            except MemoryError as error:
                last_error = error
                try:
                    import gc

                    gc.collect()
                except Exception:
                    pass
        raise MemoryError("cannot allocate minimum SignsData queue: {}".format(last_error))

    def _ota_ready(self):
        if not self.cloud.connected:
            self.quiet_gate.quiet_since_ms = None
            return False
        return self.quiet_gate.ready()

    def _reboot(self):
        from misc import Power

        Power.powerRestart()

    def _watchdog_worker(self):
        try:
            from machine import Pin

            led = Pin(Pin.GPIO24, Pin.OUT, Pin.PULL_DISABLE, 1)
        except Exception as error:
            self.logger.warn("watchdog", "LED_INIT", str(error))
            return
        while True:
            try:
                led.write(0 if led.read() else 1)
            except Exception:
                pass
            utime.sleep(1)

    def _heartbeat_worker(self):
        while True:
            utime.sleep(120)
            if self.cloud.connected:
                try:
                    self.uart.write(b"Heartbeat")
                except Exception as error:
                    self.logger.warn(
                        "uart", "HEARTBEAT", str(error), rate_limit_s=60
                    )

    def _storage_worker(self):
        """Persist batches independently of UART parsing and MQTT publishing."""
        cycles = 0
        while True:
            try:
                cycles += 1
                if cycles >= 50:
                    self.sequence.reserve_if_low()
                    cycles = 0
                ram_depth = self.ram_queue.depth()
                cloud_stats = self.cloud.stats()
                should_spill = ram_depth >= SPILL_BATCH_FRAMES and (
                    not self.cloud.connected
                    or self.ram_queue.percent() >= RAM_SPILL_HIGH_WATERMARK
                    or cloud_stats["inflight"] >= INFLIGHT_LIMIT
                )
                if should_spill:
                    written = self.delivery.spill(force=True)
                    if (
                        not written
                        and self.journal
                        and self.journal.depth() >= self.journal.capacity
                    ):
                        self.logger.error(
                            "storage", "SPOOL_FULL",
                            "Flash SignsData spool is full; old records are preserved",
                            rate_limit_s=60,
                        )
            except Exception as error:
                self.logger.error(
                    "storage", "WORKER", "Storage worker recovered",
                    extra=str(error), rate_limit_s=10,
                )
            utime.sleep_ms(20)

    def _stats_worker(self):
        while True:
            utime.sleep(60)
            uart_stats = self.uart.stats()
            delivery_stats = self.delivery.stats()
            cloud_stats = self.cloud.stats()
            log_stats = self.logger.stats()
            oldest_timestamp_ms = delivery_stats["oldest_timestamp_ms"]
            oldest_age_s = 0
            if oldest_timestamp_ms:
                try:
                    oldest_age_s = max(
                        0, (int(utime.time() * 1000) - oldest_timestamp_ms) // 1000
                    )
                except Exception:
                    oldest_age_s = 0
            extra = (
                "rx={rx},frames={frames},rx_overflow={rx_overflow},"
                "uart_errors={uart_errors},parse_errors={parse_errors},"
                "ring={ring}/{ring_cap},"
                "ram={ram}/{ram_cap},flash={flash}/{flash_cap},"
                "flash_crc={flash_crc},flash_io={flash_io},"
                "inflight={inflight},retry={retry},"
                "aux={aux},aux_drop={aux_drop},oldest_s={oldest},"
                "rejected={rejected},log_drop={log_drop}"
            ).format(
                rx=uart_stats["rx_bytes"],
                frames=uart_stats["signs_frames"],
                rx_overflow=uart_stats["rx_overflow_bytes"],
                uart_errors=uart_stats["driver_error_events"],
                parse_errors=uart_stats["sync_errors"],
                ring=uart_stats["ring_depth"],
                ring_cap=uart_stats["ring_capacity"],
                ram=delivery_stats["ram_depth"],
                ram_cap=delivery_stats["ram_capacity"],
                flash=delivery_stats["flash_depth"],
                flash_cap=delivery_stats["flash_capacity"],
                flash_crc=delivery_stats["flash_corrupt"],
                flash_io=delivery_stats["flash_io_errors"],
                inflight=cloud_stats["inflight"],
                retry=cloud_stats["retry_count"],
                aux=cloud_stats["deferred"],
                aux_drop=cloud_stats["deferred_dropped"],
                oldest=oldest_age_s,
                rejected=delivery_stats["rejected"],
                log_drop=log_stats["dropped"],
            )
            self.logger.runtime(
                "telemetry", "collector", "SignsData", "stats", "device",
                "UART and delivery queue statistics",
                value=uart_stats["signs_frames"], extra=extra
            )
            if (
                uart_stats["rx_overflow_bytes"]
                or uart_stats["driver_error_events"]
                or delivery_stats["rejected"]
            ):
                self.logger.error(
                    "uart", "DATA_LOSS", "UART/queue capacity has been exceeded",
                    extra=extra, rate_limit_s=60
                )
            if delivery_stats["flash_corrupt"] or delivery_stats["flash_io_errors"]:
                self.logger.error(
                    "storage", "SPOOL_READ", "Flash queue integrity/read error detected",
                    extra=extra, rate_limit_s=60
                )
            if not self.storage.reserve_ready and not self.ota.running:
                self.logger.error(
                    "storage", "OTA_RESERVE",
                    "Full 1.5 MiB OTA reserve is unavailable; spool expansion disabled",
                    rate_limit_s=60,
                )
            for _ in range(5):
                if not self.logger.flush_one():
                    break

    def _health_worker(self):
        # The stable bootstrap enforces a 120-second rollback deadline. Mark
        # healthy earlier so filesystem scheduling cannot create a boundary
        # race between confirmation and rollback.
        utime.sleep(OTA_HEALTH_CONFIRM_SECONDS)
        if OtaBootGuard.mark_healthy():
            self.storage.restore_reserve()
            self.logger.runtime(
                "ota", "collector", "application", "healthy", "device",
                "Application health window completed"
            )

    def start(self):
        self.logger.runtime(
            "boot", "collector", self.config.device_name, "start", "device",
            "Collector application starting",
            extra="project={},version={},ram_frames={},flash_frames={}".format(
                PROJECT_NAME,
                PROJECT_VERSION,
                self.ram_queue.capacity,
                self.journal.capacity if self.journal else 0,
            ),
        )
        self.uart.start()
        _thread.start_new_thread(self._storage_worker, ())
        self.cloud.start()
        self.ota.start()
        self.sensors.start()
        _thread.start_new_thread(self._watchdog_worker, ())
        _thread.start_new_thread(self._heartbeat_worker, ())
        _thread.start_new_thread(self._stats_worker, ())
        _thread.start_new_thread(self._health_worker, ())
        while True:
            utime.sleep(60)


def run():
    CollectorApplication().start()
