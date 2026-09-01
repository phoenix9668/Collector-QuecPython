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
    OTA_RESERVED_BYTES,
    PROJECT_NAME,
    PROJECT_VERSION,
    RAM_SPILL_HIGH_WATERMARK,
    SPILL_BATCH_FRAMES,
    load_device_config,
)
from collector_log import StructuredLogger
try:
    from collector_migration import (
        MigrationManager,
        migration_pending_on_boot,
        recover_identity_before_load,
    )
except ImportError:
    # The space-constrained 4.0.15 bridge installs the integration points one
    # OTA before the new module. It must remain bootable between both steps.
    def recover_identity_before_load():
        return None

    def migration_pending_on_boot():
        return False

    class MigrationManager:
        def __init__(self, *_args):
            pass

        def has_pending(self):
            return False

        def start(self):
            pass
from collector_ota import OtaBootGuard, OtaManager
from collector_protocol import hex_preview
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


class CollectorApplication:
    def __init__(self):
        # A committed migration must select its target identity before
        # device.json is parsed and before any MQTT worker can start.
        recover_identity_before_load()
        self.migration_mode = bool(migration_pending_on_boot())
        self.storage = StorageBudget()
        self.pending_ota = OtaBootGuard.load_pending()
        # During the post-update health window the rollback copies and the
        # 256 KiB reserve cannot coexist on the EC600M's 576 KiB /usr
        # partition.  This is an expected temporary state: defer allocation
        # until mark_healthy() removes the rollback tree instead of attempting
        # a reservation that is guaranteed to fail and reporting a false
        # storage error.
        reserve_ready = False
        if not self.pending_ota and not self.migration_mode:
            reserve_ready = self.storage.ensure_reserve()
        self.logger = StructuredLogger()
        if not reserve_ready:
            self._report_reserve_unavailable(startup=True)

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
            if self.pending_ota or self.migration_mode:
                self.logger.info(
                    "storage", "SPOOL_DEFERRED",
                    "Flash SignsData spool is deferred until maintenance completes",
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
        self.ota_exclusive = False
        self.migration_started = False
        self._ota_legacy_cloud_methods = None
        self._ota_legacy_sensor_methods = None
        self._ota_legacy_accept = None
        self.config = load_device_config()
        self.cloud = CloudClient(self.config, self.delivery, self.logger)
        self.migration = MigrationManager(
            self.config,
            self.delivery,
            self.cloud,
            self.logger,
            self._reboot,
        )
        self.ota = OtaManager(
            self.config,
            self.storage,
            self.cloud.publish_raw,
            self.logger,
            self._ota_ready,
            self._reboot,
            self.cloud.next_message_id,
            storage_prepare=self._prepare_ota_storage,
            storage_restore=self._restore_runtime_storage,
            activity_provider=lambda: self.delivery.accepted,
            exclusive_enter=self._enter_ota_mode,
            exclusive_exit=self._exit_ota_mode,
        )
        self.cloud.attach_ota(self.ota)
        self.logger.attach_cloud(self.cloud.publish_event, self.cloud.context)
        self.uart = UartPipeline(self.delivery, self.cloud, self.logger)
        self.last_uart_rx_sample_id = 0
        self.last_uart_frame_sample_id = 0
        self.sensors = SensorService(self.config, self.cloud, self.logger)
        self.migration_mode = bool(self.migration.has_pending())
        if self.migration_mode:
            self._pause_business("migration", stop_migration=False, pause_logs=False)

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

    def _report_reserve_unavailable(self, startup=False):
        """Distinguish the expected OTA health hold from a real space fault."""
        if self.pending_ota:
            return self.logger.info(
                "storage", "OTA_RESERVE_DEFERRED",
                "OTA reserve allocation is deferred until health confirmation",
                extra="reserve_kib={},startup={}".format(
                    OTA_RESERVED_BYTES // 1024, 1 if startup else 0
                ),
                rate_limit_s=60,
            )
        if getattr(self, "migration_mode", False):
            return self.logger.info(
                "storage", "MIGRATION_STORAGE_DEFERRED",
                "OTA reserve and Flash spool are deferred until migration completes",
                extra="reserve_kib={},startup={}".format(
                    OTA_RESERVED_BYTES // 1024, 1 if startup else 0
                ),
                rate_limit_s=60,
            )
        return self.logger.error(
            "storage", "OTA_RESERVE",
            "Full {} KiB OTA reserve is unavailable; spool expansion disabled".format(
                OTA_RESERVED_BYTES // 1024
            ),
            rate_limit_s=60,
        )

    def _ota_ready(self, migration_package=False):
        # Retained for constructor compatibility with older collector_ota.py.
        # New OTA code enters exclusive mode immediately and does not call it.
        return bool(self.cloud.connected)

    def _legacy_discard_delivery(self):
        """Support the first bridge OTA before collector_queue.py is updated."""
        if self._ota_legacy_accept is None:
            self._ota_legacy_accept = self.delivery.accept
            self.delivery.accept = lambda _frame, _captured_ms: True
        ram = self.delivery.ram
        journal = self.delivery.journal
        ram.lock.acquire()
        try:
            if journal:
                journal.lock.acquire()
            try:
                dropped = ram.count
                for offset in range(ram.count):
                    index = (ram.head + offset) % ram.capacity
                    ram.frames[index] = None
                    ram.sequences[index] = 0
                    ram.timestamps[index] = 0
                    ram.states[index] = ram.QUEUED
                ram.head = 0
                ram.tail = 0
                ram.count = 0
                if journal:
                    dropped += journal.count
                    journal.head = journal.tail
                    journal.count = 0
                    journal.acked = {}
                    journal.peek_failed = False
                    try:
                        journal._save_meta()
                    except Exception:
                        journal.io_errors += 1
                self.delivery.send_ceiling = None
                self.delivery.direct_persist = False
                return dropped
            finally:
                if journal:
                    journal.lock.release()
        finally:
            ram.lock.release()

    def _pause_cloud_business(self):
        pause = getattr(self.cloud, "enter_ota_mode", None)
        if pause:
            return pause()
        # 4.1.0 bridge fallback: replace only business methods on this cloud
        # instance. publish_raw remains intact for OTA progress/download.
        if self._ota_legacy_cloud_methods is None:
            self._ota_legacy_cloud_methods = (
                self.cloud._delivery_cycle,
                self.cloud.publish_properties,
                self.cloud.defer_properties,
            )
            self.cloud._delivery_cycle = lambda: None
            self.cloud.publish_properties = lambda _values, qos=0: False
            self.cloud.defer_properties = lambda _values: False
        if self.cloud.state_lock:
            self.cloud.state_lock.acquire()
        try:
            dropped = len(self.cloud.inflight)
            self.cloud.inflight = {}
            self.cloud.property_reply_ids = []
        finally:
            if self.cloud.state_lock:
                self.cloud.state_lock.release()
        return dropped

    def _pause_business(self, reason, stop_migration=False, pause_logs=False):
        """Stop all data-producing business work and discard its backlog."""
        if self.ota_exclusive:
            if stop_migration:
                self.migration.stop = True
            if pause_logs:
                self.logger.paused = True
            return True
        self.ota_exclusive = True
        if stop_migration:
            self.migration.stop = True
        cloud_dropped = self._pause_cloud_business()
        discard = getattr(self.delivery, "discard_all", None)
        data_dropped = discard() if discard else self._legacy_discard_delivery()
        pause_uart = getattr(self.uart, "pause_for_ota", None)
        if pause_uart:
            pause_uart()
        else:
            self.uart.stop = True
            try:
                self.uart.uart.set_callback(None)
            except Exception:
                pass
        pause_sensors = getattr(self.sensors, "pause_for_ota", None)
        if pause_sensors:
            pause_sensors()
        elif self._ota_legacy_sensor_methods is None:
            # Bridge fallback for the already-installed 4.1.0 sensor module.
            self._ota_legacy_sensor_methods = (
                self.sensors._wait_cloud,
                self.sensors._publish_cycle,
            )
            self.sensors._wait_cloud = lambda: False
            self.sensors._publish_cycle = lambda include_sim=False: False
        self.logger.paused = bool(pause_logs)
        try:
            print(
                "[{}] exclusive mode: UART/business stopped; discarded={} inflight={}".format(
                    str(reason).upper(), data_dropped, cloud_dropped
                )
            )
        except Exception:
            pass
        return True

    def _enter_ota_mode(self, _task=None):
        """Give OTA exclusive ownership until failure recovery or reboot."""
        return self._pause_business(
            "ota", stop_migration=True, pause_logs=True
        )

    def _exit_ota_mode(self):
        """Resume normal work only when an OTA attempt failed before reboot."""
        resume = getattr(self.delivery, "resume_accepting", None)
        if resume:
            resume()
        elif self._ota_legacy_accept is not None:
            self.delivery.accept = self._ota_legacy_accept
            self._ota_legacy_accept = None
        resume_cloud = getattr(self.cloud, "exit_ota_mode", None)
        if resume_cloud:
            resume_cloud()
        elif self._ota_legacy_cloud_methods is not None:
            (
                self.cloud._delivery_cycle,
                self.cloud.publish_properties,
                self.cloud.defer_properties,
            ) = self._ota_legacy_cloud_methods
            self._ota_legacy_cloud_methods = None
        resume_uart = getattr(self.uart, "resume_after_ota", None)
        if resume_uart:
            resume_uart()
        else:
            self.uart.stop = False
            try:
                self.uart.uart.set_callback(self.uart._callback)
                _thread.start_new_thread(self.uart._parser_worker, ())
            except Exception:
                pass
        resume_sensors = getattr(self.sensors, "resume_after_ota", None)
        if resume_sensors:
            resume_sensors()
        elif self._ota_legacy_sensor_methods is not None:
            self.sensors._wait_cloud, self.sensors._publish_cycle = (
                self._ota_legacy_sensor_methods
            )
            self._ota_legacy_sensor_methods = None
        self.migration.stop = False
        self.logger.paused = False
        self.ota_exclusive = False
        return True

    def _prepare_ota_storage(self, migration_package=False):
        """Drop business data and reclaim its Flash allocation for any OTA."""
        discard = getattr(self.delivery, "discard_all", None)
        if discard:
            discard()
        else:
            self._legacy_discard_delivery()
        if not self.delivery.detach_empty_journal():
            return False
        self.journal = None
        self.logger.clear_persistent()
        return True

    def _restore_runtime_storage(self):
        """Restore the reserve and Flash spool after OTA failure/health confirm."""
        if not self.storage.restore_reserve():
            return False
        if self.delivery.journal is not None:
            self.journal = self.delivery.journal
            return True
        spool_bytes = self.storage.spool_budget()
        if spool_bytes <= 0:
            return False
        journal = FlashJournal(
            FLASH_SPOOL_FILE,
            FLASH_SPOOL_META_0,
            FLASH_SPOOL_META_1,
            spool_bytes,
        )
        if not journal.enabled() or not self.delivery.attach_journal(journal):
            return False
        self.journal = journal
        return True

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

    def _send_heartbeat(self):
        """Keep the subordinate-controller watchdog alive during maintenance."""
        # Normal operation preserves the original network-health semantics:
        # a disconnected collector does not mask the fault from the external
        # watchdog. OTA/migration is an intentional maintenance window, so it
        # must keep heartbeats flowing even while MQTT is being reconnected.
        if not self.ota_exclusive and not self.cloud.connected:
            return False
        try:
            self.uart.write(b"Heartbeat")
            return True
        except Exception as error:
            self.logger.warn(
                "uart", "HEARTBEAT", str(error), rate_limit_s=60
            )
            return False

    def _heartbeat_worker(self):
        while True:
            self._send_heartbeat()
            utime.sleep(120)

    def _storage_worker(self):
        """Persist batches independently of UART parsing and MQTT publishing."""
        cycles = 0
        while True:
            if self.ota_exclusive:
                utime.sleep_ms(100)
                continue
            try:
                cycles += 1
                if cycles >= 50:
                    self.sequence.reserve_if_low()
                    cycles = 0
                ram_depth = self.ram_queue.depth()
                cloud_stats = self.cloud.stats()
                should_spill = (
                    ram_depth >= SPILL_BATCH_FRAMES
                    and (
                        not self.cloud.connected
                        or self.ram_queue.percent() >= RAM_SPILL_HIGH_WATERMARK
                        or cloud_stats["inflight"] >= INFLIGHT_LIMIT
                    )
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
            if self.ota_exclusive:
                continue
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
                "rx={rx},frames={frames},b2={b2},info={info},"
                "event={event_inflight}/{event_ok}/{event_reject}/"
                "{event_timeout}/{event_unmatched}/{event_retry}/{event_code},"
                "uart_sample={uart_sample},"
                "rx_overflow={rx_overflow},uart_errors={uart_errors},"
                "uart_event={uart_event},uart_read={uart_read},"
                "uart_feed={uart_feed},uart_empty={uart_empty},"
                "uart_store={uart_store},uart_last={uart_last},"
                "parse_errors={parse_errors},"
                "ring={ring}/{ring_cap}/{ring_mode},"
                "ram={ram}/{ram_cap},flash={flash}/{flash_cap},"
                "flash_crc={flash_crc},flash_io={flash_io},"
                "inflight={inflight},retry={retry},"
                "post_ok={post_ok},post_reject={post_reject},"
                "post_unmatched={post_unmatched},post_code={post_code},"
                "aux={aux},aux_drop={aux_drop},oldest_s={oldest},"
                "rejected={rejected},log_drop={log_drop}"
            ).format(
                rx=uart_stats["rx_bytes"],
                frames=uart_stats["signs_frames"],
                b2=uart_stats["voltage_frames"],
                info=uart_stats["info_frames"],
                event_inflight=cloud_stats["event_inflight"],
                event_ok=cloud_stats["event_post_success"],
                event_reject=cloud_stats["event_post_rejected"],
                event_timeout=cloud_stats["event_post_timeouts"],
                event_unmatched=cloud_stats["event_post_unmatched"],
                event_retry=cloud_stats["event_retry_count"],
                event_code=cloud_stats["last_event_reply_code"],
                uart_sample=cloud_stats["uart_sample_log"],
                rx_overflow=uart_stats["rx_overflow_bytes"],
                uart_errors=uart_stats["driver_error_events"],
                uart_event=uart_stats["event_error_events"],
                uart_read=uart_stats["read_error_events"],
                uart_feed=uart_stats["feed_error_events"],
                uart_empty=uart_stats["empty_read_events"],
                uart_store=uart_stats["storage_error_frames"],
                uart_last=uart_stats["last_driver_error"],
                parse_errors=uart_stats["sync_errors"],
                ring=uart_stats["ring_depth"],
                ring_cap=uart_stats["ring_capacity"],
                ring_mode=uart_stats["ring_mode"],
                ram=delivery_stats["ram_depth"],
                ram_cap=delivery_stats["ram_capacity"],
                flash=delivery_stats["flash_depth"],
                flash_cap=delivery_stats["flash_capacity"],
                flash_crc=delivery_stats["flash_corrupt"],
                flash_io=delivery_stats["flash_io_errors"],
                inflight=cloud_stats["inflight"],
                retry=cloud_stats["retry_count"],
                post_ok=cloud_stats["post_reply_success"],
                post_reject=cloud_stats["post_reply_rejected"],
                post_unmatched=cloud_stats["post_reply_unmatched"],
                post_code=cloud_stats["last_post_reply_code"],
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
            self._log_uart_sample(uart_stats)
            if (
                uart_stats["rx_overflow_bytes"]
                or uart_stats["driver_error_events"]
                or uart_stats["storage_error_frames"]
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
                self._report_reserve_unavailable()

    def _log_uart_sample(self, uart_stats):
        rx_sample_id = uart_stats["rx_sample_id"]
        frame_sample_id = uart_stats["frame_sample_id"]
        if (
            rx_sample_id == self.last_uart_rx_sample_id
            and frame_sample_id == self.last_uart_frame_sample_id
        ):
            return False
        try:
            # The disabled path deliberately performs no hexadecimal
            # conversion. IDs still advance so re-enabling cannot emit stale
            # UART data from an earlier 60-second window.
            if not self.cloud.uart_sample_log_enabled:
                return False
            sample_extra = "chunk_len={},raw={}".format(
                uart_stats["rx_sample_size"],
                hex_preview(uart_stats["rx_sample"][:48]),
            )
            if frame_sample_id != self.last_uart_frame_sample_id:
                sample_extra += ",frame_type={},frame_len={},frame={}".format(
                    uart_stats["frame_sample_kind"],
                    uart_stats["frame_sample_size"],
                    hex_preview(uart_stats["frame_sample"][:48]),
                )
            return self.logger.info(
                "uart", "RX_SAMPLE", "Recent UART receive sample",
                extra=sample_extra,
            )
        except Exception as error:
            self.logger.warn(
                "uart", "RX_SAMPLE", "UART sample logging failed",
                extra=str(error), rate_limit_s=60,
            )
            return False
        finally:
            self.last_uart_rx_sample_id = rx_sample_id
            self.last_uart_frame_sample_id = frame_sample_id

    def _health_worker(self):
        if not self.pending_ota:
            return
        # The stable bootstrap enforces a 120-second rollback deadline. Mark
        # healthy earlier so filesystem scheduling cannot create a boundary
        # race between confirmation and rollback.
        utime.sleep(OTA_HEALTH_CONFIRM_SECONDS)
        if OtaBootGuard.mark_healthy():
            storage_restored = True
            if not getattr(self, "migration_mode", False):
                storage_restored = self._restore_runtime_storage()
            self.pending_ota = None
            try:
                print(
                    "[OTA] version={} health confirmation completed".format(
                        PROJECT_VERSION
                    )
                )
            except Exception:
                pass
            self.logger.runtime(
                "ota", "collector", "application", "healthy", "device",
                "Application health window completed"
            )
            if getattr(self, "migration_mode", False):
                self.logger.info(
                    "storage", "MIGRATION_STORAGE_DEFERRED",
                    "Runtime storage restore deferred until migration completes",
                )
                self._start_migration()
            if not storage_restored:
                self.logger.error(
                    "storage", "OTA_RESERVE_RESTORE",
                    "OTA health confirmation completed but reserve/Flash spool restore failed",
                )

    def _start_migration(self):
        if self.migration_started:
            return False
        self.migration_started = True
        self.migration.start()
        return True

    def start(self):
        if self.pending_ota:
            try:
                print(
                    "[OTA] version={} update applied; health confirmation in {}s".format(
                        self.pending_ota.get("version", PROJECT_VERSION),
                        OTA_HEALTH_CONFIRM_SECONDS,
                    )
                )
            except Exception:
                pass
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
        if not self.migration_mode:
            self.uart.start()
        _thread.start_new_thread(self._storage_worker, ())
        self.cloud.start()
        # A migration delivered by this OTA is started only after the new app
        # passes its rollback health window. Otherwise its intentional reboot
        # would look like an application crash to the stable bootstrap.
        if not self.migration_mode or not self.pending_ota:
            self._start_migration()
        self.ota.start()
        if not self.migration_mode:
            self.sensors.start()
        _thread.start_new_thread(self._watchdog_worker, ())
        _thread.start_new_thread(self._heartbeat_worker, ())
        _thread.start_new_thread(self._stats_worker, ())
        _thread.start_new_thread(self._health_worker, ())
        while True:
            utime.sleep(60)


def run():
    CollectorApplication().start()
