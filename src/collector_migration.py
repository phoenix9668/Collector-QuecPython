try:
    import ujson as json
except ImportError:
    import json

try:
    import utime as time
except ImportError:
    import time

try:
    import _thread
except ImportError:
    _thread = None

from collector_cloud import (
    _load_mqtt_client,
    device_auth,
    dynamic_register_secret,
)
from collector_config import (
    DEVICE_CONFIG_FILE,
    DEVICE_MIGRATION_FILE,
    DEVICE_SECRET_CACHE_FILE,
    MIGRATION_CONFIRM_SECONDS,
    MIGRATION_DONE_FILE,
    MIGRATION_ROLLBACK_SECONDS,
    MIGRATION_STATE_0,
    MIGRATION_STATE_1,
    MQTT_KEEPALIVE,
    PROJECT_VERSION,
    DeviceConfig,
    atomic_json_write,
)
from collector_queue import crc32, remove_file


def ticks_ms():
    try:
        return int(time.ticks_ms())
    except Exception:
        try:
            return int(time.monotonic() * 1000)
        except Exception:
            return int(time.time() * 1000)


def ticks_diff(now, then):
    try:
        return int(time.ticks_diff(now, then))
    except Exception:
        return int(now) - int(then)


def sleep_ms(value):
    try:
        time.sleep_ms(value)
    except Exception:
        time.sleep(float(value) / 1000.0)


def epoch_seconds():
    try:
        value = int(time.time())
        return value if value >= 1000000000 else 0
    except Exception:
        return 0


def runtime_probe_params(device_name, sequence):
    return {
        "category": "migration", "targetType": "collector", "action": "probe",
        "target": str(device_name)[:32], "source": "device",
        "desc": "Identity migration target probe", "temperature": 0.0,
        "humidity": 0.0, "metricValue": 0, "version": PROJECT_VERSION,
        "seq": int(sequence) & 0x7FFFFFFF,
        "extra": "identity preflight",
    }


def probe_identity(config, secret, message_id, timeout_ms=15000):
    MQTTClient = _load_mqtt_client()
    auth = device_auth(config, secret)
    reply_topic = config.topic("thing/event/runtimeLog/post_reply")
    post_topic = config.topic("thing/event/runtimeLog/post")
    expected = str(message_id)
    state = {"code": None, "error": None, "stop": False}

    def callback(topic, message):
        if (topic.decode() if isinstance(topic, bytes) else str(topic)) != reply_topic:
            return
        try:
            value = json.loads(message.decode() if isinstance(message, bytes) else message)
            if str(value.get("id", "")) == expected:
                state["code"] = int(value.get("code", -1))
        except Exception:
            state["code"] = -1

    client = MQTTClient(
        auth["client_id"], config.mqtt_server, config.mqtt_port,
        auth["username"], auth["password"], keepalive=MQTT_KEEPALIVE,
        ssl=False, ssl_params={}, reconn=False,
    )
    client.set_callback(callback)
    try:
        client.connect(clean_session=True)
        client.subscribe(reply_topic.encode("utf-8"), 1)

        def listener():
            try:
                while not state["stop"] and state["code"] is None:
                    client.wait_msg()
            except Exception as error:
                state["error"] = error

        if _thread:
            _thread.start_new_thread(listener, ())
        payload = {
            "id": expected, "version": "1.0",
            "params": runtime_probe_params(config.device_name, int(message_id)),
            "method": "thing.event.runtimeLog.post", "sys": {"ack": 1},
        }
        client.publish(post_topic.encode("utf-8"), json.dumps(payload), qos=0)
        started = ticks_ms()
        while ticks_diff(ticks_ms(), started) < int(timeout_ms):
            if state["code"] is not None or state["error"]:
                break
            sleep_ms(100)
    finally:
        state["stop"] = True
        try:
            client.disconnect()
        except Exception:
            try:
                client.close()
            except Exception:
                pass
    return state["code"] == 200


def _read_json(path):
    for candidate in (path, path + ".tmp"):
        try:
            with open(candidate, "r") as stream:
                value = json.loads(stream.read())
            if isinstance(value, dict):
                return value
        except Exception:
            pass
    return None


def _state_wrapper(payload, generation):
    body = json.dumps(payload)
    return {
        "generation": int(generation),
        "body": body,
        "crc32": crc32(body.encode("utf-8")),
    }


def _state_paths(paths=None):
    return paths or (MIGRATION_STATE_0, MIGRATION_STATE_1)


def load_migration_state(paths=None):
    paths = _state_paths(paths)
    best = None
    best_generation = -1
    for path in paths:
        wrapper = _read_json(path)
        if not wrapper:
            continue
        try:
            body = str(wrapper["body"])
            if int(wrapper["crc32"]) != crc32(body.encode("utf-8")):
                continue
            payload = json.loads(body)
            generation = int(wrapper.get("generation", 0))
            if isinstance(payload, dict) and generation > best_generation:
                best = payload
                best_generation = generation
        except Exception:
            continue
    if best is not None:
        best["_generation"] = best_generation
    return best


def save_migration_state(payload, paths=None):
    paths = _state_paths(paths)
    current = load_migration_state(paths)
    generation = int(current.get("_generation", 0)) + 1 if current else 1
    clean = dict(payload)
    clean.pop("_generation", None)
    path = paths[generation & 1]
    atomic_json_write(path, _state_wrapper(clean, generation))
    clean["_generation"] = generation
    return clean


def remove_migration_state(paths=None):
    paths = _state_paths(paths)
    for path in paths:
        remove_file(path)
        remove_file(path + ".tmp")


def migration_pending_on_boot():
    """Return whether startup must stay in migration-only mode."""
    return bool(load_migration_state() or _read_json(DEVICE_MIGRATION_FILE))


def _valid_migration_id(value):
    value = str(value).strip()
    if not value or len(value) > 64:
        return ""
    # EC600M's reduced QuecPython ``str`` implementation does not provide the
    # CPython alphanumeric helper. Keep this validator deliberately ASCII-only so it
    # behaves identically on the module and in the host tests.
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_."
    for char in value:
        if char not in allowed:
            return ""
    return value


def _valid_aliyun_mqtt_host(value):
    host = str(value).strip().lower()
    if not host or "/" in host or ":" in host or "@" in host:
        return ""
    if not (host.endswith(".aliyuncs.com") or host.endswith(".aliyun.com")):
        return ""
    return host


def normalize_migration(data, current_config):
    if not isinstance(data, dict) or int(data.get("schema", 0)) != 1:
        raise ValueError("migration schema must be 1")
    migration_id = _valid_migration_id(data.get("migrationId", ""))
    if not migration_id:
        raise ValueError("invalid migrationId")
    source = data.get("source", {})
    if not isinstance(source, dict):
        raise ValueError("migration source must be an object")
    if str(source.get("productKey", "")) != current_config.product_key:
        raise ValueError("migration source ProductKey does not match")
    if str(source.get("deviceName", "")) != current_config.device_name:
        raise ValueError("migration source DeviceName does not match")
    target = data.get("target", {})
    if not isinstance(target, dict):
        raise ValueError("migration target must be an object")
    target = dict(target)
    if str(target.get("deviceSecret", "")).strip():
        raise ValueError("target must use ProductSecret pre-registration")
    target["mqttServer"] = _valid_aliyun_mqtt_host(target.get("mqttServer", ""))
    if not target["mqttServer"]:
        raise ValueError("target mqttServer is not an Aliyun host")
    target["mqttPort"] = int(target.get("mqttPort", 1883))
    if target["mqttPort"] != 1883:
        raise ValueError("target mqttPort must be 1883")
    target["deviceSecret"] = ""
    target_config = DeviceConfig(target)
    if not target_config.product_secret:
        raise ValueError("target ProductSecret is required")
    for allowed_host in target_config.ota_allowed_hosts:
        if not _valid_aliyun_mqtt_host(allowed_host):
            raise ValueError("target OTA allowlist contains a non-Aliyun host")
    if (
        target_config.product_key == current_config.product_key
        and target_config.device_name == current_config.device_name
    ):
        raise ValueError("target identity is unchanged")
    confirm = int(data.get("confirmSeconds", MIGRATION_CONFIRM_SECONDS))
    rollback = int(data.get("rollbackSeconds", MIGRATION_ROLLBACK_SECONDS))
    if confirm != MIGRATION_CONFIRM_SECONDS:
        raise ValueError("confirmSeconds must be {}".format(MIGRATION_CONFIRM_SECONDS))
    if rollback != MIGRATION_ROLLBACK_SECONDS or rollback <= confirm:
        raise ValueError("rollbackSeconds must be {}".format(MIGRATION_ROLLBACK_SECONDS))
    return {
        "schema": 1,
        "migrationId": migration_id,
        "source": {
            "productKey": current_config.product_key,
            "deviceName": current_config.device_name,
        },
        "target": target_config.as_dict(include_device_secret=False),
        "confirmSeconds": confirm,
        "rollbackSeconds": rollback,
    }


def _secret_cache_for(config, secret):
    return {
        "productKey": config.product_key,
        "deviceName": config.device_name,
        "deviceSecret": str(secret),
    }


def _write_identity(config_data, secret_data):
    atomic_json_write(DEVICE_CONFIG_FILE, config_data)
    if secret_data:
        atomic_json_write(DEVICE_SECRET_CACHE_FILE, secret_data)
    else:
        remove_file(DEVICE_SECRET_CACHE_FILE)
        remove_file(DEVICE_SECRET_CACHE_FILE + ".tmp")


def _identity_from_state(state, target):
    prefix = "target" if target else "source"
    config_data = state.get(prefix + "Config")
    secret_data = state.get(prefix + "Secret")
    if not isinstance(config_data, dict):
        raise ValueError("migration state missing {} config".format(prefix))
    if secret_data is not None and not isinstance(secret_data, dict):
        raise ValueError("migration state has invalid {} secret".format(prefix))
    _write_identity(config_data, secret_data)


def recover_identity_before_load():
    state = load_migration_state()
    if not state:
        return None
    stage = str(state.get("stage", ""))
    try:
        if stage in ("committed", "confirming", "complete"):
            _identity_from_state(state, True)
        else:
            _identity_from_state(state, False)
    except Exception as error:
        try:
            print("[MIGRATION] boot identity recovery failed: {}".format(error))
        except Exception:
            pass
    return state


class MigrationManager:
    def __init__(self, config, delivery, cloud, logger, reboot):
        self.config = config
        self.delivery = delivery
        self.cloud = cloud
        self.logger = logger
        self.reboot = reboot
        self.stop = False
        self.state = load_migration_state()

    def has_pending(self):
        """Return whether startup must stay in migration-only mode."""
        return bool(self.state or migration_pending_on_boot())

    def start(self):
        if _thread:
            _thread.start_new_thread(self.worker, ())

    def _usb(self, stage, description):
        try:
            print("[MIGRATION] stage={} {}".format(stage, str(description)[:160]))
        except Exception:
            pass

    def _runtime(self, action, description, extra=""):
        self._usb(action, description)
        self.logger.runtime(
            "migration",
            "collector",
            self.config.device_name,
            action,
            "device",
            description,
            extra=extra,
        )

    def _debug(self, code, description, extra=""):
        self._usb("failed", "{}: {}".format(code, description))
        self.logger.error("migration", code, description, extra=extra)

    def _save(self, stage, **values):
        state = dict(self.state or {})
        state.update(values)
        state["stage"] = stage
        self.state = save_migration_state(state)
        return self.state

    def _done(self, status, reason=""):
        migration_id = str((self.state or {}).get("migrationId", ""))
        atomic_json_write(
            MIGRATION_DONE_FILE,
            {
                "migrationId": migration_id,
                "status": status,
                "reason": str(reason)[:160],
                "version": PROJECT_VERSION,
            },
        )
        remove_file(DEVICE_MIGRATION_FILE)
        remove_file(DEVICE_MIGRATION_FILE + ".tmp")
        remove_migration_state()

    def _source_snapshot(self):
        source_secret = _read_json(DEVICE_SECRET_CACHE_FILE)
        return self.config.as_dict(include_device_secret=True), source_secret

    def _abort_precommit(self, code, reason):
        try:
            if self.state and self.state.get("sourceConfig"):
                _identity_from_state(self.state, False)
        except Exception as error:
            reason = "{}; source restore failed: {}".format(reason, error)
        self._debug(code, reason)
        self._done("failed", reason)
        self.state = None
        sleep_ms(200)
        self.reboot()

    def _rollback(self, code, reason):
        self._save("rollback", failureCode=code, failureReason=str(reason)[:160])
        self._debug(code, reason)
        self.cloud.stop = True
        try:
            self.cloud._mark_disconnected()
        except Exception:
            pass
        try:
            _identity_from_state(self.state, False)
            self._done("rolled_back", reason)
        except Exception as error:
            self._debug("ROLLBACK_WRITE", "old identity restore failed", str(error))
            return
        sleep_ms(200)
        self.reboot()

    def _commit(self):
        self._save("committing")
        self._runtime("commit", "Writing validated target identity")
        self.cloud.stop = True
        try:
            self.cloud._mark_disconnected()
        except Exception:
            pass
        _identity_from_state(self.state, True)
        self._save(
            "committed",
            confirmStartedEpoch=epoch_seconds(),
            confirmBoots=0,
        )
        self._runtime("reboot", "Target identity committed; preparing reboot")
        sleep_ms(200)
        self.reboot()

    def _start_new(self, command):
        done = _read_json(MIGRATION_DONE_FILE)
        migration_id = str(command.get("migrationId", ""))
        if done and str(done.get("migrationId", "")) == migration_id:
            self._usb("duplicate", "Ignoring completed migration {}".format(migration_id))
            remove_file(DEVICE_MIGRATION_FILE)
            sleep_ms(200)
            self.reboot()
            return
        normalized = normalize_migration(command, self.config)
        self._runtime(
            "validate",
            "Migration configuration and source identity validated",
            "source={}/{},target={}/{}".format(
                normalized["source"]["productKey"],
                normalized["source"]["deviceName"],
                normalized["target"]["productKey"],
                normalized["target"]["deviceName"],
            ),
        )
        source_config, source_secret = self._source_snapshot()
        self.state = {
            "migrationId": normalized["migrationId"],
            "sourceConfig": source_config,
            "sourceSecret": source_secret,
            "targetConfig": normalized["target"],
            "targetSecret": None,
            "confirmSeconds": normalized["confirmSeconds"],
            "rollbackSeconds": normalized["rollbackSeconds"],
        }
        self._save("preflight")
        target_config = DeviceConfig(normalized["target"])
        self._runtime("register", "Registering target identity over TLS")
        secret = dynamic_register_secret(target_config)
        if not secret:
            raise OSError("target dynamic registration returned no DeviceSecret")
        target_secret = _secret_cache_for(target_config, secret)
        self._save("registered", targetSecret=target_secret)
        probe_id = self.delivery.sequence.next() & 0x7FFFFFFF
        self._runtime("probe", "Probing target MQTT and runtimeLog acknowledgement")
        probe_ok = False
        for attempt in range(3):
            if probe_identity(target_config, secret, probe_id + attempt):
                probe_ok = True
                break
            sleep_ms(2000)
        if not probe_ok:
            raise OSError("target runtimeLog probe was not acknowledged")
        self._save("prepared")
        self._runtime(
            "ready",
            "Target identity preflight passed; business data remains disabled",
        )
        self._commit()

    def _confirm_target(self):
        first_epoch = int(self.state.get("confirmStartedEpoch", 0))
        now_epoch = epoch_seconds()
        elapsed = (
            max(0, now_epoch - first_epoch)
            if first_epoch and now_epoch >= first_epoch
            else 0
        )
        rollback_seconds = int(
            self.state.get("rollbackSeconds", MIGRATION_ROLLBACK_SECONDS)
        )
        confirm_seconds = int(
            self.state.get("confirmSeconds", MIGRATION_CONFIRM_SECONDS)
        )
        remaining_seconds = rollback_seconds - elapsed
        confirm_boots = int(self.state.get("confirmBoots", 0)) + 1
        self._save("confirming", confirmBoots=confirm_boots)
        if remaining_seconds <= 0 or confirm_boots > 3:
            self._rollback("CONFIRM_DEADLINE", "persistent confirmation deadline expired")
            return
        self._runtime(
            "confirm",
            "Target identity booted; business data remains disabled",
        )
        started = ticks_ms()
        stable_started = None
        generation = -1
        probe_id = ""
        last_probe_ms = None
        while not self.stop:
            now = ticks_ms()
            if ticks_diff(now, started) >= remaining_seconds * 1000:
                self._rollback("CONFIRM_TIMEOUT", "target confirmation timed out")
                return
            if not self.cloud.connected:
                stable_started = None
                generation = -1
                probe_id = ""
                sleep_ms(500)
                continue
            if generation != self.cloud.connection_generation:
                generation = self.cloud.connection_generation
                stable_started = None
                probe_id = ""
            if probe_id:
                result = self.cloud.event_result(probe_id)
                if result == 200:
                    stable_started = now
                    probe_id = ""
                    self._runtime("ack", "Target runtimeLog acknowledged")
                elif result is not None:
                    self._rollback(
                        "EVENT_REJECT", "target runtimeLog returned {}".format(result)
                    )
                    return
            elif stable_started is None and (
                last_probe_ms is None or ticks_diff(now, last_probe_ms) >= 3000
            ):
                params = runtime_probe_params(
                    self.config.device_name,
                    self.delivery.sequence.next() & 0x7FFFFFFF,
                )
                probe_id = self.cloud.publish_event_tracked("runtimeLog", params)
                last_probe_ms = now
            if stable_started is not None and ticks_diff(now, stable_started) >= (
                confirm_seconds * 1000
            ):
                self._runtime(
                    "success",
                    "Target identity stable; migration completed",
                )
                self._done("success")
                self.state = None
                sleep_ms(200)
                self.reboot()
                return
            sleep_ms(200)

    def worker(self):
        sleep_ms(1000)
        if self.state:
            stage = str(self.state.get("stage", ""))
            if stage in ("committed", "confirming"):
                try:
                    self._confirm_target()
                except Exception as error:
                    self._rollback("CONFIRM_WORKER", str(error))
                return
            self._abort_precommit(
                "POWER_RECOVERY",
                "pre-commit migration interrupted; old identity restored",
            )
            return
        command = _read_json(DEVICE_MIGRATION_FILE)
        if not command:
            return
        try:
            self._start_new(command)
        except Exception as error:
            self._abort_precommit("PREFLIGHT", str(error))
