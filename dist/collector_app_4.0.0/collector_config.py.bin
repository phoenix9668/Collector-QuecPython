"""Collector application constants and per-device configuration.

The repository intentionally contains no production credential.  A device must
provide /usr/config/device.json before the cloud connection is started.
"""

try:
    import ujson as json
except ImportError:  # host-side tests
    import json

try:
    import uos as os
except ImportError:  # host-side tests
    import os


PROJECT_NAME = "QuecPython_EC600M"
PROJECT_VERSION = "4.0.0"
OTA_MODULE_NAME = "collector_app"
OTA_MULTI_FILE_TARGETS = (
    "/usr/collector_app.py",
    "/usr/collector_cloud.py",
    "/usr/collector_config.py",
    "/usr/collector_log.py",
    "/usr/collector_ota.py",
    "/usr/collector_protocol.py",
    "/usr/collector_queue.py",
    "/usr/collector_sensors.py",
    "/usr/collector_uart.py",
)

DEVICE_CONFIG_FILE = "/usr/config/device.json"
DEVICE_SECRET_CACHE_FILE = "/usr/config/device_secret.json"
SEQUENCE_FILE = "/usr/data/signs_sequence.json"

UART_BAUDRATE = 115200
UART_RX_RING_TARGET = 32 * 1024
UART_RX_RING_MINIMUM = 16 * 1024
FRAME_QUEUE_TARGET = 512
FRAME_QUEUE_FALLBACKS = (512, 256, 128)
SIGNS_FRAME_SIZE = 206
INFLIGHT_LIMIT = 16
ACK_TIMEOUT_MS = 15000
ACK_RETRY_LIMIT = 3

RAM_SPILL_HIGH_WATERMARK = 75
RAM_OTA_LOW_WATERMARK = 25
SPILL_BATCH_FRAMES = 32
FLASH_SPOOL_MAX_BYTES = 4 * 1024 * 1024
FLASH_SPOOL_DIR = "/usr/data"
FLASH_SPOOL_FILE = "/usr/data/signs.queue"
FLASH_SPOOL_META_0 = "/usr/data/signs.meta.0"
FLASH_SPOOL_META_1 = "/usr/data/signs.meta.1"

OTA_MAX_APP_BYTES = 512 * 1024
OTA_DIRECTORY_BYTES = 8 * 1024
OTA_RESERVED_BYTES = 1536 * 1024
FILESYSTEM_SAFETY_BYTES = 256 * 1024
OTA_RESERVE_FILE = "/usr/.ota_space.reserve"
OTA_HEALTH_PENDING_FILE = "/usr/.ota_pending.json"
OTA_HEALTH_OK_FILE = "/usr/.ota_healthy"
OTA_UPDATER_DIR = "/usr/.updater"
OTA_QUIET_SECONDS = 60
OTA_HEALTH_CONFIRM_SECONDS = 90

LOCAL_LOG_DIR = "/usr/log"
LOCAL_LOG_MAX_BYTES = 64 * 1024
LOCAL_LOG_TOTAL_BYTES = 128 * 1024
RUNTIME_STORAGE_ALLOWANCE_BYTES = LOCAL_LOG_TOTAL_BYTES + 32 * 1024

MQTT_PORT = 1883
MQTT_KEEPALIVE = 60
DYNAMIC_REGISTER_TOPIC = "/ext/register"
DYNAMIC_REGISTER_TIMEOUT_MS = 12000
DYNAMIC_REGISTER_POLL_MS = 200
ALIYUN_SIGN_METHOD = "hmacsha256"


def _text(data, key, default=""):
    value = data.get(key, default)
    if value is None:
        return ""
    return str(value).strip()


def _validate_identity(name, value):
    if not value:
        raise ValueError("device config missing {}".format(name))
    if "/" in value or "+" in value or "#" in value:
        raise ValueError("invalid {}".format(name))
    return value


def ensure_dir(path):
    """Create a directory tree on both QuecPython and CPython."""
    if not path:
        return
    current = ""
    absolute = path.startswith("/")
    for part in path.replace("\\", "/").split("/"):
        if not part:
            continue
        if current:
            current += "/" + part
        else:
            current = ("/" if absolute else "") + part
        try:
            os.mkdir(current)
        except Exception:
            pass


def atomic_json_write(path, payload):
    parent = path.replace("\\", "/").rsplit("/", 1)[0]
    ensure_dir(parent)
    temp_path = path + ".tmp"
    with open(temp_path, "w") as stream:
        stream.write(json.dumps(payload))
        try:
            stream.flush()
        except Exception:
            pass
    try:
        os.remove(path)
    except Exception:
        pass
    os.rename(temp_path, path)


class DeviceConfig:
    def __init__(self, values):
        self.product_key = _validate_identity(
            "productKey", _text(values, "productKey")
        )
        self.device_name = _validate_identity(
            "deviceName", _text(values, "deviceName")
        )
        self.mqtt_server = _text(values, "mqttServer")
        if not self.mqtt_server:
            raise ValueError("device config missing mqttServer")
        self.mqtt_port = int(values.get("mqttPort", MQTT_PORT))
        self.device_secret = _text(values, "deviceSecret")
        self.product_secret = _text(values, "productSecret")
        if not self.device_secret and not self.product_secret:
            raise ValueError("deviceSecret or productSecret is required")
        allowed = values.get("otaAllowedHosts", [])
        if not isinstance(allowed, list):
            raise ValueError("otaAllowedHosts must be an array")
        self.ota_allowed_hosts = []
        for host in allowed:
            host = str(host).strip().lower()
            if host:
                self.ota_allowed_hosts.append(host)
        if not self.ota_allowed_hosts:
            self.ota_allowed_hosts = [
                ".aliyuncs.com",
                ".aliyun.com",
            ]
        self.cell_locator_token = _text(values, "cellLocatorToken")

    def topic(self, suffix):
        return "/sys/{}/{}/{}".format(
            self.product_key, self.device_name, suffix
        )

    def ota_topic(self, action):
        return "/ota/device/{}/{}/{}".format(
            action, self.product_key, self.device_name
        )


def load_device_config(path=DEVICE_CONFIG_FILE):
    with open(path, "r") as stream:
        data = json.loads(stream.read())
    if not isinstance(data, dict):
        raise ValueError("device config root must be an object")
    return DeviceConfig(data)


def load_cached_device_secret(config, path=DEVICE_SECRET_CACHE_FILE):
    for candidate in (path, path + ".tmp"):
        try:
            with open(candidate, "r") as stream:
                data = json.loads(stream.read())
        except Exception:
            continue
        if data.get("productKey") != config.product_key:
            continue
        if data.get("deviceName") != config.device_name:
            continue
        secret = _text(data, "deviceSecret")
        if secret:
            return secret
    return ""


def save_cached_device_secret(config, secret, path=DEVICE_SECRET_CACHE_FILE):
    atomic_json_write(
        path,
        {
            "productKey": config.product_key,
            "deviceName": config.device_name,
            "deviceSecret": secret,
        },
    )


def clear_cached_device_secret(path=DEVICE_SECRET_CACHE_FILE):
    removed = False
    for candidate in (path, path + ".tmp"):
        try:
            os.remove(candidate)
            removed = True
        except Exception:
            pass
    return removed
