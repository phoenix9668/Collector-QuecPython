import uos
import fota
import app_fota
import modem
from machine import UART
from machine import Pin
from aLiYun import aLiYun
from misc import Power
import cellLocator
import _thread
import utime
import log
import net
import checkNet
import sim
import ujson

PWM_Led = Pin(Pin.GPIO28, Pin.OUT, Pin.PULL_DISABLE, 1)

PROJECT_NAME = "QuecPython_EC200U_EU"
PROJECT_VERSION = "3.2.2"
# 获取固件型号
# ('sysname=EC200U-EUAA', 'nodename=EC200U', 'release=1.13.0', 'version=v1.12 on Thu_Apr_18_2024_9:45:12_AM', 'machine=EC200U with QUECTEL', 'qpyver=V0001')
FIRMWARE_NAME = uos.uname()[0].split("=")[1]  # EC200U-EUAA
# 获取固件版本号
FIRMWARE_VERSION = modem.getDevFwVersion()  # EC200UEUAAR03A04M08_OCPU_QPY_BETA0418

checknet = checkNet.CheckNetwork(PROJECT_NAME, PROJECT_VERSION)
TaskEnable = True  # 调用disconnect后会通过该状态回收线程资源
msg_id = 0
state = 0
mqtt_sub_msg = {}
signs_data = {}
ota_module = None

log.basicConfig(level=log.INFO)
app_log = log.getLogger("app_log")

# ProductKey = "he2maYabo9j"  # 产品标识
# ProductSecret = None  # 产品密钥（使用一机一密认证时此参数传入None，参照阿里 IoT 平台应用开发指导)
# DeviceName = "BW-XC-200-EU-001"  # 设备名称
# DeviceSecret = "1ef1e7b1db82814363c4aa1cf01faee3"

ProductKey = "k2a2jDf1SSW"  # 产品标识
ProductSecret = (
    None  # 产品密钥（使用一机一密认证时此参数传入None，参照阿里 IoT 平台应用开发指导)
)
DeviceName = "BW-XC-200-EU-001"  # 设备名称
DeviceSecret = "65771c6a0e569d20ac23254a3bbef6bd"
# DeviceName = "BW-XC-200-EU-002"  # 设备名称
# DeviceSecret = "0c07a467d808a104ab2c218c0aa52ac3"
MqttServer = "iot-31z00i0fhc1e85a.mqtt.iothub.aliyuncs.com"  # MQTT服务器地址
clientID = modem.getDevImei()


property_subscribe_topic = (
    "/sys" + "/" + ProductKey + "/" + DeviceName + "/" + "thing/service/property/set"
)
property_publish_topic = (
    "/sys" + "/" + ProductKey + "/" + DeviceName + "/" + "thing/event/property/post"
)
# 初始化OTA相关Topic
# 设备模块版本信息上报topic
ota_topic_device_inform = "/ota/device/inform/%s/%s" % (ProductKey, DeviceName)
# 设备OTA升级计划下发topic
ota_topic_device_upgrade = "/ota/device/upgrade/%s/%s" % (
    ProductKey,
    DeviceName,
)
# 设备升级进度上报topic
ota_topic_device_progress = "/ota/device/progress/%s/%s" % (
    ProductKey,
    DeviceName,
)
# 设备OTA升级计划查询topic
ota_topic_firmware_get = "/sys/%s/%s/thing/ota/firmware/get" % (
    ProductKey,
    DeviceName,
)
# 设备OTA升级计划查询应答topic
ota_topic_firmware_get_reply = "/sys/%s/%s/thing/ota/firmware/get_reply" % (
    ProductKey,
    DeviceName,
)

msg_temperature_humidity = """{{
                                "id": "{0}",
                                "version": "1.0",
                                "params": {{
                                    "Temperature": {{
                                        "value": {1}
                                    }},
                                    "Humidity": {{
                                        "value": {2}
                                    }}
                                }},
                                "method": "thing.event.property.post"
                             }}"""

msg_voltage = """{{
                    "id": "{0}",
                    "version": "1.0",
                    "params": {{
                        "BatteryVoltage": {{
                            "value": {1}
                        }},
                        "PowerVoltage": {{
                            "value": {2}
                        }}
                    }},
                    "method": "thing.event.property.post"
                 }}"""

msg_cellLocator = """{{
                    "id": "{0}",
                    "version": "1.0",
                    "params": {{
                        "CellLocator": {{
                            "Longitude": {{
                                "value": {1}
                            }},
                            "Latitude": {{
                                "value": {2}
                            }},
                            "Accuracy": {{
                            "value": {3}
                            }}
                        }}
                    }},
                    "method": "thing.event.property.post"
                 }}"""

msg_sim = """{{
                "id": "{0}",
                "version": "1.0",
                "params": {{
                    "product_information:IMSI": {{
                        "value": "{1}"
                    }},
                    "product_information:ICCID": {{
                        "value": "{2}"
                    }}
                }},
                "method": "thing.event.property.post"
             }}"""

msg_product_info_SendCommand = """{{
                                    "id": "{0}",
                                    "version": "1.0",
                                    "params": {{
                                        "product_information:SendCommand": {{
                                            "value": "{1}"
                                        }}
                                    }},
                                    "method": "thing.event.property.post"
                                 }}"""

msg_product_info_CollectorID = """{{
                                    "id": "{0}",
                                    "version": "1.0",
                                    "params": {{
                                        "product_information:CollectorID": {{
                                            "value": "{1}"
                                        }}
                                    }},
                                    "method": "thing.event.property.post"
                                 }}"""

msg_product_info_StatusInfo = """{{
                                    "id": "{0}",
                                    "version": "1.0",
                                    "params": {{
                                        "product_information:StatusInfo": {{
                                            "value": "{1}"
                                        }}
                                    }},
                                    "method": "thing.event.property.post"
                                 }}"""

msg_product_info_NetStatus = """{{
                                    "id": "{0}",
                                    "version": "1.0",
                                    "params": {{
                                        "product_information:NetStatus": {{
                                            "product_information:StageCode": {{
                                                "value": "{1}"
                                            }},
                                            "product_information:SubCode": {{
                                                "value": "{2}"
                                            }}
                                        }}
                                    }},
                                    "method": "thing.event.property.post"
                                 }}"""

msg_geoLocation = """{{
                            "id": "{0}",
                            "version": "1.0",
                            "params": {{
                                "GeoLocation": {{
                                    "Longitude": {{
                                        "value": {1}
                                    }},
                                    "Latitude": {{
                                        "value": {2}
                                    }},
                                    "Altitude": {{
                                        "value": {3}
                                    }},
                                    "CoordinateSystem": {{
                                        "value": {4}
                                    }}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

msg_signs_data = """{{
                            "id": "{0}",
                            "version": "1.0",
                            "params": {{
                                "collar_information:SignsData": {{
                                    "collar_information:CollectorID": {{
                                        "value": "{1}"
                                    }},
                                    "collar_information:RFID": {{
                                        "value": "{2}"
                                    }},
                                    "collar_information:GUID": {{
                                        "value": "{3}"
                                    }},
                                    "collar_information:restArray": {{
                                        "value": "{4}"
                                    }},
                                    "collar_information:ingestionArray": {{
                                        "value": "{5}"
                                    }},
                                    "collar_information:movementArray": {{
                                        "value": "{6}"
                                    }},
                                    "collar_information:climbArray": {{
                                        "value": "{7}"
                                    }},
                                    "collar_information:ruminateArray": {{
                                        "value": "{8}"
                                    }},
                                    "collar_information:otherArray": {{
                                        "value": "{9}"
                                    }},
                                    "collar_information:Stage": {{
                                        "value": {10}
                                    }},
                                    "collar_information:BatteryVoltage": {{
                                        "value": {11}
                                    }},
                                    "collar_information:ResetCnt": {{
                                        "value": {12}
                                    }},
                                    "collar_information:SignalStrength": {{
                                        "value": {13}
                                    }},
                                    "collar_information:UTCtime": {{
                                        "value": "{14}"
                                    }}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""


def watch_dog_task():
    while True:
        if PWM_Led.read():
            PWM_Led.write(0)
        else:
            PWM_Led.write(1)
        utime.sleep(1)


class Uart2(object):
    def __init__(
        self,
        no=UART.UART2,
        bate=115200,
        data_bits=8,
        parity=0,
        stop_bits=1,
        flow_control=0,
    ):
        self.uart = UART(no, bate, data_bits, parity, stop_bits, flow_control)
        self.uart.set_callback(self.callback)
        self.buffer = bytearray()  # 初始化缓冲区

    def set_modbus_rtu_instance(self, modbus_rtu_instance):
        self.modbus_rtu = modbus_rtu_instance

    def callback(self, para):
        app_log.debug("call para:{}".format(para))
        if 0 == para[0]:
            self.uartRead(para[2])

    def uartWrite(self, msg):
        hex_msg = [hex(x) for x in msg]
        app_log.debug("Write msg:{}".format(hex_msg))
        self.uart.write(msg)

    def uartRead(self, len):
        if len > 0:
            app_log.debug("len: {}".format(len))
            msg = self.uart.read(len)
            hex_msg = [hex(x) for x in msg]
            app_log.debug("uart2_read msg: {}".format(hex_msg))
            self.buffer += msg  # 追加读取的新数据到缓冲区
            app_log.debug("uart2 all buffer: {}".format(self.buffer))

            while True:
                frame_processed = self.process_buffer()
                if not frame_processed:
                    break
        else:
            utime.sleep_ms(10)

    def process_buffer(self):
        INFO_HEADER_PREFIX = b"\x23\x23"  # 帧头
        MIN_LENGTH = 9  # 最小帧长度，用于初始条件

        app_log.debug("buffer length: {}".format(len(self.buffer)))
        if len(self.buffer) < 9:
            return False  # 数据不足以构成任何帧

        i = 0
        while i <= len(self.buffer) - MIN_LENGTH:
            if self.buffer[i : i + 2] == INFO_HEADER_PREFIX:
                self.process_frame(self.buffer[i:])
                buffer_list = list(self.buffer)
                buffer_list.clear()
                self.buffer = bytearray(buffer_list)
            elif self.buffer[i] == 0x0E:
                if len(self.buffer) >= i + 6 and self.buffer[i + 4] == 0xB0:
                    if len(self.buffer) >= i + 206:
                        self.process_frame(self.buffer[i : i + 206])
                        buffer_list = list(self.buffer)
                        del buffer_list[: i + 206]
                        self.buffer = bytearray(buffer_list)
                        return True
                elif len(self.buffer) >= i + 6 and self.buffer[i + 4] == 0xB2:
                    self.process_frame(self.buffer[i : i + 9])
                    buffer_list = list(self.buffer)
                    del buffer_list[: i + 9]
                    self.buffer = bytearray(buffer_list)
                    return True
            i += 1

        return False

    def process_frame(self, frame):
        global msg_id, signs_data
        # 根据帧的内容处理帧数据
        # 这里应该添加帧的具体处理逻辑，可以加入帧类型分辞
        if frame[0] == 0x0E and frame[4] == 0xB0:
            hex_msg = [hex(x) for x in frame]
            signs_data["collector_id"] = hex_to_str(hex_msg[0:4], " ")
            signs_data["rfid"] = hex_to_str(hex_msg[5:11], " ")
            signs_data["guid"] = hex_to_str(hex_msg[11:43], " ")

            signs_data["rest_array"] = hex_to_str(hex_msg[43:67], " ")
            signs_data["ingestion_array"] = hex_to_str(hex_msg[67:91], " ")
            signs_data["movement_array"] = hex_to_str(hex_msg[91:115], " ")
            signs_data["climb_array"] = hex_to_str(hex_msg[115:139], " ")
            signs_data["ruminate_array"] = hex_to_str(hex_msg[139:163], " ")
            signs_data["other_array"] = hex_to_str(hex_msg[163:187], " ")

            signs_data["stage"] = int(hex_msg[187])
            signs_data["battery_voltage"] = battery_pct(
                (int(hex_msg[188]) << 8) | int(hex_msg[189])
            )
            signs_data["reset_cnt"] = (int(hex_msg[190]) << 8) | int(hex_msg[191])
            signs_data["signal_strength"] = calc_rssi_dbm(int(hex_msg[192]))
            signs_data["utc_time"] = int(round(utime.mktime(utime.localtime()) * 1000))
            app_log.debug("signs_data: {}".format(signs_data))

            msg_id += 1
            ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_signs_data.format(
                    msg_id,
                    signs_data["collector_id"],
                    signs_data["rfid"],
                    signs_data["guid"],
                    signs_data["rest_array"],
                    signs_data["ingestion_array"],
                    signs_data["movement_array"],
                    signs_data["climb_array"],
                    signs_data["ruminate_array"],
                    signs_data["other_array"],
                    signs_data["stage"],
                    signs_data["battery_voltage"],
                    signs_data["reset_cnt"],
                    signs_data["signal_strength"],
                    signs_data["utc_time"],
                ).encode("utf-8"),
            )
        elif frame[0] == 0x0E and frame[4] == 0xB2:
            battery_voltage = float(
                "%.3f" % (((frame[5] << 8) | frame[6]) / 32768 * 4.096 * 11)
            )
            app_log.info("battery_voltage = {}".format(battery_voltage))

            voltage = float(
                "%.3f" % (((frame[7] << 8) | frame[8]) / 32768 * 4.096 * 21)
            )
            app_log.info("voltage = {}".format(voltage))

            msg_id += 1
            ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_voltage.format(msg_id, battery_voltage, voltage).encode("utf-8"),
            )
        elif frame[0] == 0x23 and frame[1] == 0x23:
            msg_id += 1
            ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_product_info_StatusInfo.format(msg_id, self.buffer).encode("utf-8"),
            )
            if "##Read Memory Complete##" in self.buffer:
                collector_id = self.buffer[18:26]
                app_log.info(collector_id)
                msg_id += 1
                ali.publish(
                    property_publish_topic.encode("utf-8"),
                    msg_product_info_CollectorID.format(msg_id, collector_id).encode(
                        "utf-8"
                    ),
                )


def hex_to_str(a, b=""):
    string = "".join([hex_byte.replace("0x", b) for hex_byte in a])
    return string


def str_to_hex(s):
    list_hex = " ".join([hex(ord(c)) for c in s]).split()
    list_temp = [int(i, 16) for i in list_hex]
    byte_array = bytearray(list_temp)
    return byte_array


def calc_rssi_dbm(rssi_dec):
    """Calc the RSSI value to RSSI dBm"""
    rssi_offset = 74
    if rssi_dec >= 128:
        rssi_dbm = (rssi_dec - 256) / 2 - rssi_offset
    else:
        rssi_dbm = (rssi_dec / 2) - rssi_offset
    return float("%.2f" % rssi_dbm)


def battery_pct(battery_level):
    """Calc the battery level to battery pct"""
    result_pct = float("%.2f" % ((2 * (battery_level / 4096) * 3 - 3.0) / (4.2 - 3.0)))
    if result_pct >= 1.0:
        result_pct = 1.0
    elif result_pct < 0.0:
        result_pct = 0.0
    return result_pct


def cell_location_task():
    global msg_id
    while True:
        utime.sleep(86400)
        cell_location = cellLocator.getLocation(
            "www.queclocator.com", 80, "qa6qTK91597826z6", 8, 1
        )
        msg_id += 1
        ali.publish(
            property_publish_topic.encode("utf-8"),
            msg_cellLocator.format(
                msg_id, cell_location[0], cell_location[1], cell_location[2]
            ).encode("utf-8"),
        )


def sim_task():
    global msg_id
    while True:
        sim_imsi = sim.getImsi()
        sim_iccid = sim.getIccid()
        msg_id += 1
        ali.publish(
            property_publish_topic.encode("utf-8"),
            msg_sim.format(msg_id, sim_imsi, sim_iccid).encode("utf-8"),
        )
        utime.sleep(7200)


def power_restart():
    while True:
        utime.sleep(14400)
        Power.powerRestart()


def mqtt_sub_cb(topic, data):
    global ota_module
    topic_str = topic.decode()  # 将 bytes 转换为 str
    app_log.info(
        "Subscribe Recv: Topic={},Data={}".format(topic.decode(), data.decode())
    )
    if topic_str in (ota_topic_device_upgrade, ota_topic_firmware_get_reply):
        # OTA升级
        data = ujson.loads(data)
        app_log.info("data = {}".format(data))
        ota_module = data["data"]["module"]
        app_log.info("ota_module = {}".format(ota_module))
        ota_version = data["data"]["version"]
        if ota_module == FIRMWARE_NAME:
            # FOTA升级
            _fota = fota()
            _fota.httpDownload(url1=data.get("url"), callback=fota_callback)
        elif ota_module == PROJECT_NAME:
            # SOTA升级
            # 解析 extData 中的文件信息
            package_info = ujson.loads(data["data"]["extData"]["_package_udi"])
            ota_data = []

            # 构建下载列表
            for bin_file, py_path in package_info["files"].items():
                download_item = {
                    "url": data["data"]["url"],
                    "file_name": py_path,  # 使用指定的路径
                }
                ota_data.append(download_item)
            app_log.info("ota_data = {}".format(ota_data))
            _app_fota = app_fota.new()
            res = _app_fota.bulk_download(ota_data)
            app_log.info("res = {}".format(res))
            # SOTA上报升级结果
            ota_process = 100 if not res else -1
            if ota_process == 100:
                _app_fota.set_update_flag()
            process_data = {
                "id": 5,
                "params": {
                    "step": ota_process,
                    "desc": "desc",
                    "module": ota_module,
                },
            }
            ali.publish(ota_topic_device_progress, ujson.dumps(process_data), qos=1)
            # 升级完成之后需要重启设备
            Power.powerRestart()


# FOTA升级进度回调函数
def fota_callback(args):
    print("Download status: %s, Download process: %s" % tuple(args))
    ota_process = None
    if args[0] in (0, 1, 2) and args[1] == 100:
        ota_process = 100
    else:
        ota_process = -1
    if ota_process is not None:
        # FOTA上报升级结果
        process_data = {
            "id": 5,
            "params": {
                "step": ota_process,
                "desc": "success",
                "module": ota_module,
            },
        }
        ali.publish(ota_topic_device_progress, ujson.dumps(process_data), qos=1)
        # 升级成功后会自动重启, 此处无需手动重启
        # Power.powerRestart()


def chect_net_task():
    while True:
        stagecode, subcode = checknet.wait_network_connected(30)
        if stagecode == 3 and subcode == 1:
            app_log.info("Network connection good!")
        else:
            app_log.info(
                "Network connection failed, stage={}, state={}".format(
                    stagecode, subcode
                )
            )
            net.setModemFun(0)  # 关闭网络
            utime.sleep(5)
            net.setModemFun(1)  # 打开网络
        utime.sleep(7200)


if __name__ == "__main__":
    utime.sleep(5)
    checknet.poweron_print_once()
    stagecode, subcode = checknet.wait_network_connected(30)
    if stagecode == 3 and subcode == 1:
        app_log.info("Network connection successful!")

        uart2_inst = Uart2()

        _thread.start_new_thread(watch_dog_task, ())

        # 创建aliyun连接对象
        ali = aLiYun(ProductKey, ProductSecret, DeviceName, DeviceSecret, MqttServer)

        # 设置mqtt连接属性
        ali.setMqtt(clientID, clean_session=False, keepAlive=300)

        # 设置消息回调
        ali.setCallback(mqtt_sub_cb)

        # 订阅主题
        app_log.info(
            "Connected to aliyun, subscribed to: {}".format(property_subscribe_topic)
        )
        ali.subscribe(property_subscribe_topic.encode("utf-8"))
        ali.subscribe(ota_topic_device_upgrade, 1)
        ali.subscribe(ota_topic_firmware_get_reply, 1)
        ali.start()
        msg_id += 1
        ali.publish(
            property_publish_topic.encode("utf-8"),
            msg_product_info_NetStatus.format(msg_id, stagecode, subcode).encode(
                "utf-8"
            ),
        )
        # 上报软件版本信息
        sota_data = {
            "id": 1,
            "params": {
                "version": PROJECT_VERSION,
                "module": PROJECT_NAME,
            },
        }
        ali.publish(ota_topic_device_inform, ujson.dumps(sota_data), qos=1)

        # 上报固件版本信息
        fota_data = {
            "id": 2,
            "params": {
                "version": FIRMWARE_VERSION,
                "module": FIRMWARE_NAME,
            },
        }
        ali.publish(ota_topic_device_inform, ujson.dumps(fota_data), qos=1)

        # 查询软件升级计划
        sota_query = {
            "id": 3,
            "version": "1.0",
            "params": {
                "module": PROJECT_NAME,
            },
            "method": "thing.ota.firmware.get",
        }
        ali.publish(ota_topic_firmware_get, ujson.dumps(sota_query), qos=1)

        # 查询固件升级计划
        sota_query = {
            "id": 4,
            "version": "1.0",
            "params": {
                "module": FIRMWARE_NAME,
            },
            "method": "thing.ota.firmware.get",
        }
        ali.publish(ota_topic_firmware_get, ujson.dumps(sota_query), qos=1)

        _thread.start_new_thread(cell_location_task, ())
        _thread.start_new_thread(sim_task, ())
        _thread.start_new_thread(chect_net_task, ())

        while True:
            utime.sleep(1200)
    else:
        app_log.error(
            "Network connection failed! stagecode = {}, subcode = {}".format(
                stagecode, subcode
            )
        )
