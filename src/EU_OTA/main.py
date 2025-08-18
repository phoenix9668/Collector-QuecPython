import modem
from machine import UART
from machine import I2C
from machine import Pin
from machine import WDT
from machine import Timer
from aLiYun import aLiYun
from misc import Power
import dataCall
import cellLocator
import _thread
import utime
import log
import net
import checkNet
import sim
import ujson

PWM_Led = Pin(Pin.GPIO28, Pin.OUT, Pin.PULL_DISABLE, 1)
timer = Timer(Timer.Timer1)

PROJECT_NAME = "QuecPython_EC200U_EUAA"
PROJECT_VERSION = "3.6.6"
# 用户需要配置的APN信息，根据实际情况修改
# usrConfig = {"apn": "ctm-mobile", "username": "", "password": ""}  # KT
# usrConfig = {"apn": "smgftyz.grevpdn.zj", "username": "", "password": ""}  # 电信卡
# usrConfig = {'apn': 'shqnxx04.shm2mapn', 'username': '', 'password': ''}  # 联通卡

checknet = checkNet.CheckNetwork(PROJECT_NAME, PROJECT_VERSION)
msg_id = 0
state = True
mqtt_sub_msg = {}
signs_data = {}
ali = None

log.basicConfig(level=log.INFO)
app_log = log.getLogger("app_log")

ProductKey = "he2maYabo9j"  # 产品标识
ProductSecret = (
    None  # 产品密钥（使用一机一密认证时此参数传入None，参照阿里 IoT 平台应用开发指导)
)
# DeviceName = "BW-XC-200-EU-001"  # 设备名称
# DeviceSecret = "1ef1e7b1db82814363c4aa1cf01faee3"
# DeviceName = "BW-XC-200-EU-002"  # 设备名称
# DeviceSecret = "081223b7baaabd327d2ab274d84ea35f"
DeviceName = "BW-XC-200-EU-003"  # 设备名称
DeviceSecret = "ad28f59dd9e3044c3d739ed1819cd727"
MqttServer = "iot-06z00dcnrlb8g5r.mqtt.iothub.aliyuncs.com"

# ProductKey = "k2a2jDf1SSW"  # 产品标识
# ProductSecret = (
#     None  # 产品密钥（使用一机一密认证时此参数传入None，参照阿里 IoT 平台应用开发指导)
# )
# DeviceName = "BW-XC-200-EU-001"  # 设备名称
# DeviceSecret = "65771c6a0e569d20ac23254a3bbef6bd"
# DeviceName = "BW-XC-200-EU-002"  # 设备名称
# DeviceSecret = "8d1e6abe259f015395dac11afd690afe"
# MqttServer = "iot-31z00i0fhc1e85a.mqtt.iothub.aliyuncs.com"  # MQTT服务器地址
clientID = modem.getDevImei()

property_subscribe_topic = (
    "/sys" + "/" + ProductKey + "/" + DeviceName + "/" + "thing/service/property/set"
)
property_publish_topic = (
    "/sys" + "/" + ProductKey + "/" + DeviceName + "/" + "thing/event/property/post"
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


class SGM58031Class:
    """SGM58031 class, incloud read and write function"""

    def __init__(self, addr=0x48, alise="SGM58031"):
        # Initialization command
        self.i2c_dev = I2C(I2C.I2C0, I2C.STANDARD_MODE)  # Return I2C object
        self.i2c_addr = addr

        self.register_map = {
            "Conversion_Register": 0x0000,
            "Config_Register": 0x0000,
            "Low_Thresh_Register": 0x8000,
            "High_Thresh_Register": 0x7FFF,
            "Config1_Register": 0x0000,
            "Chip_ID_Register": 0x0000,
            "GN_Trim1_Register": 0x0000,
        }

        self.register_addr = {
            "CONVERSION_REG": 0x00,
            "CONF_REG": 0x01,
            "LOW_THRESH_REG": 0x02,
            "HIGH_THRESH_REG": 0x03,
            "CONF1_REG": 0x04,
            "CHIP_ID_REG": 0x05,
            "GN_TRIM1_REG": 0x06,
        }

        self.os_sel = {
            "No_Effect": 0x00,  # 0 = No effect
            # 1 = Start a single conversion (when in single-shot mode)
            "Start_Single_Conversion": 0x01,
        }

        self.mux_sel = {
            "AINP_AIN0_AND_AINN_AIN1": 0x00,  # 000 = AINP = AIN0 and AINN = AIN1 (default)
            "AINP_AIN0_AND_AINN_AIN3": 0x01,  # 001 = AINP = AIN0 and AINN = AIN3
            "AINP_AIN1_AND_AINN_AIN3": 0x02,  # 010 = AINP = AIN1 and AINN = AIN3
            "AINP_AIN2_AND_AINN_AIN3": 0x03,  # 011 = AINP = AIN2 and AINN = AIN3
            "AINP_AIN0_AND_AINN_GND": 0x04,  # 100 = AINP = AIN0 and AINN = GND
            "AINP_AIN1_AND_AINN_GND": 0x05,  # 101 = AINP = AIN1 and AINN = GND
            "AINP_AIN2_AND_AINN_GND": 0x06,  # 110 = AINP = AIN2 and AINN = GND
            "AINP_AIN3_AND_AINN_GND": 0x07,
        }  # 111 = AINP = AIN3 and AINN = GND

        self.pga_sel = {
            "FS_6_144V": 0x00,  # 000 = FS = +/-6.144V
            "FS_4_096V": 0x01,  # 001 = FS = +/-4.096V
            "FS_2_048V": 0x02,  # 010 = FS = +/-2.048V (default)
            "FS_1_024V": 0x03,  # 011 = FS = +/-1.024V
            "FS_0_512V": 0x04,  # 100 = FS = +/-0.512V
            "FS_0_256V": 0x05,
        }  # 101 = FS = +/-0.256V

        self.mode_sel = {
            "Continuous_Conversion_Mode": 0x00,  # 0 = Continuous conversion mode
            # 1 = Power-down single-shot mode (default)
            "Single_Shot_Mode": 0x01,
        }

        # | DR[2:0]Bits        | DR_SEL Bit in Config1Register |
        # | in Config Register | DR_SEL = 0 | DR_SEL = 1 |
        # | 000                | 6.25Hz     | 7.5Hz      |
        # | 001                | 12.5Hz     | 15Hz       |
        # | 010                | 25Hz       | 30Hz       |
        # | 011                | 50Hz       | 60Hz       |
        # | 100                | 100Hz      | 120Hz      |
        # | 101                | 200Hz      | 240Hz      |
        # | 110                | 400Hz      | 480Hz      |
        # | 111                | 800Hz      | 960HZ      |
        self.dr_sel = {
            "DR_6_25Hz_7_5Hz": 0x00,
            "DR_12_5Hz_15Hz": 0x01,
            "DR_25Hz_30Hz": 0x02,
            "DR_50Hz_60Hz": 0x03,
            "DR_100Hz_120Hz": 0x04,
            "DR_200Hz_240Hz": 0x05,
            "DR_400Hz_480Hz": 0x06,
            "DR_800Hz_960Hz": 0x07,
        }

        self.comp_mode_sel = {
            "Traditional_Comparator": 0x00,  # 0 = A traditional comparator with hysteresis (default)
            "Window_Comparator": 0x01,
        }  # 1 = A window comparator

        self.comp_pol_sel = {"Active_Low": 0x00, "Active_High": 0x01}
        self.comp_lat_sel = {"Non_Latching": 0x00, "Latching": 0x01}

        self.comp_que_sel = {
            "Assert_After_One_Conversion": 0x00,  # 00 = Assert after one conversion
            "Assert_After_Two_Conversions": 0x01,  # 01 = Assert after two conversions
            "Assert_After_Four_Conversions": 0x02,  # 10 = Assert after four conversions
            # Disable comparator (default)
            "Disable_Comparator": 0x03,
        }

        self.dr_sel_sel = {
            "DR_SEL0": 0x00,  # 0 = DR[2:0] = 000 ~ 111 for conversion rate of 6.25Hz, 12.5Hz, 25Hz,
            # 50Hz, 100Hz, 200Hz, 400Hz and 800Hz (default)
            # 1 = DR[2:0] = 000 ~ 111 for conversion rate of 7.5Hz, 15Hz, 30Hz,
            "DR_SEL1": 0x01,
        }
        # 60Hz, 120Hz, 240Hz, 480Hz and 960Hz
        self.battery_voltage = None
        self.voltage = None
        self.flip_sign = True
        self.config_reg_init()
        pass

    def config_reg_init(self, low_thresh_register=0x8000, high_thresh_register=0x7FFF):
        # Initialise the SGM58031 registers
        self.register_map["Config_Register"] = (
            (self.os_sel["Start_Single_Conversion"] << 15 & 0x8000)
            | (self.mux_sel["AINP_AIN2_AND_AINN_AIN3"] << 12 & 0x7000)
            | (self.pga_sel["FS_4_096V"] << 9 & 0x0E00)
            | (self.mode_sel["Single_Shot_Mode"] << 8 & 0x0100)
            | (self.dr_sel["DR_800Hz_960Hz"] << 5 & 0x00E0)
            | (self.comp_mode_sel["Traditional_Comparator"] << 4 & 0x0010)
            | (self.comp_pol_sel["Active_Low"] << 3 & 0x0008)
            | (self.comp_lat_sel["Non_Latching"] << 2 & 0x0004)
            | (self.comp_que_sel["Disable_Comparator"])
        )
        self.register_map["Low_Thresh_Register"] = low_thresh_register
        self.register_map["High_Thresh_Register"] = high_thresh_register
        self.register_map["Config1_Register"] = self.dr_sel_sel["DR_SEL0"] << 7 & 0x0080
        pass

    def self_verifying(self):
        # -1- Read_CHIP_ID
        tmp = self.read_register([self.register_addr["CHIP_ID_REG"]], 2)
        print("####################################################################")
        print(
            "----------------------------CHIP_ID = %x----------------------------"
            % ((tmp[0] << 8) | tmp[1])
        )
        if ((tmp[0] << 8) | tmp[1]) != 0x80:
            return False
        # -2- Set the Configuration Register
        self.write_register(
            [self.register_addr["CONF_REG"]],
            [
                self.register_map["Config_Register"] >> 8,
                (self.register_map["Config_Register"] & 0xFF),
            ],
        )
        # -3- Read the Configuration Register
        tmp = self.read_register([self.register_addr["CONF_REG"]], 2)
        print(
            "--------------------------CONF_REG = %x---------------------------"
            % ((tmp[0] << 8) | tmp[1])
        )
        if (((tmp[0] << 8) | tmp[1]) & 0x7FFF) != (
            self.register_map["Config_Register"] & 0x7FFF
        ):
            return False

        # -4- Set the Configuration1 Register
        self.write_register(
            [self.register_addr["CONF1_REG"]],
            [
                self.register_map["Config1_Register"] >> 8,
                (self.register_map["Config1_Register"] & 0xFF),
            ],
        )
        # -5- Read the Configuration1 Register
        tmp = self.read_register([self.register_addr["CONF1_REG"]], 2)
        print(
            "----------------------------CONF1_REG = %x---------------------------"
            % ((tmp[0] << 8) | tmp[1])
        )
        if ((tmp[0] << 8) | tmp[1]) != self.register_map["Config1_Register"]:
            return False

        # -6- Set the Low Thresh Register
        self.write_register(
            [self.register_addr["LOW_THRESH_REG"]],
            [
                self.register_map["Low_Thresh_Register"] >> 8,
                (self.register_map["Low_Thresh_Register"] & 0xFF),
            ],
        )
        # -7- Read the Low Thresh Register
        tmp = self.read_register([self.register_addr["LOW_THRESH_REG"]], 2)
        print(
            "---------------------Low_Thresh_Register = %x---------------------"
            % ((tmp[0] << 8) | tmp[1])
        )
        if ((tmp[0] << 8) | tmp[1]) != self.register_map["Low_Thresh_Register"]:
            return False

        # -8- Set the High Thresh Register
        self.write_register(
            [self.register_addr["HIGH_THRESH_REG"]],
            [
                self.register_map["High_Thresh_Register"] >> 8,
                (self.register_map["High_Thresh_Register"] & 0xFF),
            ],
        )
        # -9- Read the High Thresh Register
        tmp = self.read_register([self.register_addr["HIGH_THRESH_REG"]], 2)
        print(
            "--------------------High_Thresh_Register = %x---------------------"
            % ((tmp[0] << 8) | tmp[1])
        )
        if ((tmp[0] << 8) | tmp[1]) != self.register_map["High_Thresh_Register"]:
            return False
        print("--------------------sgm58031 initial completed~!--------------------")
        print("####################################################################")
        return True

    def write_register(self, register_addr, bytes_list):
        self.i2c_dev.write(
            self.i2c_addr,
            bytearray(register_addr),
            len(register_addr),
            bytearray(bytes_list),
            len(bytes_list),
        )

    def read_register(self, register_addr, length):
        bytes_list = [0x00 for _ in range(length)]
        bytes_list = bytearray(bytes_list)
        self.i2c_dev.write(
            self.i2c_addr,
            bytearray(register_addr),
            len(register_addr),
            bytearray(0x00),
            0,
        )
        self.i2c_dev.read(self.i2c_addr, bytearray(0x00), 0, bytes_list, length, 0)
        return list(bytes_list)

    def measure_adc_value(self):
        # -1- read the OS of Configuration Register
        global msg_id, state
        tmp = self.read_register([self.register_addr["CONF_REG"]], 2)
        if tmp[0] >> 7 == 1:
            if self.flip_sign:
                tmp = self.read_register([self.register_addr["CONVERSION_REG"]], 2)
                self.battery_voltage = float(
                    "%.3f" % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 11)
                )
                app_log.info("battery_voltage = {}".format(self.battery_voltage))
            else:
                tmp = self.read_register([self.register_addr["CONVERSION_REG"]], 2)
                self.voltage = float(
                    "%.3f" % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 21)
                )
                app_log.info("voltage = {}".format(self.voltage))

            msg_id += 1
            state = ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_voltage.format(msg_id, self.battery_voltage, self.voltage).encode(
                    "utf-8"
                ),
            )

        # -2- Initialise the SGM58031 peripheral
        if self.flip_sign:
            self.register_map["Config_Register"] = (
                self.register_map["Config_Register"] & 0x8FFF
            ) | (self.mux_sel["AINP_AIN0_AND_AINN_AIN1"] << 12 & 0x7000)
        else:
            self.register_map["Config_Register"] = (
                self.register_map["Config_Register"] & 0x8FFF
            ) | (self.mux_sel["AINP_AIN2_AND_AINN_AIN3"] << 12 & 0x7000)
        self.flip_sign = not self.flip_sign

        # -3- Set the Configuration Register
        self.write_register(
            [self.register_addr["CONF_REG"]],
            [
                self.register_map["Config_Register"] >> 8,
                (self.register_map["Config_Register"] & 0xFF),
            ],
        )


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
        global msg_id, signs_data, state
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
            try:
                msg_id += 1
                state = ali.publish(
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
                app_log.debug("state 1 = {}".format(state))
            except Exception as e:
                app_log.error("Failed to publish signs_data info: {}".format(e))
        elif frame[0] == 0x0E and frame[4] == 0xB2:
            battery_voltage = float(
                "%.3f" % (((frame[5] << 8) | frame[6]) / 32768 * 4.096 * 11)
            )
            app_log.info("battery_voltage = {}".format(battery_voltage))

            voltage = float(
                "%.3f" % (((frame[7] << 8) | frame[8]) / 32768 * 4.096 * 21)
            )
            app_log.info("voltage = {}".format(voltage))

            try:
                msg_id += 1
                state = ali.publish(
                    property_publish_topic.encode("utf-8"),
                    msg_voltage.format(msg_id, battery_voltage, voltage).encode(
                        "utf-8"
                    ),
                )
                app_log.debug("state 2 = {}".format(state))
            except Exception as e:
                app_log.error("Failed to publish voltage info: {}".format(e))
        elif frame[0] == 0x23 and frame[1] == 0x23:
            try:
                msg_id += 1
                state = ali.publish(
                    property_publish_topic.encode("utf-8"),
                    msg_product_info_StatusInfo.format(msg_id, self.buffer).encode(
                        "utf-8"
                    ),
                )
                app_log.debug("state 3 = {}".format(state))
            except Exception as e:
                app_log.error("Failed to publish StatusInfo info: {}".format(e))
            if "##Read Memory Complete##" in self.buffer:
                collector_id = self.buffer[18:26]
                app_log.info(collector_id)
                try:
                    msg_id += 1
                    state = ali.publish(
                        property_publish_topic.encode("utf-8"),
                        msg_product_info_CollectorID.format(
                            msg_id, collector_id
                        ).encode("utf-8"),
                    )
                    app_log.debug("state 4 = {}".format(state))
                except Exception as e:
                    app_log.error("Failed to publish collector_id info: {}".format(e))


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
    global msg_id, state
    while True:
        utime.sleep(86400)
        cell_location = cellLocator.getLocation(
            "www.queclocator.com", 80, "qa6qTK91597826z6", 8, 1
        )
        try:
            msg_id += 1
            state = ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_cellLocator.format(
                    msg_id, cell_location[0], cell_location[1], cell_location[2]
                ).encode("utf-8"),
            )
            app_log.debug("state 5 = {}".format(state))
        except Exception as e:
            app_log.error("Failed to publish cell_location info: {}".format(e))


def sim_task():
    global msg_id, state
    while True:
        sim_imsi = sim.getImsi()
        sim_iccid = sim.getIccid()
        try:
            msg_id += 1
            state = ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_sim.format(msg_id, sim_imsi, sim_iccid).encode("utf-8"),
            )
            app_log.debug("state 6 = {}".format(state))
        except Exception as e:
            app_log.error("Failed to publish SIM info: {}".format(e))
        utime.sleep(7200)


def send_heartbeat():
    while True:
        utime.sleep(120)  # 2分钟发送一次心跳包
        # 检查网络状态
        stagecode, subcode = checknet.wait_network_connected(30)
        if stagecode == 3 and subcode == 1:
            # 网络正常，发送心跳包
            uart2_inst.uartWrite(b"Heartbeat")
        else:
            # 网络不正常，不发送心跳包
            app_log.info("Network not connected, skipping heartbeat")
        # 等待一段时间后再次发送心跳包


def measure_adc_task():
    while True:
        sgm58031_dev.measure_adc_value()
        utime.sleep(1200)


def chect_net_task():
    global msg_id, ali
    while True:
        try:
            utime.sleep(60)
            stagecode, subcode = checknet.wait_network_connected(30)
            if stagecode == 3 and subcode == 1:
                app_log.info("Network connection good!")
                app_log.info("ali = {}".format(ali))
                # 先检查 ali 对象是否有效
                if ali is None:
                    # 重新创建 ali 对象
                    ali = aLiYun(
                        ProductKey, ProductSecret, DeviceName, DeviceSecret, MqttServer
                    )

                mqtt_state = ali.getAliyunSta()
                if mqtt_state == -1:
                    # 重置MQTT连接
                    try:
                        ali.setMqtt(clientID, clean_session=False, keepAlive=300)
                        ali.setCallback(mqtt_sub_cb)
                        ali.subscribe(property_subscribe_topic.encode("utf-8"))
                        ali.start()
                        utime.sleep(5)

                        # 发送网络状态
                        msg_id += 1
                        ali.publish(
                            property_publish_topic.encode("utf-8"),
                            msg_product_info_NetStatus.format(
                                msg_id, stagecode, subcode
                            ).encode("utf-8"),
                        )
                        app_log.info("aLiYun recovery successful")
                    except Exception as e:
                        app_log.error("aLiYun recovery failed: {}".format(e))
                        utime.sleep(30)  # 失败后等待较长时间再重试
                continue  # 网络正常，直接开始下一次检查

            # 网络异常，开始恢复流程
            app_log.error(
                "Network connection failed, stage={}, state={}".format(
                    stagecode, subcode
                )
            )

            # 重置网络
            net.setModemFun(0)  # 关闭网络
            utime.sleep(5)
            net.setModemFun(1)  # 打开网络
            utime.sleep(5)
        except Exception as e:
            app_log.error("Check network task error: {}".format(e))
            utime.sleep(30)  # 失败后等待较长时间再重试


def feed(t):
    wdt.feed()


def mqtt_sub_cb(topic, msg):
    global mqtt_sub_msg
    app_log.info("Subscribe Recv: Topic={},Msg={}".format(topic.decode(), msg.decode()))
    mqtt_sub_msg = ujson.loads(msg.decode())
    app_log.debug(mqtt_sub_msg["params"])


if __name__ == "__main__":
    utime.sleep(5)
    # pdpCtx = dataCall.getPDPContext(1)
    # app_log.info("pdpCtx:{}".format(pdpCtx))
    # if pdpCtx != -1:
    #     if pdpCtx[1] != usrConfig["apn"]:
    #         ret = dataCall.setPDPContext(
    #             1, 0, usrConfig["apn"], usrConfig["username"], usrConfig["password"], 0
    #         )
    #         if ret == 0:
    #             app_log.info("APN config success.")
    #             Power.powerRestart()
    #         else:
    #             app_log.info("APN config failed.")
    #     else:
    #         app_log.info("APN configured.")
    # else:
    #     app_log.info("get PDP Context failed.")

    checknet.poweron_print_once()
    stagecode, subcode = checknet.wait_network_connected(30)
    if stagecode == 3 and subcode == 1:
        app_log.info("Network connection successful!")
        wdt = WDT(20)
        timer.start(period=15000, mode=timer.PERIODIC, callback=feed)  # 使用定时器喂狗
        sgm58031_dev = SGM58031Class()
        if not sgm58031_dev.self_verifying():
            app_log.info("#----sgm58031 initial false~!----#")

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
        ali.start()

        try:
            msg_id += 1
            ali.publish(
                property_publish_topic.encode("utf-8"),
                msg_product_info_NetStatus.format(msg_id, stagecode, subcode).encode(
                    "utf-8"
                ),
            )
        except Exception as e:
            app_log.error("Failed to publish network status: {}".format(e))
        uart2_inst = Uart2()
        _thread.start_new_thread(watch_dog_task, ())
        _thread.start_new_thread(cell_location_task, ())
        _thread.start_new_thread(sim_task, ())
        _thread.start_new_thread(chect_net_task, ())
        _thread.start_new_thread(send_heartbeat, ())
        _thread.start_new_thread(measure_adc_task, ())

        while True:
            if state == True:
                pass
            else:
                try:
                    app_log.debug("state in while = {}".format(state))
                    net.setModemFun(0)  # 关闭网络
                    utime.sleep(5)
                    net.setModemFun(1)  # 打开网络
                    utime.sleep(15)
                    state = True
                except Exception as e:
                    app_log.error("Network reset failed: {}".format(e))
                    # 等待一段时间后重试
                    utime.sleep(30)
            utime.sleep(1)
    else:
        app_log.error(
            "Network connection failed! stagecode = {}, subcode = {}".format(
                stagecode, subcode
            )
        )
        Power.powerRestart()
