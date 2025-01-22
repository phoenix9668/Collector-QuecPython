from machine import UART
from machine import I2C
from machine import Pin
from umqtt import MQTTClient
from misc import Power
import dataCall
import cellLocator
import _thread
import utime
import log
import net
import checkNet
import sim
import ustruct
import ujson

PWM_Led = Pin(Pin.GPIO24, Pin.OUT, Pin.PULL_DISABLE, 1)

PROJECT_NAME = "QuecPython_EC600M"
PROJECT_VERSION = "3.2.0"

checknet = checkNet.CheckNetwork(PROJECT_NAME, PROJECT_VERSION)
TaskEnable = True  # 调用disconnect后会通过该状态回收线程资源
msg_id = 0
state = 0
mqtt_sub_msg = {}
signs_data = {}

log.basicConfig(level=log.DEBUG)
app_log = log.getLogger("app_log")


def watch_dog_task():
    while True:
        if PWM_Led.read():
            PWM_Led.write(0)
        else:
            PWM_Led.write(1)
        utime.sleep(1)


class MqttClient:
    # 说明：reconn该参数用于控制使用或关闭umqtt内部的重连机制，默认为True，使用内部重连机制。
    # 如需测试或使用外部重连机制可参考此示例代码，测试前需将reconn=False,否则默认会使用内部重连机制！
    def __init__(
        self,
        clientid,
        server,
        port,
        user=None,
        password=None,
        keepalive=0,
        ssl=False,
        ssl_params={},
        reconn=True,
    ):
        self.__clientid = clientid
        self.__pw = password
        self.__server = server
        self.__port = port
        self.__uasename = user
        self.__keepalive = keepalive
        self.__ssl = ssl
        self.__ssl_params = ssl_params
        self.topic = None
        self.qos = None
        # 网络状态标志
        self.__nw_flag = True
        # 创建互斥锁
        self.mp_lock = _thread.allocate_lock()
        # 创建类的时候初始化出mqtt对象
        self.client = MQTTClient(
            self.__clientid,
            self.__server,
            self.__port,
            self.__uasename,
            self.__pw,
            keepalive=self.__keepalive,
            ssl=self.__ssl,
            ssl_params=self.__ssl_params,
            reconn=reconn,
        )

    def connect(self):
        self.client.connect(clean_session=False)
        # 注册网络回调函数，网络状态发生变化时触发
        flag = dataCall.setCallback(self.nw_cb)
        if flag != 0:
            # 回调注册失败
            raise Exception("Network callback registration failed")

    def set_callback(self, sub_cb):
        self.client.set_callback(sub_cb)

    def error_register_cb(self, func):
        self.client.error_register_cb(func)

    def subscribe(self, topic, qos=0):
        self.topic = topic  # 保存topic ，多个topic可使用list保存
        self.qos = qos  # 保存qos
        self.client.subscribe(topic, qos)

    def publish(self, topic, msg, qos=0):
        self.client.publish(topic, msg, qos)

    def disconnect(self):
        global TaskEnable
        # 关闭wait_msg的监听线程
        TaskEnable = False
        # 关闭之前的连接，释放资源
        self.client.disconnect()

    def reconnect(self):
        # 判断锁是否已经被获取
        if self.mp_lock.locked():
            return
        self.mp_lock.acquire()
        # 重新连接前关闭之前的连接，释放资源(注意区别disconnect方法，close只释放socket资源，disconnect包含mqtt线程等资源)
        self.client.close()
        # 重新建立mqtt连接
        while True:
            net_sta = net.getState()  # 获取网络注册信息
            if net_sta != -1 and net_sta[1][0] == 1:
                call_state = dataCall.getInfo(1, 0)  # 获取拨号信息
                if (call_state != -1) and (call_state[2][0] == 1):
                    try:
                        # 网络正常，重新连接mqtt
                        self.connect()
                    except Exception as e:
                        # 重连mqtt失败, 5s继续尝试下一次
                        self.client.close()
                        utime.sleep(5)
                        continue
                else:
                    # 网络未恢复，等待恢复
                    utime.sleep(10)
                    continue
                # 重新连接mqtt成功，订阅Topic
                try:
                    # 多个topic采用list保存，遍历list重新订阅
                    if self.topic is not None:
                        self.client.subscribe(self.topic, self.qos)
                    self.mp_lock.release()
                except:
                    # 订阅失败，重新执行重连逻辑
                    self.client.close()
                    utime.sleep(5)
                    continue
            else:
                utime.sleep(5)
                continue
            break  # 结束循环
        # 退出重连
        return True

    def nw_cb(self, args):
        nw_sta = args[1]
        if nw_sta == 1:
            # 网络连接
            app_log.info("*** network connected! ***")
            self.__nw_flag = True
        else:
            # 网络断线
            app_log.info("*** network not connected! ***")
            self.__nw_flag = False

    def __listen(self):
        while True:
            try:
                if not TaskEnable:
                    break
                self.client.wait_msg()
            except OSError as e:
                # 判断网络是否断线
                if not self.__nw_flag:
                    # 网络断线等待恢复进行重连
                    self.reconnect()
                # 在socket状态异常情况下进行重连
                elif self.client.get_mqttsta() != 0 and TaskEnable:
                    self.reconnect()
                else:
                    # 这里可选择使用raise主动抛出异常或者返回-1
                    return -1

    def loop_forever(self):
        _thread.start_new_thread(self.__listen, ())


class Uart1(object):
    def __init__(
        self,
        no=UART.UART1,
        bate=115200,
        data_bits=8,
        parity=0,
        stop_bits=1,
        flow_control=0,
    ):
        self.uart = UART(no, bate, data_bits, parity, stop_bits, flow_control)
        self.uart.set_callback(self.callback)

    def callback(self, para):
        app_log.debug("call para:{}".format(para))
        if 0 == para[0]:
            self.uartRead(para[2])

    def uartWrite(self, msg):
        hex_msg = [hex(x) for x in msg]
        app_log.debug("Write msg:{}".format(hex_msg))
        self.uart.write(msg)

    def uartRead(self, len):
        msg = self.uart.read(len)
        hex_msg = [hex(x) for x in msg]
        app_log.debug("uart1_read msg: {}".format(hex_msg))


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
            elif self.buffer[i] == 0x0B:
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
        if frame[0] == 0x0B and frame[4] == 0xB0:
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
            app_log.info(signs_data)

            msg_id += 1
            mqtt_client.publish(
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
        elif frame[0] == 0x0B and frame[4] == 0xB2:
            sgm58031_dev.battery_voltage = float(
                "%.3f" % (((frame[5] << 8) | frame[6]) / 32768 * 4.096 * 11)
            )
            app_log.info("battery_voltage = {}".format(sgm58031_dev.battery_voltage))

            sgm58031_dev.voltage = float(
                "%.3f" % (((frame[7] << 8) | frame[8]) / 32768 * 4.096 * 21)
            )
            app_log.info("voltage = {}".format(sgm58031_dev.voltage))

            msg_id += 1
            mqtt_client.publish(
                property_publish_topic.encode("utf-8"),
                msg_voltage.format(
                    msg_id, sgm58031_dev.battery_voltage, sgm58031_dev.voltage
                ).encode("utf-8"),
            )
        elif frame[0] == 0x23 and frame[1] == 0x23:
            msg_id += 1
            mqtt_client.publish(
                property_publish_topic.encode("utf-8"),
                msg_product_info_StatusInfo.format(msg_id, self.buffer).encode("utf-8"),
            )
            if "##Read Memory Complete##" in self.buffer:
                collector_id = self.buffer[18:26]
                app_log.info(collector_id)
                msg_id += 1
                mqtt_client.publish(
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


def parse_loc_val(val, d):
    v = float(val) / 100
    v = int(v) + (v - int(v)) * 100 / 60
    if d == "S" or d == "W":
        v = v * -1
    return v


def parse_gprmc(data):
    """
    b'$GPRMC,111025.00,A,2517.033747,N,11019.176025,E,0.0,144.8,270920,2.3,W,A*2D\r\n'
    b'$GPRMC,,V,,,,,,,,,,N*53\r\n'
    b'$GPRMC,024443.0,A,2517.038296,N,11019.174048,E,0.0,,120201,0.0,E,A*2F\r\n'
    $GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh<CR><LF>
    <1> UTC时间,hhmmss(时分秒)格式
    <2> 定位状态,A=有效定位,V=无效定位
    <3> 纬度ddmm.mmmm(度分)格式(前面的0也将被传输)
    <4> 纬度半球N(北半球)或S(南半球)
    <5> 经度dddmm.mmmm(度分)格式(前面的0也将被传输)
    <6> 经度半球E(东经)或W(西经)
    <7> 地面速率(000.0~999.9节,前面的0也将被传输) 1节=1.852千米(km/h)
    <8> 地面航向(000.0~359.9度,以真北为参考基准,前面的0也将被传输)
    <9> UTC日期,ddmmyy(日月年)格式
    <10> 磁偏角(000.0~180.0度,前面的0也将被传输)
    <11> 磁偏角方向,E(东)或W(西)
    <12> 模式指示(仅NMEA0183 3.00版本输出,A=自主定位,D=差分,E=估算,N=数据无效)
    """
    li = data.decode().replace("$GPRMC,", "").strip().split(",")
    lat = log = speed = direct = 0
    if li[1] == "A":
        lat = round(parse_loc_val(li[2], li[3]), 6)  # 纬度
        log = round(parse_loc_val(li[4], li[5]), 6)  # 经度
        speed = float(li[6]) * 1.852
        if len(li[7]) > 0:
            direct = float(li[7])
        else:
            direct = 0
        # app_log.info('lat:{:.6f},log:{:.6f},speed:{},direct:{}'.format(lat, log, speed, direct))
    return log, lat, speed, direct


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


class AHT10Class:
    """AHT10 class, incloud reset, read, write, measurement function"""

    def __init__(self, addr=0x38, alise="AHT10"):
        # Initialization command
        self.AHT10_CALIBRATION_CMD = 0xE1
        # Trigger measurement
        self.AHT10_START_MEASURMENT_CMD = 0xAC
        # Reset
        self.AHT10_RESET_CMD = 0xBA
        self.i2c_dev = I2C(I2C.I2C1, I2C.STANDARD_MODE)  # Return I2C object
        self.i2c_addr = addr
        self.humidity = None
        self.temperature = None
        self.sensor_init()
        pass

    def sensor_init(self):
        # calibration
        self.write_data([self.AHT10_CALIBRATION_CMD, 0x08, 0x00])
        utime.sleep_ms(300)  # at last 300ms
        pass

    def ath10_reset(self):
        # reset
        self.write_data([self.AHT10_RESET_CMD])
        utime.sleep_ms(20)  # at last 20ms

    def write_data(self, data):
        self.i2c_dev.write(
            self.i2c_addr, bytearray(0x00), 0, bytearray(data), len(data)
        )

    def read_data(self, length):
        r_data = [0x00 for _ in range(length)]
        r_data = bytearray(r_data)
        self.i2c_dev.read(self.i2c_addr, bytearray(0x00), 0, r_data, length, 0)
        return list(r_data)

    def aht10_transformation_temperature(self, data):
        global msg_id
        r_data = data
        # Convert the temperature as described in the data book
        self.humidity = (r_data[0] << 12) | (r_data[1] << 4) | ((r_data[2] & 0xF0) >> 4)
        self.humidity = float("%.2f" % ((self.humidity / (1 << 20)) * 100.0))
        # print("current humidity is {0}%".format(self.humidity))
        self.temperature = ((r_data[2] & 0xF) << 16) | (r_data[3] << 8) | r_data[4]
        self.temperature = float("%.2f" % ((self.temperature * 200.0 / (1 << 20)) - 50))
        # print("current temperature is {0}°C".format(self.temperature))
        app_log.info(
            "current temperature = {}°C, current humidity = {}%".format(
                self.temperature, self.humidity
            )
        )
        msg_id += 1
        mqtt_client.publish(
            property_publish_topic.encode("utf-8"),
            msg_temperature_humidity.format(
                msg_id, self.temperature, self.humidity
            ).encode("utf-8"),
        )

    def trigger_measurement(self):
        # Trigger data conversion
        self.write_data([self.AHT10_START_MEASURMENT_CMD, 0x33, 0x00])
        utime.sleep_ms(200)  # at last delay 75ms
        # check has success
        r_data = self.read_data(6)
        # check bit7
        if (r_data[0] >> 7) != 0x0:
            print("Conversion has error")
        else:
            self.aht10_transformation_temperature(r_data[1:6])


class SGM58031Class:
    """SGM58031 class, incloud read and write function"""

    def __init__(self, addr=0x48, alise="SGM58031"):
        # Initialization command
        self.i2c_dev = I2C(I2C.I2C1, I2C.STANDARD_MODE)  # Return I2C object
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
        global msg_id
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
            mqtt_client.publish(
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


def cell_location_task():
    global msg_id
    while True:
        utime.sleep(86400)
        cell_location = cellLocator.getLocation(
            "www.queclocator.com", 80, "qa6qTK91597826z6", 8, 1
        )
        msg_id += 1
        mqtt_client.publish(
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
        mqtt_client.publish(
            property_publish_topic.encode("utf-8"),
            msg_sim.format(msg_id, sim_imsi, sim_iccid).encode("utf-8"),
        )
        utime.sleep(7200)


def power_restart():
    while True:
        utime.sleep(14400)
        Power.powerRestart()


def mqtt_sub_cb(topic, msg):
    global state, mqtt_sub_msg
    app_log.info("Subscribe Recv: Topic={},Msg={}".format(topic.decode(), msg.decode()))
    mqtt_sub_msg = ujson.loads(msg.decode())
    state = 1
    app_log.debug(mqtt_sub_msg["params"])


if __name__ == "__main__":
    utime.sleep(5)
    checknet.poweron_print_once()
    stagecode, subcode = checknet.wait_network_connected(30)
    if stagecode == 3 and subcode == 1:
        app_log.info("Network connection successful!")

        uart1_inst = Uart1()
        uart2_inst = Uart2()
        ath10_dev = AHT10Class()
        sgm58031_dev = SGM58031Class()
        if not sgm58031_dev.self_verifying():
            app_log.info("#----sgm58031 initial false~!----#")

        _thread.start_new_thread(watch_dog_task, ())

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

        ProductKey = "he2maYabo9j"  # 产品标识
        DeviceName = "BW-XC-200-036"  # 设备名称
        # DeviceName = "QH-D200-485-006"  # 设备名称
        # DeviceName = "QH-D200-485-007"  # 设备名称
        # DeviceName = "QH-D200-485-008"  # 设备名称

        property_subscribe_topic = (
            "/sys"
            + "/"
            + ProductKey
            + "/"
            + DeviceName
            + "/"
            + "thing/service/property/set"
        )
        property_publish_topic = (
            "/sys"
            + "/"
            + ProductKey
            + "/"
            + DeviceName
            + "/"
            + "thing/event/property/post"
        )

        # 创建一个mqtt实例
        mqtt_client = MqttClient(
            clientid="he2maYabo9j.BW-XC-200-036|securemode=2,signmethod=hmacsha256,timestamp=1737515423690|",
            server="iot-06z00dcnrlb8g5r.mqtt.iothub.aliyuncs.com",
            port=1883,
            user="BW-XC-200-036&he2maYabo9j",
            password="b9fbb068f1c63eb6e5b23cfa919400114ceecbb5a490e54bab160c9050e8a378",
            keepalive=60,
            reconn=True,
        )

        def mqtt_err_cb(err):
            app_log.error("thread err:%s" % err)
            mqtt_client.reconnect()  # 可根据异常进行重连

        # 设置消息回调
        mqtt_client.set_callback(mqtt_sub_cb)
        mqtt_client.error_register_cb(mqtt_err_cb)
        # 建立连接
        try:
            mqtt_client.connect()
        except Exception as e:
            app_log.error("e=%s" % e)

        # 订阅主题
        app_log.info(
            "Connected to aliyun, subscribed to: {}".format(property_subscribe_topic)
        )
        mqtt_client.subscribe(property_subscribe_topic.encode("utf-8"), qos=0)

        msg_id += 1
        mqtt_client.publish(
            property_publish_topic.encode("utf-8"),
            msg_product_info_NetStatus.format(msg_id, stagecode, subcode).encode(
                "utf-8"
            ),
        )

        mqtt_client.loop_forever()

        _thread.start_new_thread(cell_location_task, ())
        _thread.start_new_thread(sim_task, ())

        while True:
            ath10_dev.trigger_measurement()
            sgm58031_dev.measure_adc_value()
            utime.sleep(1200)
    else:
        app_log.error(
            "Network connection failed! stagecode = {}, subcode = {}".format(
                stagecode, subcode
            )
        )
