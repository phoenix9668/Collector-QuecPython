# import uos
# uos.chdir('/usr/')
import utime
import checkNet
import log
from machine import UART
from machine import I2C
from aLiYun import aLiYun
import sim
import cellLocator
import sys_bus
import _thread
import ujson

# from usr.i2c_aht10 import Aht10Class

# 设置日志输出级别
log.basicConfig(level=log.INFO)
logging = log.getLogger("UART")
uart2 = UART(UART.UART2, 115200, 8, 0, 1, 0)
uart1 = UART(UART.UART1, 115200, 8, 0, 1, 0)


def uart2_write(msg):
    global uart2
    uart2.write(msg)
    logging.info("Write msg :{}".format(msg))
    logging.info("uart_write end!")


def uart2_read():
    global uart2
    while True:
        msg_len = uart2.any()
        if (msg_len % 206 == 0) & (msg_len != 0):
            start = utime.ticks_ms()
            print(msg_len)
            msg = uart2.read(msg_len)
            hex_msg = [hex(x) for x in msg]
            logging.info("uart2_read msg: {}".format(hex_msg))

            for i in range(msg_len // 206):
                signs_data = {}
                temp = 206 * i
                signs_data['collector_id'] = hex_to_str(hex_msg[temp:temp + 4], " ")
                signs_data['rfid'] = hex_to_str(hex_msg[temp + 5:temp + 11], " ")
                signs_data['guid'] = hex_to_str(hex_msg[temp + 11:temp + 43], " ")
                signs_data['step_array'] = hex_to_str(hex_msg[temp + 43:temp + 115], " ")
                signs_data['ingestion_array'] = hex_to_str(hex_msg[temp + 115:temp + 187], " ")
                signs_data['stage'] = int(hex_msg[temp + 187])
                signs_data['battery_voltage'] = (int(hex_msg[temp + 188]) << 8) | int(hex_msg[temp + 189])
                signs_data['reset_cnt'] = (int(hex_msg[temp + 190]) << 8) | int(hex_msg[temp + 191])
                signs_data['signal_strength'] = calc_rssi_dbm(int(hex_msg[temp + 192]))
                signs_data['utc_time'] = int(round(utime.mktime(utime.localtime()) * 1000))
                print(signs_data)

                msg = {"topic": aliyunClass.publish_topic1,
                       "msg": msg_signs_data.format
                       (signs_data['collector_id'], signs_data['rfid'], signs_data['guid'], signs_data['step_array'],
                        signs_data['ingestion_array'], signs_data['stage'], signs_data['battery_voltage'],
                        signs_data['reset_cnt'], signs_data['signal_strength'], signs_data['utc_time'])}
                Handler.pub(msg)
            time_diff = utime.ticks_diff(utime.ticks_ms(), start)
            print(time_diff)
        else:
            utime.sleep_ms(10)
            continue


def uart1_read():
    global uart1
    while True:
        msg_len = uart1.any()
        if msg_len:
            print(msg_len)
            msg = uart1.read(msg_len)
            logging.info("uart1_read msg: {}".format(msg))
            gprmc_info = parse_gprmc(msg)
            print(gprmc_info)

            msg = {"topic": aliyunClass.publish_topic1,
                   "msg": msg_geoLocation.format
                   (gprmc_info[0], gprmc_info[1], gprmc_info[2], 1)}
            Handler.pub(msg)
        else:
            continue
        utime.sleep(60)


def hex_to_str(a, b=""):
    string = ''.join([hex_byte.replace('0x', b) for hex_byte in a])
    return string


class SysTopicClass(object):
    RRPC = "rrpc"
    OTA = "ota"
    PUB = "pub"
    SUB = "sub"


class ALiYunClass(object):
    def __init__(self):
        ALiYunClass.inst = self
        self.ProductKey = "a1GugRmVZzw"  # 产品标识
        self.ProductSecret = 'MqLQtdNniH3hKf1r'  # 产品密钥（一机一密认证此参数传入None）
        self.DeviceSecret = None  # 设备密钥（一型一密认证此参数传入None）
        self.DeviceName = "BX-XC-200-001"  # 设备名称

        self.subscribe_topic1 = "/sys" + "/" + self.ProductKey + "/" + \
                                self.DeviceName + "/" + "thing/service/property/set"
        self.publish_topic1 = "/sys" + "/" + self.ProductKey + "/" + \
                              self.DeviceName + "/" + "thing/event/property/post"

        # 创建aliyun连接对象
        self.ali = aLiYun(
            self.ProductKey,
            self.ProductSecret,
            self.DeviceName,
            self.DeviceSecret)
        # 设置mqtt连接属性
        client_id = self.ProductKey + "." + self.DeviceName  # 自定义字符（不超过64）
        self.ali.setMqtt(client_id, clean_session=False, keepAlive=60, reconn=True)  # False True

        # 设置回调函数
        self.ali.setCallback(self.ali_sub_cb)

    def ali_sub_cb(self, topic, msg):  # 回调函数
        if topic.decode().find(sys_topic.RRPC) != -1:
            sys_bus.publish(sys_topic.RRPC, {"topic": topic, "msg": msg})
        elif topic.decode().find(sys_topic.OTA) != -1:
            sys_bus.publish(sys_topic.OTA, {"topic": topic, "msg": msg})
        else:
            sys_bus.publish(sys_topic.SUB, {"topic": topic, "msg": msg})

    def ali_start(self):
        # 运行
        self.ali.start()
        print('Runing')
        # aLiYun.disconnect()

    def ali_subscribe(self):
        # 订阅主题
        self.ali.subscribe(self.subscribe_topic1, qos=0)
        # self.ali.subscribe(self.subscribe_topic2, qos=0)

    def ali_publish(self, topic, msg):
        ret = self.ali.getAliyunSta()
        # print(ret)
        if ret == 0:
            try:
                self.ali.publish(msg.get('topic'), msg.get("msg"), qos=0)
            except BaseException:
                print('send fail!!!')


class Handler(object):
    @classmethod
    def sub(cls, topic, msg):
        print(
            "Subscribe Recv: Topic={},Msg={}".format(
                msg.get('topic').decode(),
                msg.get("msg").decode()))

    @classmethod
    def pub(cls, msg):
        sys_bus.publish(sys_topic.PUB, msg)
        utime.sleep_ms(2000)

    @classmethod
    def ota(cls, topic, msg):
        """处理完ota的信息后，同步发送"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """同步publish，同步情况下会等待所有topic对应的处理函数处理完才会退出"""
        sys_bus.publish_sync(sys_topic.PUB, msg)

    @classmethod
    def rrpc(cls, topic, msg):
        """发布rrpc执行下列操作"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """异步publish， """
        sys_bus.publish(sys_topic.PUB, msg)


class AHT10Class:
    """ AHT10 class, incloud reset, read, write, measurement function"""

    def __init__(self, addr=0x38, alise="AHT10"):
        # Initialization command
        self.AHT10_CALIBRATION_CMD = 0xE1
        # Trigger measurement
        self.AHT10_START_MEASURMENT_CMD = 0xAC
        # Reset
        self.AHT10_RESET_CMD = 0xBA
        self.i2c_log = log.getLogger(alise)
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
        self.i2c_dev.write(self.i2c_addr,
                           bytearray(0x00), 0,
                           bytearray(data), len(data))

    def read_data(self, length):
        r_data = [0x00 for _ in range(length)]
        r_data = bytearray(r_data)
        self.i2c_dev.read(self.i2c_addr,
                          bytearray(0x00), 0,
                          r_data, length,
                          0)
        return list(r_data)

    def aht10_transformation_temperature(self, data):
        r_data = data
        # Convert the temperature as described in the data book
        self.humidity = (r_data[0] << 12) | (r_data[1] << 4) | ((r_data[2] & 0xF0) >> 4)
        self.humidity = float('%.2f' % ((self.humidity / (1 << 20)) * 100.0))
        # print("current humidity is {0}%".format(self.humidity))
        self.temperature = ((r_data[2] & 0xf) << 16) | (r_data[3] << 8) | r_data[4]
        self.temperature = float('%.2f' % ((self.temperature * 200.0 / (1 << 20)) - 50))
        # print("current temperature is {0}°C".format(self.temperature))
        msg = {"topic": aliyunClass.publish_topic1,
               "msg": msg_temperature_humidity.format(self.temperature, self.humidity)}
        Handler.pub(msg)
        utime.sleep(5)

    def trigger_measurement(self):
        while True:
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
    """ SGM58031 class, incloud read and write function"""

    def __init__(self, addr=0x48, alise="SGM58031"):
        # Initialization command
        self.i2c_log = log.getLogger(alise)
        self.i2c_dev = I2C(I2C.I2C1, I2C.STANDARD_MODE)  # Return I2C object
        self.i2c_addr = addr

        self.register_map = {'Conversion_Register': 0x0000, 'Config_Register': 0x0000, 'Low_Thresh_Register': 0x8000,
                             'High_Thresh_Register': 0x7FFF, 'Config1_Register': 0x0000, 'Chip_ID_Register': 0x0000,
                             'GN_Trim1_Register': 0x0000}

        self.register_addr = {'CONVERSION_REG': 0x00, 'CONF_REG': 0x01, 'LOW_THRESH_REG': 0x02,
                              'HIGH_THRESH_REG': 0x03, 'CONF1_REG': 0x04, 'CHIP_ID_REG': 0x05, 'GN_TRIM1_REG': 0x06}

        self.os_sel = {'No_Effect': 0x00,  # 0 = No effect
                       'Start_Single_Conversion': 0x01}  # 1 = Start a single conversion (when in single-shot mode)

        self.mux_sel = {'AINP_AIN0_AND_AINN_AIN1': 0x00,  # 000 = AINP = AIN0 and AINN = AIN1 (default)
                        'AINP_AIN0_AND_AINN_AIN3': 0x01,  # 001 = AINP = AIN0 and AINN = AIN3
                        'AINP_AIN1_AND_AINN_AIN3': 0x02,  # 010 = AINP = AIN1 and AINN = AIN3
                        'AINP_AIN2_AND_AINN_AIN3': 0x03,  # 011 = AINP = AIN2 and AINN = AIN3
                        'AINP_AIN0_AND_AINN_GND': 0x04,  # 100 = AINP = AIN0 and AINN = GND
                        'AINP_AIN1_AND_AINN_GND': 0x05,  # 101 = AINP = AIN1 and AINN = GND
                        'AINP_AIN2_AND_AINN_GND': 0x06,  # 110 = AINP = AIN2 and AINN = GND
                        'AINP_AIN3_AND_AINN_GND': 0x07}  # 111 = AINP = AIN3 and AINN = GND

        self.pga_sel = {'FS_6_144V': 0x00,  # 000 = FS = +/-6.144V
                        'FS_4_096V': 0x01,  # 001 = FS = +/-4.096V
                        'FS_2_048V': 0x02,  # 010 = FS = +/-2.048V (default)
                        'FS_1_024V': 0x03,  # 011 = FS = +/-1.024V
                        'FS_0_512V': 0x04,  # 100 = FS = +/-0.512V
                        'FS_0_256V': 0x05}  # 101 = FS = +/-0.256V

        self.mode_sel = {'Continuous_Conversion_Mode': 0x00,  # 0 = Continuous conversion mode
                         'Single_Shot_Mode': 0x01}  # 1 = Power-down single-shot mode (default)

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
        self.dr_sel = {'DR_6_25Hz_7_5Hz': 0x00,
                       'DR_12_5Hz_15Hz': 0x01,
                       'DR_25Hz_30Hz': 0x02,
                       'DR_50Hz_60Hz': 0x03,
                       'DR_100Hz_120Hz': 0x04,
                       'DR_200Hz_240Hz': 0x05,
                       'DR_400Hz_480Hz': 0x06,
                       'DR_800Hz_960Hz': 0x07}

        self.comp_mode_sel = {'Traditional_Comparator': 0x00,  # 0 = A traditional comparator with hysteresis (default)
                              'Window_Comparator': 0x01}  # 1 = A window comparator

        self.comp_pol_sel = {'Active_Low': 0x00, 'Active_High': 0x01}
        self.comp_lat_sel = {'Non_Latching': 0x00, 'Latching': 0x01}

        self.comp_que_sel = {'Assert_After_One_Conversion': 0x00,  # 00 = Assert after one conversion
                             'Assert_After_Two_Conversions': 0x01,  # 01 = Assert after two conversions
                             'Assert_After_Four_Conversions': 0x02,  # 10 = Assert after four conversions
                             'Disable_Comparator': 0x03}  # Disable comparator (default)

        self.dr_sel_sel = {'DR_SEL0': 0x00,  # 0 = DR[2:0] = 000 ~ 111 for conversion rate of 6.25Hz, 12.5Hz, 25Hz,
                           # 50Hz, 100Hz, 200Hz, 400Hz and 800Hz (default)
                           'DR_SEL1': 0x01}  # 1 = DR[2:0] = 000 ~ 111 for conversion rate of 7.5Hz, 15Hz, 30Hz,
        # 60Hz, 120Hz, 240Hz, 480Hz and 960Hz
        self.battery_voltage = None
        self.voltage = None
        self.config_reg_init()
        pass

    def config_reg_init(self, low_thresh_register=0x8000, high_thresh_register=0x7FFF):
        # Initialise the SGM58031 registers
        self.register_map['Config_Register'] = (self.os_sel['Start_Single_Conversion'] << 15 & 0x8000) | \
                                               (self.mux_sel['AINP_AIN2_AND_AINN_AIN3'] << 12 & 0x7000) | \
                                               (self.pga_sel['FS_4_096V'] << 9 & 0x0E00) | \
                                               (self.mode_sel['Single_Shot_Mode'] << 8 & 0x0100) | \
                                               (self.dr_sel['DR_800Hz_960Hz'] << 5 & 0x00E0) | \
                                               (self.comp_mode_sel['Traditional_Comparator'] << 4 & 0x0010) | \
                                               (self.comp_pol_sel['Active_Low'] << 3 & 0x0008) | \
                                               (self.comp_lat_sel['Non_Latching'] << 2 & 0x0004) | \
                                               (self.comp_que_sel['Disable_Comparator'])
        self.register_map['Low_Thresh_Register'] = low_thresh_register
        self.register_map['High_Thresh_Register'] = high_thresh_register
        self.register_map['Config1_Register'] = (self.dr_sel_sel['DR_SEL0'] << 7 & 0x0080)
        pass

    def self_verifying(self):
        # -1- Read_CHIP_ID
        tmp = self.read_register([self.register_addr['CHIP_ID_REG']], 2)
        print("####################################################################")
        print("----------------------------CHIP_ID = %x----------------------------"
              % ((tmp[0] << 8) | tmp[1]))
        if ((tmp[0] << 8) | tmp[1]) != 0x80:
            return False
        # -2- Set the Configuration Register
        self.write_register([self.register_addr['CONF_REG']],
                            [self.register_map['Config_Register'] >> 8,
                             (self.register_map['Config_Register'] & 0xff)])
        # -3- Read the Configuration Register
        tmp = self.read_register([self.register_addr['CONF_REG']], 2)
        print("--------------------------CONF_REG = %x---------------------------"
              % ((tmp[0] << 8) | tmp[1]))
        if (((tmp[0] << 8) | tmp[1]) & 0x7fff) != (self.register_map['Config_Register'] & 0x7fff):
            return False

        # -4- Set the Configuration1 Register
        self.write_register([self.register_addr['CONF1_REG']],
                            [self.register_map['Config1_Register'] >> 8,
                             (self.register_map['Config1_Register'] & 0xff)])
        # -5- Read the Configuration1 Register
        tmp = self.read_register([self.register_addr['CONF1_REG']], 2)
        print("----------------------------CONF1_REG = %x---------------------------"
              % ((tmp[0] << 8) | tmp[1]))
        if ((tmp[0] << 8) | tmp[1]) != self.register_map['Config1_Register']:
            return False

        # -6- Set the Low Thresh Register
        self.write_register([self.register_addr['LOW_THRESH_REG']],
                            [self.register_map['Low_Thresh_Register'] >> 8,
                             (self.register_map['Low_Thresh_Register'] & 0xff)])
        # -7- Read the Low Thresh Register
        tmp = self.read_register([self.register_addr['LOW_THRESH_REG']], 2)
        print("---------------------Low_Thresh_Register = %x---------------------"
              % ((tmp[0] << 8) | tmp[1]))
        if ((tmp[0] << 8) | tmp[1]) != self.register_map['Low_Thresh_Register']:
            return False

        # -8- Set the High Thresh Register
        self.write_register([self.register_addr['HIGH_THRESH_REG']],
                            [self.register_map['High_Thresh_Register'] >> 8,
                             (self.register_map['High_Thresh_Register'] & 0xff)])
        # -9- Read the High Thresh Register
        tmp = self.read_register([self.register_addr['HIGH_THRESH_REG']], 2)
        print("--------------------High_Thresh_Register = %x---------------------"
              % ((tmp[0] << 8) | tmp[1]))
        if ((tmp[0] << 8) | tmp[1]) != self.register_map['High_Thresh_Register']:
            return False
        print("--------------------sgm58031 initial completed~!--------------------")
        print("####################################################################")
        return True

    def write_register(self, register_addr, bytes_list):
        self.i2c_dev.write(self.i2c_addr,
                           bytearray(register_addr), len(register_addr),
                           bytearray(bytes_list), len(bytes_list))

    def read_register(self, register_addr, length):
        bytes_list = [0x00 for _ in range(length)]
        bytes_list = bytearray(bytes_list)
        self.i2c_dev.write(self.i2c_addr,
                           bytearray(register_addr), len(register_addr),
                           bytearray(0x00), 0)
        self.i2c_dev.read(self.i2c_addr,
                          bytearray(0x00), 0,
                          bytes_list, length,
                          0)
        return list(bytes_list)

    def measure_adc_value(self):
        flip_sign = True
        while True:
            utime.sleep(4)
            # -1- read the OS of Configuration Register
            tmp = self.read_register([self.register_addr['CONF_REG']], 2)
            if tmp[0] >> 7 == 1:
                if flip_sign:
                    tmp = self.read_register([self.register_addr['CONVERSION_REG']], 2)
                    self.battery_voltage = float('%.3f' % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 11))
                    print("battery_voltage = {}".format(self.battery_voltage))
                else:
                    tmp = self.read_register([self.register_addr['CONVERSION_REG']], 2)
                    self.voltage = float('%.3f' % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 21))
                    print("voltage = {}".format(self.voltage))

                msg = {"topic": aliyunClass.publish_topic1,
                       "msg": msg_voltage.format(self.battery_voltage, self.voltage)}
                Handler.pub(msg)

            # -2- Initialise the SGM58031 peripheral
            if flip_sign:
                self.register_map['Config_Register'] = (self.register_map['Config_Register'] & 0x8fff) | \
                                                       (self.mux_sel['AINP_AIN0_AND_AINN_AIN1'] << 12 & 0x7000)
            else:
                self.register_map['Config_Register'] = (self.register_map['Config_Register'] & 0x8fff) | \
                                                       (self.mux_sel['AINP_AIN2_AND_AINN_AIN3'] << 12 & 0x7000)
            flip_sign = not flip_sign

            # -3- Set the Configuration Register
            self.write_register([self.register_addr['CONF_REG']],
                                [self.register_map['Config_Register'] >> 8,
                                 (self.register_map['Config_Register'] & 0xff)])


def get_cell_location_and_sim_info():
    while True:
        cell_location = cellLocator.getLocation("www.queclocator.com", 80, "1111111122222222", 8, 1)
        sim_imsi = sim.getImsi()
        sim_iccid = sim.getIccid()
        msg = {"topic": aliyunClass.publish_topic1,
               "msg": msg_cellLocator_sim.format
               (cell_location[0], cell_location[1], cell_location[2], sim_imsi, sim_iccid)}
        Handler.pub(msg)
        utime.sleep(60)


def parse_loc_val(val, d):
    v = float(val) / 100
    v = int(v) + (v - int(v)) * 100 / 60
    if d == 'S' or d == 'W':
        v = v * -1
    return v


def parse_gprmc(data):
    """
    b'$GPRMC,111025.00,A,2517.033747,N,11019.176025,E,0.0,144.8,270920,2.3,W,A*2D\r\n'
    b'$GPRMC,,V,,,,,,,,,,N*53\r\n'
    b'$GPRMC,024443.0,A,2517.038296,N,11019.174048,E,0.0,,120201,0.0,E,A*2F\r\n'
    $GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh<CR><LF>
    <1> UTC时间，hhmmss（时分秒）格式
    <2> 定位状态，A=有效定位，V=无效定位
    <3> 纬度ddmm.mmmm（度分）格式（前面的0也将被传输）
    <4> 纬度半球N（北半球）或S（南半球）
    <5> 经度dddmm.mmmm（度分）格式（前面的0也将被传输）
    <6> 经度半球E（东经）或W（西经）
    <7> 地面速率（000.0~999.9节，前面的0也将被传输） 1节=1.852千米（km/h)
    <8> 地面航向（000.0~359.9度，以真北为参考基准，前面的0也将被传输）
    <9> UTC日期，ddmmyy（日月年）格式
    <10> 磁偏角（000.0~180.0度，前面的0也将被传输）
    <11> 磁偏角方向，E（东）或W（西）
    <12> 模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）
    """
    li = data.decode().replace('$GPRMC,', '').strip().split(',')
    lat = log = speed = direct = 0
    if li[1] == 'A':
        lat = round(parse_loc_val(li[2], li[3]), 6)  # 纬度
        log = round(parse_loc_val(li[4], li[5]), 6)  # 经度
        speed = float(li[6]) * 1.852
        if len(li[7]) > 0:
            direct = float(li[7])
        else:
            direct = 0
        # logging.info('lat:{:.6f},log:{:.6f},speed:{},direct:{}'.format(lat, log, speed, direct))
    return log, lat, speed, direct


def calc_rssi_dbm(rssi_dec):
    """Calc the RSSI value to RSSI dBm"""
    rssi_offset = 74
    if rssi_dec >= 128:
        rssi_dBm = (rssi_dec - 256) / 2 - rssi_offset
    else:
        rssi_dBm = (rssi_dec / 2) - rssi_offset
    return float('%.2f' % rssi_dBm)


if __name__ == '__main__':
    PROJECT_NAME = "QuecPython"
    PROJECT_VERSION = "1.0.0"
    check_net = checkNet.CheckNetwork(PROJECT_NAME, PROJECT_VERSION)
    check_net.poweron_print_once()
    check_net.wait_network_connected()

    aliyunClass = ALiYunClass()
    aliyunClass.ali_subscribe()

    sys_topic = SysTopicClass()
    sys_bus.subscribe(sys_topic.RRPC, Handler.rrpc)
    sys_bus.subscribe(sys_topic.OTA, Handler.ota)
    sys_bus.subscribe(sys_topic.SUB, Handler.sub)
    sys_bus.subscribe(sys_topic.PUB, aliyunClass.ali_publish)

    ath10_dev = AHT10Class()
    sgm58031_dev = SGM58031Class()
    if not sgm58031_dev.self_verifying():
        print("#----sgm58031 initial false~!----#")

    msg_temperature_humidity = """{{
                            "id": "123",
                            "version": "1.0",
                            "params": {{
                                "Temperature": {{
                                    "value": {0}
                                }},
                                "Humidity": {{
                                    "value": {1}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    msg_voltage = """{{
                            "id": "2345",
                            "version": "1.0",
                            "params": {{
                                "BatteryVoltage": {{
                                    "value": {0}
                                }},
                                "voltage": {{
                                    "value": {1}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    msg_cellLocator_sim = """{{
                            "id": "456",
                            "version": "1.0",
                            "params": {{
                                "CellLocator": {{
                                    "Longitude": {{
                                        "value": {0}
                                    }},
                                    "Latitude": {{
                                        "value": {1}
                                    }},
                                    "Accuracy": {{
                                    "value": {2}
                                    }}
                                }},
                                "product_information:IMSI": {{
                                    "value": "{3}"
                                }},
                                "product_information:ICCID": {{
                                    "value": "{4}"
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    msg_geoLocation = """{{
                            "id": "5676",
                            "version": "1.0",
                            "params": {{
                                "GeoLocation": {{
                                    "Longitude": {{
                                        "value": {0}
                                    }},
                                    "Latitude": {{
                                        "value": {1}
                                    }},
                                    "Altitude": {{
                                        "value": {2}
                                    }},
                                    "CoordinateSystem": {{
                                        "value": {3}
                                    }}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    msg_signs_data = """{{
                            "id": "789",
                            "version": "1.0",
                            "params": {{
                                "collar_information:SignsData": {{
                                    "collar_information:CollectorID": {{
                                        "value": "{0}"
                                    }},
                                    "collar_information:RFID": {{
                                        "value": "{1}"
                                    }},
                                    "collar_information:GUID": {{
                                        "value": "{2}"
                                    }},
                                    "collar_information:StepArray": {{
                                        "value": "{3}"
                                    }},
                                    "collar_information:IngestionArray": {{
                                        "value": "{4}"
                                    }},
                                    "collar_information:Stage": {{
                                        "value": {5}
                                    }},
                                    "collar_information:BatteryVoltage": {{
                                        "value": {6}
                                    }},
                                    "collar_information:ResetCnt": {{
                                        "value": {7}
                                    }},
                                    "collar_information:SignalStrength": {{
                                        "value": {8}
                                    }},
                                    "collar_information:UTCtime": {{
                                        "value": "{9}"
                                    }}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    _thread.start_new_thread(get_cell_location_and_sim_info, ())
    _thread.start_new_thread(ath10_dev.trigger_measurement, ())
    _thread.start_new_thread(sgm58031_dev.measure_adc_value, ())
    _thread.start_new_thread(uart2_read, ())
    _thread.start_new_thread(uart1_read, ())

    aliyunClass.ali_start()

    while True:
        utime.sleep(4)
        message = "temperature = {}, humidity = {}".format(ath10_dev.temperature, ath10_dev.humidity)
        uart2_write(message)
