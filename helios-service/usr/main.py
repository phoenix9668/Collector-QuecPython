# import uos
# uos.chdir('/usr/')
from machine import UART
from machine import I2C
from aLiYun import aLiYun
import cellLocator
import sys_bus
import _thread
import utime
from usr.bin.guard import GuardContext
import ujson

# from usr.i2c_aht10 import Aht10Class
uart2 = UART(UART.UART2, 115200, 8, 0, 1, 0)
uart1 = UART(UART.UART1, 115200, 8, 0, 1, 0)
signs_data = {}
msg_id = 0


def uart2_write(msg):
    global uart2
    uart2.write(msg)
    app_log.info("uart2_write msg :{}".format(msg))


def uart2_read():
    global uart2, msg_id
    while True:
        msg_len = uart2.any()
        if (msg_len % 206 == 0) & (msg_len != 0):
            start = utime.ticks_ms()
            app_log.info(msg_len)
            msg = uart2.read(msg_len)
            hex_msg = [hex(x) for x in msg]
            app_log.info("uart2_read msg: {}".format(hex_msg))

            for i in range(msg_len // 206):
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
                app_log.info(signs_data)

                msg_id += 1
                msg = {"topic": aliyunClass.property_publish_topic,
                       "msg": msg_signs_data.format
                       (msg_id, signs_data['collector_id'], signs_data['rfid'],
                        signs_data['guid'], signs_data['step_array'],
                        signs_data['ingestion_array'], signs_data['stage'], signs_data['battery_voltage'],
                        signs_data['reset_cnt'], signs_data['signal_strength'], signs_data['utc_time'])}
                Handler.pub(msg)
            time_diff = utime.ticks_diff(utime.ticks_ms(), start)
            app_log.info("time_diff = {}".format(time_diff))
        elif msg_len:
            app_log.info(msg_len)
            message = uart2.read(msg_len).decode()
            app_log.info("uart2_read msg: {}".format(message))
            msg_id += 1
            msg = {"topic": aliyunClass.property_publish_topic,
                   "msg": msg_product_info_StatusInfo.format(msg_id, message)}
            Handler.pub(msg)
            if "##Read Memory Complete##" in message:
                collector_id = message[18:26]
                app_log.info(collector_id)
                msg_id += 1
                msg = {"topic": aliyunClass.property_publish_topic,
                       "msg": msg_product_info_CollectorID.format(msg_id, collector_id)}
                Handler.pub(msg)
        else:
            utime.sleep_ms(10)
            continue


def uart1_read():
    global uart1, msg_id
    msg_len = uart1.any()
    if msg_len:
        app_log.info("msg_len = {}".format(msg_len))
        msg = uart1.read(msg_len)
        app_log.info("uart1_read msg: {}".format(msg))
        gprmc_info = parse_gprmc(msg)
        app_log.info("gprmc_info = {}".format(gprmc_info))

        msg_id += 1
        msg = {"topic": aliyunClass.property_publish_topic,
               "msg": msg_geoLocation.format
               (msg_id, gprmc_info[0], gprmc_info[1], gprmc_info[2], 1)}
        Handler.pub(msg)


def hex_to_str(a, b=""):
    string = ''.join([hex_byte.replace('0x', b) for hex_byte in a])
    return string


def str_to_hex(s):
    list_hex = ' '.join([hex(ord(c)) for c in s]).split()
    list_temp = [int(i, 16) for i in list_hex]
    byte_array = bytearray(list_temp)
    return byte_array


class SysTopicClass(object):
    RRPC = "rrpc"
    OTA = "ota"
    PUB = "pub"
    SUB = "sub"


class ALiYunClass(object):
    def __init__(self):
        ALiYunClass.inst = self
        self.ProductKey = "a1GugRmVZzw"  # ????????????
        self.ProductSecret = 'MqLQtdNniH3hKf1r'  # ????????????????????????????????????????????????None???
        self.DeviceSecret = None  # ????????????????????????????????????????????????None???
        self.DeviceName = "BX-XC-200-003"  # ????????????

        self.property_subscribe_topic = "/sys" + "/" + self.ProductKey + "/" + \
                                        self.DeviceName + "/" + "thing/service/property/set"
        self.property_publish_topic = "/sys" + "/" + self.ProductKey + "/" + \
                                      self.DeviceName + "/" + "thing/event/property/post"

        # ??????aliyun????????????
        self.ali = aLiYun(
            self.ProductKey,
            self.ProductSecret,
            self.DeviceName,
            self.DeviceSecret)
        # ??????mqtt????????????
        client_id = self.ProductKey + "." + self.DeviceName  # ???????????????????????????64???
        self.ali.setMqtt(client_id, clean_session=False, keepAlive=60, reconn=True)  # False True

        # ??????????????????
        self.ali.setCallback(self.ali_sub_cb)

    def ali_sub_cb(self, topic, msg):  # ????????????
        if topic.decode().find(sys_topic.RRPC) != -1:
            sys_bus.publish(sys_topic.RRPC, {"topic": topic, "msg": msg})
        elif topic.decode().find(sys_topic.OTA) != -1:
            sys_bus.publish(sys_topic.OTA, {"topic": topic, "msg": msg})
        else:
            sys_bus.publish(sys_topic.SUB, {"topic": topic, "msg": msg})

    def ali_start(self):
        self.ali.start()
        print('Running')

    def ali_stop(self):
        self.ali.disconnect()

    def ali_subscribe(self):
        self.ali.subscribe(self.property_subscribe_topic, qos=0)

    def ali_publish(self, topic, msg):
        if not self.ali.getAliyunSta():
            try:
                self.ali.publish(msg.get('topic'), msg.get("msg"), qos=0)
            except BaseException:
                print('send fail!!!')


class Handler(object):
    @classmethod
    def sub(cls, topic, msg):
        global msg_id
        print(
            "Subscribe Recv: Topic={},Msg={}".format(
                msg.get("topic").decode(),
                msg.get("msg").decode()))
        msg_dict = ujson.loads(msg.get("msg").decode())
        app_log.info(msg_dict)
        if 'params' in msg_dict.keys():
            params_dict = msg_dict['params']
            if 'product_information:SendCommand' in params_dict.keys():
                app_log.info(params_dict['product_information:SendCommand'])
                msg_id += 1
                msg = {"topic": aliyunClass.property_publish_topic,
                       "msg": msg_product_info_SendCommand.format
                       (msg_id, params_dict['product_information:SendCommand'])}
                Handler.pub(msg)
                hex_array = str_to_hex(params_dict['product_information:SendCommand'])
                uart2_write(hex_array)

    @classmethod
    def pub(cls, msg):
        sys_bus.publish(sys_topic.PUB, msg)
        utime.sleep_ms(2000)

    @classmethod
    def ota(cls, topic, msg):
        """?????????ota???????????????????????????"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """??????publish?????????????????????????????????topic??????????????????????????????????????????"""
        sys_bus.publish_sync(sys_topic.PUB, msg)

    @classmethod
    def rrpc(cls, topic, msg):
        """??????rrpc??????????????????"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """??????publish??? """
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
        global msg_id
        r_data = data
        # Convert the temperature as described in the data book
        self.humidity = (r_data[0] << 12) | (r_data[1] << 4) | ((r_data[2] & 0xF0) >> 4)
        self.humidity = float('%.2f' % ((self.humidity / (1 << 20)) * 100.0))
        # print("current humidity is {0}%".format(self.humidity))
        self.temperature = ((r_data[2] & 0xf) << 16) | (r_data[3] << 8) | r_data[4]
        self.temperature = float('%.2f' % ((self.temperature * 200.0 / (1 << 20)) - 50))
        # print("current temperature is {0}??C".format(self.temperature))
        app_log.info("current temperature = {}??C, current humidity = {}%".format(self.temperature, self.humidity))
        msg_id += 1
        msg = {"topic": aliyunClass.property_publish_topic,
               "msg": msg_temperature_humidity.format(msg_id, self.temperature, self.humidity)}
        Handler.pub(msg)

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
    """ SGM58031 class, incloud read and write function"""

    def __init__(self, addr=0x48, alise="SGM58031"):
        # Initialization command
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
        self.flip_sign = True
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
        # -1- read the OS of Configuration Register
        global msg_id
        tmp = self.read_register([self.register_addr['CONF_REG']], 2)
        if tmp[0] >> 7 == 1:
            if self.flip_sign:
                tmp = self.read_register([self.register_addr['CONVERSION_REG']], 2)
                self.battery_voltage = float('%.3f' % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 11))
                app_log.info("battery_voltage = {}".format(self.battery_voltage))
            else:
                tmp = self.read_register([self.register_addr['CONVERSION_REG']], 2)
                self.voltage = float('%.3f' % (((tmp[0] << 8) | tmp[1]) / 32768 * 4.096 * 21))
                app_log.info("voltage = {}".format(self.voltage))

            msg_id += 1
            msg = {"topic": aliyunClass.property_publish_topic,
                   "msg": msg_voltage.format(msg_id, self.battery_voltage, self.voltage)}
            Handler.pub(msg)

        # -2- Initialise the SGM58031 peripheral
        if self.flip_sign:
            self.register_map['Config_Register'] = (self.register_map['Config_Register'] & 0x8fff) | \
                                                   (self.mux_sel['AINP_AIN0_AND_AINN_AIN1'] << 12 & 0x7000)
        else:
            self.register_map['Config_Register'] = (self.register_map['Config_Register'] & 0x8fff) | \
                                                   (self.mux_sel['AINP_AIN2_AND_AINN_AIN3'] << 12 & 0x7000)
        self.flip_sign = not self.flip_sign

        # -3- Set the Configuration Register
        self.write_register([self.register_addr['CONF_REG']],
                            [self.register_map['Config_Register'] >> 8,
                             (self.register_map['Config_Register'] & 0xff)])


def get_cell_location():
    global msg_id
    while True:
        cell_location = cellLocator.getLocation("www.queclocator.com", 80, "qa6qTK91597826z6", 8, 1)
        msg_id += 1
        msg = {"topic": aliyunClass.property_publish_topic,
               "msg": msg_cellLocator.format
               (msg_id, cell_location[0], cell_location[1], cell_location[2])}
        Handler.pub(msg)
        utime.sleep(86400)


def get_sim():
    global msg_id
    sim_imsi = net_ser.sim.getImsi()
    sim_iccid = net_ser.sim.getIccid()
    msg_id += 1
    msg = {"topic": aliyunClass.property_publish_topic,
           "msg": msg_sim.format(msg_id, sim_imsi, sim_iccid)}
    Handler.pub(msg)


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
    <1> UTC?????????hhmmss?????????????????????
    <2> ???????????????A=???????????????V=????????????
    <3> ??????ddmm.mmmm??????????????????????????????0??????????????????
    <4> ????????????N??????????????????S???????????????
    <5> ??????dddmm.mmmm??????????????????????????????0??????????????????
    <6> ????????????E???????????????W????????????
    <7> ???????????????000.0~999.9???????????????0?????????????????? 1???=1.852?????????km/h)
    <8> ???????????????000.0~359.9??????????????????????????????????????????0??????????????????
    <9> UTC?????????ddmmyy?????????????????????
    <10> ????????????000.0~180.0???????????????0??????????????????
    <11> ??????????????????E????????????W?????????
    <12> ??????????????????NMEA0183 3.00???????????????A=???????????????D=?????????E=?????????N=???????????????
    """
    li = data.decode().replace('$GPRMC,', '').strip().split(',')
    lat = log = speed = direct = 0
    if li[1] == 'A':
        lat = round(parse_loc_val(li[2], li[3]), 6)  # ??????
        log = round(parse_loc_val(li[4], li[5]), 6)  # ??????
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
    return float('%.2f' % rssi_dbm)


def log_callback(*args, **kwargs):
    app_log.debug("log_callback,args:{},kwargs:{}".format(args, kwargs))


def net_callback(*args, **kwargs):
    """??????????????????"""
    app_log.debug("net_callback,args:{},kwargs:{}".format(args, kwargs))


if __name__ == '__main__':
    utime.sleep(20)
    # ????????????
    guard_context = GuardContext()
    guard_context.refresh()
    # ????????????
    net_ser = guard_context.get_server("net")
    log_ser = guard_context.get_server("log")
    log_ser.set_level("INFO")
    pm_ser = guard_context.get_server("pm")
    pm_ser.lock()
    pm_ser.count()
    # ????????????????????????, ?????????????????????, 1???????????????0?????????
    pm_ser.auto_sleep(1)
    # ????????????
    net_ser.subscribe(net_callback)
    log_ser.subscribe(log_callback)
    # ??????app_log
    app_log = guard_context.get_logger("app_log")
    app_log.info("net status: {}".format(net_ser.get_net_status()))
    net_ser.wait_connect(5)

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
                        "Voltage": {{
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
                                    "collar_information:StepArray": {{
                                        "value": "{4}"
                                    }},
                                    "collar_information:IngestionArray": {{
                                        "value": "{5}"
                                    }},
                                    "collar_information:Stage": {{
                                        "value": {6}
                                    }},
                                    "collar_information:BatteryVoltage": {{
                                        "value": {7}
                                    }},
                                    "collar_information:ResetCnt": {{
                                        "value": {8}
                                    }},
                                    "collar_information:SignalStrength": {{
                                        "value": {9}
                                    }},
                                    "collar_information:UTCtime": {{
                                        "value": "{10}"
                                    }}
                                }}
                            }},
                            "method": "thing.event.property.post"
                         }}"""

    aliyunClass.ali_start()

    _thread.start_new_thread(get_cell_location, ())
    _thread.start_new_thread(uart2_read, ())

    while True:
        utime.sleep(120)
        uart1_read()
        ath10_dev.trigger_measurement()
        sgm58031_dev.measure_adc_value()
        get_sim()
