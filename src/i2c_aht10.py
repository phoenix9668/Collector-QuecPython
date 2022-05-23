import log
from machine import I2C
import utime as time


class Aht10Class:
    """ AHT10 class, incloud reset, read, write, measurement function"""

    def __init__(self, addr=0x38, alise="Ath10"):
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
        time.sleep_ms(300)  # at last 300ms
        pass

    def ath10_reset(self):
        # reset
        self.write_data([self.AHT10_RESET_CMD])
        time.sleep_ms(20)  # at last 20ms

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
        self.humidity = (r_data[0] << 12) | (
                r_data[1] << 4) | ((r_data[2] & 0xF0) >> 4)
        self.humidity = ((self.humidity / (1 << 20)) * 100.0)
        self.humidity = round(self.humidity, 2)
        print("current humidity is {0}%".format(self.humidity))
        self.temperature = ((r_data[2] & 0xf) << 16) | (
                r_data[3] << 8) | r_data[4]
        self.temperature = (self.temperature * 200.0 / (1 << 20)) - 50
        self.temperature = round(self.temperature, 2)
        print("current temperature is {0}°C".format(self.temperature))
        time.sleep(5)

    def trigger_measurement(self):
        while True:
            # Trigger data conversion
            self.write_data([self.AHT10_START_MEASURMENT_CMD, 0x33, 0x00])
            time.sleep_ms(200)  # at last delay 75ms
            # check has success
            r_data = self.read_data(6)
            # check bit7
            if (r_data[0] >> 7) != 0x0:
                print("Conversion has error")
            else:
                self.aht10_transformation_temperature(r_data[1:6])

