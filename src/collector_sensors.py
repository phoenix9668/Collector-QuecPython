"""Low-frequency sensors and device information tasks for EC600M."""

import _thread
import utime
from machine import I2C

from collector_config import (
    CELL_LOCATOR_HOST,
    CELL_LOCATOR_LOCATION_TYPE,
    CELL_LOCATOR_PORT,
    CELL_LOCATOR_TIMEOUT,
    CELL_LOCATOR_TOKEN,
)


class AHT10:
    ADDRESS = 0x38

    def __init__(self):
        self.i2c = I2C(I2C.I2C1, I2C.STANDARD_MODE)
        self.i2c.write(self.ADDRESS, bytearray(0), 0, bytearray([0xE1, 0x08, 0]), 3)
        utime.sleep_ms(300)

    def measure(self):
        self.i2c.write(self.ADDRESS, bytearray(0), 0, bytearray([0xAC, 0x33, 0]), 3)
        utime.sleep_ms(200)
        data = bytearray(6)
        self.i2c.read(self.ADDRESS, bytearray(0), 0, data, 6, 0)
        if data[0] & 0x80:
            raise OSError("AHT10 busy")
        raw_humidity = (data[1] << 12) | (data[2] << 4) | ((data[3] & 0xF0) >> 4)
        raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        humidity = float("%.2f" % (raw_humidity * 100.0 / (1 << 20)))
        temperature = float("%.2f" % (raw_temperature * 200.0 / (1 << 20) - 50))
        return temperature, humidity


class SGM58031:
    ADDRESS = 0x48
    CONVERSION = 0x00
    CONFIG = 0x01
    CONFIG1 = 0x04
    CHIP_ID = 0x05

    def __init__(self):
        self.i2c = I2C(I2C.I2C1, I2C.STANDARD_MODE)
        self.battery_selected = True
        # Single-shot, +/-4.096 V, 800 Hz, comparator disabled.
        self.base_config = 0x8000 | (1 << 9) | (1 << 8) | (7 << 5) | 3
        self._write(self.CONFIG1, 0)
        self._select_channel(self.battery_selected)

    def _write(self, register, value):
        self.i2c.write(
            self.ADDRESS,
            bytearray([register]),
            1,
            bytearray([(value >> 8) & 0xFF, value & 0xFF]),
            2,
        )

    def _read(self, register):
        data = bytearray(2)
        self.i2c.write(self.ADDRESS, bytearray([register]), 1, bytearray(0), 0)
        self.i2c.read(self.ADDRESS, bytearray(0), 0, data, 2, 0)
        return (data[0] << 8) | data[1]

    def verify(self):
        return self._read(self.CHIP_ID) == 0x80

    def _select_channel(self, battery_channel):
        # Existing hardware wiring: AIN2-AIN3 is battery (x11), AIN0-AIN1
        # is external power (x21).
        mux = 3 if battery_channel else 0
        self._write(self.CONFIG, self.base_config | (mux << 12))

    def measure(self):
        # The previous conversion belongs to the channel selected on the last call.
        raw = self._read(self.CONVERSION)
        if self.battery_selected:
            battery = float("%.3f" % (raw / 32768.0 * 4.096 * 11))
            power = None
        else:
            power = float("%.3f" % (raw / 32768.0 * 4.096 * 21))
            battery = None
        self.battery_selected = not self.battery_selected
        self._select_channel(self.battery_selected)
        return battery, power


class SensorService:
    SENSOR_INTERVAL_SECONDS = 1200
    SIM_INTERVAL_SECONDS = 7200
    RETRY_SECONDS = 5

    def __init__(self, config, cloud, logger):
        self.config = config
        self.cloud = cloud
        self.logger = logger
        self.aht10 = None
        self.adc = None
        self.battery = 0.0
        self.power = 0.0
        self.ota_paused = False

    def pause_for_ota(self):
        self.ota_paused = True
        return True

    def resume_after_ota(self):
        self.ota_paused = False
        return True

    def start(self):
        try:
            self.aht10 = AHT10()
        except Exception as error:
            self.logger.error("sensor", "AHT10_INIT", str(error))
        try:
            self.adc = SGM58031()
            if not self.adc.verify():
                self.logger.warn("sensor", "SGM_ID", "SGM58031 chip ID mismatch")
        except Exception as error:
            self.logger.error("sensor", "SGM_INIT", str(error))
            self.adc = None
        _thread.start_new_thread(self._sensor_worker, ())
        _thread.start_new_thread(self._sim_worker, ())
        _thread.start_new_thread(self._location_worker, ())

    def _wait_cloud(self):
        while not self.cloud.connected and not self.ota_paused:
            utime.sleep(1)
        return not self.ota_paused

    def _collect_sensor_values(self):
        values = {}
        if self.aht10:
            try:
                temperature, humidity = self.aht10.measure()
                values["Temperature"] = temperature
                values["Humidity"] = humidity
            except Exception as error:
                self.logger.warn(
                    "sensor", "AHT10_READ", str(error), rate_limit_s=300
                )
        if self.adc:
            try:
                # Allow the channel selected during initialization to finish,
                # then read both alternating channels for a complete snapshot.
                utime.sleep_ms(5)
                for index in range(2):
                    battery, power = self.adc.measure()
                    if battery is not None:
                        self.battery = battery
                        values["BatteryVoltage"] = battery
                    if power is not None:
                        self.power = power
                        values["PowerVoltage"] = power
                    if index == 0:
                        utime.sleep_ms(5)
            except Exception as error:
                self.logger.warn(
                    "sensor", "SGM_READ", str(error), rate_limit_s=300
                )
        return values

    def _collect_sim_values(self):
        try:
            import sim

            return {
                "product_information:IMSI": sim.getImsi(),
                "product_information:ICCID": sim.getIccid(),
            }
        except Exception as error:
            self.logger.warn("sim", "READ", str(error), rate_limit_s=300)
            return {}

    def _publish_cycle(self, include_sim=False):
        if self.ota_paused:
            return False
        values = self._collect_sensor_values()
        if include_sim:
            values.update(self._collect_sim_values())
        if not values:
            return False
        try:
            return bool(self.cloud.publish_properties(values, qos=1 if include_sim else 0))
        except Exception as error:
            self.logger.warn(
                "sensor", "PUBLISH", str(error), rate_limit_s=300
            )
            return False

    def _sensor_worker(self):
        startup_pending = True
        while True:
            if not self._wait_cloud():
                utime.sleep(1)
                continue
            if self._publish_cycle(include_sim=startup_pending):
                startup_pending = False
                utime.sleep(self.SENSOR_INTERVAL_SECONDS)
            else:
                utime.sleep(self.RETRY_SECONDS)

    def _sim_worker(self):
        # The sensor worker includes SIM identity in the startup snapshot.
        utime.sleep(self.SIM_INTERVAL_SECONDS)
        while True:
            if not self._wait_cloud():
                utime.sleep(1)
                continue
            values = self._collect_sim_values()
            try:
                published = bool(values) and self.cloud.publish_properties(values)
            except Exception as error:
                self.logger.warn("sim", "PUBLISH", str(error), rate_limit_s=300)
                published = False
            utime.sleep(
                self.SIM_INTERVAL_SECONDS if published else self.RETRY_SECONDS
            )

    def _location_worker(self):
        import cellLocator

        # Preserve the address, token, parameters and 24-hour interval used by
        # the original EC600M Collector application.
        while True:
            utime.sleep(86400)
            if self.ota_paused:
                continue
            try:
                location = cellLocator.getLocation(
                    CELL_LOCATOR_HOST,
                    CELL_LOCATOR_PORT,
                    CELL_LOCATOR_TOKEN,
                    CELL_LOCATOR_LOCATION_TYPE,
                    CELL_LOCATOR_TIMEOUT,
                )
                self.cloud.publish_properties(
                    {
                        "CellLocator": {
                            "Longitude": {"value": location[0]},
                            "Latitude": {"value": location[1]},
                            "Accuracy": {"value": location[2]},
                        }
                    }
                )
            except Exception as error:
                self.logger.warn("location", "READ", str(error), rate_limit_s=3600)
