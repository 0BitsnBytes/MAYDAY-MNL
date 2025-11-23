import asyncio
from bmp280 import BMP280
import smbus2
import math

class AsyncBMP:
    def __init__(self, sea_level_hpa=1013.25, bus_num=1, address=0x76):
        self.bus = smbus2.SMBus(bus_num)
        self.bmp = BMP280(i2c_dev=self.bus)

        self.sea_level = sea_level_hpa
        self.latest = None

    def read_sensor(self):
        """Read temperature, pressure, altitude (meters)."""
        try:
            temp = self.bmp.get_temperature()
            pressure = self.bmp.get_pressure()

            # Calculate altitude using barometric formula
            altitude = 44330 * (1.0 - (pressure / self.sea_level) ** (1/5.255))

            return {
                "temperature": temp,
                "pressure": pressure,
                "altitude": altitude
            }

        except Exception:
            return None

    async def read_loop(self):
        """Async background reader."""
        while True:
            data = self.read_sensor()
            if data:
                self.latest = data

            await asyncio.sleep(0.05)  # 20 Hz sampling

    def get(self):
        """Return latest sample or None."""
        return self.latest
