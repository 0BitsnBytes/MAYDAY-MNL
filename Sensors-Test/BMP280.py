import time
from smbus2 import SMBus
from bmp280 import BMP280

# Initialise the BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus , i2c_addr=0x76)

while True:
    temperature = bmp280.get_temperature()
    pressure = bmp280.get_pressure()
    print(f"{temperature:05.2f}*C {pressure:05.2f}hPa")
    time.sleep(0.5)
