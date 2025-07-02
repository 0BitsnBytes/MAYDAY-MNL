from mpu6050 import mpu6050
import smbus2
from bmp280 import BMP280
import math

anomalies = {
    pitch : 45,
    bank : 30,
    gForceP : 2.7,
    gForceN : -1,
    descentRateUnder1000 : 2000,
    descentRateOver1000 : 6000,
    ascentRate : 5000,

}

bus = smbus2.SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
sensor = mpu6050(0x68)

def bmp280Data():

    global temperature , pressure , pressure_hpa , altitude
    temperature = bmp280.get_temperature()  # °C
    pressure = bmp280.get_pressure()        # Pa
    pressure_hpa = pressure / 100            # convert to hPa

    sea_level_pressure = 1013.25  # hPa adjust this
    altitude = bmp280.get_altitude(sea_level_pressure=sea_level_pressure)


def mpuData(sensor):

    global gyro_data , accel_data
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()

def gForce():
    
    x = accel_data['x']
    y = accel_data['y']
    z = accel_data['z']
    return math.sqrt(x**2 + y**2 + z**2)


print(f"Total g-force magnitude: {gforce():.3f} g")

if __name__ == "__main__":
    while True:
        mpuData()
