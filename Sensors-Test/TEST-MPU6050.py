from mpu6050 import mpu6050
from time import sleep
import math

sensor = mpu6050(0x68)

while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()

    print("Accelerometer data")
    print("x: " + str(accel_data['x']))
    print("y: " + str(accel_data['y']))
    print("z: " + str(accel_data['z']))

    g_force = math.sqrt(
        accel_data['x']**2 +
        accel_data['y']**2 +
        accel_data['z']**2
    ) / 9.80665


    print(f"G-Force: {g_force:.2f} g")


    print("Gyroscope data")
    print("x: " + str(gyro_data['x']))
    print("y: " + str(gyro_data['y']))
    print("z: " + str(gyro_data['z']))

    print("Temp: " + str(temp) + " C")
    sleep(0.01)
