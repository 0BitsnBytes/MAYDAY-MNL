#!/usr/bin/env python3
from mpu6050 import mpu6050
import time

def main():
    print("Initializing MPU6050...")
    mpu = mpu6050(0x68)

    while True:
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        temp = mpu.get_temp()

        print("=== MPU6050 DATA ===")
        print(f"Accel (g): {accel}")
        print(f"Gyro (deg/s): {gyro}")
        print(f"Temp (C): {temp}") 
        print("---------------------")
        time.sleep(0.5)

        print("hello")

if __name__ == "__main__":
    main()
