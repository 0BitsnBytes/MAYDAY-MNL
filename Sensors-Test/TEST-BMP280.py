import time 
from bmp280 import BMP280 
import smbus2 
SEA_LEVEL_PRESSURE = 1013.25 # hPa 

def calculate_altitude(pressure_hpa, sea_level_hpa=SEA_LEVEL_PRESSURE): 
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** (1/5.255)) 

def main(): 
    print("Initializing BMP280...") 
    bus = smbus2.SMBus(1) # I2C bus 1 
    bmp = BMP280(i2c_dev=bus) 
    while True: 
        temp = bmp.get_temperature() 
        pres = bmp.get_pressure() 
        alt = calculate_altitude(pres) 
        print("=== BMP280 DATA ===") 
        print(f"Temperature: {temp:.2f} C") 
        print(f"Pressure: {pres:.2f} hPa") 
        print(f"Altitude: {alt:.2f} m") 
        print("---------------------") 
        time.sleep(0.5) 

if __name__ == "__main__": 
    main()
