from mpu6050 import mpu6050
from bmp280 import BMP280
import smbus2
import serial
import pynmea2
import math
import time

# ──────────────── Configuration ────────────────
anomalies = {
    "pitch": 45,
    "bank": 30,
    "gForceP": 2.7,
    "gForceN": -1,
    "descentRateUnder1000": 60960,     # 2000 ft/min
    "ascentRateOver1000": 182880,      # 6000 ft/min
    "ascentRate": 152400,              # 5000 ft/min
    "descentRate": 152400              # 5000 ft/min
}

# ──────────────── Global Variables ────────────────
gyro_data = {}
accel_data = {}
accel_x = accel_y = accel_z = 0.0
g_force = 0.0
pitch_angle = 0.0
bank_angle = 0.0
latitude = longitude = altitude = ground_speed_kmph = 0.0
altitude_prev = 0.0

# ──────────────── Sensor Setup ────────────────
def sensors_init():
    global bus, bmp280, sensor, gps_serial, accel_scale_modifier

    bus = smbus2.SMBus(1)
    bmp280 = BMP280(i2c_dev=bus)
    sensor = mpu6050(0x68)

    # Manually set accelerometer to ±4g
    ACCEL_CONFIG = 0x1C
    bus.write_byte_data(0x68, ACCEL_CONFIG, 0x08)  # AFS_SEL = 1 → ±4g

    accel_scale_modifier = 8192.0  # LSB/g for ±4g

    # Initialize GPS
    gps_serial = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)


# ──────────────── Read GPS Data ────────────────
def gps_data():
    global latitude, longitude, altitude, ground_speed_kmph

    try:
        line = gps_serial.readline().decode('ascii', errors='replace')
        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
            msg = pynmea2.parse(line)

            if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                latitude = msg.latitude
                longitude = msg.longitude

            if hasattr(msg, 'altitude'):
                altitude = float(msg.altitude)

            if hasattr(msg, 'spd_over_grnd'):
                ground_speed_kmph = float(msg.spd_over_grnd) * 1.852  # knots → km/h

    except Exception as e:
        print(f"[GPS Error] {e}")

# ──────────────── Read BMP280 Data ────────────────
def bmp_data():
    global temperature, pressure, pressure_hpa, altitude

    try:
        temperature = bmp280.get_temperature()  # °C
        pressure = bmp280.get_pressure()        # Pa
        pressure_hpa = pressure / 100.0         # hPa
        altitude = bmp280.get_altitude()        # Meters (if available in library)

    except Exception as e:
        print(f"[BMP Error] {e}")

# ──────────────── Read MPU6050 Data ────────────────
def mpu_data():
    global gyro_data, accel_data, accel_x, accel_y, accel_z

    try:
        gyro_data = sensor.get_gyro_data()       # °/s
        raw_accel = sensor.get_accel_data()      # Already scaled as g by mpu6050 lib

        # But we override scaling manually to match ±4g config
        raw_x = sensor.read_i2c_word(0x3B)
        raw_y = sensor.read_i2c_word(0x3D)
        raw_z = sensor.read_i2c_word(0x3F)

        accel_x = raw_x / accel_scale_modifier
        accel_y = raw_y / accel_scale_modifier
        accel_z = raw_z / accel_scale_modifier

        gForce()

    except Exception as e:
        print(f"[MPU Error] {e}")


# ──────────────── Calculate Total G-Force ────────────────
def gForce():
    global g_force
    g_force = math.sqrt(accel_x ** 2 + accel_y ** 2 + accel_z ** 2)

# ──────────────── Get Pitch Angle ────────────────
def get_pitch(dt):
    global pitch_angle

    try:
        gyro_y = gyro_data['y']
        accel_pitch = math.degrees(math.atan2(accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)))
        pitch_from_gyro = pitch_angle + gyro_y * dt

        alpha = 0.98
        pitch_angle = alpha * pitch_from_gyro + (1 - alpha) * accel_pitch

        return pitch_angle

    except Exception as e:
        print(f"[Pitch Error] {e}")
        return None

# ──────────────── Get Bank (Roll) Angle ────────────────
def get_bank(dt):
    global bank_angle

    try:
        gyro_x = gyro_data['x']
        accel_bank = math.degrees(math.atan2(accel_y, accel_z))
        bank_from_gyro = bank_angle + gyro_x * dt

        alpha = 0.98
        bank_angle = alpha * bank_from_gyro + (1 - alpha) * accel_bank

        return bank_angle

    except Exception as e:
        print(f"[Bank Error] {e}")
        return None

# ──────────────── Anomaly Detection ────────────────
def detect_anomaly(vertical_rate=None):
    if abs(pitch_angle) >= anomalies['pitch']:
        return True, "Pitch Angle Exceeded"

    elif abs(bank_angle) >= anomalies['bank']:
        return True, "Bank Angle Exceeded"

    elif g_force >= anomalies['gForceP']:
        return True, "High G-Force"

    elif g_force <= anomalies['gForceN']:
        return True, "Negative G-Force"

    if vertical_rate is not None:
        if altitude < 1000:
            if vertical_rate < -anomalies['descentRateUnder1000']:
                return True, "Rapid Descent (<1000m)"
        else:
            if vertical_rate > anomalies['ascentRateOver1000']:
                return True, "Rapid Ascent (>1000m)"

        if vertical_rate > anomalies['ascentRate']:
            return True, "Rapid Ascent"

        elif vertical_rate < -anomalies['descentRate']:
            return True, "Rapid Descent"

    return False, None

# ──────────────── Main Loop ────────────────
if __name__ == "__main__":
    sensors_init()

    last_time = time.time()
    altitude_prev = 0.0

    print("📡 Starting sensor read loop...")

    while True:
        try:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            mpu_data()
            bmp_data()
            gps_data()

            pitch = get_pitch(dt)
            bank = get_bank(dt)

            # Calculate vertical rate (cm/min)
            vertical_rate = ((altitude - altitude_prev) / dt) * 60 * 100
            altitude_prev = altitude

            # Print sensor data (optional)
            print(f"Pitch: {pitch:.1f}°, Bank: {bank:.1f}°, G: {g_force:.2f}g, Alt: {altitude:.2f}m, VRate: {vertical_rate:.1f} cm/min")

            # Detect anomalies
            anomaly, reason = detect_anomaly(vertical_rate)
            if anomaly:
                print(f"⚠️  Anomaly Detected: {reason}")

            time.sleep(0.2)

        except KeyboardInterrupt:
            print("Exiting.")
            break
