from vosk import Model, KaldiRecognizer
from datetime import datetime           # Date recorder
from mpu6050 import mpu6050             # Gyro and accel package
from bmp280 import BMP280               # Barometric pressure package
import pynmea2                          # GPS module package
import smbus2                           # i2c protocol library
import socket                           # Data transmission library
import serial                           # Serial communication protocol
import math                             # Extra math functions
import time                             # Time management library
import json
import csv                              # CSV file handler
import os
import pyaudio

# ──────────────── Global Variables ────────────────
mpu_pitch_angle = 0.0
mpu_bank_angle = 0.0
mpu_g_force = 0.0

gps_lat = 0.0
gps_long = 0.0
gps_alt = 0.0
gps_ground_speed_kmph = 0.0

bmp_temp = 0.0
bmp_pressure = 0.0

voice_recognition = None

# ──────────────── Global Configs / CONSTANTS ────────────
IP_ADDRESS = '192.168.68.50'
DEBUG_MODE = True
CHECK_FREQ = 5

ACCEL_CONFIG = 0x1C              # AFS_SEL = 1 → ±4g
MPU6050_ADDRESS = 0x68           # MPU 6050 i2c Address
ACCEL_SCALE_MODIFIER = 8192.0    # LSB/g for ±4g
ALPHA = 0.4

FEET_TO_CM = 30.48
KNOTS_TO_KMPH = 1.852
KPA_TO_HPA = 10

SPEECH_PHRASES = ["mayday", "emergency"]
SAMPLE_RATE = 16000                         # lower than 16000 to reduce CPU, still OK quality
BUFFER_SIZE = 4000                          # small buffer for responsiveness

# ──────────────── Triggers to activation ────────────────
triggers = {
    "pitch": 45,
    "bank": 30,
    "gForceP": 2.5,
    "gForceN": -1,
    "descentRateUnder1000": 2000 * FEET_TO_CM,    # 2000 ft/min
    "ascentRateOver1000": 6000 * FEET_TO_CM,      # 6000 ft/min
    "ascentRate": 5000 * FEET_TO_CM,              # 5000 ft/min
    "descentRate": 5000 * FEET_TO_CM,             # 5000 ft/min
    "cabin_pressure": 70 * KPA_TO_HPA,            # 75 kPa
    "lowTemp" : 10,                               # °C
    "highTemp" : 50                               # °C
}

# ──────────────── Log File Setup ────────────────
log_filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Write headers if the file is new
with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "timestamp", "latitude", "longitude", "altitude_m", "ground_speed_kmph",
        "pitch_deg", "bank_deg", "g_force", "vertical_rate_cm_min", "cabin_pressure", "cabin_temperature",
        "voice_recognised", "anomaly_detected", "anomaly_reason", "anomaly_value"
    ])

# ──────────────── Data Logger ─────────────────
def data_log(anomaly, reason, amount):

    global gps_lat, gps_long, gps_alt, gps_ground_speed_kmph, mpu_pitch_angle, mpu_bank_angle, mpu_g_force, vertical_rate, bmp_pressure, bmp_temp, voice_recognition

    with open(log_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            datetime.now().isoformat(),
            gps_lat,
            gps_long,
            gps_alt,
            gps_ground_speed_kmph,
            mpu_pitch_angle,
            mpu_bank_angle,
            mpu_g_force,
            vertical_rate,
            bmp_pressure,
            bmp_temp,
            str(voice_recognition),
            anomaly,
            reason if anomaly else "",
            amount if anomaly else ""
        ])

# ──────────────── Sensor Setup ────────────────
def sensors_init():
    
    global mpu_sensor, bmp_sensor, gps_sensor, speech_recognizer, stream, p

    # Manually set accelerometer to ±4g
    bus = smbus2.SMBus(1) # init i2c protocol
    mpu_sensor = mpu6050(MPU6050_ADDRESS) # MPU 6050 i2c protocol (Gryro & Accel package)
    bus.write_byte_data(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08)  # AFS_SEL = 1 → ±4g

    # Barometric pressure and temperature package
    bmp_sensor = BMP280(i2c_dev=bus)

    # Initialize GPS
    gps_sensor = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)

    # Initialize Speech Recog Model
    speech_model_path = os.path.expanduser("~/vosk-model")
    speech_model = Model(speech_model_path)

    grammar = json.dumps(SPEECH_PHRASES)
    speech_recognizer = KaldiRecognizer(speech_model, SAMPLE_RATE, grammar)

    # Initializing and Streaming Microphone
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=BUFFER_SIZE)
    stream.start_stream()

# ──────────────── Read GPS Data ────────────────
def gps_data():

    global gps_lat, gps_long, gps_alt, gps_ground_speed_kmph

    try:
        gps_out = gps_sensor.readline().decode('ascii', errors='replace')
        if gps_out.startswith('$GPGGA') or gps_out.startswith('$GPRMC'):
            msg = pynmea2.parse(gps_out)

            if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                gps_lat = msg.latitude
                gps_long = msg.longitude

            if hasattr(msg, 'altitude'):
                gps_alt = round(float(msg.altitude), 0)

            if hasattr(msg, 'spd_over_grnd'):
                gps_ground_speed_kmph = round(float(msg.spd_over_grnd) * KNOTS_TO_KMPH, 0)  # knots → km/h

    except Exception as e:
        print(f"[GPS Error] {e}")

# ──────────────── Read BMP280 Data ────────────────
def bmp_data():

    global bmp_temp, bmp_pressure #, altitude

    try:
        bmp_temp = round(bmp_sensor.get_temperature(), 1)  # °C
        pressure = bmp_sensor.get_pressure()      # Pa
        bmp_pressure = round(pressure / 100.0, 0)         # hPa
        # altitude = bmp_sensor.get_altitude()        # Meters (if available in library)

    except Exception as e:
        print(f"[BMP Error] {e}")

# ──────────────── Read MPU6050 Data ────────────────
def mpu_data(dt):

    global mpu_g_force, mpu_pitch_angle, mpu_bank_angle

    try:
        gyro_data = mpu_sensor.get_gyro_data()       # °/s
        accel_data = mpu_sensor.get_accel_data()       

        # But we override scaling manually to match ±4g config
        accel_x = accel_data['x'] / ACCEL_SCALE_MODIFIER
        accel_y = accel_data['y'] / ACCEL_SCALE_MODIFIER
        accel_z = accel_data['z'] / ACCEL_SCALE_MODIFIER

        # ──────────────── Calculate G Force ────────────────
        mpu_g_force = math.sqrt(accel_x ** 2 + accel_y ** 2 + accel_z ** 2)

        # ──────────────── Get Pitch and Bank (Roll) Angle ────────────────
        gyro_y = gyro_data['y']
        accel_pitch = math.degrees(math.atan2(accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)))
        pitch_from_gyro = mpu_pitch_angle + gyro_y 

        mpu_pitch_angle = round(ALPHA * pitch_from_gyro + (1 - ALPHA) * accel_pitch, 1)

        # ──────────────── Get Bank (Roll) Angle ────────────────=
        gyro_x = gyro_data['x']
        accel_bank = math.degrees(math.atan2(accel_y, accel_z))
        bank_from_gyro = mpu_bank_angle + gyro_x

        mpu_bank_angle = round(ALPHA * bank_from_gyro + (1 - ALPHA) * accel_bank, 1)

    except Exception as e:
        print(f"[MPU Read Error] {e}")

# ──────────────── USB Microphone data ────────────────
def voice_data():
    try:
        data = stream.read(BUFFER_SIZE, exception_on_overflow=False)
        if speech_recognizer.AcceptWaveform(data):
            result = json.loads(speech_recognizer.Result())
            text = result.get("text", "").lower()
            #print(f"? Recognized: {text}")
            if any(phrase in text for phrase in SPEECH_PHRASES):
                return text
            else:
                return None

    except KeyboardInterrupt:
        print("\n? Stopped by user.")

# ──────────────── Anomaly Detection ────────────────
def detect_anomaly(vertical_rate):
    if abs(mpu_pitch_angle) >= triggers['pitch']:
        return True, "Pitch Angle Exceeded" , mpu_pitch_angle

    if abs(mpu_bank_angle) >= triggers['bank']:
        return True, "Bank Angle Exceeded" , mpu_bank_angle

    if mpu_g_force >= triggers['gForceP']:
        return True, "High G-Force" , mpu_g_force

    if mpu_g_force <= triggers['gForceN']:
        return True, "High Negative G-Force" , mpu_g_force
    
    if bmp_pressure < triggers['cabin_pressure']:
        return True , "Low Cabin Pressure" , bmp_pressure
    
    if bmp_temp > triggers["highTemp"]:
        return True , "High Temperature" , bmp_temp
    
    if bmp_temp < triggers["lowTemp"]:
        return True , "Low Temperature" , bmp_temp
    
    if voice_recognition != None:
        return True, "Distress Signal" , voice_recognition

    if vertical_rate is not None:
        if gps_alt < 1000:
            if vertical_rate < -triggers['descentRateUnder1000']:
                return True, "Rapid Descent (<1000m)" , vertical_rate
        else:
            if vertical_rate > triggers['ascentRateOver1000']:
                return True, "Rapid Ascent (>1000m)" , vertical_rate

        if vertical_rate > triggers['ascentRate']:
            return True, "Rapid Ascent" , vertical_rate

        elif vertical_rate < -triggers['descentRate']:
            return True, "Rapid Descent" , vertical_rate

    return False, None, None

# ──────────────── Anomaly data send ────────────────
def send_data(data):
    port = 5000

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((IP_ADDRESS, port))
        s.sendall(data.encode())

# ──────────────── Logged data send ────────────────
def send_log_file():
    port = 5001  # Use a different port for log file transfer

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((IP_ADDRESS, port))

            with open(log_filename, 'r') as f:
                contents = f.read()

            s.sendall(contents.encode())
            print("✅ Log file sent successfully.")

    except Exception as e:
        print(f"[Log File Send Error] {e}")

# ──────────────── Main Loop ────────────────
if __name__ == "__main__":
     
    sensors_init()

    prev_time = time.time()
    prev_altitude = 0.0

    print("📡 Starting sensor read loop...")

    while True:
        try:
            current_time = time.time()
            dt = current_time - prev_time # Delta time
            prev_time = current_time
            voice_recognition = None

            mpu_data(dt)
            bmp_data()
            gps_data()

            print(dt)

            # Calculate vertical rate (cm/min)
            vertical_rate = ((gps_alt - prev_altitude) / dt) * 60 * 100
            prev_altitude = gps_alt
   
            voice_recognition = voice_data()
            
            if DEBUG_MODE:
                # Print sensor data (optional)
                print(f"""
                      Pitch: {mpu_pitch_angle:.1f}°, 
                      Bank: {mpu_bank_angle:.1f}°, 
                      G-force: {mpu_g_force:.2f}g, 
                      Alt: {gps_alt:.2f}m, 
                      Vertical Rate: {vertical_rate:.1f} cm/min, 
                      Pressure: {bmp_pressure:.1f} hPa, 
                      Temperature: {bmp_temp:.1f} °C,
                      Distress Signal: {voice_recognition},
                      Latitude: {gps_lat},
                      Longitude: {gps_long}""")

            # Detect triggers
            anomaly, reason, amount = detect_anomaly(vertical_rate)
            if anomaly:
                print(f"⚠️ Anomaly Detected: {reason}")

                data_package = {
                    "latitude" : gps_lat,
                    "longitude" : gps_long,
                    "altitude" : gps_alt,
                    "ground_speed_kmph" : gps_ground_speed_kmph,
                    "anomaly" : [reason, amount]
                }

                send_data(str(data_package))  # Send minimal info
                send_log_file()               # Send entire log

            data_log(anomaly, reason, amount)
            time.sleep(CHECK_FREQ)

        except KeyboardInterrupt:
            print("Exiting.")
            stream.stop_stream()
            stream.close()
            p.terminate()
            break
