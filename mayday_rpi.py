from datetime import datetime           # Date recorder
from mpu6050 import mpu6050             # Gyro and accel package
from bmp280 import BMP280               # Barometric pressure package
import sounddevice as sd                # Real-time audio input from mic
import numpy as np                      # For numerical operations
import threading                        # For running parallel background tasks
import pynmea2                          # GPS module package
import whisper                          # Speech-to-text transcription
import smbus2                           # i2c protocol library
import socket                           # Data transmission library
import serial                           # Serial communication protocol
import queue                            # Thread-safe FIFO queue
import torch
import math                             # Extra math functions
import time                             # Time management library
import csv                              # CSV file handler
import re                               # String manipulation


# ──────────────── Global Variables ────────────────
mpu_pitch_angle = 0.0
mpu_bank_angle = 0.0
mpu_g_force = 0.0
vertical_velocity = 0.0  # m/s

gps_lat = 0.0
gps_long = 0.0
gps_alt = 0.0
gps_ground_speed_kmph = 0.0

bmp_temp = 0.0
bmp_pressure = 0.0

voice_recognition = None

# ──────────────── Global Configs / CONSTANTS ────────────
IP_ADDRESS = '192.168.68.50'
DEBUG_MODE = False
CHECK_FREQ = 1 # >= 0.5

ACCEL_CONFIG = 0x1C              # AFS_SEL = 1 → ±4g
MPU6050_ADDRESS = 0x68           # MPU 6050 i2c Address
ACCEL_SCALE_MODIFIER = 8192.0    # LSB/g for ±4g
ALPHA = 0.4

FEET_TO_CM = 30.48
KNOTS_TO_KMPH = 1.852
KPA_TO_HPA = 10

SPEECH_PHRASES = ["mayday", "emergency", "help"]
SAMPLE_RATE = 16000                          # lower than 16000 to reduce CPU, still OK quality
BUFFER_SIZE = 16000                          # small buffer for responsiveness

# Whisper Parameters
CHUNK_DURATION = 3  # seconds
CHANNELS = 1

# ──────────────── Triggers to activation ────────────────
triggers = {
    "pitch": 45,
    "bank": 30,
    "gForceP": 2,
    "gForceN": -0.5,
    "ascentRate": 1.52,   # m/s ascent threshold
    "descentRate": -1.52, # m/s descent threshold
    "cabin_pressure": 70 * KPA_TO_HPA,           # 75 kPa
    "lowTemp" : 10,                              # °C
    "highTemp" : 50,                             # °C
    "distress" : SPEECH_PHRASES
}

# ──────────────── Log File Setup ────────────────
log_filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Write headers if the file is new
with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "timestamp", "lat", "long", "alt_m", "gr_speed_kmph", "pitch_deg", "bank_deg", 
        "g_force", # "vert_rate_cm_min", 
        "cabin_pressure", "cabin_temp", "voice_recog", "anomaly", "a_reason", "a_val"
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
            #vertical_rate,
            bmp_pressure,
            bmp_temp,
            str(voice_recognition),
            anomaly,
            reason if anomaly else "",
            amount if anomaly else ""
        ])

# ──────────────── Sensor Setup ────────────────
def sensors_init():
    
    global mpu_sensor, bmp_sensor, gps_sensor, audio_queue, latest_transcription, model

    # Manually set accelerometer to ±4g
    bus = smbus2.SMBus(1)                                     # init i2c protocol
    mpu_sensor = mpu6050(MPU6050_ADDRESS)                     # MPU 6050 i2c protocol (Gryro & Accel package)
    bus.write_byte_data(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08)  # AFS_SEL = 1 → ±4g

    # Barometric pressure and temperature package
    bmp_sensor = BMP280(i2c_dev=bus)

    # Initialize GPS
    gps_sensor = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)

    # Initialize Speech Recog Model
    # Shared queue and transcription result
    audio_queue = queue.Queue()
    latest_transcription = None

    # Load Whisper model
    model = whisper.load_model("tiny")  # or "tiny" for Raspberry Pi

# ──────────────── Read GPS Data ────────────────
def gps_data():
    global gps_lat, gps_long, gps_alt, gps_ground_speed_kmph

    try:
        for _ in range(10):  # Try 10 lines per call
            gps_out = gps_sensor.readline().decode('ascii', errors='replace').strip()
            
            if gps_out.startswith('$GPGGA') or gps_out.startswith('$GPRMC'):
                msg = pynmea2.parse(gps_out)

                if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                    gps_lat = msg.latitude
                    gps_long = msg.longitude

                if hasattr(msg, 'altitude'):
                    gps_alt = round(float(msg.altitude), 0)

                if hasattr(msg, 'spd_over_grnd'):
                    gps_ground_speed_kmph = round(float(msg.spd_over_grnd) * KNOTS_TO_KMPH, 0)
                
                break  # exit loop after successful parse

    except Exception as e:
        print(f"[GPS Error] {e}")


# ──────────────── Read BMP280 Data ────────────────
def bmp_data():

    global bmp_temp, bmp_pressure #, altitude

    try:
        bmp_temp = round(bmp_sensor.get_temperature(), 1)   # °C
        pressure = bmp_sensor.get_pressure()                # Pa
        bmp_pressure = round(pressure,2)                    # hPa
        # altitude = bmp_sensor.get_altitude()              # Meters (if available in library)

    except Exception as e:
        print(f"[BMP Error] {e}")

# ──────────────── Read MPU6050 Data ────────────────
def mpu_data(dt):

    global mpu_g_force, mpu_pitch_angle, mpu_bank_angle, vertical_accel

    try:
        gyro_data = mpu_sensor.get_gyro_data()  # °/s
        accel_data = mpu_sensor.get_accel_data()       

        # But we override scaling manually to match ±4g config
        accel_x = accel_data['x'] 
        accel_y = accel_data['y'] 
        accel_z = accel_data['z'] 

        # ──────────────── Calculate G Force ────────────────
        mpu_g_force = round(math.sqrt(accel_x**2 + accel_y**2 + accel_z**2) / 9.81, 2)

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

        # ───── Estimate Vertical Acceleration (in m/s²) ─────        
        
        #gyro_y = gyro_data['y']
        #accel_pitch = math.degrees(math.atan2(accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)))
        #pitch_from_gyro = mpu_pitch_angle + gyro_y * dt  # Note: multiply gyro by dt here
        #mpu_pitch_angle = round(ALPHA * pitch_from_gyro + (1 - ALPHA) * accel_pitch, 1)
        
        #gyro_x = gyro_data['x']
        #accel_bank = math.degrees(math.atan2(accel_y, accel_z))
        #bank_from_gyro = mpu_bank_angle + gyro_x * dt  # multiply by dt
        #mpu_bank_angle = round(ALPHA * bank_from_gyro + (1 - ALPHA) * accel_bank, 1)
        
        # Gravity removal with pitch and bank
        #g = 9.81
        #pitch_rad = math.radians(mpu_pitch_angle)
        #bank_rad = math.radians(mpu_bank_angle)
        #gravity_z = g * math.cos(pitch_rad) * math.cos(bank_rad)
        #linear_accel_z = accel_z * g - gravity_z
        #vertical_velocity += linear_accel_z * dt
        #vertical_velocity = max(min(vertical_velocity, 30), -30)  # Adjust limits as needed

    except Exception as e:
        print(f"[MPU Read Error] {e}")

# ──────────────── USB Microphone data ────────────────
def whisper_worker():
    global voice_recognition
    buffer = np.empty((0, CHANNELS), dtype=np.float32)

    while True:
        try:
            data = audio_queue.get(timeout=0.1)
            buffer = np.concatenate((buffer, data), axis=0)

            if len(buffer) >= SAMPLE_RATE * CHUNK_DURATION:
                audio_chunk = buffer[:SAMPLE_RATE * CHUNK_DURATION]
                buffer = buffer[SAMPLE_RATE * CHUNK_DURATION:]

                # Flatten and convert for VAD
                mono_audio = audio_chunk.flatten()

                # If speech detected, transcribe with Whisper
                result = model.transcribe(mono_audio, fp16=False, language="en")
                transcription = result['text'].strip().lower()

                # Skip weird characters
                if not re.match(r'^[a-zA-Z0-9\s.,!?\'-]+$', transcription):
                    print("🚫 Ignoring non-English or corrupted transcription.")
                    continue

                print(f"🎤 Transcribed: {transcription}")

                for phrase in SPEECH_PHRASES:
                    if re.search(r'\b' + re.escape(phrase) + r'\b', transcription):
                        voice_recognition = phrase
                        print(f"🚨 Keyword Detected: {phrase}")
                        break

        except queue.Empty:
            continue

# Audio callback for sounddevice
def audio_callback(indata, frames, time_info, status):
    if status:
        print(status)
    audio_queue.put(indata.copy())

# Start Whisper background thread
# ──────────────── Start Whisper Thread ────────────────
def start_whisper_thread():
    global audio_stream  # Keep the stream from being garbage collected

    # Start background thread to run Whisper
    threading.Thread(target=whisper_worker, daemon=True).start()

    # Create and start microphone audio stream
    audio_stream = sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        callback=audio_callback
    )
    audio_stream.start()



# ──────────────── Anomaly Detection ────────────────
def detect_anomaly(vertical_rate = None):
    global voice_recognition

    if abs(mpu_pitch_angle) >= triggers['pitch']:
        return True, "Pitch Angle Exceeded" , mpu_pitch_angle

    if abs(mpu_bank_angle) >= triggers['bank']:
        return True, "Bank Angle Exceeded" , mpu_bank_angle
    
    if bmp_pressure < triggers['cabin_pressure']:
        return True , "Low Cabin Pressure" , bmp_pressure
    
    if bmp_temp > triggers["highTemp"]:
        return True , "High Temperature" , bmp_temp
    
    if mpu_g_force >= triggers['gForceP']:
        return True, "High G-Force" , mpu_g_force

    if mpu_g_force <= triggers['gForceN']:
        return True, "High Negative G-Force" , mpu_g_force
    
    if bmp_temp < triggers["lowTemp"]:
        return True , "Low Temperature" , bmp_temp
    
    #if vertical_velocity > triggers['ascentRate']:
    #    return True, "Rapid Ascent", vertical_velocity

    #if vertical_velocity < triggers['descentRate']:
    #    return True, "Rapid Descent", vertical_velocity

    if voice_recognition in triggers["distress"]:
        word = voice_recognition
        voice_recognition = None
        return True, "Distress Signal" , word

    return False, None, None

# ──────────────── Anomaly data send ────────────────
def send_data(data):
    port = 5050

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((IP_ADDRESS, port))
        s.sendall(data.encode())

# ──────────────── Logged data send ────────────────
def send_log_file():
    port = 5050  # Use a different port for log file transfer

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
    start_whisper_thread()

    prev_time = time.time()
    prev_altitude = 0.0

    print("📡 Starting sensor read loop...")

    while True:
        try:
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            gps_data()

            if gps_lat == 0.0 and gps_long == 0.0:
                print("🛰️ Waiting for GPS fix...")
                #continue

            mpu_data(dt)
            bmp_data()

            # Cap vertical_velocity to avoid drift explosion
            #vertical_velocity = max(min(vertical_velocity, 100), -100)  # m/s
            #vertical_rate = vertical_velocity  # in m/s now, just rename if you want

            if latest_transcription is not None:  
                voice_recognition = latest_transcription
                latest_transcription = None


            if DEBUG_MODE:
                print(f"""
Pitch: {mpu_pitch_angle:.1f}°, 
Bank: {mpu_bank_angle:.1f}°, 
G-force: {mpu_g_force:.2f}g, 
Alt: {gps_alt:.2f} m,  
Pressure: {bmp_pressure:.1f} hPa, 
Temperature: {bmp_temp:.1f} °C,
Distress Signal: {voice_recognition},
Latitude: {gps_lat},
Longitude: {gps_long}""")


            anomaly, reason, amount = detect_anomaly() #vertical_rate)
            if anomaly:
                print(f"⚠️ Anomaly Detected: {reason}")

            data_log(anomaly, reason, amount)
            time.sleep(CHECK_FREQ)

        except KeyboardInterrupt:
            print("Exiting.")
            break
