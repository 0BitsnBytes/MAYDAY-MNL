import asyncio
import time
from collections import deque
from datetime import datetime
import math
from scripts.mpu_async import AsyncMPU
from scripts.bmp_async import AsyncBMP
from scripts.gps_async import AsyncGPS
from scripts.anomaly_rules import THRESHOLDS
from scripts.logger import create_log, append_row
from scripts.audio_async import AsyncAudio
import socket

# -----------------------
# CONFIG
# -----------------------
ALPHA = 0.4 
DEBUG_MODE = False
LOG_RATE = 2  # seconds between writes (20 Hz)
DEQUE_LEN_ALT = 5 # seconds
DEQUE_LEN_PRESS = 5
DEQUE_LEN_TEMP = 30 
IP_ADDRESS = "192.168.3.102"
# -----------------------

global_pitch_ang = 0
global_bank_ang  = 0
global_temp = 0.0
global_pressure = 0.0

# Create log file
log_file, LOG_FIELDS = create_log()

# History buffers for event-detection (timestamped tuples)
ALT_HISTORY = deque(maxlen=DEQUE_LEN_ALT)    # elements: (timestamp, altitude_m)
TEMP_HISTORY = deque(maxlen=DEQUE_LEN_TEMP)  # elements: (timestamp, speed_m_s)
PRESS_HISTORY = deque(maxlen=DEQUE_LEN_PRESS)  # elements: (timestamp, pressure_hpa)

# Helper: trim histories to window seconds
def trim_history(history, window_s):
    now = time.time()
    while history and (now - history[0][0]) > window_s:
        history.popleft()

# Anomaly detection function using THRESHOLDS
def detect_anomalies(audio, mpu, bmp):
    global global_pitch_ang, global_bank_ang, global_temp, global_pressure
    g_force = rate_alt_change = temp_change = pressure_change = 0.0
    anomaly = False
    value = []
    reason = []

    """
    Returns (anomaly_bool, reason_str, value)
    Keeps logic and reasons similar to your old script.
    """
    #print(audio)

    if mpu:
        # ???????????????? Get Pitch and Bank (Roll) Angle ????????????????
        gyro_data = mpu["gyro"]
        accel = mpu["accel"]

        # get Pitch
        global_pitch_ang = gyro_data['y']
        """
        gyro_y = gyro_data['y']
        accel_pitch = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y'] ** 2 + accel['z'] ** 2)))
        pitch_from_gyro = global_pitch_ang + gyro_y 
        global_pitch_ang = round(ALPHA * pitch_from_gyro + (1 - ALPHA) * accel_pitch, 1) 
        """
        
        #print(f"pitch angle : {global_pitch_ang}")
        
        # get Bank
        global_bank_ang = gyro_data['z']
        """
        gyro_x = gyro_data['x']
        accel_bank = math.degrees(math.atan2(accel['y'], accel['z']))
        bank_from_gyro = global_bank_ang + gyro_x
        global_bank_ang = round(ALPHA * bank_from_gyro + (1 - ALPHA) * accel_bank, 1)
        #print(f"bank angle : {global_bank_ang}")
        """

        
        # get g_force
        g_force = round(math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2) / 9.81, 2)
        #print(f"G force : {g_force}")

    #print("bmp data : ",bmp)
    if bmp and bmp.get("altitude") is not None:
        # append history and trim
        ALT_HISTORY.append(bmp["altitude"])
        rate_alt_change = (ALT_HISTORY[-1] - ALT_HISTORY[0])/LOG_RATE*DEQUE_LEN_ALT
    
    if bmp and bmp.get("temprature") is not None:
        # append history and trim
        TEMP_HISTORY.append(bmp["temprature"])
        temp_change = (ALT_HISTORY[-1] - ALT_HISTORY[0])
    
    if bmp and bmp.get("pressure") is not None:
        # append history and trim
        PRESS_HISTORY.append(bmp["pressure"])
        pressure_change = (PRESS_HISTORY[-1] - PRESS_HISTORY[0])

    ## return anomaly 
    if audio:
        for word in THRESHOLDS["TRIGGER_WORDS"]:
            for i in range(len(audio)):
                if word in audio[i]:
                    audio[i] = 'PROCESSED'
                    anomaly = True
                    value.append(word)
                    reason.append("TRIGGER_WORD")
                    #return True, "TRIGGER_WORD", word

    #print(PRESS_HISTORY[-1])
    if PRESS_HISTORY[-1] < THRESHOLDS["LOW_CABIN_PRESSURE"]:
        anomaly = True
        value.append(PRESS_HISTORY[-1])
        reason.append("LOW_CABIN_PRESSURE")
        #return True, "LOW_CABIN_PRESSURE", PRESS_HISTORY[-1]

    #print(temp_change)
    if abs(temp_change) > THRESHOLDS["TEMP_CHANGE"]:
        anomaly = True
        value.append(round(temp_change,2))
        reason.append("TEMP_CHANGE")
        #return True, "TEMP_CHANGE", round(temp_change,2)
    
    #print(pressure_change)
    if abs(pressure_change) > THRESHOLDS["PRESSURE_CHANGE"]:
        anomaly = True
        value.append(round(pressure_change,2))
        reason.append("PRESSURE_CHANGE")
        #return True, "PRESSURE_CHANGE", round(pressure_change,2)
    
    #print(rate_alt_change)
    if abs(rate_alt_change) > THRESHOLDS["ALT_DROP_M"]:
        anomaly = True
        value.append(round(rate_alt_change,2))
        reason.append("ALTITUDE_CHANGE")
        #return True, "ALTITUDE_CHANGE", round(rate_alt_change,2) 
      
    if g_force <= THRESHOLDS["G_FORCE_LOW"]:
        anomaly = True
        value.append(g_force)
        reason.append("LOW_G_FORCE")
        #return True, "LOW_G_FORCE", g_force  
    
    if g_force >= THRESHOLDS["G_FORCE_HIGH"]:
        anomaly = True
        value.append(g_force)
        reason.append("HIGH_G_FORCE")
        #return True, "HIGH_G_FORCE", g_force
        
    if abs(global_bank_ang) >= THRESHOLDS["BANK_DEG"]:
        anomaly = True
        value.append(global_bank_ang)
        reason.append("BANK_EXCEEDED")
        #return True, "BANK_EXCEEDED", global_bank_ang

    if abs(global_pitch_ang) >= THRESHOLDS["PITCH_DEG"]:
        anomaly = True
        value.append(global_pitch_ang)
        reason.append("PITCH_EXCEEDED")
        #return True, "PITCH_EXCEEDED", global_pitch_ang

    return anomaly, reason, value

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

            with open(log_file, 'r') as f:
                contents = f.read()

            s.sendall(contents.encode())
            print("✅ Log file sent successfully.")

    except Exception as e:
        print(f"[Log File Send Error] {e}")

# -----------------------
# Main async orchestrator
# -----------------------
async def main():
    print("📡 MAYDAY initializing sensors...")
    gps = AsyncGPS()       # /dev/serial0 @ 9600
    mpu = AsyncMPU()       # MPU6050
    bmp = AsyncBMP()       # BMP280
    audio = AsyncAudio()   # Audio

    # start background loops
    asyncio.create_task(gps.read_loop())
    asyncio.create_task(mpu.read_loop())
    asyncio.create_task(bmp.read_loop())
    asyncio.create_task(audio.read_loop())


    print("Sensors started. Logging to:", log_file)

    try:
        while True:
            tstamp = datetime.now().isoformat()

            gps_data = gps.get()
            mpu_data = mpu.get()
            bmp_data = bmp.get()
            audio_data = audio.get()

            #print(audio_data)

            # Prepare row dict for logger
            row = {
                "timestamp": tstamp,
                "lat":  gps_data.get("lat")   if gps_data else "",
                "lon":  gps_data.get("lon")   if gps_data else "",
                "alt":  gps_data.get("alt")   if gps_data else "",
                "sats": gps_data.get("sats")  if gps_data else "",
                "fix":  gps_data.get("fix")   if gps_data else "",
                "speed":gps_data.get("speed") if gps_data else "",

                "accel_x": mpu_data["accel"]["x"] if mpu_data else "",
                "accel_y": mpu_data["accel"]["y"] if mpu_data else "",
                "accel_z": mpu_data["accel"]["z"] if mpu_data else "",
                "gyro_x":  mpu_data["gyro"]["x"]  if mpu_data else "",
                "gyro_y":  mpu_data["gyro"]["y"]  if mpu_data else "",
                "gyro_z":  mpu_data["gyro"]["z"]  if mpu_data else "",
                "temp_c":  mpu_data["temp"] if mpu_data else "",
                #"pitch_deg": mpu_data["pitch"] if mpu_data else "",
                #"bank_deg":  mpu_data["bank"]  if mpu_data else "",
                #"g_force":   mpu_data["g_force"] if mpu_data else "",

                "bmp_temp_c": bmp_data["temperature"] if bmp_data else "",
                "bmp_pressure_hpa": bmp_data["pressure"] if bmp_data else "",
                "bmp_alt_m": bmp_data["altitude"] if bmp_data else ""
            }

            # Detect anomalies using the function that references THRESHOLDS
            #print(f"bmp_data = {bmp_data}, gps = {gps_data}, mpu_data = {mpu_data}")
            anomaly = reason = val = None
            if bmp_data and mpu_data:
                anomaly, reason, val = detect_anomalies(audio_data, mpu_data, bmp_data)
            row["anomaly"] = anomaly
            row["anom_reason"] = reason
            row["anom_val"] = val

            # voice 

            # Append to CSV
            append_row(log_file, LOG_FIELDS, row)

            # Debug printing
            if DEBUG_MODE:
                print(row)
            if anomaly:
                print("⚠️ ANOMALY:", reason, val)
                send_log_file()

            # time.sleep(1)
            await asyncio.sleep(LOG_RATE)

    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt — shutting down.")
    finally:
        print("Stopping. Last log:", log_file)

if __name__ == "__main__":
    asyncio.run(main()) 
