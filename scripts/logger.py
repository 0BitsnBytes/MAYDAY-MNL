import os
import csv
from datetime import datetime

LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

def create_log():
    fname = os.path.join(LOG_DIR, f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    fields = [
        "timestamp",
        "lat","lon","alt","sats","fix","speed_kmh",
        "accel_x","accel_y","accel_z",
        "gyro_x","gyro_y","gyro_z",
        "temp_c","pitch_deg","bank_deg","g_force",
        "bmp_temp_c","bmp_pressure_hpa","bmp_alt_m",
        "anomaly","anom_reason","anom_val"
    ]
    with open(fname, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
    return fname, fields

def append_row(fname, fields, data_dict):
    # ensure all fields present
    row = {k: data_dict.get(k, "") for k in fields}
    with open(fname, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writerow(row)
