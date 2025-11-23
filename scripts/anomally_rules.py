THRESHOLDS = {
    "PITCH_DEG": 25,           # degrees
    "BANK_DEG": 70,            # degrees
    "G_FORCE_HIGH": 1.5,       # G
    "G_FORCE_LOW": 0.5,        # G
    "ALT_DROP_M": 1.5,          # meters drop threshold within ALT_WINDOW_S
    "TEMP_CHANGE" : 1.0,       # degrees
    "PRESSURE_CHANGE": 0.08,      # satellites minimum
    "LOW_CABIN_PRESSURE": 912.45, # m/s drop threshold within SPEED_WINDOW_S
    "SPEED_WINDOW_S": 2.0,     # seconds for speed change
    "PRESSURE_JUMP_HPA": 2.0,  # hPa change in PRESSURE_WINDOW_S
    "PRESSURE_WINDOW_S": 1.0,
    "TRIGGER_WORDS" : ["mayday", "help", "emergency", "rescue"]
}
