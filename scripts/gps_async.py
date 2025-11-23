import asyncio
import serial
import pynmea2

class AsyncGPS:
    def __init__(self, port="/dev/serial0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.latest = None   # will store latest parsed GPS dict

    def parse_line(self, line):
        """Parse a single NMEA line into a dict."""
        try:
            msg = pynmea2.parse(line)
        except:
            return None

        data = {}

        # --- GGA: fix status, sats, altitude ---
        if isinstance(msg, pynmea2.types.talker.GGA):
            data["lat"] = msg.latitude
            data["lon"] = msg.longitude
            data["sats"] = int(msg.num_sats)
            data["alt"] = float(msg.altitude)
            data["fix"] = int(msg.gps_qual)
            data["timestamp"] = str(msg.timestamp)
            return data

        # --- RMC: lat, lon, speed, timestamp ---
        if isinstance(msg, pynmea2.types.talker.RMC):
            data["lat"] = msg.latitude
            data["lon"] = msg.longitude
            data["speed"] = float(msg.spd_over_grnd)  # knots
            data["fix"] = 1 if msg.status == "A" else 0
            data["timestamp"] = f"{msg.datestamp} {msg.timestamp}"
            return data

        return None

    async def read_loop(self):
        """Continuously read GPS in background."""
        while True:
            try:
                line = self.ser.readline().decode("ascii", "ignore").strip()
                if line.startswith("$"):
                    parsed = self.parse_line(line)
                    if parsed:
                        self.latest = parsed
            except:
                pass

            # yield control to event loop
            await asyncio.sleep(0.01)

    def get(self):
        """Return latest GPS fix OR None."""
        return self.latest
