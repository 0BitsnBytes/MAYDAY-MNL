import serial
import pynmea2

def check_gps():
    gps_port = '/dev/serial0'  # adjust if needed
    baudrate = 9600

    try:
        ser = serial.Serial(gps_port, baudrate=baudrate, timeout=1)
        print("? Reading GPS data. Press Ctrl+C to stop.\n")

        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line.startswith('$'):
                continue

            try:
                msg = pynmea2.parse(line)

                if isinstance(msg, pynmea2.GGA):
                    fix_quality = msg.gps_qual
                    satellites = msg.num_sats
                    print(f"? GGA ? Fix quality: {fix_quality}, Satellites: {satellites}")

                    if fix_quality > 0:
                        print(f"? Location: {msg.latitude:.5f} {msg.lat_dir}, {msg.longitude:.5f} {msg.lon_dir}")
                        print(f"   Altitude: {msg.altitude} {msg.altitude_units}")
                    else:
                        print("? No fix yet?")

                elif isinstance(msg, pynmea2.RMC):
                    status = msg.status
                    if status == 'A':
                        print(f"? RMC ? Active Fix: Lat {msg.latitude:.5f}, Lon {msg.longitude:.5f}")
                    else:
                        print("? RMC ? No valid fix yet")

            except pynmea2.ParseError:
                continue

    except KeyboardInterrupt:
        print("\n? Stopped.")
    except Exception as e:
        print(f"? Error: {e}")

if __name__ == "__main__":
    check_gps()
