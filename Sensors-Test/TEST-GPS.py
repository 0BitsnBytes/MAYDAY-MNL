#!/usr/bin/env python3
import serial
import pynmea2

def main():
    print("Opening GPS on /dev/serial0 @ 9600 baud...")

    gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

    while True:
        try:
            line = gps.readline().decode("ascii", errors="ignore").strip()

            if not line.startswith("$"):
                continue

            msg = pynmea2.parse(line)

            if isinstance(msg, pynmea2.types.talker.GGA):
                print("=== GPS GGA Fix ===")
                print(f"Latitude:  {msg.latitude}")
                print(f"Longitude: {msg.longitude}")
                print(f"Satellites: {msg.num_sats}")
                print(f"Altitude:  {msg.altitude} m")
                print("---------------------")

            elif isinstance(msg, pynmea2.types.talker.RMC):
                print("=== GPS RMC ===")
                print(f"Lat: {msg.latitude}, Lon: {msg.longitude}")
                print(f"Speed: {msg.spd_over_grnd} knots")
                print(f"Status: {'FIX' if msg.status=='A' else 'NO FIX'}")
                print("---------------------")

        except pynmea2.ParseError:
            pass  # ignore junk
        except KeyboardInterrupt:
            print("Exiting.")
            break

if __name__ == "__main__":
    main()
