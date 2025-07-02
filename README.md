# ✈️ Project MAYDAY

**Project MAYDAY** is a real-time emergency detection and flight data transmission system designed for aircraft. Inspired by real-life incidents like Air France 447 and MH370, this project aims to ensure that even if a black box is lost, critical data isn’t.

Using a Raspberry Pi 5, a combination of onboard sensors, and offline voice recognition, MAYDAY detects signs of an in-flight emergency — such as extreme G-forces, unusual flight angles, or the pilot calling “Mayday” — and instantly transmits key flight data to the cloud.

---

## 📌 Features

- ✅ Real-time G-force, altitude, and orientation monitoring
- 🎙️ Offline voice trigger detection with `vosk` (“Mayday” call)
- 📤 Cloud-ready: Transmits critical sensor data when anomalies are detected
- 💾 Local logging for offline fallback
- 🧠 Runs fully offline until upload is needed

---

## 🔧 Hardware Used

| Component        | Model/Type                  | Purpose                                     |
|------------------|-----------------------------|--------------------------------------       |
| Microcontroller  | Raspberry Pi 5              | Core computer                               |
| IMU Sensor       | MPU6050                     | Accelerometer + Gyroscope                   |
| Barometer        | BMP280                      | Pressure, Altitude                          |
| USB Microphone   | Any USB mic (e.g. CM108)    | Voice recognition trigger                   |
| GPS Module       | u-blox Neo-6M               | Real-time latitude, longitude, ground speed |

---

## 🧪 How It Works

1. Continuously reads sensor data (altitude, acceleration, orientation)
2. Calculates:
   - G-forces
   - Pitch and bank angles
   - Descent/ascent rate
3. Listens for the phrase “Mayday” using offline voice recognition (Vosk)
4. If an anomaly is detected:
   - Uploads recent flight data to a cloud endpoint (or local server)
   - Logs the event for review

