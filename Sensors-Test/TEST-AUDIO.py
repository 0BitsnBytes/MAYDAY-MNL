from vosk import Model, KaldiRecognizer
import pyaudio
import json
import time

# Words to detect
TRIGGER_WORDS = ["mayday", "help", "emergency", "rescue"]

def send_signal(word):
    print(f"*** TRIGGER ACTIVATED: {word} ***")
    # Add GPIO or any signal here
    # Example:
    # import RPi.GPIO as GPIO
    # GPIO.output(18, GPIO.HIGH)
    # time.sleep(1)
    # GPIO.output(18, GPIO.LOW)

# Load Vosk model
model = Model("vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 48000)

# Audio input
p = pyaudio.PyAudio()
stream = p.open(
    rate=48000,
    channels=1,
    format=pyaudio.paInt16,
    input_device_index=1,
    input=True,
    frames_per_buffer=8000
)
stream.start_stream()

print("Listening for:", TRIGGER_WORDS)
while True:
    data = stream.read(4000, exception_on_overflow=False)

    if recognizer.AcceptWaveform(data):
        result = json.loads(recognizer.Result())
        text = result.get("text", "").lower()

        if text:
            print("Heard:", text)

            # Check for any trigger word
            for word in TRIGGER_WORDS:
                if word in text:
                    send_signal(word)
                    print("Restarting listening...\n")
                    time.sleep(0.5)
                    break
