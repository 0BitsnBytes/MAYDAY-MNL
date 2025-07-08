import sounddevice as sd
import numpy as np
import whisper
import queue
import threading
import time

# Parameters
SAMPLE_RATE = 16000
CHUNK_DURATION = 3  # seconds
CHANNELS = 1

# Thread-safe queue to hold recorded audio
audio_queue = queue.Queue()

# Load Whisper model (small or tiny for faster inference on Pi)
model = whisper.load_model("tiny")  # or "tiny"

def audio_callback(indata, frames, time_info, status):
    """Called by sounddevice for each audio block."""
    if status:
        print(status)
    audio_queue.put(indata.copy())

def record_audio():
    """Continuously record audio chunks and put them in queue."""
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, callback=audio_callback):
        print("Recording... Press Ctrl+C to stop.")
        while True:
            time.sleep(1)

def transcribe_worker():
    """Take audio chunks from queue, transcribe, print text."""
    buffer = np.empty((0, CHANNELS), dtype=np.float32)

    while True:
        # Get new audio data from queue
        try:
            data = audio_queue.get(timeout=1)
            buffer = np.concatenate((buffer, data), axis=0)

            # If buffer is longer than chunk size, process it
            if len(buffer) >= SAMPLE_RATE * CHUNK_DURATION:
                # Whisper expects float32 numpy array, 16kHz
                audio_chunk = buffer[:SAMPLE_RATE * CHUNK_DURATION].flatten()
                buffer = buffer[SAMPLE_RATE * CHUNK_DURATION:]

                print("Transcribing chunk...")
                result = model.transcribe(audio_chunk, fp16=False, language="en")
                print("Detected Text:", result['text'].strip())
        except queue.Empty:
            pass

if __name__ == "__main__":
    try:
        # Start recording thread
        threading.Thread(target=record_audio, daemon=True).start()
        # Start transcribing loop in main thread
        transcribe_worker()
    except KeyboardInterrupt:
        print("\nStopped.")
