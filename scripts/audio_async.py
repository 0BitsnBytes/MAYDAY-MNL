import asyncio
from vosk import Model, KaldiRecognizer
from collections import deque
import pyaudio
import json

class AsyncAudio:
    def __init__(self):
        # Load model
        self.model = Model("vosk-model-small-en-us-0.15")
        self.recognizer = KaldiRecognizer(self.model, 48000)

        # list of words
        self.WORD_HISTORY = deque(maxlen=10)

        # PyAudio setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            rate=48000,
            channels=1,
            format=pyaudio.paInt16,
            input_device_index=1,
            input=True,
            frames_per_buffer=8000,
        )
        self.stream.start_stream()

        self.latest = None

    async def read_loop(self):
        """Run indefinitely; real async behavior because blocking code is threaded."""
        while True:
            # Offload PyAudio read (blocking) to a thread
            data = await asyncio.to_thread(
                self.stream.read,
                4000,
                exception_on_overflow=False
            )

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").lower()

                if text:
                    self.WORD_HISTORY.append(text)
            
            await asyncio.sleep(0)  # yield to event loop

    def get(self):
        return self.WORD_HISTORY


# Example usage:
# async def main():
#     mic = AsyncAudio()
#     await mic.read_loop()
# asyncio.run(main())
