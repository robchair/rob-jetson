import os
import sounddevice as sd
import soundfile as sf

SR = 16000
DUR = 1.3  # seconds per clip

COMMANDS = ["forward", "back", "left", "right", "stop", "manual"]
ROOT = "component_tests/data"
DEVICE_INDEX = 1                # USB Audio Device index

def record_command(cmd: str, idx: int):
    os.makedirs(os.path.join(ROOT, cmd), exist_ok=True)
    filename = os.path.join(ROOT, cmd, f"{cmd}_{idx:02d}.wav")
    sd.default.device = (DEVICE_INDEX, None)
    print(f"\nRecording {cmd} #{idx} -> {filename}")
    input("Press ENTER, then speak the command...")
    audio = sd.rec(int(DUR * SR), samplerate=SR, channels=1, dtype='float32')
    sd.wait()
    sf.write(filename, audio, SR)
    print("Saved.")

if __name__ == "__main__":
    for cmd in COMMANDS:
        for i in range(1, 6):  # 5 samples per command
            record_command(cmd, i)
