from pylsl import StreamInlet, resolve_streams
import numpy as np
from collections import deque
import time

print("Looking for EEG stream...")
streams = resolve_streams()
eeg_stream = [s for s in streams if s.type() == 'EEG']

if not eeg_stream:
    print("No EEG stream found!")
    exit()

inlet = StreamInlet(eeg_stream[0])
buffer = deque(maxlen=256)

print("Waiting for data...\n")

count = 0
start = time.time()

while time.time() - start < 30:
    sample, ts = inlet.pull_sample(timeout=1.0)
    if sample:
        buffer.append(sample)
        count += 1

        if count % 50 == 0:
            print(f"Received {count} samples, buffer size: {len(buffer)}")

        if len(buffer) >= 256:
            data = np.array(list(buffer))
            channel = data[:, 1]  # AF7

            fft_vals = np.abs(np.fft.rfft(channel))
            freqs = np.fft.rfftfreq(len(channel), d=1.0 / 256.0)

            alpha = np.mean(fft_vals[(freqs >= 8) & (freqs <= 12)])
            beta = np.mean(fft_vals[(freqs >= 13) & (freqs <= 30)])

            ratio = beta / alpha if alpha > 0 else 0
            attention = min(ratio / 3.0, 1.0)

            elapsed = time.time() - start
            phase = "RELAX" if elapsed < 15 else "FOCUS"
            print(f"[{phase}] Alpha:{alpha:.1f} Beta:{beta:.1f} Ratio:{ratio:.2f} Attention:{attention:.2f}")

    else:
        print("No sample received...")

print(f"\nTotal samples received: {count}")
