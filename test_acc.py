from pylsl import StreamInlet, resolve_streams
import time

print("Looking for ACC stream...")
streams = resolve_streams()
acc_stream = [s for s in streams if s.type() == 'ACC']

if not acc_stream:
    print("No ACC stream found!")
    exit()

inlet = StreamInlet(acc_stream[0])
print("Connected! Tilt your head around...\n")

while True:
    sample, ts = inlet.pull_sample()
    x, y, z = sample
    print(f"X: {x:+7.2f}  Y: {y:+7.2f}  Z: {z:+7.2f}")
    time.sleep(0.1)
