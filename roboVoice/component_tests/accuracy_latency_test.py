import os
import glob
import time
import statistics
import sys

import yaml
import numpy as np
import soundfile as sf  # lightweight WAV reader

# Execute this script from roboVoice room
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(THIS_DIR)  # parent directory of component_tests

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from asr import ASR
from kws_map import CommandMapper

SR = 16000
DATA_ROOT = os.path.join(THIS_DIR, "data")


def load_cfg(path=os.path.join(PROJECT_ROOT, "config.yaml")):
    with open(path) as f:
        return yaml.safe_load(f)


def load_audio_sf(path, target_sr=SR):
    """Load a mono float32 waveform using soundfile. We assume recordings are SR=target_sr."""
    audio, sr = sf.read(path, dtype="float32")  # audio shape: (N,) or (N, channels)
    if audio.ndim == 2:
        audio = audio.mean(axis=1)  # mix down to mono
    if sr != target_sr:
        # Recordings should already be at 16 kHz due to SR
        # If there is a sampling error exception
        raise ValueError(f"Expected sample rate {target_sr}, got {sr} for {path}")
    return audio


def main():
    cfg = load_cfg()
    asr = ASR(
        model_size=cfg["model_size"],
        device=cfg["device"],
        compute_type=cfg["compute_type"],
    )
    mapper = CommandMapper(cfg["commands"], score_cutoff=80)

    y_true = []
    y_pred = []
    latencies = []

    if not os.path.isdir(DATA_ROOT):
        raise RuntimeError(f"DATA_ROOT does not exist: {DATA_ROOT}")

    for cmd_label in sorted(os.listdir(DATA_ROOT)):
        cmd_dir = os.path.join(DATA_ROOT, cmd_label)
        if not os.path.isdir(cmd_dir):
            continue
        for wav_path in glob.glob(os.path.join(cmd_dir, "*.wav")):
            audio = load_audio_sf(wav_path, SR)

            t0 = time.perf_counter()
            text = asr.transcribe(audio)
            cmd = mapper.map_text(text)
            t1 = time.perf_counter()
            latency = t1 - t0

            y_true.append(cmd_label)
            y_pred.append(cmd)
            latencies.append(latency)

            print(f"{wav_path}: text={text!r}  -> cmd={cmd}, latency={latency:.3f}s")

    if not y_true:
        print("No samples found under", DATA_ROOT)
        return

    # Accuracy
    correct = sum(1 for yt, yp in zip(y_true, y_pred) if yt == yp)
    total = len(y_true)
    acc = correct / total

    # Latency stats
    median_lat = statistics.median(latencies)
    p95_lat = float(np.percentile(latencies, 95))

    print("\n=== RESULTS ===")
    print(f"Total samples: {total}")
    print(f"Accuracy: {acc*100:.1f}%  (correct={correct}, total={total})")
    print(f"Median latency: {median_lat:.3f} s")
    print(f"95th percentile latency: {p95_lat:.3f} s")

    # Accuracy and latency boolean checks
    acc_pass = acc >= 0.90
    lat_pass = (median_lat <= 0.7) and (p95_lat <= 0.7)

    print(f"\nPass accuracy spec (>= 90%)? {'YES' if acc_pass else 'NO'}")
    print(f"Pass latency spec (median <= 0.7s, p95 <= 0.7s)? {'YES' if lat_pass else 'NO'}")


if __name__ == "__main__":
    main()
