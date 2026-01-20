# roboVoice/main_realtime_asr.py
import os
import yaml
from roboVoice.audio_io import AudioStream
from roboVoice.vad import VADSegmenter
from roboVoice.asr import ASR
from roboVoice.kws_map import CommandMapper

# Load absolute path
def load_cfg(path="config.yaml"):
    base = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(base, "config.yaml")

    with open(path, "r") as f:
        return yaml.safe_load(f)

def run_realtime(on_command=None):
    """
    on_command: optional callback(cmd: str | None)
      - If provided, we call it with high-level command (e.g. 'forward', 'left', or None).
      - If not provided, we just print to stdout (current behavior).
    """
    cfg = load_cfg()

    stream = AudioStream(
        samplerate=cfg["sample_rate"],
        block_ms=cfg["block_duration_ms"],
        device_index=cfg.get("device_index"),
    )

    vcfg = cfg["vad"]
    vad = VADSegmenter(
        sr=cfg["sample_rate"],
        th=vcfg["threshold"],
        min_speech_ms=vcfg["min_speech_ms"],
        max_speech_ms=vcfg["max_speech_ms"],
        min_silence_ms=vcfg["min_silence_ms"],
        max_window_ms=vcfg["max_window_ms"],
        vad_stride=vcfg["stride"],
    )

    asr = ASR(model_size=cfg["model_size"],
              device=cfg["device"],
              compute_type=cfg["compute_type"])

    mapper = CommandMapper(cfg["commands"], score_cutoff=80)

    print("Listening for a command: (forward/left/right/back/stop/manual)")
    stream.start()
    try:
        while True:
            chunk = stream.read()
            seg = vad.push(chunk)
            if seg is None:
                continue

            MAX_SEG_SEC = 1.0
            max_len = int(MAX_SEG_SEC * cfg["sample_rate"])
            if len(seg) > max_len:
                seg = seg[-max_len:]

            text = asr.transcribe(seg)
            cmd = mapper.map_text(text)

            if on_command is not None:
                on_command(cmd)
            else:
                print(f"[transcript] {text!r}  ->  [cmd] {cmd}")
    except KeyboardInterrupt:
        stream.stop()

if __name__ == "__main__":
    # Standalone mode (what you’ve been using)
    run_realtime()

