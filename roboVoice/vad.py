# vad.py
import torch
import numpy as np

class VADSegmenter:
    def __init__(
        self,
        sr=16000,
        th=0.7,              # was 0.5 increase for strictness
        min_speech_ms=250,
        max_speech_ms=800,   # shorten max segment length
        min_silence_ms=300,
        max_window_ms=1500,  # analyze at most 1.5s of history
        vad_stride=3,        # only run VAD every 3rd chunk
    ):
        self.sr = sr
        self.th = th
        self.min_speech = int(min_speech_ms / 1000 * self.sr)
        self.max_speech = int(max_speech_ms / 1000 * self.sr)
        self.min_silence = int(min_silence_ms / 1000 * self.sr)
        self.max_window = int(max_window_ms / 1000 * self.sr)
        self.vad_stride = vad_stride
        self.step = 0

        # load silero VAD
        self.model, utils = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
            trust_repo=True,
        )
        (self.get_speech_ts, _, self.read_audio, _, _) = utils

        self.buf = np.zeros(0, dtype=np.float32)

    def push(self, chunk: np.ndarray):
        """
        chunk: (N, 1) float32 from sounddevice
        Returns:
          - None if no complete speech segment yet
          - 1D np.ndarray of mono audio when a segment is ready
        """
        mono = chunk.astype(np.float32).flatten()
        self.buf = np.concatenate([self.buf, mono])

        # hard cap history to last max_window samples
        if len(self.buf) > self.max_window:
            self.buf = self.buf[-self.max_window:]

        # need at least min_speech samples before even try
        if len(self.buf) < self.min_speech:
            return None

        # only run VAD every vad_stride calls to save CPU
        self.step += 1
        if self.step % self.vad_stride != 0:
            return None

        ts = self.get_speech_ts(
            self.buf,
            self.model,
            sampling_rate=self.sr,
            threshold=self.th,
            min_speech_duration_ms=self.min_speech / self.sr * 1000,
            min_silence_duration_ms=self.min_silence / self.sr * 1000,
            return_seconds=False,
        )

        if not ts:
            return None

        last = ts[-1]
        start, end = last["start"], last["end"]

        # Case 1:seen silence after speech -> emit that segment
        if len(self.buf) - end >= self.min_silence:
            seg = self.buf[start:end].copy()
            self.buf = self.buf[end:]  # drop used samples
            return seg

        # Case 2: speech going on too long -> force cut
        if end - start >= self.max_speech:
            seg = self.buf[start:end].copy()
            self.buf = self.buf[end:]
            return seg

        # Otherwise keep accumulating more audio
        return None

