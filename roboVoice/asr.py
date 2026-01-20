from faster_whisper import WhisperModel

class ASR:
    def __init__(self, model_size="tiny.en", device="cpu", compute_type="int8"):
        self.model = WhisperModel(model_size, device=device, compute_type=compute_type)

    def transcribe(self, audio_16k_mono: "np.ndarray[float32]") -> str:
        segments, info = self.model.transcribe(audio_16k_mono, language="en", beam_size=1, vad_filter=False)
        text = "".join(seg.text for seg in segments).strip()
        return text.lower()
