# audio_io.py
import sounddevice as sd
import numpy as np
from queue import Queue
from scipy.signal import resample_poly

TARGET_SR = 16000          # what VAD/ASR expect
USB_MIC_INDEX = 24         # USB Audio Device index

class AudioStream:
    def __init__(self, samplerate=TARGET_SR, block_ms=30, device_index=None):
        # samplerate is the TARGET rate for downstream (VAD/ASR)
        self.target_sr = samplerate
        self.block_tgt = int(self.target_sr * block_ms / 1000)
        self.q = Queue()

        # force device_index to be an int; default to our USB mic
        if device_index is None:
            self.device_index = USB_MIC_INDEX
        else:
            # if it comes in as a string force to int
            self.device_index = int(device_index)

        self.device_sr = None
        self.channels_opened = 1
        self.stream = None
        self._residual = np.zeros(0, dtype=np.float32)

    def _mixdown(self, x: np.ndarray) -> np.ndarray:
        """(frames, channels) -> mono float32"""
        if x.ndim == 1:
            return x.astype(np.float32)
        return x.mean(axis=1).astype(np.float32)

    def _resample_to_target(self, x: np.ndarray) -> np.ndarray:
        """Resample from device_sr to target_sr using polyphase."""
        if self.device_sr == self.target_sr:
            return x
        return resample_poly(x, up=self.target_sr, down=int(self.device_sr)).astype(np.float32)

    def _callback(self, indata, frames, time, status):
        if status:
            print(status)
        mono = self._mixdown(indata.copy())
        mono_16k = self._resample_to_target(mono)

        if len(mono_16k) == 0:
            return

        buf = np.concatenate([self._residual, mono_16k])
        n = (len(buf) // self.block_tgt) * self.block_tgt
        if n > 0:
            out_blocks = buf[:n].reshape(-1, self.block_tgt)
            for b in out_blocks:
                # (block_tgt,) -> (block_tgt, 1) for VAD
                self.q.put(b.reshape(-1, 1).astype(np.float32))
            self._residual = buf[n:]
        else:
            self._residual = buf

    def start(self):
        in_id = int(self.device_index)
        print(f"[audio] Using input device index {in_id}")

        dev_info = sd.query_devices(in_id, kind='input')
        self.device_sr = float(dev_info['default_samplerate']) or 44100.0

        try:
            self.channels_opened = 1
            self.stream = sd.InputStream(
                device=in_id,
                channels=1,
                samplerate=self.device_sr,
                blocksize=0,
                dtype='float32',
                callback=self._callback
            )
            self.stream.start()
        except Exception as e_mono:
            print(f"[audio] Mono open failed ({e_mono}), retrying with stereo…")
            self.channels_opened = 2
            self.stream = sd.InputStream(
                device=in_id,
                channels=2,
                samplerate=self.device_sr,
                blocksize=0,
                dtype='float32',
                callback=self._callback
            )
            self.stream.start()

        print(f"[audio] Opened device {in_id} at {self.device_sr:.0f} Hz "
              f"({self.channels_opened} ch), resampling -> {self.target_sr} Hz")

    def read(self):
        return self.q.get()

    def stop(self):
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None

