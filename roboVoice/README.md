# roboVoice

## 1. For computer/laptop running:

### Set up virtual environment:
```bash
python3 -m venv venv && source venv/bin/activate
pip install faster-whisper torch torchaudio sounddevice numpy scipy rapidfuzz pyyaml
pip install silero-vad==5.1 

#OR 
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

### Set device:

Run `python3 list_devices.py` and find the index of the microphone connected to computer.

In the `config.yml`, set device_index variable to the number of the microphone from running the above line.

```bash
# ...
device_index: 2
# ...
``` 

### Run Inference in Real Time:

```bash
python main_realtime_asr.py
```

## 2. For computer 

### Set device index: 

Run the following to find the device index:

```bash
python -m sounddevice
```

Ex: ` 24 USB Audio Device: - (hw:2,0), ALSA (1 in, 0 out)` is index 24

