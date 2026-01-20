import serial
import threading
import time

class SerialBridge:
    def __init__(self, port="/dev/ttyACM0", baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()

    def open(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                print(f"[serial] Connected to {self.port} @ {self.baud}")
                break
            except serial.SerialException as e:
                print(f"[serial] Failed to open {self.port}: {e}, retrying in 2s...")
                time.sleep(2)

    def send(self, cmd_str: str):
        """Send a single line command to the Arduino."""
        if not self.ser:
            print("[serial] Not connected, dropping:", cmd_str)
            return
        line = (cmd_str.strip() + "\n").encode("ascii")
        with self.lock:
            self.ser.write(line)
            self.ser.flush()
        # Optional: print for debugging
        print(f"[serial] -> {cmd_str.strip()}")
