# controller/arbiter.py
import threading
import time
import serial  # pip install pyserial
from typing import Optional

class SerialCommandSender:
    """
    Thin wrapper around a serial.Serial port that sends newline-terminated
    ASCII command strings, e.g. "forward\n".
    """

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 9600):
        self._lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        # Give Arduino time to reset after opening the port
        time.sleep(2.0)
        print(f"[serial] Opened {port} @ {baud} baud")

    def send(self, cmd: str | None):
        """Send a single command line over serial."""
        if not cmd:
            return
        line = (cmd.strip() + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(line)
            self.ser.flush()
        print(f"[serial] -> {cmd!r}")


class CommandArbiter:
    """
    Central arbiter:
      - Receives commands from roboVoice (voice),
        later from LiDAR and other modules.
      - Enforces that LiDAR 'stop' overrides everything.
      - Maps canonical commands -> Arduino serial strings.
    """

    def __init__(self, serial_port: str = "/dev/ttyACM0", baud: int = 9600):
        self.sender = SerialCommandSender(serial_port, baud)
        self._lock = threading.Lock()

        # Safety state
        self._lidar_stop_active = False
        self._last_sent: str | None = None

    # -------- mapping --------

    @staticmethod
    def _map_voice_to_serial(cmd: str | None) -> str | None:
        """
        Map canonical voice commands to the exact strings
        the Arduino expects in loop():
            forward  -> "forward"
            back     -> "backward"
            left     -> "turnLeft"
            right    -> "turnRight"
            stop     -> "stop"
            manual   -> "quit"
        """
        if cmd is None:
            return None

        mapping = {
            "forward": "forward",
            "back": "backward",
            "left": "turnLeft",
            "right": "turnRight",
            "stop": "stop",
            "manual": "quit",
        }
        return mapping.get(cmd)

    def _translate_cmd(self, src_cmd: Optional[str]) -> Optional[str]:
        """
        Map high-level commands from voice/keyboard into the exact
        string the Arduino sketch expects over Serial.
        """
        if src_cmd is None:
            return None

        src = src_cmd.strip().lower()

        mapping = {
            # forward
            "forward": "forward",

            # back
            "back": "backward",
            "backward": "backward",

            # left
            "left": "turnLeft",
            "turnleft": "turnLeft",

            # right
            "right": "turnRight",
            "turnright": "turnRight",

            # stop / quit
            "stop": "stop",
            "quit": "quit",
        }

        serial_cmd = mapping.get(src)
        if serial_cmd is None:
            print(f"[arbiter] Unknown voice cmd {src_cmd!r}, ignoring")
        return serial_cmd

    def _send_if_changed(self, serial_cmd: str | None):
        """Avoid spamming identical commands repeatedly."""
        if not serial_cmd:
            return
        #if serial_cmd == self._last_sent:
            #return
        self._last_sent = serial_cmd
        self.sender.send(serial_cmd)

    # -------- event handlers --------

    def on_voice_command(self, cmd: str | None):
        """
        Called from roboVoice thread with canonical commands:
        'forward', 'back', 'left', 'right', 'stop', 'manual', or None.
        """
        if cmd is None:
            return

        with self._lock:
            # If LiDAR has an active STOP, ignore everything except 'stop'.
            if self._lidar_stop_active and cmd != "stop":
                print(f"[arbiter] Ignoring voice cmd {cmd!r} due to LiDAR STOP")
                return

            #serial_cmd = self._map_voice_to_serial(cmd)
            serial_cmd = self._translate_cmd(cmd)
            if serial_cmd is None:
                print(f"[arbiter] Unknown voice cmd {cmd!r}, ignoring")
                return

            self._send_if_changed(serial_cmd)
    
    def on_keyboard_command(self, cmd):
        """
        Called by keyboard_control.py with the same canonical commands as voice.
        For now we treat keyboard and voice identically.
        """
        self.on_voice_command(cmd)

    def on_lidar_event(self, event: str):
        """
        Called by LIDAR module:
          - "stop"    -> assert hard stop, override all motion
          - "forward" -> front is clear again, allow motion
          - "clear"   -> alias for "forward"
        """
        with self._lock:
            if event == "stop":
                print("[arbiter] LiDAR STOP asserted")
                self._lidar_stop_active = True
                # Always send a hard stop when LiDAR hits
                self._send_if_changed("stop")

            elif event in ("forward", "clear"):
                if self._lidar_stop_active:
                    print("[arbiter] LiDAR STOP cleared")
                self._lidar_stop_active = False
                # Force next motion command to be sent, even if same as last
                self._last_sent = None


    def shutdown(self):
        """Optional: call this on clean exit."""
        with self._lock:
            try:
                self.sender.send("stop")
            except Exception:
                pass

