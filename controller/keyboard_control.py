# keyboard_control.py (FIXED)

import time
from typing import Optional
from readchar import readkey, key
from arbiter import CommandArbiter #arbiter sends to the arduino

def main():
    arbiter = CommandArbiter(
        serial_port="/dev/ttyACM0",
        baud=9600,
    )

    print("Keyboard control:")
    print("  ↑ : forward")
    print("  ↓ : backward")
    print("  ← : turnLeft")
    print("  → : turnRight")
    print("SPACE : stop")
    print("  q : quit\n")

    try:
        while True:
            k = readkey()
            cmd: Optional[str] = None

            if k == key.UP:
                cmd = "forward"
            elif k == key.DOWN:
                cmd = "backward"
            elif k == key.LEFT:
                cmd = "turnLeft"
            elif k == key.RIGHT:
                cmd = "turnRight"
            elif k == " ":
                cmd = "stop"
            elif k in ("q","Q"):
                arbiter.on_keyboard_command("stop")
                break
#this is the code that takes the command and sends to arbiter based on keyboard entry
            if cmd is not None:
                print(f"[keyboard] -> {cmd}")
                arbiter.on_keyboard_command(cmd)

    except KeyboardInterrupt:
        arbiter.on_keyboard_command("stop")
        time.sleep(0.2)

if __name__ == "__main__":
    main()

