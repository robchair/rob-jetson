# main_controller.py

import sys
import threading
from readchar import readkey, key

# ---------------------------------------------------------------------
# FIXED IMPORT PATHS
# These force Python to use your actual project folders,
# and prevent fallback to old or installed versions.
# ---------------------------------------------------------------------
sys.path.insert(0, "/home/rob/rob/controller")
sys.path.insert(0, "/home/rob/rob")
sys.path.insert(0, "/home/rob/rob/roboVoice")

# Now imports work correctly
from arbiter import CommandArbiter
from roboVoice.main_realtime_asr import run_realtime
from lidar.lidar_front_safety import run_lidar_loop


# ---------------------------------------------------------------------
# MAIN CONTROLLER
# ---------------------------------------------------------------------
def main():
    arbiter = CommandArbiter(
        serial_port="/dev/ttyACM0",
        baud=9600
    )

    # -------------------------------------------------------
    # VOICE THREAD
    # -------------------------------------------------------
    def voice_thread():
        def on_voice_cmd(cmd: str | None):
            arbiter.on_voice_command(cmd)

        print("[voice] Starting voice recognition thread...")
        run_realtime(on_command=on_voice_cmd)

    t_voice = threading.Thread(target=voice_thread, daemon=True)
    t_voice.start()

    # -------------------------------------------------------
    # LIDAR SAFETY THREAD
    # -------------------------------------------------------
    def lidar_thread():
        def on_lidar_event(cmd):
            arbiter.on_lidar_event(cmd)

        print("[lidar] Starting front safety check...")
        run_lidar_loop(on_lidar_event)

    t_lidar = threading.Thread(target=lidar_thread, daemon=True)
    t_lidar.start()

    # -------------------------------------------------------
    # KEYBOARD CONTROL THREAD
    # -------------------------------------------------------
    def keyboard_thread():
        print("\n[keyboard] Controls ready:")
        print("  ↑ : forward")
        print("  ↓ : backward")
        print("  ← : turnLeft")
        print("  → : turnRight")
        print("SPACE : stop")
        print("  q : quit\n")

        while True:
            k = readkey()
            cmd = None

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
            elif k.lower() == "q":
                print("[keyboard] Quit command received.")
                arbiter.on_keyboard_command("stop")
                break

            if cmd is not None:
                print(f"[keyboard] -> {cmd}")
                arbiter.on_keyboard_command(cmd)

    t_keyboard = threading.Thread(target=keyboard_thread, daemon=True)
    t_keyboard.start()

    # -------------------------------------------------------
    print("[controller] Controller running. Press Ctrl+C to exit.")
    try:
        while True:
            threading.Event().wait(1.0)
    except KeyboardInterrupt:
        print("[controller] Shutting down.")
        arbiter.shutdown()


if __name__ == "__main__":
    main()
