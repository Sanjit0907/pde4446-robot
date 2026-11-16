#!/usr/bin/env python3
"""Keyboard teleop for firmware/teleop_test (Arduino Uno on USB serial).

    pip install pyserial
    python tools/teleop.py COM5            # Windows
    python tools/teleop.py /dev/ttyACM0    # Linux     (macOS: /dev/cu.usbmodemXXXX)

Keys (latched: press once, the robot keeps going until you change/stop):
    w s a d     forward / backward / pivot left / pivot right
    j k         left wheel only / right wheel only (diagnostics)
    space or x  stop            + -  speed        r  reset counts      q  quit
The script re-sends the current command 10x per second as a heartbeat. The Arduino stops by itself
if the heartbeat stops (dead-man), so closing the window or unplugging the cable stops the robot.
Telemetry from the Arduino is shown on one live status line and logged to logs/teleop_<time>.log.
"""
import argparse
import datetime
import os
import sys
import threading
import time

try:
    import serial  # type: ignore
except ImportError:
    sys.exit("pyserial is missing: pip install pyserial")

HEARTBEAT_S = 0.1
MOVE_KEYS = set("wsadjk")


class Keyboard:
    """Non-blocking single-key reader for Windows and POSIX terminals."""

    def __enter__(self):
        if os.name == "nt":
            import msvcrt
            self._ms = msvcrt
        else:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if os.name != "nt":
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def get(self, timeout):
        if os.name == "nt":
            end = time.time() + timeout
            while time.time() < end:
                if self._ms.kbhit():
                    return self._ms.getwch()
                time.sleep(0.01)
            return None
        import select
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else None


class Reader(threading.Thread):
    def __init__(self, ser, logfile):
        super().__init__(daemon=True)
        self.ser, self.logfile = ser, logfile
        self.latest = ""
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException:
                break
            if not line:
                continue
            self.logfile.write(line + "\n")
            if line.startswith("E,"):
                self.latest = line
            elif line.startswith("#"):
                print("\r" + " " * 100 + "\r" + line)


def pretty(line):
    p = line.split(",")
    if len(p) < 12:
        return line
    return (f"L {p[2]:>7} ticks {p[4]:>6} mm {p[6]:>5} mm/s | R {p[3]:>7} ticks {p[5]:>6} mm {p[7]:>5} mm/s"
            f" | pwm {p[8]:>4}/{p[9]:>4} | mode {p[10]} drive {p[11]}" + (f" | dist {p[12]} cm" if len(p) > 12 else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    os.makedirs("logs", exist_ok=True)
    path = datetime.datetime.now().strftime("logs/teleop_%Y%m%d_%H%M%S.log")
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(2.0)  # the Uno resets when the port opens
    ser.reset_input_buffer()

    with open(path, "w") as logfile:
        reader = Reader(ser, logfile)
        reader.start()
        current = "x"
        print("Keys: w s a d | j k | space=stop | + - | r | q=quit")
        print(f"logging to {path}\n")
        last_sent = 0.0
        try:
            with Keyboard() as kb:
                while True:
                    ch = kb.get(0.03)
                    if ch:
                        ch = ch.lower()
                        if ch == "q":
                            break
                        if ch in MOVE_KEYS:
                            current = ch
                        elif ch in (" ", "x"):
                            current = "x"
                            ser.write(b"x")
                        elif ch in "+-=_rp?":
                            ser.write(ch.encode())
                    now = time.time()
                    if current != "x" and now - last_sent >= HEARTBEAT_S:
                        ser.write(current.encode())
                        last_sent = now
                    status = pretty(reader.latest) if reader.latest else "(waiting for telemetry)"
                    sys.stdout.write("\r" + status[:118].ljust(118))
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            for _ in range(3):
                ser.write(b"x")
                time.sleep(0.02)
            reader.running = False
            ser.close()
            print("\nstopped. log:", path)


if __name__ == "__main__":
    main()
