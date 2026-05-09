"""
pc_controller.py — Keyboard controller for the robot via ESP1 serial bridge.

Usage:
    python pc_controller.py [PORT] [BAUD]

    Default port: COM3 (Windows) or /dev/ttyUSB0 (Linux/Mac)
    Default baud: 115200

Controls:
    W / S       Forward / Reverse  (increases RPM by STEP each press)
    A / D       Spin left / Spin right
    SPACE       Stop (sets RPM to 0)
    0-5, T      Mode switches:
                  0 = MANUAL_DRIVE
                  1 = WALL_FOLLOWING
                  2 = CENTERING
                  3 = LOW_TOWER
                  4 = ATTACK_NEXUS
                  5 = ATTACK_TOP_TOWER
                  T = VIVE_NAV (mode 10)
    +  /  -     Increase / decrease target RPM by STEP
    Q           Quit

Requires:
    pip install pyserial keyboard
    (On Linux/Mac, run with sudo for keyboard access)
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

try:
    import keyboard
except ImportError:
    print("Missing dependency: pip install keyboard")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200
RPM_STEP     = 10      # RPM change per keypress
RPM_MAX      = 150
RPM_MIN      = 0

# ── State ──────────────────────────────────────────────────────────────
current_rpm   = 60
left_dir      = 1
right_dir     = 1

# ── Serial helpers ──────────────────────────────────────────────────────
def send(ser: serial.Serial, msg: str):
    line = msg.strip() + "\n"
    ser.write(line.encode())
    print(f"  >> {msg}")

def send_drive(ser):
    send(ser, f"RPM:{current_rpm},L:{left_dir},R:{right_dir}")

def send_stop(ser):
    send(ser, "STOP")

def send_mode(ser, mode: int):
    send(ser, f"MODE:{mode}")

# ── Serial reader thread (prints ESP1 responses) ───────────────────────
def reader_thread(ser: serial.Serial):
    while True:
        try:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"  <- {line}")
        except Exception:
            break

# ── Main ───────────────────────────────────────────────────────────────
def main():
    global current_rpm, left_dir, right_dir

    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    # Auto-detect port if default isn't found
    if port == DEFAULT_PORT:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports and DEFAULT_PORT not in ports:
            port = ports[0]
            print(f"[Auto] Using port {port}")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1.5)  # let ESP1 boot
        print(f"[Serial] Connected to {port} @ {baud}")
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        print("Available ports:", [p.device for p in serial.tools.list_ports.comports()])
        sys.exit(1)

    # Start background reader
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    print("\n=== Robot Controller ===")
    print("  W/S = fwd/rev   A/D = spin   SPACE = stop")
    print("  +/- = RPM up/dn   0-5 / T = mode   Q = quit")
    print(f"  Starting RPM: {current_rpm}")
    print("========================\n")

    def on_key(event):
        global current_rpm, left_dir, right_dir
        key = event.name.lower()

        if key == 'w':
            left_dir, right_dir = 1, 1
            current_rpm = min(current_rpm + RPM_STEP, RPM_MAX)
            send_drive(ser)

        elif key == 's':
            left_dir, right_dir = -1, -1
            current_rpm = min(current_rpm + RPM_STEP, RPM_MAX)
            send_drive(ser)

        elif key == 'a':
            left_dir, right_dir = -1, 1
            send_drive(ser)

        elif key == 'd':
            left_dir, right_dir = 1, -1
            send_drive(ser)

        elif key == 'space':
            current_rpm = 0
            send_stop(ser)

        elif key == '+' or key == '=':
            current_rpm = min(current_rpm + RPM_STEP, RPM_MAX)
            print(f"  [RPM -> {current_rpm}]")
            send_drive(ser)

        elif key == '-':
            current_rpm = max(current_rpm - RPM_STEP, RPM_MIN)
            print(f"  [RPM -> {current_rpm}]")
            send_drive(ser)

        elif key == '0': send_mode(ser, 0)
        elif key == '1': send_mode(ser, 1)
        elif key == '2': send_mode(ser, 2)
        elif key == '3': send_mode(ser, 3)
        elif key == '4': send_mode(ser, 4)
        elif key == '5': send_mode(ser, 5)
        elif key == 't': send_mode(ser, 10)

        elif key == 'q':
            print("Quitting...")
            send_stop(ser)
            ser.close()
            sys.exit(0)

    keyboard.on_press(on_key)
    keyboard.wait()   # block until 'q' triggers sys.exit()

if __name__ == "__main__":
    main()
