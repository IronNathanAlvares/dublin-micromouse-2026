"""mouse.py - the whole build / flash / check loop in one command.

    python tools/mouse.py ports                  list connected boards
    python tools/mouse.py build                  compile the firmware (no board needed)
    python tools/mouse.py upload                 compile + flash the firmware, then open the monitor
    python tools/mouse.py monitor                Serial Monitor; everything is also saved in logs/
    python tools/mouse.py monitor --send s w m   ...and type those commands automatically
    python tools/mouse.py debug 06_tof_3_imu     flash a Debug Kit step (or i2c_scan, encoder_test, ...)
    python tools/mouse.py test                   all offline tests: solver, mms protocol, virtual robot

Options: --port COM5 (otherwise the first connected board is used).
Close the Arduino IDE's Serial Monitor first: only one program can use the port.
Each monitor session is saved to logs/<date-time>.txt, so you can paste real
output to a teammate or an AI agent (see docs/CONTEXT.md).
"""
import argparse
import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIRMWARE = os.path.join(ROOT, "firmware", "micromouse")
FQBN = "esp32:esp32:esp32c6:CDCOnBoot=cdc"  # ESP32C6 Dev Module, USB CDC On Boot: Enabled
BAUD = "115200"


def arduino_cli():
    found = shutil.which("arduino-cli")
    candidates = [
        found,
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"),
        r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli",
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    sys.exit("Can't find arduino-cli. Install the Arduino IDE 2 (it includes it) or arduino-cli itself.")


def run(args, **kw):
    print("> " + " ".join(os.path.basename(a) if i == 0 else a for i, a in enumerate(args)), flush=True)
    return subprocess.run(args, **kw)


def ports():
    out = subprocess.run([arduino_cli(), "board", "list", "--format", "json"],
                         capture_output=True, text=True).stdout
    found = []
    for p in json.loads(out or "{}").get("detected_ports", []):
        port = p.get("port", {})
        if port.get("protocol") == "serial":
            found.append((port.get("address"), port.get("properties", {}).get("pid", ""),
                          port.get("label", "")))
    return found


def pick_port(requested):
    if requested:
        return requested
    found = ports()
    if not found:
        sys.exit("No board found. Check the USB cable carries data (many only charge), then try again.\n"
                 "Still nothing? Hold BOOT, tap RESET, release BOOT, and retry.")
    return found[0][0]


def build(sketch):
    build_dir = os.path.join(ROOT, ".build", os.path.basename(sketch))
    r = run([arduino_cli(), "compile", "--fqbn", FQBN, "--warnings", "all", "--build-path", build_dir, sketch])
    if r.returncode:
        sys.exit("Build failed (see above).")
    return build_dir


def upload(sketch, port):
    build_dir = build(sketch)
    r = run([arduino_cli(), "upload", "--fqbn", FQBN, "-p", port, "--input-dir", build_dir, sketch])
    if r.returncode:
        sys.exit("Upload failed. Is the Serial Monitor open somewhere else? Try: hold BOOT, tap RESET, "
                 "release BOOT, then upload again.")
    time.sleep(1.5)  # let the board reboot and USB re-appear


def monitor(port, send, seconds):
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    log_path = os.path.join(ROOT, "logs", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".txt")
    print(f"Monitor on {port} at {BAUD} baud. Saving to {os.path.relpath(log_path, ROOT)}")
    print("Type a command letter + Enter to send it (h = help). Ctrl+C to quit.\n", flush=True)
    proc = subprocess.Popen([arduino_cli(), "monitor", "-p", port, "--config", f"baudrate={BAUD}", "--quiet"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    log = open(log_path, "w", encoding="utf-8")

    def reader():
        for line in proc.stdout:
            stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(f"[{stamp}] {line}")
            log.flush()

    threading.Thread(target=reader, daemon=True).start()

    def write(cmd):
        log.write(f"[sent] {cmd}\n")
        proc.stdin.write(cmd)  # the firmware reads single characters
        proc.stdin.flush()

    try:
        time.sleep(2)
        for cmd in send:
            write(cmd)
            time.sleep(1.5)
        if seconds:
            time.sleep(seconds)
        else:
            for line in sys.stdin:
                write(line.strip())
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        log.close()
        print(f"\nSaved {os.path.relpath(log_path, ROOT)}")


def debug_sketch(name):
    hits = [d for d in glob.glob(os.path.join(ROOT, "debug-kit", "**", name), recursive=True) if os.path.isdir(d)]
    if not hits:
        names = sorted(os.path.basename(os.path.dirname(p))
                       for p in glob.glob(os.path.join(ROOT, "debug-kit", "**", "*.ino"), recursive=True))
        sys.exit(f"No Debug Kit sketch '{name}'. Choose from: {', '.join(names)}")
    return hits[0]


def test():
    gpp = shutil.which("g++") or sys.exit("g++ (MinGW) is needed for the offline tests.")
    ok = True
    exe = ".exe" if os.name == "nt" else ""
    steps = [
        ("build mms simulator mouse", [gpp, "-std=c++17", "-O2", "-static", "-I../firmware/micromouse",
                                       "-o", "mouse" + exe, "Main.cpp", "API.cpp"], os.path.join(ROOT, "sim")),
        ("solver on 30 random mazes", [sys.executable, "tools/fake_mms.py", "30", "16"], ROOT),
        ("solver on the event maze", [sys.executable, "tools/fake_mms.py", "--files", "mazes/dublin2026.txt"], ROOT),
        ("build virtual robot (real firmware)", [gpp, "-std=c++17", "-O2", "-static", "-Isim/robot/fake",
                                                 "-Ifirmware/micromouse", "-o", "sim/robot/robot_sim" + exe,
                                                 "sim/robot/robot_sim.cpp"], ROOT),
        ("virtual robot: full competition on the event maze",
         [os.path.join(ROOT, "sim", "robot", "robot_sim" + exe), "mazes/dublin2026.txt", "1"], ROOT),
    ]
    for label, cmd, cwd in steps:
        print(f"\n=== {label}", flush=True)
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        print((r.stdout + r.stderr).strip()[-800:])
        if r.returncode:
            ok = False
            print(f"!!! FAILED: {label}")
    print("\nALL TESTS PASSED" if ok else "\nSOME TESTS FAILED")
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["ports", "build", "upload", "monitor", "debug", "test"])
    ap.add_argument("name", nargs="?", help="Debug Kit sketch for 'debug'")
    ap.add_argument("--port")
    ap.add_argument("--send", nargs="*", default=[], help="commands to type into the monitor")
    ap.add_argument("--seconds", type=float, help="close the monitor after this long")
    a = ap.parse_args()

    if a.command == "ports":
        found = ports()
        print("\n".join(f"{addr}  {label}" for addr, _, label in found) or "No boards connected.")
    elif a.command == "build":
        build(FIRMWARE)
        print("Build OK")
    elif a.command == "upload":
        port = pick_port(a.port)
        upload(FIRMWARE, port)
        monitor(port, a.send, a.seconds)
    elif a.command == "monitor":
        monitor(pick_port(a.port), a.send, a.seconds)
    elif a.command == "debug":
        if not a.name:
            sys.exit("Which step? e.g. python tools/mouse.py debug 01_led_red")
        sketch = debug_sketch(a.name)
        port = pick_port(a.port)
        upload(sketch, port)
        monitor(port, a.send, a.seconds)
    elif a.command == "test":
        test()


if __name__ == "__main__":
    main()
