Import("env")

if hasattr(env, "IsCleanTarget") and env.IsCleanTarget():
    print("[CALIBRATION] Skipped: clean target")
    Return()

import os
import re
import subprocess

from platformio.public import list_serial_ports


# ---------------------------------------------------------------------------
# Find PlatformIO's esptool
# ---------------------------------------------------------------------------

def find_esptool():
    package_dir = env.PioPlatform().get_package_dir("tool-esptoolpy")

    if not package_dir:
        raise RuntimeError(
            "PlatformIO package 'tool-esptoolpy' was not found.\n"
            "Run: pio pkg install --tool platformio/tool-esptoolpy"
        )

    # Depending on the esptool version, it may be installed under one of
    # these names.
    candidates = [
        os.path.join(package_dir, "esptool.py"),
        os.path.join(package_dir, "esptool"),
    ]

    for path in candidates:
        if os.path.isfile(path):
            return path

    raise RuntimeError(
        "Could not find esptool inside:\n"
        + package_dir
    )


# ---------------------------------------------------------------------------
# Get Python executable used by PlatformIO
# ---------------------------------------------------------------------------

def get_python():
    python = env.get("PYTHONEXE")

    if not python:
        raise RuntimeError(
            "PlatformIO PYTHONEXE is not available"
        )

    return python


# ---------------------------------------------------------------------------
# Try to read MAC from one serial port
# ---------------------------------------------------------------------------

def get_mac_from_port(port):
    esptool = find_esptool()
    python = get_python()

    print(f"[CALIBRATION] Trying {port}...")

    command = [
        python,
        esptool,
        "--port",
        port,
        "read_mac",
    ]

    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10,
        )
    except subprocess.TimeoutExpired:
        print(f"[CALIBRATION] {port}: timeout")
        return None

    output = result.stdout

    if result.returncode != 0:
        print(f"[CALIBRATION] {port}: not ESP8266")
        return None

    # esptool output usually contains:
    #
    # MAC: f0:24:f9:d2:ec:87
    #
    match = re.search(
        r"\bMAC:\s*([0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5})\b",
        output,
    )

    if not match:
        print(
            f"[CALIBRATION] {port}: MAC not found"
        )
        return None

    mac = match.group(1).upper().replace(":", "")

    print(
        f"[CALIBRATION] ESP8266 found on {port}: {mac}"
    )

    return mac


# ---------------------------------------------------------------------------
# Find ESP8266 among all serial ports
# ---------------------------------------------------------------------------

def find_esp8266():
    ports = list_serial_ports()

    if not ports:
        raise RuntimeError(
            "No serial ports found."
        )

    print("[CALIBRATION] Serial ports:")

    for info in ports:
        port = info["port"]

        print(
            f"  {port}"
            f"  VID={info.get('vid')}"
            f"  PID={info.get('pid')}"
            f"  description={info.get('description')}"
        )

    print()

    found = []

    for info in ports:
        port = info["port"]

        mac = get_mac_from_port(port)

        if mac is not None:
            found.append((port, mac))

    if not found:
        raise RuntimeError(
            "No ESP8266 was found on available serial ports."
        )

    if len(found) > 1:
        message = [
            "Multiple ESP8266 devices were found:"
        ]

        for port, mac in found:
            message.append(
                f"  {port} -> {mac}"
            )

        message.append(
            ""
            "Please specify upload_port in platformio.ini "
            "when multiple ESP8266 boards are connected."
        )

        raise RuntimeError("\n".join(message))

    return found[0][1]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

mac = find_esp8266()

header = f"calibration_{mac}.hpp"

env.Append(
    CPPDEFINES=[
        ("CALIBRATION_HEADER", f'\\"{header}\\"'),
    ]
)

print()
print("============================================================")
print(f"[CALIBRATION] MAC:    {mac}")
print(f"[CALIBRATION] HEADER: {header}")
print("============================================================")
print()
