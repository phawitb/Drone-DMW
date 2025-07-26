# receiver.py
import serial
import serial.tools.list_ports
import time
import json

def find_lora_serial_ports():
    keywords = ["usbserial", "ch340", "cp210", "ftdi", "uart", "lora"]
    matched_ports = []
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(keyword in desc for keyword in keywords):
            matched_ports.append(port.device)
    return matched_ports

# รอจนกว่าจะเจอ LoRa
while not find_lora_serial_ports():
    print('🔴 Not found TELE device')
    time.sleep(1)

serial_port = find_lora_serial_ports()[0]
baudrate = 57600
ser = serial.Serial(serial_port, baudrate, timeout=1)

print(f"🔵 Receiver connected to {serial_port}")

while True:
    try:
        raw = ser.readline()
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception as e:
            print("⚠️ Decode error:", e)
            continue

        if line.startswith("{") and line.endswith("}"):
            try:
                data = json.loads(line)
                print(f"🔵 Received dict: {data}")
            except json.JSONDecodeError:
                print(f"⚠️ JSON parse failed: {line}")
        else:
            if line:
                print(f"⚠️ Non-JSON line: {line}")
    except Exception as e:
        print("❌ Receiver error:", e)
        break
