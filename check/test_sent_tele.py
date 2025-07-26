# sender.py
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

def sent_tele(data):
    json_str = json.dumps(data)
    ser.write((json_str + "\n").encode("utf-8"))  # ensure utf-8 encode
    print(f"🟢 Sent: {json_str}")

# รอจนกว่าจะเจอ LoRa
while not find_lora_serial_ports():
    print('🔴 Not found TELE device')
    time.sleep(1)

serial_port = find_lora_serial_ports()[0]
baudrate = 57600
ser = serial.Serial(serial_port, baudrate, timeout=1)

print(f"🟢 Sender connected to {serial_port}")

DRONE_ID = "D0001"
counter = 1

while True:
    try:
        data = {
            "drone_id": DRONE_ID,
            "counter": counter,
            "timestamp": time.time(),
            "status": "ok"
        }

        sent_tele(data)
        
        counter += 1
        time.sleep(3)
    except Exception as e:
        print("❌ Sender error:", e)
        break
