import serial
import serial.tools.list_ports
import time
import json
import string
import base64
import requests
import random
from pathlib import Path

BASE_URL = "http://127.0.0.1:5000"
IMG_DIR = Path("fake_imgs")  # ต้องมีรูป .jpg/.png อยู่ในโฟลเดอร์นี้

def find_lora_serial_ports():
    keywords = ["usbserial", "ch340", "cp210", "ftdi", "uart", "lora"]
    matched_ports = []
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(keyword in desc for keyword in keywords):
            matched_ports.append(port.device)
    return matched_ports

def clean_line(line):
    printable = set(string.printable)
    return ''.join(filter(lambda x: x in printable, line))

def encode_image_base64(filepath):
    try:
        with open(filepath, "rb") as f:
            encoded = base64.b64encode(f.read()).decode("utf-8")
            mime = "image/jpeg" if str(filepath).lower().endswith(".jpg") else "image/png"
            return f"data:{mime};base64,{encoded}"
    except Exception as e:
        print("❌ Failed to read image file:", e)
        return None

def save_base64_image(base64_string, output_dir="received_images"):
    try:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        if base64_string.startswith("data:"):
            base64_string = base64_string.split(",", 1)[1]

        img_data = base64.b64decode(base64_string)
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        file_path = output_dir / f"image_{timestamp}.jpg"

        with open(file_path, "wb") as f:
            f.write(img_data)
        print(f"🖼️ Saved image to {file_path}")
        return str(file_path)
    except Exception as e:
        print("❌ Error saving base64 image:", e)

def post_drone(data):
    payload = {
        "lat": data.get("lat", data.get("drone_lat")),
        "lon": data.get("lon", data.get("drone_lon")),
        "altitude": data.get("altitude", data.get("drone_alt")),
        "timestamp": data.get("timestamp")
    }
    try:
        r = requests.post(f"{BASE_URL}/api/send", json=payload)
        print(f"🚁 Sent current_drone | Status: {r.status_code}")
    except Exception as e:
        print("❌ Failed to send drone data:", e)

def post_object(data):
    image_b64 = data.get("image_base64")

    # 🔁 ถ้าไม่มี image_base64 → สุ่มจาก fake_imgs
    if not image_b64:
        image_files = list(IMG_DIR.glob("*.jpg")) + list(IMG_DIR.glob("*.png"))
        if image_files:
            random_image = random.choice(image_files)
            print(f"🎲 Using random image: {random_image}")
            image_b64 = encode_image_base64(random_image)
        else:
            print("⚠️ No images found in fake_imgs/")
            return

    payload = {
        "class": data.get("class"),
        "lat": data.get("lat", data.get("drone_lat")),
        "lon": data.get("lon", data.get("drone_lon")),
        "timestamp": data.get("timestamp"),
        "image_base64": image_b64
    }

    try:
        r = requests.post(f"{BASE_URL}/api/add_obj", json=payload)
        print(f"🧍 Sent object '{payload['class']}' | Status: {r.status_code}")
    except Exception as e:
        print("❌ Failed to send object data:", e)

# --- Serial Setup ---
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
        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if "{" in line:
            json_start = line.index("{")
            json_str = clean_line(line[json_start:])

            try:
                data = json.loads(json_str)
                print("✅ Parsed JSON:", data)

                # 🔄 แยกตาม type
                if data.get("type") == "object":
                    post_object(data)
                elif data.get("type") == "current_drone":
                    post_drone(data)
                else:
                    print("⚠️ Unknown type or missing 'type' field:", data)

            except json.JSONDecodeError as je:
                print("❌ JSON Decode Error:", je)
        else:
            print("⚠️ Non-JSON line skipped:", line)

    except Exception as e:
        print("❌ Receiver exception:", e)
        time.sleep(0.5)
