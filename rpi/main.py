from pymavlink import mavutil
from collections import defaultdict
import json
import cv2
import pandas as pd
import time
import os
import serial.tools.list_ports
from ultralytics import YOLO
import threading

def sent_tele(data):
    json_str = json.dumps(data)
    ser.write((json_str + "\n").encode("utf-8"))  # ensure utf-8 encode
    print(f" Sent: {json_str}")

def log_detection_data(path, data_dict):
    df = pd.DataFrame([data_dict])
    if not os.path.exists(path):
        df.to_csv(path, index=False)
    else:
        df.to_csv(path, mode='a', header=False, index=False)

def is_new_detection(name, box, threshold=50):
    """
    Determine whether an object is new based on class name and bounding box.
    Threshold defines how far (in pixels) a box can move and still be considered the same object.
    """
    global last_detected_objects
    if name not in last_detected_objects:
        last_detected_objects[name] = []

    for prev_box in last_detected_objects[name]:
        # Compare center positions
        x1, y1, x2, y2 = box
        prev_x1, prev_y1, prev_x2, prev_y2 = prev_box
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        pcx, pcy = (prev_x1 + prev_x2) / 2, (prev_y1 + prev_y2) / 2

        if abs(cx - pcx) < threshold and abs(cy - pcy) < threshold:
            return False  # Already detected

    # New object detected
    last_detected_objects[name].append(box)
    return True

def detection_obj(frame,filtered_data):
    # global frame

    original_height, original_width = frame.shape[:2]
    new_width = 320
    aspect_ratio = original_height / original_width
    new_height = int(new_width * aspect_ratio)
    resized = cv2.resize(frame, (new_width, new_height))

    t0 = time.time()
    results = model(resized)
    print(f"Inference time: {time.time() - t0:.2f} s")

    boxes = results[0].boxes
    if boxes is not None:
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls_id]

            if cls_id in TARGET_CLASSES:
                xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                if is_new_detection(name, xyxy):
                    print(f"🟢 🟢 🟢 Detected: {name} ({conf:.2f}) {filtered_data} box: {xyxy}")

                    timestamp_str = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
                    img_path = os.path.join(DETECTION_DIR, f"{name}_{time.time():.0f}.png")

                    # Log detection
                    log_data = {
                        **filtered_data,
                        "class": name,
                        "conf": round(conf, 3),
                        "bbox_x1": round(xyxy[0], 2),
                        "bbox_y1": round(xyxy[1], 2),
                        "bbox_x2": round(xyxy[2], 2),
                        "bbox_y2": round(xyxy[3], 2),
                        "img_path": img_path,
                        "timestamp": timestamp_str,
                    }

                    log_data["type"] = "object"

                    # Save detection image
                    cv2.imwrite(img_path, frame)
                    print(f'saved {img_path}')

                    log_detection_data(DETECTION_CSV, log_data)
                    print(f'saved log {img_path}')

                    print("🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
                    sent_tele(log_data)

def find_lora_serial_ports():
    keywords = ["usbserial", "ch340", "cp210", "ftdi", "uart", "lora"]
    matched_ports = []
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(keyword in desc for keyword in keywords):
            matched_ports.append(port.device)
    return matched_ports

def get_opencv_usb_camera_ids(max_devices=10):
    available_ids = []
    for i in range(max_devices):
        dev_path = f"/dev/video{i}"
        if os.path.exists(dev_path):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                available_ids.append(i)
                cap.release()
    return available_ids

def find_drone_data_ports():
    keywords = ["cuav", "pixhawk", "fmu"]
    ports = []
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(k in desc for k in keywords):
            ports.append(port.device)
    return ports

def format_srt_time(seconds):
    hrs = int(seconds // 3600)
    mins = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    millis = int((seconds % 1) * 1000)
    return f"{hrs:02}:{mins:02}:{secs:02},{millis:03}"

# === Wait for devices ===
while True:
    drone_ports = find_drone_data_ports()
    cam_ids = get_opencv_usb_camera_ids()
    tele = find_lora_serial_ports()
    if drone_ports and cam_ids:
        print("✅ Drone:", drone_ports)
        print("✅ Camera:", cam_ids)
        print("✅ Tele:", tele)
        break
    print("⏳ Waiting for devices...")
    time.sleep(2)

# === CONFIGURATION ===
DRONE_ID = "D00001"
DRONE_PORT = drone_ports[0]
CAM_ID = cam_ids[0]
TELE = tele[0] if tele else None
SAVE_INTERVAL = 3  # seconds
N = 50  # samples for csv log

model = YOLO("yolov8n.pt")
TARGET_CLASSES = [0, 2]  # 0: person, 2: car
frame = None
last_detected_objects = {}

ser = serial.Serial(TELE, 57600, timeout=1)

sent_tele({'DRONE_ID':DRONE_ID,'DRONE_PORT':DRONE_PORT,'CAM_ID':CAM_ID,'TELE':TELE})

cap = cv2.VideoCapture(CAM_ID)
if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

ret, _ = cap.read()
if not ret:
    print("❌ Cannot read camera frame")
    cap.release()
    exit()
print("✅ Camera initialized")

# === MAVLink ===
master = mavutil.mavlink_connection(DRONE_PORT, baud=57600)
print("🔄 Waiting for heartbeat...")
master.wait_heartbeat()
print("✅ Heartbeat received")

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

# === Output folders ===
start_time = int(time.time())
OUTPUT_DIR = f"output/{start_time}"
IMAGES_DIR = os.path.join(OUTPUT_DIR, "images")
VIDEO_DIR = os.path.join(OUTPUT_DIR, "vdo")
DETECTION_DIR = os.path.join(OUTPUT_DIR, "detections")
DETECTION_CSV = os.path.join(DETECTION_DIR, "detection_datas.csv")
os.makedirs(DETECTION_DIR, exist_ok=True)
os.makedirs(IMAGES_DIR, exist_ok=True)
os.makedirs(VIDEO_DIR, exist_ok=True)

ALL_LOG_PATH = os.path.join(OUTPUT_DIR, "all_logs.csv")
FILTERED_LOG_PATH = os.path.join(OUTPUT_DIR, "filter_logs.csv")
VIDEO_PATH = os.path.join(VIDEO_DIR, "drone_vdo.mp4")
SUBTITLE_PATH = os.path.join(VIDEO_DIR, "drone_data.srt")

# === Measure actual FPS ===
print("⏳ Measuring real FPS...")
frame_times = []
for _ in range(30):
    ret, _ = cap.read()
    frame_times.append(time.time())
    time.sleep(0.01)

duration = frame_times[-1] - frame_times[0]
fps = max(1.0, min(len(frame_times) / duration, 30))  # clamp to 1–30 fps
print(f"✅ Real FPS measured: {fps:.2f}")

# === Video writer ===
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(VIDEO_PATH, fourcc, fps, (width, height))

# === Subtitle writer ===
srt_file = open(SUBTITLE_PATH, "a", encoding="utf-8")

# === CSV logger ===
def get_drone_logs():
    sum_data = defaultdict(float)
    count_data = defaultdict(int)
    for _ in range(N):
        msg = master.recv_match(blocking=True)
        if not msg or msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        for k, v in data.items():
            if isinstance(v, (int, float)):
                sum_data[k] += v
                count_data[k] += 1
    avg_data = {k: sum_data[k] / count_data[k] for k in sum_data}
    filtered = {
        "drone_lat": avg_data.get("lat", 0) / 1e7,
        "drone_lon": avg_data.get("lon", 0) / 1e7,
        "drone_alt": avg_data.get("alt", 0) / 1000,
        "drone_yaw": avg_data.get("yaw", 0),
        "drone_pitch": avg_data.get("pitch", 0),
        "drone_roll": avg_data.get("roll", 0),
        "drone_heading_deg": avg_data.get("hdg", 0) / 100,
    }
    return avg_data, filtered

def append_csv_row(path, data_dict):
    df = pd.DataFrame([data_dict])
    if not os.path.exists(path):
        df.to_csv(path, index=False)
    else:
        df.to_csv(path, mode='a', header=False, index=False)

# === Main loop ===
print("🎥 Recording... Press 'q' to quit")
frame_index = 0
script_start = time.time()
last_save_time = time.time()

threads = {}
i = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    now = time.time()
    elapsed = now - script_start

    video_writer.write(frame)

    drone_data = {
        "lat": 0, "lon": 0, "alt": 0,
        "yaw": 0, "pitch": 0, "roll": 0, "hdg": 0
    }
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "GLOBAL_POSITION_INT":
            drone_data["lat"] = msg.lat / 1e7
            drone_data["lon"] = msg.lon / 1e7
            drone_data["alt"] = msg.alt / 1000
        elif msg.get_type() == "ATTITUDE":
            drone_data["yaw"] = msg.yaw
            drone_data["pitch"] = msg.pitch
            drone_data["roll"] = msg.roll
        elif msg.get_type() == "VFR_HUD":
            drone_data["hdg"] = msg.heading

    subtitle_text = (
        f"Lat: {drone_data['lat']:.7f}, Lon: {drone_data['lon']:.7f}, Alt: {drone_data['alt']:.2f} m\n"
        f"Yaw: {drone_data['yaw']:.2f}, Pitch: {drone_data['pitch']:.2f}, "
        f"Roll: {drone_data['roll']:.2f}, Heading: {drone_data['hdg']:.2f}°"
    )

    # Write subtitle line
    srt_file.write(f"{frame_index + 1}\n")
    srt_file.write(f"{format_srt_time(elapsed)} --> {format_srt_time(elapsed + 1.0 / fps)}\n")
    srt_file.write(subtitle_text + "\n\n")
    srt_file.flush()

    if now - last_save_time >= SAVE_INTERVAL:
        all_data, filtered_data = get_drone_logs()
        timestamp_str = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(now))
        all_data["timestamp"] = timestamp_str
        filtered_data["timestamp"] = timestamp_str
        append_csv_row(ALL_LOG_PATH, all_data)
        append_csv_row(FILTERED_LOG_PATH, filtered_data)
        img_path = os.path.join(IMAGES_DIR, f"{timestamp_str}.png")
        cv2.imwrite(img_path, frame)
        print(f"✅ Saved at {timestamp_str}")

        print("Filtered:\n", json.dumps(filtered_data, indent=2))
        print(f"🟢 Current Drone: {filtered_data}")
        filtered_data['drone_id'] = DRONE_ID
        filtered_data['type'] = "current_drone"
        sent_tele(filtered_data)



        threads[str(i)] = threading.Thread(target=detection_obj, args=(frame,filtered_data,))
        threads[str(i)].start()
        i += 1

        last_save_time = now

    frame_index += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === Cleanup ===
cap.release()
video_writer.release()
srt_file.close()
cv2.destroyAllWindows()
print(f"✅ Video saved to {VIDEO_PATH}")
print(f"✅ Subtitle saved to {SUBTITLE_PATH}")
