from ultralytics import YOLO
import cv2
import time
import threading

# Load model
model = YOLO("yolov8n.pt")

# Only detect person and car
TARGET_CLASSES = [0, 2]  # 0: person, 2: car

# Shared frame variable
frame = None

def detection_obj():
    global frame
    resized = cv2.resize(frame, (320, 320))
    t0 = time.time()
    results = model(resized)
    print(f"Inference time: {time.time() - t0:.2f} s")

    # Check results
    boxes = results[0].boxes
    if boxes is not None:
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls_id]
            if cls_id in TARGET_CLASSES:
                print(f"🟢 Detected: {name} ({conf:.2f})")

# Open camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

last_infer_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    now = time.time()
    if now - last_infer_time > 3:  # Run YOLO every 3 seconds
        thread1 = threading.Thread(target=detection_obj)
        thread1.start()
        last_infer_time = now

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
