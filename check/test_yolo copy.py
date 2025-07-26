from ultralytics import YOLO
import cv2
import time

model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

last_infer_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    now = time.time()
    if now - last_infer_time > 3:  # run YOLO every 3 seconds
        resized = cv2.resize(frame, (320, 320))
        t0 = time.time()
        results = model(resized)
        print(f"Inference time: {time.time() - t0:.2f} s")

        # Print detections (class name and confidence)
        boxes = results[0].boxes
        if boxes is not None:
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls_id]
                print(f"🟢 Detected: {name} ({conf:.2f})")

        last_infer_time = now

    # No display, so waitKey not needed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
