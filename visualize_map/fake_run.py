import requests
import time
import random
import base64
from datetime import datetime
from pathlib import Path

BASE_URL = "http://127.0.0.1:5000"

OBJECT_CLASSES = ['Tank', 'Person', 'Airplane', 'Helicopter', 'Car']
IMG_DIR = Path("fake_imgs")  # folder with .png/.jpg images

# starting coordinates
lat = 13.7367
lon = 100.5231

def encode_image_base64(filepath):
    mime = "image/png" if filepath.suffix.lower() == ".png" else "image/jpeg"
    with open(filepath, "rb") as f:
        encoded = base64.b64encode(f.read()).decode("utf-8")
        return f"data:{mime};base64,{encoded}"

def post_drone(lat, lon, altitude):
    data = {
        "lat": lat,
        "lon": lon,
        "altitude": altitude,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }
    r = requests.post(f"{BASE_URL}/api/send", json=data)
    print(f"🚁 Drone posted: {data['lat']:.6f}, {data['lon']:.6f} | Status: {r.status_code}")

def post_object(obj_class, lat, lon, image_path):
    image_base64 = encode_image_base64(image_path)
    data = {
        "class": obj_class,
        "lat": lat,
        "lon": lon,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "image_base64": image_base64
    }
    r = requests.post(f"{BASE_URL}/api/add_obj", json=data)
    print(f"🧍 Object {obj_class} posted with image: {image_path.name} | Status: {r.status_code}")

def run_simulation():
    global lat, lon
    image_files = list(IMG_DIR.glob("*.[pj][pn]g"))  # .png and .jpg

    if not image_files:
        print("❌ No images found in fake_imgs/")
        return

    for i in range(20):
        lat += random.uniform(-0.0002, 0.0002)
        lon += random.uniform(-0.0002, 0.0002)
        altitude = random.uniform(100, 150)

        post_drone(lat, lon, altitude)

        # post object every 3 steps
        if i % 3 == 0:
            obj_class = random.choice(OBJECT_CLASSES)
            image_path = random.choice(image_files)
            obj_lat = lat + random.uniform(-0.0005, 0.0005)
            obj_lon = lon + random.uniform(-0.0005, 0.0005)
            post_object(obj_class, obj_lat, obj_lon, image_path)

        time.sleep(1)

if __name__ == "__main__":
    run_simulation()
