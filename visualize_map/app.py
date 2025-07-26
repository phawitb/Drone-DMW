from flask import Flask, request, jsonify, render_template, send_from_directory
import os, csv, base64
from datetime import datetime

app = Flask(__name__, static_url_path='/static')

# === STEP 1: Create next flight directory ===
def get_next_flight_dir():
    os.makedirs("logs", exist_ok=True)
    existing = [d for d in os.listdir("logs") if d.startswith("flight") and os.path.isdir(os.path.join("logs", d))]
    nums = [int(d.replace("flight", "")) for d in existing if d.replace("flight", "").isdigit()]
    next_num = max(nums, default=0) + 1
    flight_path = os.path.join("logs", f"flight{next_num}")
    os.makedirs(flight_path, exist_ok=True)
    os.makedirs(os.path.join(flight_path, "images"), exist_ok=True)
    return flight_path

# === STEP 2: Setup paths ===
FLIGHT_DIR = get_next_flight_dir()
DRONE_LOG_PATH = os.path.join(FLIGHT_DIR, "drone_log.csv")
OBJECT_LOG_PATH = os.path.join(FLIGHT_DIR, "objects_log.csv")
IMG_DIR = os.path.join(FLIGHT_DIR, "images")

print(f"🔄 Logging to: {FLIGHT_DIR}")

# === STEP 3: Runtime memory ===
latest_data = {
    'lat': None,
    'lon': None,
    'altitude': None,
    'timestamp': None
}
objects = []
flight_ready = False

# === ROUTES ===

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/api/start_flight', methods=['POST'])
def start_flight():
    global flight_ready, FLIGHT_DIR, DRONE_LOG_PATH, OBJECT_LOG_PATH, IMG_DIR
    flight_ready = True
    FLIGHT_DIR = get_next_flight_dir()
    DRONE_LOG_PATH = os.path.join(FLIGHT_DIR, "drone_log.csv")
    OBJECT_LOG_PATH = os.path.join(FLIGHT_DIR, "objects_log.csv")
    IMG_DIR = os.path.join(FLIGHT_DIR, "images")
    print(f"🚀 New flight started: {FLIGHT_DIR}")
    return jsonify({'status': 'Flight started', 'ready': flight_ready})

@app.route('/api/status')
def get_status():
    return jsonify({'ready': flight_ready})

@app.route('/api/send', methods=['POST'])
def send_data():
    if not flight_ready:
        return jsonify({'error': 'Flight not started'}), 403

    data = request.json
    required = ['lat', 'lon', 'altitude', 'timestamp']
    if not all(k in data for k in required):
        return jsonify({'error': 'Missing required fields'}), 400

    latest_data.update({
        'lat': float(data['lat']),
        'lon': float(data['lon']),
        'altitude': float(data['altitude']),
        'timestamp': str(data['timestamp'])
    })

    try:
        os.makedirs(os.path.dirname(DRONE_LOG_PATH), exist_ok=True)
        with open(DRONE_LOG_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([data['timestamp'], data['lat'], data['lon'], data['altitude']])
    except Exception as e:
        print(f"[Error] Writing drone log failed: {e}")

    return jsonify({'status': 'drone updated'}), 200

@app.route('/api/latest')
def get_latest():
    if not flight_ready or latest_data['lat'] is None:
        return jsonify({'ready': False})
    return jsonify({**latest_data, 'ready': True})

@app.route('/api/add_obj', methods=['POST'])
def add_object():
    if not flight_ready:
        return jsonify({'error': 'Flight not started'}), 403

    data = request.json
    required = ['class', 'lat', 'lon', 'timestamp']
    if not all(k in data for k in required):
        return jsonify({'error': 'Missing fields'}), 400

    obj = {
        'class': data['class'],
        'lat': float(data['lat']),
        'lon': float(data['lon']),
        'timestamp': str(data['timestamp']),
        'image_base64': data.get('image_base64')
    }
    objects.append(obj)

    try:
        with open(OBJECT_LOG_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([data['timestamp'], data['class'], data['lat'], data['lon']])
    except Exception as e:
        print(f"[Error] Writing object log failed: {e}")

    if obj['image_base64']:
        try:
            os.makedirs(IMG_DIR, exist_ok=True)
            header, encoded = obj['image_base64'].split(',', 1)
            img_data = base64.b64decode(encoded)
            fname = f"{obj['timestamp'].replace(':','-')}_{obj['class']}.png"
            img_path = os.path.join(IMG_DIR, fname)
            with open(img_path, 'wb') as img_file:
                img_file.write(img_data)
        except Exception as e:
            print(f"[Warning] Could not save image: {e}")

    return jsonify({'status': 'object added', 'count': len(objects)})

@app.route('/api/objects')
def get_objects():
    return jsonify(objects)

@app.route('/api/clear')
def clear_data():
    global flight_ready
    latest_data.update({
        'lat': None,
        'lon': None,
        'altitude': None,
        'timestamp': None
    })
    objects.clear()
    flight_ready = False
    return jsonify({'status': 'cleared'})

@app.route('/history')
def history():
    if not os.path.exists("logs"):
        return render_template('history.html', flights=[])

    # flight_dirs = sorted([
    #     d for d in os.listdir("logs") 
    #     if d.startswith("flight") and os.path.isdir(os.path.join("logs", d))
    # ])

    flight_dirs = sorted(
        [
            d for d in os.listdir("logs")
            if d.startswith("flight") and os.path.isdir(os.path.join("logs", d))
        ],
        key=lambda x: int(x.replace("flight", ""))
    )


    flights_data = []

    for flight in flight_dirs:
        flight_path = os.path.join("logs", flight)
        drone_log_path = os.path.join(flight_path, "drone_log.csv")
        object_log_path = os.path.join(flight_path, "objects_log.csv")
        image_dir = os.path.join(flight_path, "images")

        drone_path = []
        object_list = []

        if os.path.exists(drone_log_path):
            with open(drone_log_path, newline='') as f:
                reader = csv.reader(f)
                for row in reader:
                    drone_path.append({
                        'timestamp': row[0],
                        'lat': float(row[1]),
                        'lon': float(row[2]),
                        'altitude': float(row[3])
                    })

        if os.path.exists(object_log_path):
            with open(object_log_path, newline='') as f:
                reader = csv.reader(f)
                for row in reader:
                    timestamp, class_name, lat, lon = row
                    img_file = f"{timestamp.replace(':','-')}_{class_name}.png"
                    img_path = f"/logs/{flight}/images/{img_file}" if os.path.exists(os.path.join(image_dir, img_file)) else None

                    object_list.append({
                        'timestamp': timestamp,
                        'class': class_name,
                        'lat': float(lat),
                        'lon': float(lon),
                        'image_url': img_path
                    })

        flights_data.append({
            'flight': flight,
            'path': drone_path,
            'objects': object_list
        })

    return render_template('history.html', flights=flights_data)

@app.route('/logs/<path:filename>')
def serve_logs(filename):
    return send_from_directory('logs', filename)

if __name__ == '__main__':
    app.run(debug=True)
