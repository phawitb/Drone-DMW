from pymavlink import mavutil

# Connect to ArduPilot via USB serial port
master = mavutil.mavlink_connection('/dev/tty.usbmodem143203', baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system (system ID %u component ID %u)" %
      (master.target_system, master.target_component))

# Request data stream (1Hz)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Track GPS status
gps_fix_type = None
satellites = None

# Read messages until GPS is fixed or 50 messages max
for _ in range(50):
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    msg_type = msg.get_type()

    if msg_type == "GPS_RAW_INT":
        gps_fix_type = msg.fix_type
        satellites = msg.satellites_visible
        print(f"[GPS_RAW_INT] fix_type: {gps_fix_type}, satellites_visible: {satellites}")

        if gps_fix_type >= 2:
            print("✅ GPS FIXED")
        else:
            print("❌ GPS NOT FIXED")

    elif msg_type == "GLOBAL_POSITION_INT":
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # Convert mm to meters

        if lat == 0 and lon == 0:
            print("⚠️ GPS position invalid (lat/lon = 0)")
        else:
            print(f"[GPS] Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f} m")

    else:
        # Optional: print other types you care about
        pass
