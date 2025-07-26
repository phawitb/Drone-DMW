from pymavlink import mavutil
from collections import defaultdict
import json

## config recieve drone data ---------------------------------
# Connect
# master = mavutil.mavlink_connection('/dev/tty.usbmodem144203', baud=57600)
master = mavutil.mavlink_connection('/dev/tty.usbmodem14301', baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()

# Request data stream
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Accumulators
sum_data = defaultdict(float)
count_data = defaultdict(int)

N = 50  # number of samples

## ------------------------------------------------------------------

def get_drone_logs():

    for _ in range(N):
        msg = master.recv_match(blocking=True)
        if not msg or msg.get_type() == "BAD_DATA":
            continue

        data = msg.to_dict()
        for k, v in data.items():
            if isinstance(v, (int, float)):
                sum_data[k] += v
                count_data[k] += 1

    # Averaged result
    avg_data = {k: sum_data[k] / count_data[k] for k in sum_data}
    print('avg_data:',json.dumps(avg_data, indent=2))

    # Filter only necessary features for ML input
    filtered = {
        "drone_lat": avg_data.get("lat", 0) / 1e7,
        "drone_lon": avg_data.get("lon", 0) / 1e7,
        "drone_alt": avg_data.get("alt", 0) / 1000,
        "drone_yaw": avg_data.get("yaw", 0),            # radians
        "drone_pitch": avg_data.get("pitch", 0),        # radians
        "drone_roll": avg_data.get("roll", 0),          # radians
        "drone_heading_deg": avg_data.get("hdg", 0) / 100  # degrees
        # You will manually add: bbox_x, bbox_y, object_lat, object_lon
    }

    print("\n🎯 Filtered ML Input Features:")
    print('filtered:',json.dumps(filtered, indent=2))

    return avg_data,filtered


logs = get_drone_logs()
print('logs',logs)








"""
avg_data: {
  "time_boot_ms": 1756673.0,
  "roll": -0.008044913422054378,
  "pitch": -0.004692491609603167,
  "yaw": 1.716524600982666,
  "rollspeed": -9.347469313070178e-05,
  "pitchspeed": -0.0006404063897207379,
  "yawspeed": 0.00037388340570032597,
  "lat": 143027878.33333334,
  "lon": 1011636286.2857143,
  "alt": 26046.349999745686,
  "relative_alt": -567.5,
  "vx": 0.0,
  "vy": 0.0,
  "vz": 0.0,
  "hdg": 14761.5,
  "onboard_control_sensors_present": 1399913519.0,
  "onboard_control_sensors_enabled": 1383078959.0,
  "onboard_control_sensors_health": 1165007919.0,
  "load": 209.0,
  "voltage_battery": 0.0,
  "current_battery": -0.6666666666666666,
  "battery_remaining": -1.0,
  "drop_rate_comm": 0.0,
  "errors_comm": 0.0,
  "errors_count1": 0.0,
  "errors_count2": 0.0,
  "errors_count3": 0.0,
  "errors_count4": 0.0,
  "Vcc": 4289.5,
  "Vservo": 0.0,
  "flags": 85.5,
  "brkval": 0.0,
  "freemem": 65535.0,
  "freemem32": 550128.0,
  "nav_roll": -0.00035503937397152185,
  "nav_pitch": 0.00017767406825441867,
  "nav_bearing": 147.0,
  "target_bearing": 0.0,
  "wp_dist": 0.0,
  "alt_error": 0.6606668531894684,
  "aspd_error": 0.0,
  "xtrack_error": 0.0,
  "seq": 0.0,
  "total": 0.0,
  "mission_state": 1.0,
  "mission_mode": 0.0,
  "airspeed": 0.3250655382871628,
  "groundspeed": 0.005405463743954897,
  "heading": 147.0,
  "throttle": 0.0,
  "climb": 0.004221464041620493,
  "time_usec": 1756581797.5714285,
  "port": 0.0,
  "servo1_raw": 0.0,
  "servo2_raw": 0.0,
  "servo3_raw": 0.0,
  "servo4_raw": 0.0,
  "servo5_raw": 0.0,
  "servo6_raw": 0.0,
  "servo7_raw": 0.0,
  "servo8_raw": 0.0,
  "servo9_raw": 0.0,
  "servo10_raw": 0.0,
  "servo11_raw": 0.0,
  "servo12_raw": 0.0,
  "servo13_raw": 0.0,
  "servo14_raw": 0.0,
  "servo15_raw": 0.0,
  "servo16_raw": 0.0,
  "chancount": 0.0,
  "chan1_raw": 0.0,
  "chan2_raw": 0.0,
  "chan3_raw": 0.0,
  "chan4_raw": 0.0,
  "chan5_raw": 0.0,
  "chan6_raw": 0.0,
  "chan7_raw": 0.0,
  "chan8_raw": 0.0,
  "chan9_raw": 0.0,
  "chan10_raw": 0.0,
  "chan11_raw": 0.0,
  "chan12_raw": 0.0,
  "chan13_raw": 0.0,
  "chan14_raw": 0.0,
  "chan15_raw": 0.0,
  "chan16_raw": 0.0,
  "chan17_raw": 0.0,
  "chan18_raw": 0.0,
  "rssi": 255.0,
  "xacc": 17.0,
  "yacc": 23.0,
  "zacc": -1000.0,
  "xgyro": 2.5,
  "ygyro": -1.8333333333333333,
  "zgyro": 0.0,
  "xmag": -192.66666666666666,
  "ymag": -109.0,
  "zmag": 119.66666666666667,
  "id": 0.0,
  "temperature": 6489.272727272727,
  "press_abs": 1005.0069885253906,
  "press_diff": 0.0,
  "temperature_press_diff": 0.0,
  "fix_type": 4.0,
  "eph": 95.0,
  "epv": 145.0,
  "vel": 32.0,
  "cog": 29624.5,
  "satellites_visible": 11.0,
  "alt_ellipsoid": 12476.0,
  "h_acc": 5412.0,
  "v_acc": 7452.5,
  "vel_acc": 1514.5,
  "hdg_acc": 0.0,
  "time_unix_usec": 1751251145976180.0,
  "breach_status": 0.0,
  "breach_count": 0.0,
  "breach_type": 0.0,
  "breach_time": 0.0,
  "breach_mitigation": 1.0,
  "omegaIx": 0.000795660336734727,
  "omegaIy": 0.00098742067348212,
  "omegaIz": -0.0006251560116652399,
  "accel_weight": 0.0,
  "renorm_val": 0.0,
  "error_rp": 0.0003813790681306273,
  "error_yaw": 0.00041586090810596943,
  "altitude": -0.5649999976158142,
  "lng": 1011649593.0,
  "spacing": 0.0,
  "terrain_height": 0.0,
  "current_height": 39.04999923706055,
  "pending": 336.0,
  "loaded": 0.0,
  "velocity_variance": 0.0,
  "pos_horiz_variance": 0.0012574883294291794,
  "pos_vert_variance": 0.010567399440333247,
  "compass_variance": 9.718826549942605e-05,
  "terrain_alt_variance": 0.0,
  "airspeed_variance": 0.0,
  "vibration_x": 0.012441049329936504,
  "vibration_y": 0.01906130276620388,
  "vibration_z": 0.00834288727492094,
  "clipping_0": 0.0,
  "clipping_1": 0.0,
  "clipping_2": 0.0,
  "battery_function": 0.0,
  "type": 1.0,
  "current_consumed": 0.0,
  "energy_consumed": 0.0,
  "time_remaining": 0.0,
  "charge_state": 6.0,
  "mode": 0.0,
  "fault_bitmask": 0.0,
  "MCU_temperature": 3909.0,
  "MCU_voltage": 3288.0,
  "MCU_voltage_min": 3276.0,
  "MCU_voltage_max": 3302.0,
  "param_value": 6512.0,
  "param_type": 6.0,
  "param_count": 1029.0,
  "param_index": 65535.0,
  "autopilot": 3.0,
  "base_mode": 81.0,
  "custom_mode": 0.0,
  "system_status": 3.0,
  "mavlink_version": 3.0,
  "grid_spacing": 100.0,
  "mask": 7.205759403792794e+16
}

🎯 Filtered ML Input Features:
filtered: {
  "drone_lat": 14.302787833333335,
  "drone_lon": 101.16362862857143,
  "drone_alt": 26.046349999745686,
  "drone_yaw": 1.716524600982666,
  "drone_pitch": -0.004692491609603167,
  "drone_roll": -0.008044913422054378,
  "drone_heading_deg": 147.615
}

"""
