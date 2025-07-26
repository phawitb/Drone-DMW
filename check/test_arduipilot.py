from pymavlink import mavutil

# ใช้พอร์ตที่คุณใช้ได้จริง
port = '/dev/tty.usbmodem14301'
baud = 115200

# สร้าง MAVLink connection
master = mavutil.mavlink_connection(port, baud=baud)

# รอ heartbeat (จะค้างหากยัง boot ไม่เสร็จ)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✅ Heartbeat from system {master.target_system}, component {master.target_component}")

# อ่านข้อมูล MAVLink แบบถอดรหัส
while True:
    msg = master.recv_match(blocking=True)
    if msg:
        print(f"[{msg.get_type()}] {msg.to_dict()}")
