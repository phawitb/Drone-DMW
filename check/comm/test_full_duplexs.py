# full_duplex.py
import threading
import serial
import time
import serial.tools.list_ports

def find_lora_serial_ports():
    """
    Returns a list of serial ports likely connected to a LoRa module (e.g., LR900F).
    Filters ports by common USB-UART identifiers like CH340, CP210, FTDI, etc.
    """
    keywords = ["usbserial", "ch340", "cp210", "ftdi", "uart", "lora"]
    matched_ports = []

    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(keyword in desc for keyword in keywords):
            matched_ports.append(port.device)

    return matched_ports

while not find_lora_serial_ports():
    print('not found TELE')
print(f'found TELE: {find_lora_serial_ports()}')

# ระบุพอร์ตที่เชื่อมกับ LR900F
serial_port = find_lora_serial_ports()[0] #"/dev/tty.usbserial-4"  # หรือ "/dev/tty.SLAB_USBtoUART" สำหรับ macOS
baudrate = 57600

ser = serial.Serial(serial_port, baudrate, timeout=1)

def sender():
    while True:
        try:
            msg = input("🟢 You: ")
            ser.write((msg + "\n").encode())
        except Exception as e:
            print("Sender error:", e)
            break

def receiver():
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(f"\n🔵 Friend: {line}\n🟢 You: ", end='', flush=True)
        except Exception as e:
            print("Receiver error:", e)
            break

# สร้าง thread แยกส่ง/รับ
threading.Thread(target=receiver, daemon=True).start()
sender()  # รัน sender ใน thread หลัก
