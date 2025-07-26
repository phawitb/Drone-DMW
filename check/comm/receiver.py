# receiver.py
import serial

ser = serial.Serial('/dev/tty.usbserial-4', 57600, timeout=1)
while True:
    msg = ser.readline().decode().strip()
    if msg:
        print("Received:", msg)
