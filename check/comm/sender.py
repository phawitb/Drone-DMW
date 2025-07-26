# sender.py
import serial
import time

ser = serial.Serial('/dev/tty.usbserial-0001', 57600, timeout=1)
while True:
    ser.write(b"Hello from PC1\n")
    print("Sent.")
    time.sleep(1)
