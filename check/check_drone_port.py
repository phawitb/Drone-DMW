import serial.tools.list_ports
import cv2
import glob
import os

def get_opencv_usb_camera_ids(max_devices=10):
    """
    Returns a list of available USB camera IDs usable by OpenCV.
    Filters only real /dev/video* devices.
    """
    available_ids = []
    for i in range(max_devices):
        dev_path = f"/dev/video{i}"
        if os.path.exists(dev_path):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                available_ids.append(i)
                cap.release()
    return available_ids


def find_drone_data_ports():
    """
    Returns a list of all serial ports likely connected to a drone (CUAV, Pixhawk, FMU).
    """
    keywords = ["cuav", "pixhawk", "fmu"]
    ports = []

    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(k in desc for k in keywords):
            ports.append(port.device)

    return ports


d = find_drone_data_ports()
print(d)

c = get_opencv_usb_camera_ids()
print(c)