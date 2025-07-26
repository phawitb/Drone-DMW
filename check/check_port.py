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


serial_ports = find_lora_serial_ports()

if serial_ports:
    print("✅ Found LoRa serial port(s):")
    for port in serial_ports:
        print(f" - {port}")
else:
    print("❌ No LoRa serial ports detected.")
