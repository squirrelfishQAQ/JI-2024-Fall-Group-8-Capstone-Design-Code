import serial

SERIAL_PORT = '/dev/tty.usbserial-110'  # Replace with your serial port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)