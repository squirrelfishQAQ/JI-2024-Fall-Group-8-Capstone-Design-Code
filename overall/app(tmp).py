from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
from threading import Lock
import random

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# Serial Configuration
SERIAL_PORT = '<your_port>'  # Replace with your serial port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Background Thread
thread = None
thread_lock = Lock()

data_lock = Lock()

voltages = [0,0,0,0]
currents = [0,0,0,0]

# Soc1 = 0
# SoC2 = 0

def management():
    serial_data = read_serial_data()
    voltages = get_data.get("voltages", [0, 0, 0, 0])
    currents = get_data.get("currents", [0, 0, 0, 0])


def read_serial_data():
    """Read and parse data from the serial port."""
    global voltages, currents
    if ser.in_waiting > 0:
        try:
            # Read a line of serial data
            line = ser.readline().decode('utf-8').strip()
            
            # Check if the line starts with the expected format
            if line.startswith("voltage & Current:"):
                # Remove the label and split the numeric data
                values = line.replace("voltage & Current:", "").strip().split(", ")
                
                # Convert the values to floats
                values = [float(v) for v in values]
                
                if len(values) == 8:  # Ensure we have exactly 8 values
                    # Separate voltages and currents
                    with data_lock:
                        voltages = values[:4]
                        currents = values[4:]
                    return {"voltages": voltages, "currents": currents}
            print("Unexpected data format:", line)
        except Exception as e:
            print("Error reading serial data:", e)
    return None

def update_power_data():
    """Update power data based on serial input."""
    while True:
        global voltages, currents

        # global SoC1, SoC2

        # serial_data = read_serial_data()
        # management("get_data")
        with data_lock:
        #     # Extract voltages and currents
        #     voltages = get_data.get("voltages", [0, 0, 0, 0])
        #     currents = get_data.get("currents", [0, 0, 0, 0])
            
            # Calculate power values
            solar = voltages[2] * currents[2]
            apply = voltages[3] * currents[3]
            charge1 = voltages[1] * currents[1]
            charge2 = voltages[0] * currents[0]
            
            # Waiting for battery percentage calculation
            # bat1_pcnt = SoC1
            # bat2_pcnt = SoC2

            power_data = {
                'solar': solar,
                'apply': apply,
                'charge1': charge1,
                'charge2': charge2,
                'consume1': - charge1,
                'consume2': - charge2,
                
                # Waiting for battery percentage calculation
                'pct1': random.randint(40, 100),
                'pct2': random.randint(40, 100),
                # 'pct1': bat1_pcnt,
                # 'pct2': bat2_pcnt,
            }
            socketio.emit('power_update', power_data)
        socketio.sleep(2)  # Update every 2 seconds

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    """Start the thread if it's not already running."""
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(update_power_data)

if __name__ == '__main__':
    try:
        socketio.run(app, host='0.0.0.0', port=8000, debug=True)
    finally:
        ser.close()  # Close the serial port when exiting


