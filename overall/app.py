from flask import Flask, render_template
from flask_socketio import SocketIO
import random  # For simulating power data (Delete after completely Serial)
from threading import Lock, Thread
import time
import serial

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# Serial Set up
SERIAL_PORT = '<your_port>'  # Replace with your serial port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Background Thread
thread = None
thread_lock = Lock()

# Setup for threading management and website
data_lock = Lock()
voltages = [0,0,0,0]
currents = [0,0,0,0]
Period=1/10
# Battery Percentage
SoC1=0
SoC2=0
# ??
Enabled=False

Battery1Ref_Current=0
PVRef_Voltage=15

OutletRef_Voltage=24

Flag_OverCharging=0
Flag_PVOptimized=0

Capacities=6.00*60*60
BatterDecayFactor=1
BatterDecayFactor=1

BatteryVoltageList1=[]
BatteryVoltageList2=[]

loopTimeStamp=time.time()

def SoCIntegral():
    global currents, voltages, SoC1, SoC2, Enabled, Period
    # put your code here, and add any necessary global variables as you wish
    # This function will be called by management function every period

def findCapacityByVoltage(v):
    cap = 0.0
    if 3.40 < v <= 3.65:
        cap = (-2500.0 / 3 * v + 8500.0 / 3)
    elif 3.37 < v <= 3.40:
        cap = (-42500.0 * v + 137000.0)
    elif 3.33 < v <= 3.37:
        cap = (-50000 * v + 161000)
    elif 3.28 < v <= 3.33:
        cap = (-100000.0 / 3 * v + 325000.0 / 3)
    elif 3.21 <= v < 3.28:
        cap = (-20000.0 * v + 66600.0)
    elif 3.12 <= v < 3.21:
        cap = (-25000.0 / 3 * v + 92000.0 / 3)
    elif 2.9 <= v < 3.12:
        cap = (-20000.0 / 11 * v + 10991.0)
    elif 0 <= v < 2.9:
        cap = (-500.0 / 3 * v + 19100.0 / 3)
    else:
        cap = 6000.0
    return (6000.0 - cap)

def management():
    global BatteryVoltageList1, BatteryVoltageList2, loopTimeStamp, Period, SoC1, SoC2
    while True:
        serial_data = read_serial_data()
        newTimeStamp = time.time()
        Period = newTimeStamp - loopTimeStamp
        loopTimeStamp = newTimeStamp

        voltages = serial_data.get("voltages", [0, 0, 0, 0])
        currents = serial_data.get("currents", [0, 0, 0, 0])
        if not Enabled:
            BatteryVoltageList1.append(voltages[1])
            #BatteryVoltageList2.append(voltages[0])
            
        else:
            if not hasCalculatedIniCapacity:
                SoC1 = findCapacityByVoltage(sum(BatteryVoltageList1)/len(BatteryVoltageList1))
                #SoC2 = findCapacityByVoltage(sum(BatteryVoltageList2)/len(BatteryVoltageList2))
                BatteryVoltageList1=[]
                #BatteryVoltageList2=[]
                hasCalculatedIniCapacity = True
            SoCIntegral()

def read_serial_data():
    """Read and parse data from the serial port."""
    global voltages, currents, ser
    while True:
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
                        #Why added break here?
                        break 
                print("Unexpected data format:", line)
            except Exception as e:
                print("Error reading serial data:", e)
    return None

def update_power_data():
    while True:
        global voltages, currents, SoC1, SoC2

        with data_lock:
            # Calculate power values
            solar = voltages[2] * currents[2]
            apply = voltages[3] * currents[3]
            charge1 = voltages[1] * currents[1]
            charge2 = voltages[0] * currents[0]
            pct1 = SoC1
            pct2 = SoC2
            # Prepare Data for sending
        power_data = {
            'solar': solar,
            'apply': apply,
            'charge1': charge1,
            'charge2': charge2,
            'consume1': - charge1,
            'consume2': - charge2,
            'pct1': pct1,
            'pct2': pct2,
        }
        # Send data
        socketio.emit('power_update', power_data)
        # Update every 2 seconds
        socketio.sleep(2)  
        
@socketio.on('start_stream')
def start_stream():
    """Start streaming data."""
    # global streaming
    # streaming = True
    print("Data streaming started.")

@socketio.on('stop_stream')
def stop_stream():
    """Stop streaming data."""
    # global streaming
    # streaming = False
    print("Data streaming stopped.")

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
    # Start the management function in a separate thread
    management_thread = Thread(target=management, daemon=True)
    management_thread.start()

    # Start the Flask website
    socketio.run(app, host='0.0.0.0', port=8000, debug=True)
