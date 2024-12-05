from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
from threading import Lock
import random
import time

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
currents = [0,0,0,0] # Negative value for providing power, positive consuming power

# Global varibles for Management

Period=1/10

SoC1=0
SoC2=0

ref0 = 0 #reference current for battery
ref1 = 17 #reference voltage for Solar panel
ref2 = 24 #reference voltage for Outlet

Enabled=False

Battery1Ref_Current=0
PVRef_Voltage=15

OutletRef_Voltage=24

OverCharging = False
PVOptimized = False

Capacities=6.00*60*60
BatterDecayFactor=1.0
BatterDecayFactor=1.0

BatteryVoltageList1=[]
BatteryVoltageList2=[]

loopTimeStamp=time.time()

segments = [
    (2.95, 0.0),
    (3.15, 0.05),
    (3.2, 0.08),
    (3.3, 0.33),
    (3.32, 0.66),
    (3.34, 0.75),
    (3.35, 0.95),
    (3.4, 1.0)
]

def findCapacityByVoltage(v):
    if v <= segments[0][0]:
        return 0.0
    elif segments[0][0] < v <= segments[1][0]:
        return ((v-segments[0][0])/(segments[1][0]-segments[0][0])*(segments[1][1]-segments[0][1])+segments[0][1])
    elif segments[1][0] < v <= segments[2][0]:
        return ((v-segments[1][0])/(segments[2][0]-segments[1][0])*(segments[2][1]-segments[1][1])+segments[1][1])
    elif segments[2][0] < v <= segments[3][0]:
        return ((v-segments[2][0])/(segments[3][0]-segments[2][0])*(segments[3][1]-segments[2][1])+segments[2][1])
    elif segments[3][0] < v <= segments[4][0]:
        return ((v-segments[3][0])/(segments[4][0]-segments[3][0])*(segments[4][1]-segments[3][1])+segments[3][1])
    elif segments[4][0] < v <= segments[5][0]:
        return ((v-segments[4][0])/(segments[5][0]-segments[4][0])*(segments[5][1]-segments[4][1])+segments[4][1])
    elif segments[5][0] < v <= segments[6][0]:
        return ((v-segments[5][0])/(segments[6][0]-segments[5][0])*(segments[6][1]-segments[5][1])+segments[5][1])
    elif segments[6][0] < v <= segments[7][0]:
        return ((v-segments[6][0])/(segments[7][0]-segments[6][0])*(segments[7][1]-segments[6][1])+segments[6][1])
    elif v > segments[7][0]:
        return 1.0


SolarPowerlist = []
SolarRefVList = []
IterationCount = 0

def PVDynamicOptimizing(SolarPower, loopCount):
    global PVOptimized, SolarPowerlist, SolarRefVList, ref1
    SolarUpdateInterval = 10
    if not PVOptimized:
        SolarPowerlist.append[SolarPower]
        if loopCount == SolarUpdateInterval:
            SolarRefVList.append(ref1)
            IterationCount = 1
            ref1 = ref1 - 1 #Try change the 
        elif loopCount % SolarUpdateInterval == 0:
            IterationCount = 1 + IterationCount
            target = sum(SolarPowerlist[IterationCount*SolarUpdateInterval-5:IterationCount*SolarUpdateInterval-1])/5 # Compute the average power before and after change
            target_old = sum(SolarPowerlist[(IterationCount-1)*SolarUpdateInterval-5:(IterationCount-1)*SolarUpdateInterval-1])/5
            input = ref1
            input_old = SolarRefVList[-1]
            SolarRefVList.append(ref1)
            grad = (target - target_old)/(input-input_old)
            alpha = 0.5 #parameters to adjust
            beta = 20
            if grad < 5:
                PVOptimized = True
                IterationCount = 0
                SolarPowerlist = [target]
            ref1 = input - alpha*(input-input_old) + beta*grad/target
    else:
        if abs(SolarPower-SolarPowerlist[0]) > 3:
            # Threshold for the change of solar power
            PVOptimized = False
            SolarPowerlist = []
            SolarRefVList = []


def PIfeedbackControl(Kp, Ki, DeltaT, targetY, refY, control_old, error_old):
    error = refY - targetY
    control = control_old + Ki*DeltaT*error + Kp*(error-error_old)
    return {"control": control, "error": error}

def management():
    global BatteryVoltageList1, BatteryVoltageList2, loopTimeStamp, Period, SoC1, SoC2, PVOptimized, ref0, ref1, ref2
    ref0_old = 0
    ref1_old = 17
    OptmLoopCount = 1 # used for solar panel optimizing update
    ref3 = 0 # Try to minimize the load on port 0
    error = 0
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
            SoC1 = SoC1 + currents[1] * Period / (6000.0 * 3.6) #Positive current means consuming power/recharging
            #SoC2 = SoC2 + currents[0] * Period / (6000.0 * 3.6) # Current-time integral for battery charge
            # PV panel Optimizing and Control
            if not OverCharging:
                DynamicOptimizing(voltages[2]*current[2],OptmLoopCount)
                if PVOptimized:
                    OptmLoopCount = 1
                else:
                    OptmLoopCount = OptmLoopCount + 1
            else:
                controlData = PIfeedbackControl(-5.0, -10.0, Period, currents[0],ref3,ref1_old,error)
                ref1 = controlData.get("control",ref1)
                error = controlData.get("error",error)

            # Battery control
            if not OverCharging:
                controlData = PIfeedbackControl(-5.0, -10.0, Period, currents[0],ref3,ref0_old,error)
            # Battery monitoring. todo: switching flag "OverCharging" regarding to the current and voltage of both battery port
            # todo: regulate the current and voltage of batteries in case of over charging/discharging

            
def read_serial_data():
    """Read and parse data from the serial port."""
    global voltages, currents, ser
    while True
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
                        break
                print("Unexpected data format:", line)
            except Exception as e:
                print("Error reading serial data:", e)
    return None

def update_power_data():
    """Update power data based on serial input."""
    while True:
        global voltages, currents
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
            # bat1_pcnt = 2 * 3
            # bat2_pcnt = 3 * 4

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


