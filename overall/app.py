from flask import Flask, render_template
from flask_socketio import SocketIO
from threading import Lock, Thread
import time
import serial

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode='threading')

# Serial Set up
SERIAL_PORT = '/dev/tty.usbserial-10'  # Replace with your serial port (e.g., 'COM3' or '/dev/ttyUSB0')
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
ref0 = 0 #reference current for battery
ref1 = 17 #reference voltage for Solar panel
ref2 = 24 #reference voltage for Outlet
# ??
Enabled=False

Battery1Ref_Current=0
PVRef_Voltage=15

OutletRef_Voltage=24

OverCharging = False
PVOptimized = False

Capacities=6.00*60*60

Battery1Low = False
Battery2Low = False

BatteryVoltageList1=[]
BatteryVoltageList2=[]

LowPowerCount=0

BatteryVoltageList2=[]

loopTimeStamp=time.time()

hasCalculatedIniCapacity=False

# Check the threads availability
# running = True

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
            ref1 = ref1 - 0.5 #Try change the ref voltage
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
                SolarRefVList = [ref1]
            ref1 = input - alpha*(input-input_old) + beta*grad/target
    else:
        if abs(SolarPower-SolarPowerlist[0]) > 3:
            # Threshold for the change of solar power
            PVOptimized = False
            SolarPowerlist = [ref1]
            # SolarRefVList = []


def PIfeedbackControl(Kp, Ki, DeltaT, targetY, refY, control_old, error_old):
    error = refY - targetY
    control = control_old + Ki*DeltaT*error + Kp*(error-error_old)
    return {"control": control, "error": error}

def management():
    global BatteryVoltageList1, BatteryVoltageList2, loopTimeStamp, Period, SoC1, SoC2, PVOptimized, ref0, ref1, ref2, SolarRefVList, Battery1Low, Battery2Low, hasCalculatedIniCapacity
    global OverCharging, LowPowerCount

    LowPowerCount = 0
    OptmLoopCount = 1 # used for solar panel optimizing update
    ref3 = 0 # Try to minimize the load on port 0
    error = 0
    while True:
        newTimeStamp = time.time()
        with data_lock:
            Period = newTimeStamp - loopTimeStamp
            loopTimeStamp = newTimeStamp

        serial_data = read_serial_data()
        if not serial_data:
            time.sleep(0.1)
            continue
        with data_lock:
            voltages = serial_data.get("voltages", [0, 0, 0, 0])
            currents = serial_data.get("currents", [0, 0, 0, 0])

        if not Enabled:
            BatteryVoltageList1.append(voltages[1])
            print(BatteryVoltageList1)
            #BatteryVoltageList2.append(voltages[0])
            time.sleep(1)
        else:
            time.sleep(1)
            if not hasCalculatedIniCapacity:
                SoC1 = findCapacityByVoltage(sum(BatteryVoltageList1)/len(BatteryVoltageList1)/8)
                #SoC2 = findCapacityByVoltage(sum(BatteryVoltageList2)/len(BatteryVoltageList2)/8)
                BatteryVoltageList1=[]
                #BatteryVoltageList2=[]
                hasCalculatedIniCapacity = True
            SoC1 = SoC1 + currents[1] * Period / (6000.0 * 3.6) #Positive current means consuming power/recharging
            #SoC2 = SoC2 + currents[0] * Period / (6000.0 * 3.6) # Current-time integral for battery charge
            # PV panel Optimizing and Control
            
            if ref0>2.0:
                OverCharging = True
            if voltages[1]>27.6:
                ref0 = ref0 - 1.0*(voltages[1]-27.6)*Period
                if voltages[0]>27.6:
                    OverCharging = True

            #batteries auto balancing
            if (SoC1-SoC2)>0.1:
                if ref0>0: ref3 = ref0*(1+(SoC1-SoC2)*2)
                elif ref0<0: ref3 = ref0/(1+(SoC1-SoC2)*2)
            elif (SoC2-SoC1)>0.1:
                if ref0>0: ref3 = ref0/(1-(SoC1-SoC2)*2)
                elif ref0<0: ref3 = ref0*(1-(SoC1-SoC2)*2)

            if voltages[0]>27.6:
                ref3 = ref3 - 2.0*(voltages[0]-27.6)*Period
            ##
            if SoC2<0.15:
                Battery2Low = True
                if SoC1<0.15: 
                    print("Warning: Not enough battery charge, system closed\n")
                    ref2=0
            elif SoC1<0.15:
                Battery1Low = True
            
            if Battery2Low:
                if ref3<0: ref3=0
                if SoC2 > 0.3: Battery2Low = False

            if ref3>2.0: ref3=1.9
            elif ref3<-5.0: ref3=-4.0

            if current[0]<-4:
                LowPowerCount = 1+LowPowerCount
            else:
                LowPowerCount = 0
            if LowPowerCount>10
                ref2 = 0
                print("Warning: System power supply is insufficient. Please reduce the appliance power.\n")

            # PV and control
            if not OverCharging:
                PVDynamicOptimizing(-voltages[2]*currents[2],OptmLoopCount)
                if PVOptimized:
                    OptmLoopCount = 1
                else:
                    OptmLoopCount = OptmLoopCount + 1
            else:
                controlData = PIfeedbackControl(-1.0, -1.0, Period, currents[0], ref3, ref1, error)
                ref1 = controlData.get("control",ref1)
                error = controlData.get("error",error)

            # Battery control
            if not OverCharging:
                controlData = PIfeedbackControl(-0.5, -1, Period, currents[0], ref3, ref0, error)
                ref0 = controlData.get("control",0)
                error = controlData.get("error",error)

            else:
                if ref1 < SolarRefVList[-1]:
                    ref1 = SolarRefVList[-1] #if the solar power needed exceed the max power, restart the solar power optimizing
                    PVOptimized = False
                    OptmLoopCount = 1
                    OverCharging = False # Change the charging 
            
            if ref0>2.0: ref0=1.9
            elif ref0<-5.0: ref0=-4.5
            
            if Battery1Low:
                if ref0<0: ref0=0
                if SoC1 > 0.3: Battery1Low = False
            #Write the ref data to the router:
            send_serial_data()
        time.sleep(1)

def send_serial_data():
    global ref0, ref1, ref2, ser
    try:
        with data_lock:
            formatted_data = f"ref0: {ref0:.3f}\n"
            ser.write(formatted_data.encode('utf-8'))
            time.sleep(0.002)
            formatted_data = f"ref1: {ref1:.3f}\n"
            ser.write(formatted_data.encode('utf-8'))
            time.sleep(0.002)
            formatted_data = f"ref2: {ref2:.3f}\n"
            ser.write(formatted_data.encode('utf-8'))
            time.sleep(0.002)
    except Exception as e:
        print(f"Error sending serial data: {e}")

def read_serial_data():
    """Read and parse data from the serial port."""
    global voltages, currents, ser
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                # Check input format
                if line.startswith("voltage & Current:"):
                    values = line.replace("voltage & Current:", "").strip().split(", ")
                    values = [float(v) for v in values]
                    if len(values) == 8:
                        with data_lock:
                            voltages = values[:4]
                            currents = values[4:]
                            print(voltages)
                            print(currents)
                        return {"voltages": voltages, "currents": currents}
        except ValueError:
            print("Error: Invalid data format received from serial.")
            continue
        except Exception as e:
            print(f"Unexpected error while reading serial: {e}")
            continue            
    return None

def update_power_data():
    while True:
        global voltages, currents, SoC1, SoC2
        with data_lock:
            # Calculate power values
            solar = round(voltages[2] * currents[2], 1)
            apply = round(voltages[3] * currents[3], 1)
            charge1 = round(voltages[1] * currents[1], 1)
            charge2 = round(voltages[0] * currents[0], 1)
            pct1 = round(SoC1 * 100, 1)
            pct2 = round(SoC2 * 100, 1)
            # Prepare Data for sending
        power_data = {
            'solar': - solar,
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
    global ref2, ser, Enabled
    """Start streaming data."""
    Enabled = True
    text = "PIDstart\n"
    # ref2 = 24
    with data_lock:
        ser.write(text.encode('utf-8'))
        time.sleep(0.01)
    print("Data streaming started.")

@socketio.on('stop_stream')
def stop_stream():
    global ser, Enabled
    """Stop streaming data."""
    # global streaming
    # streaming = 
    Enabled = False 
    text = "disableAll\n"
    with data_lock:
        ser.write(text.encode('utf-8'))
        time.sleep(0.01)
    print("Data streaming stopped.")

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    """Start the thread if it's not already running."""
    global thread
    if thread is None:
        thread = socketio.start_background_task(update_power_data)

if __name__ == '__main__':
    # Start the management thread
    management_thread = Thread(target=management, daemon=True)
    management_thread.start()
    
    # Start Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=800)
    