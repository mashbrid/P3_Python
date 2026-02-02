import time
import board
import busio
import pwmio
import digitalio
import microcontroller
from analogio import AnalogIn
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

# -------------------------
# CONSTANTS
# -------------------------
SET_MIN_WINDOW = 5.0   # seconds
SET_MIN_BUFFER = 1.05  # multiplier for min step
SET_MAX_WINDOW = 5.0   # max step uses 5-second raw measurement

# -------------------------
# SETUP FSR & HAPTIC
# -------------------------
fsr_in = AnalogIn(board.A3)
motor_pwm = pwmio.PWMOut(board.D0, frequency=1000, duty_cycle=0)

# -------------------------
# SETUP ACCELEROMETER
# -------------------------
imupwr = digitalio.DigitalInOut(board.IMU_PWR)
imupwr.direction = digitalio.Direction.OUTPUT
imupwr.value = True
time.sleep(0.1)
imu_i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
sensor = LSM6DS3TRC(imu_i2c)

# -------------------------
# BLE RADIO
# -------------------------
ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)
ble.name = "Paddle003"

# -------------------------
# ONBOARD LEDS
# -------------------------
ledred = digitalio.DigitalInOut(board.LED_RED)
ledred.direction = digitalio.Direction.OUTPUT
ledred.value = True

ledblue = digitalio.DigitalInOut(board.LED_BLUE)
ledblue.direction = digitalio.Direction.OUTPUT
ledblue.value = True

ledgreen = digitalio.DigitalInOut(board.LED_GREEN)
ledgreen.direction = digitalio.Direction.OUTPUT
ledgreen.value = True

BLINK_ON_DURATION = 0.05
BLINK_OFF_DURATION = 5
LAST_BLINK_TIME = -1

step_now = 1  # initialize step

# -------------------------
# LOAD VARIABLES FROM NVM
# -------------------------
fsr_on = microcontroller.nvm[0] == 1
print("fsr_on:", fsr_on)

step1 = (microcontroller.nvm[1] << 8) | microcontroller.nvm[2]
step10 = (microcontroller.nvm[3] << 8) | microcontroller.nvm[4]

haptic2 = microcontroller.nvm[5]
haptic3 = microcontroller.nvm[6]
haptic4 = microcontroller.nvm[7]
haptic5 = microcontroller.nvm[8]
haptic6 = microcontroller.nvm[9]
haptic7 = microcontroller.nvm[10]
haptic8 = microcontroller.nvm[11]
haptic9 = microcontroller.nvm[12]
haptic10 = microcontroller.nvm[13]

xangle_on = microcontroller.nvm[14] == 1
xangle = microcontroller.nvm[15] * -1
xhaptic = microcontroller.nvm[16]
xtime = microcontroller.nvm[17] * 0.1

# Load Haptic Start from NVM
try:
    hstart = int(microcontroller.nvm[18])
except Exception:
    hstart = 4  # default
if hstart < 2 or hstart > 10:
    hstart = 4
microcontroller.nvm[18] = hstart  # always sync NVM

# Print NVM values for debugging
print("step1:", step1)
print("step10:", step10)
print("hstart:", hstart)
print("haptic2:", haptic2)
print("haptic3:", haptic3)
print("haptic4:", haptic4)
print("haptic5:", haptic5)
print("haptic6:", haptic6)
print("haptic7:", haptic7)
print("haptic8:", haptic8)
print("haptic9:", haptic9)
print("haptic10:", haptic10)
print("xangle_on:", xangle_on)
print("xangle:", xangle)
print("xhaptic:", xhaptic)
print("xtime:", xtime)

# -------------------------
# CALCULATE FSR STEPS
# -------------------------
fsr_increment = (step10 - step1) / 9
step2 = step1 + fsr_increment
step3 = step2 + fsr_increment
step4 = step3 + fsr_increment
step5 = step4 + fsr_increment
step6 = step5 + fsr_increment
step7 = step6 + fsr_increment
step8 = step7 + fsr_increment
step9 = step8 + fsr_increment

# -------------------------
# DOUBLE BUZZ & GREEN LED STARTUP
# -------------------------
for _ in range(2):
    ledgreen.value = False
    motor_pwm.duty_cycle = int(10 * 65535 / 100)
    time.sleep(0.2)
    ledgreen.value = True
    motor_pwm.duty_cycle = 0
    time.sleep(0.2)

# -------------------------
# HELPER FUNCTIONS
# -------------------------
def get_voltage(pin):
    return pin.value

x_negative_start_time = None

def check_xangle():
    global x_negative_start_time
    x, y, z = sensor.acceleration
    if x < xangle:
        if x_negative_start_time is None:
            x_negative_start_time = time.monotonic()
        elif time.monotonic() - x_negative_start_time >= xtime:
            for _ in range(3):
                motor_pwm.duty_cycle = int(xhaptic * 65535 / 100)
                time.sleep(0.08)
                motor_pwm.duty_cycle = 0
                time.sleep(0.08)
            x_negative_start_time = None
    else:
        x_negative_start_time = None

flashcounter = 1

def check_step():
    global flashcounter, step_now

    fsr_value = get_voltage(fsr_in)  # always read FSR

    if not fsr_on:
        motor_pwm.duty_cycle = 0

    # Step checks
    if fsr_value <= step1:
        step_now = 1
        if fsr_on:
            motor_pwm.duty_cycle = 0
        flashcounter += 1
    elif fsr_value <= step2:
        step_now = 2
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic2 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step3:
        step_now = 3
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic3 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step4:
        step_now = 4
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic4 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step5:
        step_now = 5
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic5 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step6:
        step_now = 6
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic6 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step7:
        step_now = 7
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic7 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step8:
        step_now = 8
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic8 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    elif fsr_value <= step9:
        step_now = 9
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic9 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1
    else:
        step_now = 10
        if fsr_on and step_now >= hstart:
            motor_pwm.duty_cycle = int(haptic10 * 65535 / 100)
        else:
            motor_pwm.duty_cycle = 0
        flashcounter = 1

    if flashcounter > 3000:
        ledgreen.value = False
        time.sleep(0.1)
        ledgreen.value = True
        flashcounter = 1

# -------------------------
# SET_MIN FUNCTION
# -------------------------
def set_min_from_fsr():
    global step1, step2, step3, step4, step5, step6, step7, step8, step9, fsr_increment

    start_time = time.monotonic()
    max_fsr = 0

    while time.monotonic() - start_time < SET_MIN_WINDOW:
        fsr_value = get_voltage(fsr_in)
        if fsr_value > max_fsr:
            max_fsr = fsr_value
        time.sleep(0.01)

    if max_fsr < 50:
        uart.write("ERROR: SET_MIN FAILED\n")
        return

    step1 = int(max_fsr * SET_MIN_BUFFER)
    max_allowed_step1 = step10 * 0.5
    if step1 > max_allowed_step1:
        step1 = int(max_allowed_step1)

    microcontroller.nvm[1] = (step1 >> 8) & 0xFF
    microcontroller.nvm[2] = step1 & 0xFF

    fsr_increment = (step10 - step1) / 9
    step2 = step1 + fsr_increment
    step3 = step2 + fsr_increment
    step4 = step3 + fsr_increment
    step5 = step4 + fsr_increment
    step6 = step5 + fsr_increment
    step7 = step6 + fsr_increment
    step8 = step7 + fsr_increment
    step9 = step8 + fsr_increment

    uart.write(f"STEP1:{step1}\n")

# -------------------------
# SET_MAX FUNCTION
# -------------------------
def set_max_from_fsr():
    global step10, step2, step3, step4, step5, step6, step7, step8, step9, fsr_increment

    start_time = time.monotonic()
    max_fsr = 0

    while time.monotonic() - start_time < SET_MAX_WINDOW:
        fsr_value = get_voltage(fsr_in)
        if fsr_value > max_fsr:
            max_fsr = fsr_value
        time.sleep(0.01)

    if max_fsr < 2 * step1:
        uart.write("ERROR: MAX < 2x MIN\n")
        return

    step10 = int(max_fsr)  # raw max, no multiplication

    microcontroller.nvm[3] = (step10 >> 8) & 0xFF
    microcontroller.nvm[4] = step10 & 0xFF

    fsr_increment = (step10 - step1) / 9
    step2 = step1 + fsr_increment
    step3 = step2 + fsr_increment
    step4 = step3 + fsr_increment
    step5 = step4 + fsr_increment
    step6 = step5 + fsr_increment
    step7 = step6 + fsr_increment
    step8 = step7 + fsr_increment
    step9 = step8 + fsr_increment

    uart.write(f"STEP10:{step10}\n")

# -------------------------
# TURN ON STEP FOR APP
# -------------------------
def get_turn_on_step():
    if haptic2 > 0: return 2
    if haptic3 > 0: return 3
    if haptic4 > 0: return 4
    if haptic5 > 0: return 5
    if haptic6 > 0: return 6
    if haptic7 > 0: return 7
    if haptic8 > 0: return 8
    if haptic9 > 0: return 9
    if haptic10 > 0: return 10
    return 0

# -------------------------
# MAIN LOOP
# -------------------------
last_transmission = time.monotonic()
last_hstart_sent = None

while True:
    if not ble.connected:
        ble.start_advertising(advertisement)
        print("Advertising...")
        while not ble.connected:
            check_step()
            if xangle_on:
                check_xangle()
            time.sleep(0.05)
        ble.stop_advertising()
        print("Connected!")
        uart.write(f"HSTART:{hstart}\n")  # initial sync on connect
        last_hstart_sent = hstart

    while ble.connected:
        check_step()
        if xangle_on:
            check_xangle()

        current_time = time.monotonic()

        # Send periodic status updates only if value changed or first time
        if current_time - last_transmission >= 0.01:
            fsr_value = get_voltage(fsr_in)
            x, _, _ = sensor.acceleration
            xangle_value = round(x, 2)
            uart.write(f"FSR:{fsr_value}\n")
            uart.write(f"XANGLE:{xangle_value}\n")
            uart.write(f"STEP:{step_now}\n")
            last_transmission = current_time

        if uart.in_waiting:
            try:
                data = uart.read(uart.in_waiting)
                if data:
                    try:
                        command = data.decode('utf-8').strip()
                        if command == "TOGGLE_FSR":
                            fsr_on = not fsr_on
                            microcontroller.nvm[0] = 1 if fsr_on else 0
                            uart.write("FSR ENABLED\n" if fsr_on else "FSR DISABLED\n")
                        elif command == "TOGGLE_XANGLE":
                            xangle_on = not xangle_on
                            microcontroller.nvm[14] = 1 if xangle_on else 0
                            uart.write("XANGLE ENABLED\n" if xangle_on else "XANGLE DISABLED\n")
                        elif command == "GET_FSR":
                            uart.write("FSR ENABLED\n" if fsr_on else "FSR DISABLED\n")
                        elif command == "GET_XANGLE":
                            uart.write("XANGLE ENABLED\n" if xangle_on else "XANGLE DISABLED\n")
                        elif command.startswith("SET_STEP2:"):
                            try:
                                haptic_value = int(command.split(":")[1])
                                haptic2 = haptic_value
                                microcontroller.nvm[5] = haptic2
                                uart.write(f"Step 2 set to {haptic2}\n")
                            except ValueError:
                                uart.write("ERROR: Invalid value for Step 2\n")
                        elif command.startswith("SET_STEP3:"):
                            try:
                                haptic_value = int(command.split(":")[1])
                                haptic3 = haptic_value
                                microcontroller.nvm[6] = haptic3
                                uart.write(f"Step 3 set to {haptic3}\n")
                            except ValueError:
                                uart.write("ERROR: Invalid value for Step 3\n")
                        elif command == "SET_MIN":
                            uart.write("SET_MIN START\n")
                            set_min_from_fsr()
                        elif command == "SET_MAX":
                            uart.write("SET_MAX START\n")
                            set_max_from_fsr()
                        elif command.startswith("HSTART:"):
                            try:
                                new_hstart = int(command.split(":")[1])
                                if 2 <= new_hstart <= 10:
                                    hstart = new_hstart
                                    microcontroller.nvm[18] = hstart
                                    uart.write(f"HSTART:{hstart}\n")
                                else:
                                    uart.write("ERROR: HSTART out of range\n")
                            except ValueError:
                                uart.write("ERROR: Invalid HSTART value\n")
                        elif command == "GET_HSTART":
                            uart.write(f"HSTART:{hstart}\n")
                        else:
                            uart.write("ERROR: Unknown command\n")
                    except UnicodeDecodeError:
                        uart.write("ERROR: Invalid command format\n")
            except Exception as e:
                print(f"Error: {e}")
                uart.write("ERROR: Something went wrong processing the data\n")

        time.sleep(0.05)
