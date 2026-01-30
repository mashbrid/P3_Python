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

# Print NVM values for debugging
print("step1:", step1)
print("step10:", step10)
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
        
    if fsr_value <= step1:
        step_now = 1
        if fsr_on:
            motor_pwm.duty_cycle = 0
        flashcounter += 1
    elif fsr_value <= step2:
        step_now = 2
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic2 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step3:
        step_now = 3
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic3 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step4:
        step_now = 4
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic4 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step5:
        step_now = 5
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic5 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step6:
        step_now = 6
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic6 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step7:
        step_now = 7
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic7 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step8:
        step_now = 8
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic8 * 65535 / 100)
        flashcounter = 1
    elif fsr_value <= step9:
        step_now = 9
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic9 * 65535 / 100)
        flashcounter = 1
    else:
        step_now = 10
        if fsr_on:
            motor_pwm.duty_cycle = int(haptic10 * 65535 / 100)
        flashcounter = 1

    if flashcounter > 3000:
        ledgreen.value = False
        time.sleep(0.1)
        ledgreen.value = True
        flashcounter = 1

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

    last_transmission = time.monotonic()

    while ble.connected:
        check_step()
        if xangle_on:
            check_xangle()
        # Send initial status messages upon connection
        uart.write("FSR ENABLED\n" if fsr_on else "FSR DISABLED\n")
        uart.write("XANGLE ENABLED\n" if xangle_on else "XANGLE DISABLED\n")

        time.sleep(0.05)
        # Check if there is any data waiting to be read
        if uart.in_waiting:
            print(f"Data waiting: {uart.in_waiting} bytes")
            try:
                # Read all available data
                data = uart.read(uart.in_waiting)
                if data is not None:
                    # Decode the data (utf-8)
                    try:
                        command = data.decode('utf-8').strip()
                        # Handle commands
                        if command == "TOGGLE_FSR":
                            fsr_on = not fsr_on
                            microcontroller.nvm[0] = 1 if fsr_on else 0  # Save to NVM
                            response = "FSR ENABLED\n" if fsr_on else "FSR DISABLED\n"
                            uart.write(response)
                        elif command == "TOGGLE_XANGLE":
                            xangle_on = not xangle_on
                            if xangle_on:
                                microcontroller.nvm[14] = 1  # Save to NVM
                            else:
                                microcontroller.nvm[14] = 0  # Save to NVM
                            if xangle_on:
                                response = "XANGLE ENABLED\n"
                            else:
                                response = "XANGLE DISABLED\n"
                            uart.write(response)
                        elif command == "GET_FSR":
                            if fsr_on:
                                response = "FSR ENABLED\n"
                            else:
                                response = "FSR DISABLED\n"
                            uart.write(response)
                        elif command == "GET_XANGLE":
                            if xangle_on:
                                response = "XANGLE ENABLED\n"
                            else:
                                response = "XANGLE DISABLED\n"
                            uart.write(response)
                        elif command.startswith("SET_STEP2:"):
                            try:
                                # Extract the value from the command
                                haptic_value = int(command.split(":")[1])
                                haptic2 = haptic_value  # Update the step2 variable
                                microcontroller.nvm[5] = (haptic2)
                                uart.write(f"Step 2 set to {haptic2}\n")
                            except ValueError:
                                uart.write("ERROR: Invalid value for Step 2\n")
                        else:
                            uart.write("ERROR: Unknown command\n")
                    except UnicodeDecodeError:
                        uart.write("ERROR: Invalid command format\n")
                else:
                    print("No data received (None).")
            except Exception as e:
                print(f"Error: {e}")
                uart.write("ERROR: Something went wrong processing the data\n")
        else:
            # Periodically send FSR and xangle values
            current_time = time.monotonic()
            if current_time - last_transmission >= .01:  # Send every second
                # Read FSR and xangle values
                fsr_value = get_voltage(fsr_in)
                x, _, _ = sensor.acceleration
                xangle_value = round(x, 2)

                # Prepare data to send
                fsr_message = f"FSR:{fsr_value}\n"
                xangle_message = f"XANGLE:{xangle_value}\n"
                step_message = f"STEP:{step_now}\n"  # Add step_now message
                # print(step_now)

                # Send data over UART
                uart.write(fsr_message)
                uart.write(xangle_message)
                uart.write(step_message)

                last_transmission = current_time
        time.sleep(0.05)  # Reduced sleep time for responsiveness
