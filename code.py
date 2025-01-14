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

# Setup FSR and Haptic
fsr_in = AnalogIn(board.A3)  # Pin 3 analog input from FSR
motor_pwm = pwmio.PWMOut(board.D0, frequency=1000, duty_cycle=0)  # Motor on D5

# Setup Accelerometer
imupwr = digitalio.DigitalInOut(board.IMU_PWR)
imupwr.direction = digitalio.Direction.OUTPUT
imupwr.value = True
time.sleep(0.1)
imu_i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
sensor = LSM6DS3TRC(imu_i2c)

# Initialize BLE radio
ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)
ble.name = "Paddle003"

# set up on board LED colors
ledred = digitalio.DigitalInOut(board.LED_RED)
ledred.direction = digitalio.Direction.OUTPUT
ledred.value = True  # turn off red LED
ledblue = digitalio.DigitalInOut(board.LED_BLUE)
ledblue.direction = digitalio.Direction.OUTPUT
ledblue.value = True  # turn off blue LED
ledgreen = digitalio.DigitalInOut(board.LED_GREEN)
ledgreen.direction = digitalio.Direction.OUTPUT
ledgreen.value = True  # turn off green LED

# values to blink LEDs when connected
BLINK_ON_DURATION = 0.05
BLINK_OFF_DURATION = 5
LAST_BLINK_TIME = -1

step_now = 1  # initialize variable

# Load variables from NVM
fsr_on = microcontroller.nvm[0] == 1
print("fsr_0n:", (fsr_on))
step1 = (microcontroller.nvm[1] << 8) | microcontroller.nvm[2]
# step1 = 11000
# microcontroller.nvm[1] = (step1 >> 8) & 0xFF  # High byte of step1
# microcontroller.nvm[2] = step1 & 0xFF  # Low byte of step1print("step1:", (step1))
time.sleep(.1)
step10 = (microcontroller.nvm[3] << 8) | microcontroller.nvm[4]
print("step10:", (step10))
haptic2 = microcontroller.nvm[5]
print("haptic2:", (haptic2))
haptic3 = microcontroller.nvm[6]
print("haptic3:", (haptic3))
haptic4 = microcontroller.nvm[7]
print("haptic4:", (haptic4))
haptic5 = microcontroller.nvm[8]
print("haptic5:", (haptic5))
haptic6 = microcontroller.nvm[9]
print("haptic6:", (haptic6))
haptic7 = microcontroller.nvm[10]
print("haptic7:", (haptic7))
haptic8 = microcontroller.nvm[11]
print("haptic8:", (haptic8))
haptic9 = microcontroller.nvm[12]
print("haptic9:", (haptic9))
haptic10 = microcontroller.nvm[13]
print("haptic10:", (haptic10))
xangle_on = microcontroller.nvm[14] == 1
print("xangle_on:", (xangle_on))
xangle = microcontroller.nvm[15] * -1
print("xangle:", (xangle))
xhaptic = microcontroller.nvm[16]
print("xhaptic:", (xhaptic))
xtime = microcontroller.nvm[17] * .1
print("xtime:", (xtime))

# Variables to track X state
x_negative_start_time = None  # Timestamp when X becomes negative

# Set values for FSR resistance to each step
fsr_increment = (step10 - step1) / 9
step2 = step1 + fsr_increment
step3 = step2 + fsr_increment
step4 = step3 + fsr_increment
step5 = step4 + fsr_increment
step6 = step5 + fsr_increment
step7 = step6 + fsr_increment
step8 = step7 + fsr_increment
step9 = step8 + fsr_increment

#  Flash green LED twice and run haptice twice to signal startup OK
ledgreen.value = False
motor_pwm.duty_cycle = int(10 * 65535 / 100)  # 10% intensity
time.sleep(0.2)
ledgreen.value = True
motor_pwm.duty_cycle = 0  # Haptic off
time.sleep(0.2)
ledgreen.value = False
motor_pwm.duty_cycle = int(10 * 65535 / 100)  # 10% intensity
time.sleep(0.2)
ledgreen.value = True
motor_pwm.duty_cycle = 0  # Haptic off

# Global variables
flashcounter = 1  # Green LED heartbeat flash if Pic 1 for several cycles

def get_voltage(pin):
    return pin.value

def show_xyz():
    x, y, z = sensor.acceleration  # Acceleration in m/s^2
    print("x is ", (x))
    print("y is ", (y))
    print("z is ", (z))
    time.sleep(0.1)

def check_xangle():
    global x_negative_start_time  # Declare x_negative_start_time as global
    # Read accelerometer data
    x, y, z = sensor.acceleration  # Acceleration in m/s^2

    # Check if X is less than 0
    if x < xangle:
        if x_negative_start_time is None:
            # Start tracking time when X first becomes negative
            x_negative_start_time = time.monotonic()
        elif time.monotonic() - x_negative_start_time >= xtime:
            # If X has been negative for the threshold duration, pulse the haptic
            print("X negative > 1 second, trigger haptic.")
            motor_pwm.duty_cycle = int(xhaptic * 65535 / 100)
            time.sleep(0.08)
            motor_pwm.duty_cycle = 0  # Haptic off
            time.sleep(0.08)
            motor_pwm.duty_cycle = int(xhaptic * 65535 / 100)
            time.sleep(0.08)
            motor_pwm.duty_cycle = 0  # Haptic off
            time.sleep(0.08)
            motor_pwm.duty_cycle = int(xhaptic * 65535 / 100)
            time.sleep(0.08)
            motor_pwm.duty_cycle = 0  # Haptic off
            time.sleep(0.08)
            x_negative_start_time = None  # Reset after pulsing
    else:
        # Reset tracking if X is not negative
        x_negative_start_time = None

def check_step():
    time.sleep(0.05)
    global flashcounter, step_now
    fsr_value = get_voltage(fsr_in)
    if fsr_value <= step1:  # step1 always off
        step_now = 1
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = 0  # Haptic off
        flashcounter = flashcounter + 1  # increment flashcounter
    if fsr_value > step1 and fsr_value <= step2:  # step2
        step_now = 2
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic2 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step2 and fsr_value <= step3:  # step3
        step_now = 3
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = haptic3
        flashcounter = 1
    if fsr_value > step3 and fsr_value <= step4:  # step4
        step_now = 4
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic4 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step4 and fsr_value <= step5:  # step5
        step_now = 5
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic5 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step5 and fsr_value <= step6:  # step6
        step_now = 6
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic6 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step6 and fsr_value <= step7:  # step7
        step_now = 7
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic7 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step7 and fsr_value <= step8:  # step8
        step_now = 8
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic8 * 65535 / 100)
        flashcounter = 1
    if fsr_value > step8 and fsr_value <= step9:  # step9
        step_now = 9
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic9 * 65535 / 100)  # 100% intensity
        flashcounter = 1
    if fsr_value > step9:  # step10
        step_now = 10
        print(f"Step {step_now}: {get_voltage(fsr_in)}")
        motor_pwm.duty_cycle = int(haptic10 * 65535 / 100)  # 100% intensity
        flashcounter = 1
    if (flashcounter) > 3000:  # flash LED if sitting in step1
        ledgreen.value = False  # turns LED on
        time.sleep(0.1)
        ledgreen.value = True  # turns LED off
        flashcounter = 1  # reset LED flashcounter
# save_variables_to_nvm()
while True:
    # Advertise BLE if not connected
    if not ble.connected:
        ble.start_advertising(advertisement)
        print("Advertising...")
        while not ble.connected:
            if fsr_on:
                check_step()
            if xangle_on:
                check_xangle()
            # show_xyz()
            time.sleep(0.05)
        ble.stop_advertising()
        print("Connected!")

    last_transmission = time.monotonic()

    while ble.connected:
        if fsr_on:
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
                print(step_now)

                # Send data over UART
                uart.write(fsr_message)
                uart.write(xangle_message)
                uart.write(step_message)

                last_transmission = current_time
        time.sleep(0.05)  # Reduced sleep time for responsiveness
