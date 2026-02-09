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
SET_MIN_WINDOW = 5.0
SET_MIN_BUFFER = 1.05
SET_MAX_WINDOW = 5.0

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

step_now = 1
# -------------------------
# LOAD VARIABLES FROM NVM
# -------------------------
fsr_on = microcontroller.nvm[0] == 1
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
hstart = int(microcontroller.nvm[18])
# -------------------------
# STARTUP DEBUG PRINTS
# -------------------------
print("fsr_on:", fsr_on)
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
print("hstart:", hstart)
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
# DOUBLE HAPTIC & GREEN LED STARTUP CONFIRMATION
# -------------------------
for _ in range(2):
    ledgreen.value = False
    motor_pwm.duty_cycle = int(10 * 65535 / 100)  # 10% haptic pulse
    time.sleep(0.1)
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
    x, _, _ = sensor.acceleration
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
    fsr_value = get_voltage(fsr_in)

    if not fsr_on:
        motor_pwm.duty_cycle = 0

    if fsr_value <= step1:
        step_now = 1
        motor_pwm.duty_cycle = 0
        flashcounter += 1
    elif fsr_value <= step2:
        step_now = 2
        motor_pwm.duty_cycle = int(haptic2 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step3:
        step_now = 3
        motor_pwm.duty_cycle = int(haptic3 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step4:
        step_now = 4
        motor_pwm.duty_cycle = int(haptic4 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step5:
        step_now = 5
        motor_pwm.duty_cycle = int(haptic5 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step6:
        step_now = 6
        motor_pwm.duty_cycle = int(haptic6 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step7:
        step_now = 7
        motor_pwm.duty_cycle = int(haptic7 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step8:
        step_now = 8
        motor_pwm.duty_cycle = int(haptic8 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    elif fsr_value <= step9:
        step_now = 9
        motor_pwm.duty_cycle = int(haptic9 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1
    else:
        step_now = 10
        motor_pwm.duty_cycle = int(haptic10 * 65535 / 100) if fsr_on and step_now >= hstart else 0
        flashcounter = 1

# -------------------------
# MAIN LOOP
# -------------------------
last_transmission = time.monotonic()

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
        time.sleep(0.1)  # allow notifications to be enabled

    while ble.connected:
        check_step()
        if xangle_on:
            check_xangle()

        now = time.monotonic()
        if now - last_transmission >= 0.02:
            fsr_value = get_voltage(fsr_in)
            x, _, _ = sensor.acceleration
            uart.write(f"FSR:{fsr_value}\n")
            uart.write(f"XANGLE:{round(x, 2)}\n")
            uart.write(f"STEP:{step_now}\n")
            last_transmission = now

        if uart.in_waiting:
            data = uart.read(uart.in_waiting)
            if data:
                text = data.decode("utf-8")
                lines = text.split("\n")

            for raw in lines:
                command = raw.strip().upper()
                if not command:
                    continue

                if command == "GET_FSR_STATE":
                    uart.write("FSR ENABLED\n" if fsr_on else "FSR DISABLED\n")

                elif command == "GET_XANGLE_STATE":
                    uart.write("XANGLE ENABLED\n" if xangle_on else "XANGLE DISABLED\n")

                elif command == "TOGGLE_FSR":
                    fsr_on = not fsr_on
                    microcontroller.nvm[0] = 1 if fsr_on else 0
                    uart.write("FSR ENABLED\n" if fsr_on else "FSR DISABLED\n")

                elif command == "TOGGLE_XANGLE":
                    xangle_on = not xangle_on
                    microcontroller.nvm[14] = 1 if xangle_on else 0
                    uart.write("XANGLE ENABLED\n" if xangle_on else "XANGLE DISABLED\n")


        time.sleep(0.02)
