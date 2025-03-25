import RPi.GPIO as GPIO
from gpiozero import Button, LED, Buzzer
import time
import csv
import math
import smbus
from datetime import datetime
import joblib
import pandas as pd

# GPIO Pins Setup
TRIG, ECHO = 23, 24
RED_LED, YELLOW_LED, GREEN_LED, BLUE_MORSE_LED_PIN = 16, 20, 21, 26
BUZZER_PIN = 18
BUTTON = Button(25)

# Initialize Components
BLUE_MORSE_LED = LED(BLUE_MORSE_LED_PIN)
BUZZER = Buzzer(BUZZER_PIN)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([RED_LED, YELLOW_LED, GREEN_LED], GPIO.OUT)

# MPU6050 I2C Setup
bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

# Load trained model and label encoder
model, label_encoder = joblib.load("edl_trained_model_rf.pkl")


# Read raw 16-bit data from MPU register
def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value


# Compute pitch and roll angles
def get_tilt_angle():
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    pitch = math.degrees(math.atan2(Ax, math.sqrt(Ay ** 2 + Az ** 2)))
    roll = math.degrees(math.atan2(Ay, math.sqrt(Ax ** 2 + Az ** 2)))
    return pitch, roll


# Compute G-Force from accelerometer
def get_g_force():
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    g_force = math.sqrt(Ax ** 2 + Ay ** 2 + Az ** 2)
    return round(g_force, 2)


# Get altitude from ultrasonic sensor
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    end_time = time.time()
    while GPIO.input(ECHO) == 1:
        end_time = time.time()
    distance = (end_time - start_time) * 34300 / 2
    return round(distance, 2)


MORSE_CODE_DICT = {
    'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.',
    'F': '..-.', 'G': '--.', 'H': '....', 'I': '..', 'J': '.---',
    'K': '-.-', 'L': '.-..', 'M': '--', 'N': '-.', 'O': '---',
    'P': '.--.', 'Q': '--.-', 'R': '.-.', 'S': '...', 'T': '-',
    'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-', 'Y': '-.--',
    'Z': '--..', '1': '.----', '2': '..---', '3': '...--', '4': '....-',
    '5': '.....', '6': '-....', '7': '--...', '8': '---..', '9': '----.', '0': '-----'
}


def send_morse_code(message, output_device):
    for char in message.upper():
        if char in MORSE_CODE_DICT:
            morse_sequence = MORSE_CODE_DICT[char]
            for symbol in morse_sequence:
                if symbol == ".":
                    output_device.on()
                    time.sleep(0.1)
                elif symbol == "-":
                    output_device.on()
                    time.sleep(0.5)

                output_device.off()
                time.sleep(0.1)
            time.sleep(0.5)
        else:
            time.sleep(1)


# Main EDL monitoring and prediction loop
def landing_sequence():
    safe_landing_triggered = False
    prev_distance = get_distance()
    prev_time = time.time()
    t = 0

    print("Timestamp | T+Time | Altitude (cm) | Speed (cm/s) | Pitch (°) | Roll (°) | G-Force (g) | Prediction")

    try:
        while True:
            t += 1
            parachute_deployed = False
            current_distance = get_distance()
            current_time = time.time()
            time_diff = current_time - prev_time
            descent_speed = (prev_distance - current_distance) / time_diff if time_diff > 0 else 0

            pitch, roll = get_tilt_angle()
            g_force = get_g_force()
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Create input for model prediction
            input_features = pd.DataFrame(
                [[current_distance, descent_speed, pitch, roll, g_force]],
                columns=["Altitude (cm)", "Speed (cm/s)", "Pitch (°)", "Roll (°)", "G-Force (g)"]
            )
            prediction_encoded = model.predict(input_features)[0]
            prediction_label = label_encoder.inverse_transform([prediction_encoded])[0]

            print(f"{timestamp} | T+{t}s | {current_distance:.2f} | {descent_speed:.2f} | "
                  f"{pitch:.2f} | {roll:.2f} | {g_force:.2f} | {prediction_label}")

            # Tilty Warning
            if prediction_label in [
                "Unstable Roll - Adjust the Roll for stable landing",
                "Unstable Pitch - Adjust the Pitch for stable landing"
            ]:
                print("Tilt Risk Detected!")
                print("Excessive tilt detected: pitch or roll exceeded threshold.")
                BUZZER.on()
                time.sleep(1)
                BUZZER.off()

            # AI Prediction display logic combined with LED status
            if prediction_label != "Stable":
                print("AI Assessment: Critical landing failure detected. Immediate action required. Initiating Entry Phase Emergency Protocol.")
                GPIO.output(RED_LED, True)
            elif prediction_label == "Stable" and current_distance < 8:
                print("AI Assessment: Descent stable and within landing parameters. Touchdown confirmed.")
                GPIO.output(GREEN_LED, True)

            if current_distance < 10:
                if g_force < 0.2:
                    print("Free fall detected close to ground!")
                    print("Free fall detected below 10cm altitude.")
                elif g_force > 2.0:
                    print("Impact detected near surface! Sudden deceleration suggests a hard landing. Assess structural integrity and initiate emergency diagnostics.")
                    print("High G-force impact detected below 10cm altitude.")
                    BUZZER.on()
                    time.sleep(1)
                    BUZZER.off()

            if prediction_label != "Stable" and not parachute_deployed:
                print("Deploying Parachute...")
                BUZZER.on()
                time.sleep(1)
                BUZZER.off()
                parachute_deployed = True
                time.sleep(2)

            # EDL phase control
            GPIO.output([RED_LED, YELLOW_LED, GREEN_LED], False)
            if current_distance > 40:
                GPIO.output(RED_LED, True)
                print(f"[{timestamp}] Entry - (Too High)")
            elif 30 < current_distance <= 40:
                GPIO.output(YELLOW_LED, True)
                print(f"[{timestamp}] Descent (Parachute Deploy Zone)")
            elif 25 < current_distance <= 30:
                GPIO.output(YELLOW_LED, True)
                print(f"[{timestamp}] Descent (Heat Shield Separation Zone)")
            elif 8 < current_distance <= 25:
                GPIO.output(YELLOW_LED, True)
                print(f"[{timestamp}] Powered Descent - FIRE THE ROCKETS")
            elif current_distance <= 8.5 and not safe_landing_triggered:
                GPIO.output(GREEN_LED, True)
                print(f"[{timestamp}] SAFE LANDING")
                send_morse_code("SAFE LANDING", BLUE_MORSE_LED)
                safe_landing_triggered = True
                break

            prev_distance = current_distance
            prev_time = current_time
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTelemetry interrupted. Cleaning up GPIO...")
        GPIO.cleanup()


# Run the sequence
landing_sequence()


