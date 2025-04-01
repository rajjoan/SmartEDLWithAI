import warnings
warnings.filterwarnings("ignore", category=UserWarning)
# Import necessary libraries for GPIO, sensors, timing, AI model, and logging
import RPi.GPIO as GPIO
from gpiozero import Button, LED, Buzzer
import time
import math
import smbus
import joblib
import csv
from datetime import datetime

# GPIO Pins Setup
TRIG, ECHO = 23, 24
RED_LED, YELLOW_LED, GREEN_LED, BLUE_MORSE_LED_PIN = 16, 20, 21, 26
BUZZER_PIN = 18
BUTTON = Button(25)

# Initialize LED and buzzer hardware
BLUE_MORSE_LED = LED(BLUE_MORSE_LED_PIN)
BUZZER = Buzzer(BUZZER_PIN)

# Create a CSV file and write headers to log sensor readings and AI predictions
LOG_FILE = "edl_landing_log.csv"
with open(LOG_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "T+Time", "Altitude (cm)", "Speed (cm/s)", "Pitch (°)", "Roll (°)", "G-Force (g)", "AI Prediction"])

# Setup GPIO pins for trigger, echo, and LED output
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([RED_LED, YELLOW_LED, GREEN_LED], GPIO.OUT)

# Setup I2C communication for MPU6050 accelerometer/gyroscope
bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

# Load trained AI model for classification
model, label_encoder = joblib.load("edl_trained_model_rf_edited.pkl")

# Function to read 16-bit raw data from MPU6050 registers
def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

# Function to calculate pitch and roll from accelerometer data
def get_tilt_angle():
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    pitch = math.degrees(math.atan2(Ax, math.sqrt(Ay**2 + Az**2)))
    roll = math.degrees(math.atan2(Ay, math.sqrt(Ax**2 + Az**2)))
    return pitch, roll

# Function to compute total G-force from accelerometer readings
def get_g_force():
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    g_force = math.sqrt(Ax**2 + Ay**2 + Az**2)
    return round(g_force, 2)

# Function to compute distance using ultrasonic sensor
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

# Function to simulate thruster activation (via buzzer)
def activate_thrusters(duration=2):
    print("Activate emergency thrusters")
    BUZZER.on()
    time.sleep(duration)
    BUZZER.off()
   

# Function to correct lander orientation using tilt data and log maneuver actions
def correct_tilt(pitch, roll):
    actions = []
    if pitch > 10:
        actions.append("Firing rear thruster to pitch forward")
    elif pitch < -10:
        actions.append("Firing front thruster to pitch backward")
    if roll > 10:
        actions.append("Firing left thruster to roll right")
    elif roll < -10:
        actions.append("Firing right thruster to roll left")
    for action in actions:
        print(action)
    if actions:
        print("Tilt correction in progress using directional thrusters.")
        BUZZER.on()
        time.sleep(3)
        BUZZER.off()

# Main landing sequence loop: reads sensor data, predicts state, logs, and makes decisions
def landing_sequence():
    # Flags and timers for mission control
    safe_landing_triggered = False
    t = 0
    parachute_deployed = False
    prev_distance = get_distance()
    prev_time = time.time()

    try:
        while True:
            t += 1
            parachute_deployed = False
            # Gather sensor data and compute descent speed
            current_distance = get_distance()
            current_time = time.time()
            time_diff = current_time - prev_time
            descent_speed = (prev_distance - current_distance) / time_diff if time_diff > 0 else 0
            descent_speed = round(descent_speed, 2)
            altitude = get_distance()
            # descent_speed = 5 - (altitude / 40)
            pitch, roll = get_tilt_angle()
            g_force = get_g_force()

            # AI prediction based on current sensor readings
            inputs = [[altitude, descent_speed, pitch, roll, g_force]]
            prediction = model.predict(inputs)[0]
            ai_prediction = label_encoder.inverse_transform([prediction])[0]

            # Log telemetry and AI output to CSV and console
            print(f"T+{t}s | Altitude: {altitude}cm | AI Prediction: {ai_prediction} | Descent Speed: {descent_speed} cm per sec | Pitch: {pitch:.2f}° | Roll: {roll:.2f}° | G: {g_force}g")
            with open(LOG_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([current_time, f"T+{t}s", altitude, descent_speed, pitch, roll, g_force, ai_prediction])
                
            # If prediction is 'Failed', determine failure type and initiate maneuvers
            if ai_prediction == "Failed":
                if abs(pitch) >= 35 or abs(roll) >= 35:
                    print("Failure due to extreme tilt")
                    correct_tilt(pitch, roll)
                    
                elif descent_speed >= 10:
                    print("Failure due to high descent speed")
                    activate_thrusters(duration=6)
                    
                elif g_force > 2.5:
                    print("Failure due to high impact G-force")
                    BUZZER.on()
                    time.sleep(2)
                    BUZZER.off()
                    

            # If prediction is 'Unstable', simulate parachute deployment
            if ai_prediction == "Unstable" and not parachute_deployed:
                print("Deploying Parachute...")
                BUZZER.on()
                time.sleep(1)
                BUZZER.off()
                parachute_deployed = True
                time.sleep(5)
           

            # Set LED indicators based on altitude to represent EDL phases
            GPIO.output([RED_LED, YELLOW_LED, GREEN_LED], False)
            if altitude > 40:
                GPIO.output(RED_LED, True)
                print("Entry Phase")
            elif 30 < altitude <= 40:
                GPIO.output(YELLOW_LED, True)
                print("Descent - Parachute Prep")
            elif 25 < altitude <= 30:
                GPIO.output(YELLOW_LED, True)
                print("Descent - Heat Shield Prep")
            elif 8 < altitude <= 25:
                GPIO.output(YELLOW_LED, True)
                print("Descent - Back Shell and Rockets")
            elif altitude < 8 and not safe_landing_triggered:
                GPIO.output(GREEN_LED, True)
                print("SAFE LANDING")
                safe_landing_triggered = True
                break
             
            prev_distance = current_distance
            prev_time = current_time
            time.sleep(1)

    # Exit on interrupt and cleanup GPIO
    except KeyboardInterrupt:
        print("\nInterrupted. Cleaning up...")
        GPIO.cleanup()

# Start the main sequence
landing_sequence()