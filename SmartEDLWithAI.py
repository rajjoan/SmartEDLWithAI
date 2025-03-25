
import RPi.GPIO as GPIO
from gpiozero import Button, LED, Buzzer
import time
import math
import smbus
import joblib
from datetime import datetime

# GPIO Pins Setup
TRIG, ECHO = 23, 24
RED_LED, YELLOW_LED, GREEN_LED, BLUE_MORSE_LED_PIN = 16, 20, 21, 26
BUZZER_PIN = 18
BUTTON = Button(25)

# Initialize Components
BLUE_MORSE_LED = LED(BLUE_MORSE_LED_PIN)
BUZZER = Buzzer(BUZZER_PIN)


import csv
LOG_FILE = "edl_landing_log.csv"
with open(LOG_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "T+Time", "Altitude (cm)", "Speed (cm/s)", "Pitch (°)", "Roll (°)", "G-Force (g)", "AI Prediction"])
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([RED_LED, YELLOW_LED, GREEN_LED], GPIO.OUT)

bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

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

def get_g_force():
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    g_force = math.sqrt(Ax**2 + Ay**2 + Az**2)
    return round(g_force, 2)

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

def ai_predict(model, altitude, descent_speed, pitch, roll, g_force):
    inputs = [[altitude, descent_speed, pitch, roll, g_force]]
    prediction = model.predict(inputs)[0]
    return prediction

def activate_thrusters(duration=2):
    BUZZER.on()
    time.sleep(duration)
    BUZZER.off()

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
        print("🛠 Tilt correction in progress using directional thrusters.")
        BUZZER.on()
        time.sleep(3)
        BUZZER.off()

def landing_sequence():
    model = joblib.load("landing_model.pkl")
    safe_landing_triggered = False
    t = 0
    parachute_deployed = False
    try:
        while True:
            t += 1
            altitude = get_distance()
            descent_speed = 5 - (altitude / 40)
            pitch, roll = get_tilt_angle()
            g_force = get_g_force()
            ai_prediction = ai_predict(model, altitude, descent_speed, pitch, roll, g_force)
            print(f"T+{t}s | Altitude: {altitude}cm | AI: {ai_prediction} | Pitch: {pitch:.2f}° | Roll: {roll:.2f}° | G: {g_force}g")
            print(f"AI: {ai_prediction} | Alt: {altitude} | Speed: {descent_speed} | Pitch: {pitch} | Roll: {roll} | G: {g_force}")

            with open(LOG_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, f"T+{t}s", altitude, descent_speed, pitch, roll, g_force, ai_prediction])
            if abs(pitch) > 20 or abs(roll) > 20:
                print("Tilt Risk Detected! Excessive tilt detected: pitch or roll exceeded threshold.")
                BUZZER.on()
                time.sleep(1)
                BUZZER.off()

            if ai_prediction == "Failed":
                if abs(pitch) >= 35 or abs(roll) >= 35:
                    print("Failure due to extreme tilt")
                    correct_tilt(pitch, roll)
                elif descent_speed >= 10:
                    print("Failure due to high descent speed")
                    print("Applying maximum thruster burn to decelerate...")
                    activate_thrusters(duration=6)
                    print("High descent speed. Maximum thrust applied to avoid crash.")
                elif g_force > 2.5:
                    print("Failure due to high impact G-force")
                    print("Impact detected. G-force exceeded safe threshold. Running diagnostics.")
                    BUZZER.on()
                    time.sleep(2)
                    BUZZER.off()

            if ai_prediction == "Unstable" and not parachute_deployed:
                print("Deploying Parachute...")
                BUZZER.on()
                time.sleep(1)
                BUZZER.off()
                parachute_deployed = True
                time.sleep(5)

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

            time.sleep(1)
    except KeyboardInterrupt:
        print("\nInterrupted. Cleaning up...")
        GPIO.cleanup()

# Start
landing_sequence()