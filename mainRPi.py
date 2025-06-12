#!/usr/bin/python3
from rpi_lcd import LCD
import RPi.GPIO as GPIO
import serial as s
import time
import math
import threading
import os

anguloServo = 0
velocidadMotor = 0
temperatures = []

data_lock = threading.Lock()

def leer_serial(ser):
    global anguloServo, velocidadMotor, temperatures
    while True:
        line = ser.readline()
        if line:
            try:
                text = line.decode("utf-8").strip()
                with data_lock:
                    if text.startswith("S"):
                        duty_servo = int(text[1:])
                        anguloServo=int((180*(duty_servo-1024))/3072)
                        servo.ChangeDutyCycle(int(100*duty_servo/4096))

                    elif text.startswith("M"):
                        duty_motor = int(text[1:])
                        if duty_motor ==1024:
                            rpmMotor=0
                            motor.ChangeDutyCycle(0)
                        else:
                            rpmMotor = 1250 + (3750 / 4095) * duty_motor
                        velocidadMotor = int(rpmMotor * math.pi / 30)
                        motor.ChangeDutyCycle(int(100*duty_motor/4096))
                    else:
                        temp = float(text)
                        temperatures.append(temp)
            except ValueError:
                print(f"Error al interpretar la línea: {line}")
        else:
            time.sleep(0.01)

def guardar_temps(temps):
    with open("temps.dat", "w") as archivo:
        for temp in temps:
            archivo.write(str(temp) + "\n")

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(24,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)
motor = GPIO.PWM(18,400)
servo = GPIO.PWM(24,400)
servo.start(0)
motor.start(0)
lcd = LCD()
lcd.clear()
puerto_serie = s.Serial("/dev/serial0", 115200, timeout=1)
time.sleep(2)

serial_thread = threading.Thread(target=leer_serial, args=(puerto_serie,))
serial_thread.daemon = True
serial_thread.start()

last_temperature_dump = time.time()
try:
    while True:
        with data_lock:
            current_motor_velocity = velocidadMotor
            current_servo_angle = anguloServo
            total_temps = len(temperatures)
        
        lcd.text(f"Vel:{current_motor_velocity} RAD/s", 1)
        lcd.text(f"Servo:{current_servo_angle} grados", 2)

        if total_temps >= 1000:
            GPIO.output(26, GPIO.HIGH)
            time.sleep(0.5)
            with data_lock:
                current_temperatures = temperatures.copy()
                temperatures.clear() 
            if current_temperatures:
                guardar_temps(current_temperatures)
                #print("Temperaturas guardadas:", current_temperatures)
            last_temperature_dump = time.time()
            GPIO.output(26, GPIO.LOW)
        time.sleep(0.1)
except KeyboardInterrupt:
    os.system("pkill gnuplot_x11")
    os.system("pkill plotTemp.gp");

