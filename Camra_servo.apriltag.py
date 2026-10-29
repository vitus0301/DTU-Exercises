from pyb import Servo
from pid import PID

import sensor
#import image
import time
#import math

sensor.reset()

sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
# Setting VFLIP and HMIRROR, rotates the image 180
sensor.set_vflip(True)
sensor.set_hmirror(True)

sensor.skip_frames(time=2000)
#sensor.set_auto_gain(False)  # must turn this off to prevent image washout...

clock = time.clock()

# Sensor center
x0 = sensor.width()/2
y0 = sensor.height()/2

# PID konstanter. Skal justeres til
Kp = 0.11
Ki = 0.0145
Kd = 0.01

tKp = 0.11
tKi = 0.0145
tKd = 0.01

servo_pan = Servo(1)  # P7
servo_tilt = Servo(2)  # P8

# Definér full range på servoerne. Tilt servoens fysiske begrænsning sættes med første parameter
print(servo_pan.calibration())
print(servo_tilt.calibration())
#servo_pan.calibration(480, 2490, 1500, 2490, 2200)
#print(servo_tilt.calibration())
#servo_tilt.calibration(1800, 2490, 1500, 2490, 2200)

servo_pan.angle(0, 500)
servo_tilt.angle(-10, 500)

pan_pid = PID(p=Kp, i=Ki, d=Kd, imax=120)
tilt_pid = PID(p=tKp, i=tKi, d=tKd, imax=120)

# Funktion for at finde det AprilTag med bedste fit
def find_best(tags):
    best_margin = 0.0
    for tag in tags:
        if tag.decision_margin > best_margin:
            best_tag = tag
            best_margin = tag.decision_margin
    return best_tag


# Skriv parametrene ud
print("Parameters pit  used: Kp=%f, Ki=%f, Kd=%f" % (Kp, Ki, Kd))
print("Parameters tilt used: tKp=%f, tKi=%f, tKd=%f" % (tKp, tKi, tKd))

while True:
    clock.tick()
    img = sensor.snapshot()
    tags = img.find_apriltags()

    if tags:
        tag = find_best(tags)
        #img.draw_rectangle(tag.rect, color=(255, 0, 0))
        #img.draw_cross(tag.cx, tag.cy)  # , color=(0, 255, 0))
        img.draw_detection(tag, color1=(255, 0, 0), color2=(0, 255, 0))

        pan_error = tag.cx - x0
        # if abs(pan_error) <= 1:
        #    pan_error = 0
        pan_output = pan_pid.get_pid(pan_error, 1)
        pan_angle = servo_pan.angle() - pan_output
        dp = int(clock.avg())
        setTime = time.ticks_ms()
        servo_pan.angle(pan_angle, dp)
        print("Time[ms]: %d, Error_pan  : %d, Output: %f, CurAng: %d, DesAng: %f" % (time.ticks_ms(), pan_error, pan_output, servo_pan.angle(), pan_angle))
        time.sleep_ms(dp)  # Vent på at servoen er i position

        tilt_error = tag.cy - y0
        # if abs(tilt_error) <= 1:
        #    tilt_error = 0
        tilt_output = tilt_pid.get_pid(tilt_error, -1)
        tilt_angle = servo_tilt.angle() - tilt_output
        dt = int(clock.avg())
        setTime = time.ticks_ms()
        servo_tilt.angle(tilt_angle, dt)
        print("Time[ms]: %d, Error_tilt : %d, Output: %f, CurAng: %d, DesAng: %f" % (time.ticks_ms(), tilt_error, tilt_output, servo_tilt.angle(), tilt_angle))
        time.sleep_ms(dt)  # Vent på at servoen er i position
