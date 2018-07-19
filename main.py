# Untitled - By: selin - Per Tem 19 2018

# 25 x 25 square plate and ball system
# PID calculations:
# in milimeters, input will range from -250 to 250
# and output from -45 to 45. 0 deg neutral position is actually the 90 deg pos. on the 180 deg servo
# mapping the angle of the board and servo like this gives us 1/4 of a degree precision. might be subject to change
# two PID instances for the two servo's

# Camera sensor is 640x480 px, the very center is (320,240)
# try to align the center of board, or re-calibrate according to the center position

import sensor, image, time, pyb, PID

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

clock = time.clock()

# position var.s
posx = 0
posy = 0

# angle var.s
alpha = 0.0
theta = 0.0

Kp = 0.3
Ki = 0.2
Kd = 0.01
# subject to change.

pid1 = PID.PID(Kp, Ki, Kd)
pid1.SetPoint = 0
pid1.setSampleTime(0.01)

pid2 = PID.PID(Kp, Ki, Kd)
pid2.SetPoint = 0
pid2.setSampleTime(0.01)

s1 = pyb.Servo(1)   # create a servo object on position P7
s1.angle(90) # neutral position
s2 = pyb.Servo(2)   # create a servo object on position P8
s2.angle(90) # nutral position


threshold_index = 0 # 0 for red, 1 for green, 2 for blue, see below

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds

pxtomm = None  # scale for conversion


while(True):
    clock.tick()
    img = sensor.snapshot()

    alpha = s1.angle()
    theta = s2.angle()


    # ---------------
    #  image processing

    area = 0
    found = None

    for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=200, area_threshold=200, merge=True):

        if blob.area() > area
            found = blob

        if found
            x = found.cx()
            y = found.cy()

            # perform calculation
            # to convert pos. relative to center, in mm's
            #------------
            posx = (x-320) / pxtomm
            posy = (y-240) / pxtomm
            #------------

            img.draw_cross(x, y)

        else
            posx = 0
            posy = 0

    # ---------------


    #  PID control and calculations

    alphaout = pid1.update(posx)
    thetaout = pid2.update(posy)

    alpha = alphaout*2 + 90
    theta = thetaout*2 + 90

    s1.angle(alpha)
    s2.angle(theta)

    print(posx)
    print(posy)
    print(clock.fps())
