
#This program is developed by Han Li

#This file try to let robot through a predefined maze by ArUco tags

#This file include head file as follows
#please ensure necessary packages are available to be called before running
import picamera
import picamera.array
import time
import GP2Y
import math
import sys
import cv2
from cv2 import aruco
import pigpio

# These lines define the pins
STBY=23
PWMA=20
PWMB=27
AIN1=22
AIN2=21
BIN1=24
BIN2=25

#This line gives access to Pi's GPIO
pi=pigpio.pi()

#set the GPIO mode for pins
pi.set_mode(23,pigpio.OUTPUT)
pi.set_mode(20,pigpio.OUTPUT)
pi.set_mode(27,pigpio.OUTPUT)
pi.set_mode(22,pigpio.OUTPUT)
pi.set_mode(21,pigpio.OUTPUT)
pi.set_mode(24,pigpio.OUTPUT)
pi.set_mode(25,pigpio.OUTPUT)

#turn the standby mode of IC off
pi.write(STBY,1)

#Set the robot move forward
def goForward():
    pi.set_PWM_dutycycle(PWMA, 200)
    pi.set_PWM_dutycycle(PWMB, 200)

    #set the motor driving forward
    pi.write(AIN1,1)
    pi.write(AIN2,0)
    pi.write(BIN1,0)
    pi.write(BIN2,1)

#Set the robot turn left by 90 degrees
def turnLeft():
    #Lower duty cycle make it more robust when turning left
    pi.set_PWM_dutycycle(PWMA, 120)
    pi.set_PWM_dutycycle(PWMB, 120)

    #set the motor turning left
    pi.write(AIN1,1)
    pi.write(AIN2,0)
    pi.write(BIN1,1)
    pi.write(BIN2,0)

    #this time let it turn 90 degrees with PWM dutycycle 120
    time.sleep(2.9)

#Let the robot turn left by 90 degrees
#The working principle of it is the same as function turnLeft()
def turnRight():
    pi.set_PWM_dutycycle(PWMA, 120)
    pi.set_PWM_dutycycle(PWMB, 120)

    #set the motor turning right
    pi.write(AIN1,0)
    pi.write(AIN2,1)
    pi.write(BIN1,0)
    pi.write(BIN2,1)

    time.sleep(2.9)

#Set the robot stop
def stopMotor():
    #set the motor stop
    pi.write(AIN1,1)
    pi.write(AIN2,1)
    pi.write(BIN1,1)
    pi.write(BIN2,1)

    #use time sleep to ensure it is stoped
    time.sleep(1)
    #turn the standby mode of IC off
    pi.write(STBY,1)
    #close Pi's GPIO here because the program comes to end when this function is called
    pi.stop()


# Initialise the camera and create a reference to it
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.rotation = 180
rawCapture = picamera.array.PiRGBArray(camera, size=camera.resolution)

# Allow the camera time to warm up
time.sleep(0.1)

# Setup aruco dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_parameters = aruco.DetectorParameters_create()

# Create counter for FPS
frame_count = 0
start_time = time.time()

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)

    #these lines call the IR sensor to measure distance and display it
    GP2Y.distcalc()
    lk = GP2Y.Distance
    print("Distance(cm): ", str.format('{0:.2f}',lk))

    #The allowable range of objects is 15cm to ensure IR and ArUco has sufficient time to detect
    if lk<15 :
        #This is a predefined map so robot can get the steering information through tags
        if ids==1 :
            turnLeft()
        elif ids==2 :
            turnRight()
        elif ids==3 :
            turnRight()
        elif ids==4 :
            turnLeft()
        #The foifth tag is detected means reaching the end point, stop running at here
        elif ids==5 :
            stopMotor()
            break
    #if no obstacle or Tags detected, go forward
    goForward()

    #these lines deal with the image processing work
    frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
    frame_count += 1
    average_fps = frame_count / ( time.time() - start_time )
    cv2.putText(frame_markers,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

    # Show the frame
    cv2.imshow("Frame", frame_markers)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
