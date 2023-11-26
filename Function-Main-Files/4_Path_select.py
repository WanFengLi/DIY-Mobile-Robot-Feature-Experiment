
#This program is developed by Han Li

#This file try to let robot choosing shorter path by itself using blob detection

#This file include head file as follows
#please ensure necessary packages are available to be called before running
import picamera
import picamera.array
import time
import cv2
import pigpio
import GP2Y
import math
import sys

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

#this counter control the state of robot
#when it is lower than 36 the robot is in detection state
#when it is higher than 36 the robot is in running state
#in detection state it will continue to reset as 0 when start point is detected
#so it will only start counting when the camera leaves the starting point
ProcessCount = 0

#This counter control the thread of robot
#It will make each pattern be detected three times before next pattern use camera
ThreadCount = 0

#these variable shows which path is detected after detection state
#if path detected return 1 ,else return 0
ShortPath = 0
LongPath = 0

#this variable shows if red star is detected to detect start and end point
StartAndEndPoint=0
#this variable shows if green oval is detected to detect path and go through maze
ShortPathGreenOval=0
#this variable shows if blue square is detected to detect path and go through maze
LongPathBlueSquare=0

#Thses lines define the parameters used in color detection
redMin = (165, 40, 80)
redMax = (179, 255, 255)
greenMin = (55, 40, 80)
greenMax = (75, 255, 255)
blueMin = (105, 40, 80)
blueMax = (135, 255, 255)

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

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #these lines call the IR sensor to measure distance and display it
    GP2Y.distcalc()
    lk = GP2Y.Distance
    print("Distance(cm): ", str.format('{0:.2f}',lk))

    #These lines Initialize the filter so that it can be used later
    #The data here are meaningless because they will only be used inside the "if condition"
    mask = cv2.inRange(hsv, redMin, redMax)
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    params = cv2.SimpleBlobDetector_Params()
    params.thresholdStep = 255
    params.minRepeatability = 1
    params.blobColor = 255
    params.filterByCircularity = True
    params.minCircularity = 0.6
    params.maxCircularity = 1
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByColor = False
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 10000000
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(masked_image)
    StartAndEndPoint = len(keypoints)

    #The first three-ninths of a detection cycle will detect red stars
    if ThreadCount is (1 or 2 or 3):
        #these lines are filters that detects the blob we are interested in
        #first use mask to filter out patterns of other colors
        mask = cv2.inRange(hsv, redMin, redMax)
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        params = cv2.SimpleBlobDetector_Params()
        #Then find the pattern we want according to the nature of the graphic
        params.thresholdStep = 255
        params.minRepeatability = 1
        params.blobColor = 255

        #Circularity is the key to detecting different patterns in this program
        params.filterByCircularity = True
        params.minCircularity = 0
        params.maxCircularity = 0.6

        #This program does't use Convexity for judgment
        #enable this parameter when more precise detection is needed
        params.filterByConvexity = False
        #params.minConvexity = 0.3
        #params.maxConvexity = 1

        params.filterByInertia = False
        params.filterByColor = False

        #These lines will filter out too small blobs because they are usually disturbing factors
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000000

        #Pass the test result information
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(masked_image)

        #store the information of blob detection
        StartAndEndPoint = len(keypoints)

    #The midle three-ninths of a detection cycle will detect blue squares
    #The detection principle is the same as the previous one, please check above if you need
    if ThreadCount is (4 or 5 or 6):
        mask = cv2.inRange(hsv, blueMin, blueMax)
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        params = cv2.SimpleBlobDetector_Params()
        params.thresholdStep = 255
        params.minRepeatability = 1
        params.blobColor = 255

        params.filterByCircularity = True
        params.minCircularity = 0.3
        params.maxCircularity = 0.8

        params.filterByConvexity = False
        #params.minConvexity = 0.3
        #params.maxConvexity = 1

        params.filterByInertia = False
        params.filterByColor = False

        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000000

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(masked_image)
        LongPathBlueSquare = len(keypoints)

    #The last three-ninths of a detection cycle will detect green ovals
    #The detection principle is the same as the previous one, please check above if you need
    if ThreadCount is (7 or 8 or 9):
        mask = cv2.inRange(hsv, greenMin, greenMax)
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        params = cv2.SimpleBlobDetector_Params()
        params.thresholdStep = 255
        params.minRepeatability = 1
        params.blobColor = 255

        params.filterByCircularity = True
        params.minCircularity = 0.6
        params.maxCircularity = 1.0

        params.filterByConvexity = False
        #params.minConvexity = 0.3
        #params.maxConvexity = 1

        params.filterByInertia = False
        params.filterByColor = False

        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000000

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(masked_image)
        ShortPathGreenOval = len(keypoints)


    #here comes to detection step
    #The robot is in detection state with ProcessCount lower than 36
    if  ProcessCount <= 36:
        #in the beginning the robot should go forward to approach start point
        if lk>18:
            if StartAndEndPoint >= 1:
                goForward()
                ProcessCount = 0
        #at the start point the robot should turn right immediately to start detection
        if lk<=18:
            if StartAndEndPoint >= 1:
                turnRight()
                ProcessCount=0
                StartAndEndPoint=0

        #if blue square is detected the robot will know the long path to go through maze
        if LongPathBlueSquare >= 1:
            LongPath = 1
            print("There is a way to go")

        #if green oval is detected the robot will know the short path to go through maze
        if ShortPathGreenOval >= 1:
            ShortPath = 1
            print("There is a shorter way")

        #these lines will set the robot rotate 90 degrees after each detection cycle
        if ProcessCount is 18:
            turnRight()
        if ProcessCount is 27:
            turnRight()
        if ProcessCount is 36:
            turnRight()
        #turn two times to adjust to the best angle to start running step
            turnRight()

    #here comes to running step
    #The robot is in running state with ProcessCount higher than 36
    elif  ProcessCount > 36:
        #Prefer short paths if it has
        if ShortPath is 1:
            #when blob is too far just approach it
            if lk>18:
                if StartAndEndPoint >= 1:
                    goForward()
                if ShortPathGreenOval >= 1:
                    goForward()
            #when certain blob is detected, run as it told
            if lk<=18:
                if StartAndEndPoint >= 1:
                    stopMotor()
                    break
                if ShortPathGreenOval >= 1:
                    turnLeft()
        #if no short path the robot will choose long path to go
        elif ShortPath is 0:
            if LongPath is 1:
                #when blob is too far just approach it
                if lk>18:
                    if StartAndEndPoint >= 1:
                        goForward()
                    if LongPathBlueSquare >= 1:
                        goForward()
                #when certain blob is detected, run as it told
                if lk<=18:
                    if StartAndEndPoint >= 1:
                        stopMotor()
                        break
                    if LongPathBlueSquare >= 1:
                        turnRight()
            #if no path detected show it in putty window
            else:
                print("I didn't find a path to go")

    #Display the value every time for verification
    ProcessCount=ProcessCount+1
    print("the ProcessCount now is ",ProcessCount)

    #The thread counter will cycle between 1-9
    ThreadCount=ThreadCount+1
    if ThreadCount is 10:
        ThreadCount=ThreadCount-9
    print("the ThreadCount now is ",ThreadCount)

    #these lines mark the detected pattern so we can validate it on screen
    color=(0,0,255)
    flags= cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    kp_image = cv2.drawKeypoints(mask,keypoints,None,color,flags)

    # Show the frame
    cv2.imshow("Frame", kp_image)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
