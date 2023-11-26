
#This program is developed by Han Li

#This file try to let robot return according to original path

#This program does not use odometer to record turning movements
#All it record is the parameter of straight paths desparately
#This program use predefined map as what we interested in is recording and returning

#This file include head file as follows
#please ensure necessary packages are available to be called before running
import GP2Y
import math
import sys
import time
import cv2
import pigpio

# These lines define the pins
STBY=23
PWMA=20
PWMB=27
AIN1=22
AIN2=21
BIN1=24
BIN2=25
AENC1 = 18
AENC2 = 19
BENC1 = 17
BENC2 = 16

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

#we predefined that the map has three straight lines
#these variables pass the value of counter in each straight sections
Road1CountLeft=0
Road1CountRight=0
Road2CountLeft=0
Road2CountRight=0
Road2CountLeft=0
Road2CountRight=0
LeftCount=0
RightCount=0

#these variables record the converted times
Road1Time=0
Road2Time=0
Road3Time=0


#this vairable count is used to distinguish different road sections
ProcessCount=1

#This function will read the pin's level to get the pulse count
def pulseA(gpio, level, tick):
    LeftCount = LeftCount + 1
#this count is for another wheel
def pulseB(gpio, level, tick):
    RightCount = RightCount + 1

#This function assists the function above to complete the counting task
pi.callback(18, pigpio.RISING_EDGE, pulseA)
pi.callback(17, pigpio.RISING_EDGE, pulseB)

#turn the standby mode of IC off
pi.write(STBY,1)

#here define some functions to move
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


#in the guide of this loop robot will go through with predefined map
while True:
    #these lines call the IR sensor to measure distance
    GP2Y.distcalc()
    lk = GP2Y.Distance
    print("Distance(cm): ", str.format('{0:.2f}',lk))

    #this program use IR to avoid obstacles
    if(lk<=10):
        #first determine which road the robot is currently on
        if ProcessCount is 1:

            #when obstacles detected record the data of count
            Road1CountLeft=LeftCount
            Road1CountRight=RightCount
            Road1Time=(Road1CountLeft+Road1CountRight)/(2*300)

            #after record and convert data set the robot turning to go next
            turnRight()

            #the turning and previous path change the count on LeftCount and RightCount
            #clear up the data before we use it to record next road section
            LeftCount=0
            RightCount=0
            #change the process state to respond next road
            ProcessCount=ProcessCount+1

        #this part have the same principle with last if condition
        elif ProcessCount is 2:
            Road2CountLeft=LeftCount
            Road2CountRight=RightCount
            Road2Time=(Road2CountLeft+Road2CountRight)/(2*300)
            turnLeft()
            LeftCount=0
            RightCount=0
            ProcessCount=ProcessCount+1

        #When the third obstacle appearce we stop the robot at end point
        #this part have the same principle with last if condition
        elif ProcessCount is 3:
            Road3CountLeft=LeftCount
            Road3CountRight=RightCount
            Road3Time=(Road3CountLeft+Road3CountRight)/(2*300)
            LeftCount=0
            RightCount=0
            ProcessCount=ProcessCount+1
            #after counting convert we let robot turn 180 degrees
            #in this way it will be ready to return
            turnRight()
            turnRight()
            stopMotor()
            break

    #if no obstacle seen, go forward
    goForward()

#these lines reproduces the path to go back to start point
goForward()
time.sleep(Road3Time)
turnLeft()

goForward()
time.sleep(Road2Time)
turnRight()

goForward()
time.sleep(Road1Time)
stopMotor()

#close Pi's GPIO
pi.write(STBY,1)
pi.stop()
