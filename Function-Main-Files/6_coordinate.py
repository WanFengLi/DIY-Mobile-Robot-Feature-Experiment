
#This program is developed by Han Li

#This file try to let robot return according to original path

#This program does not use odometer to record turning movements
#All it record is the parameter of straight paths desparately
#This program use predefined map as what we interested in is recording and returning
#the path from start point to end point looks like letter "c" which gives a short path to return
#The idea can be achieved only with the condition that robot turns 90 degrees every time it turns

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
AENC1=18
AENC2=19
BENC1=17
BENC2=16

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
RoadTime=0
Road1Time=0
Road2Time=0
Road3Time=0

#this variable stand for the coordinate of robot
#1 shows the robot faces the x-axis direction
#2 shows the robot faces the y-axis direction
#3 shows the robot faces the opposite direction of the x-axis
#4 shows the robot faces the opposite direction of the y-axis
#this variable will be limited in 1-4
CoordinateState=1

#these variables record the converted coordinate of each condition
CurrentCoordinateX=0
CurrentCoordinateY=0
Coordinate1X=0
Coordinate1Y=0
Coordinate2X=0
Coordinate2Y=0
CoordinateEndX=0
CoordinateEndY=0

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


#The robot moves from start point to end point in this loop
while True:
    #these lines call the IR sensor to measure distance
    GP2Y.distcalc()
    lk = GP2Y.Distance
    print("Distance(cm): ", str.format('{0:.2f}',lk))

    #this program use IR to avoid obstacles
    if(lk<=10):
        #first determine which road tvhe robot is currently on
        if ProcessCount is 1:

            #when obstacles detected record the data of count
            Road1CountLeft=LeftCount
            Road1CountRight=RightCount

            #convert data to the time
            Road1Time=(Road1CountLeft+Road1CountRight)/(2*300)
            #record the time and pass it to coordinate formula
            RoadTime=Road1Time

            #after record and convert data set the robot turning to go next
            turnRight()
            #when turn right the orientation of the robot changes
            #Turn right: +1    turn left: -1
            CoordinateState=CoordinateState+1

            #the turning and previous path will change the count on LeftCount and RightCount
            #clear up the data before we use it to record next road section
            LeftCount=0
            RightCount=0
            #change the process state to respond next road
            ProcessCount=ProcessCount+1

        #this part have the same principle with last if condition
        elif ProcessCount is 2:
            #before the calculate, record the coordinate value of the previous node
            Coordinate1X=CurrentCoordinateX
            Coordinate1Y=CurrentCoordinateY
            #this part have the same principle with last if condition
            Road2CountLeft=LeftCount
            Road2CountRight=RightCount
            Road2Time=(Road2CountLeft+Road2CountRight)/(2*300)
            RoadTime=Road2Time
            turnRight()
            CoordinateState=CoordinateState+1
            LeftCount=0
            RightCount=0
            ProcessCount=ProcessCount+1

        #When the third obstacle appearce we stop the robot at end point
        #this part have the same principle with last if condition
        elif ProcessCount is 3:

            Coordinate2X=CurrentCoordinateX
            Coordinate2Y=CurrentCoordinateY

            Road3CountLeft=LeftCount
            Road3CountRight=RightCount

            Road3Time=(Road3CountLeft+Road3CountRight)/(2*300)
            RoadTime=Road3Time
            LeftCount=0
            RightCount=0
            ProcessCount=ProcessCount+1
            #after counting convert we let robot turn 180 degrees
            #in this way it will be ready to return
            turnRight()
            CoordinateState=CoordinateState+1
            turnRight()
            CoordinateState=CoordinateState+1
            stopMotor()
            break

    #if no obstacle seen, go forward
    goForward()

    if CoordinateState>4:
        CoordinateState=CoordinateState-4
    if CoordinateState<1:
        CoordinateState=CoordinateState+4

    if CoordinateState==1:
        CurrentCoordinateX=CurrentCoordinateX+RoadTime
    elif CoordinateState==2:
        CurrentCoordinateY=CurrentCoordinateY+RoadTime
    elif CoordinateState==3:
        CurrentCoordinateX=CurrentCoordinateX-RoadTime
    elif CoordinateState==4:
        CurrentCoordinateY=CurrentCoordinateX-RoadTime

#after the loop we record the coordinate of end point
CoordinateEndX=CurrentCoordinateX
CoordinateEndY=CurrentCoordinateY

#these lines reset robot to coordinate state 1
if CoordinateState==2:
    turnLeft()
elif CoordinateState==3:
    turnLeft()
    turnLeft()
elif CoordinateState==4:
    turnRight()

#these lines let robot take x-coordinate back to the origin
if CoordinateEndX>0:
    #Adjust the direction so robot will point to the y-axis of its map
    turnRight()
    turnRight()
    goForward()
    time.sleep(CoordinateEndX)
else:
    #its in right direction now so just go forward to back
    goForward()
    time.sleep(((-1)*CoordinateEndX))

#these lines reset robot to coordinate state 1
if CoordinateState==2:
    turnLeft()
elif CoordinateState==3:
    turnLeft()
    turnLeft()
elif CoordinateState==4:
    turnRight()

#these lines let robot take y-coordinate back to the origin
if CoordinateEndY>0:
    turnLeft()
    goForward()
    time.sleep(CoordinateEndY)
else:
    turnRight()
    goForward()
    time.sleep(((-1)*CoordinateEndY))

#close Pi's GPIO
pi.write(STBY,1)
pi.stop()
