
#This program is developed by Han Li

#This file try to let robot avoid things by IR Sensor

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

#This line gives access to Pi's GPIO
pi=pigpio.pi()

#Use a counter to control the running time
Mycount=0

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
#Re-evaluate whether to continue this state every 0.2 seconds
def go_forward():
    pi.set_PWM_dutycycle(PWMA, 250)
    pi.set_PWM_dutycycle(PWMB, 250)

    #set the motor driving forward
    pi.write(AIN1,1)
    pi.write(AIN2,0)
    pi.write(BIN1,0)
    pi.write(BIN2,1)

    time.sleep(0.2)

#Set the robot turn left by 90 degrees
#Robot will always turn left when obstacles are seen
def turn_left():
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

#This loop set robot keep running
#you can change the value of Myaccount to control running time
while (Mycount<200):
    #these lines call the IR sensor to measure distance
    GP2Y.distcalc()
    lk = GP2Y.Distance
    print("Distance(cm): ", str.format('{0:.2f}',lk))
    #if the obstacle is in 10cm or closer, turn left
    if(lk<=10):
        turn_left()
    #if no obstacle seen, go forward
    go_forward()
    #if count over 200 times stop running
    Mycount=Mycount+1

#set the motor stop
pi.write(AIN1,1)
pi.write(AIN2,1)
pi.write(BIN1,1)
pi.write(BIN2,1)

time.sleep(1)

#close Pi's GPIO
pi.stop()
