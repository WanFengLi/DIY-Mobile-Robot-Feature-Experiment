#Reading data from a Sharp GP2Y0E02B from a Raspi
#Written by Alexandre Gee
#WARNING: USE AT YOUR OWN RISK
#CONTAINS SPAGHETTI CODE
import smbus
import math

#Registers.
#Note a lot of these registers may
#stay dormant or not have any function (yet).
#All the registers are from the application note.
#Device address
bus = smbus.SMBus(1)            #If Pi Model A, set to 0, else keep it at 1.
GP2Y_ID = 0x40                  #Taken from i2cdetect

Hold_Bit = 0x01
Shift_Bit = 0x35
Dist1 = 0x5E                    #Distance[11:4]
Dist2 = 0x5F                    #Distance[3:0]
Dist128 = 0x01                  #Maximum display 128cm
Dist64 = 0x02                   #Maximum display 64cm
Enable = 0x00
Disable = 0x01

MEPW = 0x13                     #Maximum Emitting Pulse Width.
n1 = 0x07                       #320us, default, 26mA 
n2 = 0x06                       #240us, 22mA
n3 = 0x05                       #160us, 18mA 
n4 = 0x04                       #80us, 14ma
n5 = 0x03                       #40us, 12mA
############################################################
####################DISTANCE AND DATAREADING################
############################################################

def dataread(adr):
    return bus.read_byte_data(GP2Y_ID, adr)

def distcalc():
    global Distance
    D1 = dataread(Dist1)
    D2 = dataread(Dist2)
    SBit = dataread(Shift_Bit)
    S1proc = ((((D1 * 16) + D2) / 16) / math.pow(2, SBit))
    Distance = S1proc
    return Distance

############################################################
def datawrite(adr, data):
    bus.write_byte_data(GP2Y_ID, adr, data)
    
def maxdisplay(dis):
    if dis == 0 or 1:
        if dis == 1:
            disx = Dist128
            datawrite(Shift_Bit, disx)
        elif dis == 0:
            disx = Dist64
            datawrite(Shift_Bit, disx)
    
def pwidth(dis):
    if dis == 1 or 2 or 3 or 4 or 5:
        if dis == 1:
            nbit = n1
        elif dis == 2:
            nbit = n2
        elif dis == 3:
            nbit = n3
        elif dis == 4:
            nbit = n4
        elif dis == 5:
            nbit = n5
        datawrite(MEPW, nbit)