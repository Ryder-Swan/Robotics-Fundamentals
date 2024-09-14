# Robotic Arm Manipulator Program


#Import Statements
import serial
import numpy as np
import time
import matplotlib


#Define Variables
COMPort = 'COM3'

#Servo ROMs
# Base = 0-180 deg
# Shoulder = 0-135 deg
# Elbow = 0-135 deg
# Wrist = 0-90 deg
# Wrist Rotation (WristR) = 0-180 deg
# Gripper = 0-90 deg

#Angles defined along x axis.


#Define Functions


#Connect to lynx motion controller
def initLynx(Port):
    connection = serial.Serial(Port, 9600, timeout=0.5)
    return connection        


#Set Servo Positions
def setServoPos(Angles):
    
    global ser
    
    #Convert angles to servo positions and then into serial commands
    baseCmd = '#0P%d\r' %(Angles[0]*5 + 950)
    shoulderCmd = '#1P%d\r' %(Angles[1]*-8.889 + 2000)
    elbowCmd = '#2P%d\r' %(Angles[2]*8.889 + 700)
    wristCmd = '#3P%d\r' %(Angles[3]*11.11 + 500)
    wristRotCmd = '#4P%d\r' %(Angles[4]*20 + 1800)
    gripperCmd = '#5P%d\r' %(Angles[5]*-14.4 -1300)
    
    ser.write(baseCmd.encode('utf-8'))
    ser.write(shoulderCmd.encode('utf-8'))
    ser.write(elbowCmd.encode('utf-8'))
    ser.write(wristCmd.encode('utf-8'))
    ser.write(wristRotCmd.encode('utf-8'))
    ser.write(gripperCmd.encode('utf-8'))

    
#Kinematic Calculations
    
    
#Reverse Kinematic Calculations
    
    
"""Main Loop"""

numOfPoints = 2

#Define Servo Positions
servoAngles = [[0,90,45,0,90,90],
               [-90,0,135,90,0,0]]

#Define Time Delays
timeDelays = [2,2,]


#Run Functions
ser = initLynx(COMPort)

i = 0
while (i <= numOfPoints-1):
    setServoPos(servoAngles[i])
    print('servoAngles: %d' %(i))
    time.sleep(timeDelays[i])
    i = i + 1



#Close the Program
ser.close()