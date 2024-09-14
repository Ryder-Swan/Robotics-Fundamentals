# Robotic Arm Manipulator Program


#Import Statements
import serial
import numpy as np
import time
import matplotlib
import math


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
def fwdKinematicCalc(Angles):
    
    #Define Link lengths
    a0 = 2.718
    a1 = 5.753
    a2 = 7.375
    a3 = 3.955
    
    #Define the rotation matrix for each joint
    #A0 = #Base rotation matrix
    
    #A1 - Shoulder Matrix
    A1 = [[math.cos(Angles[1]), -math.sin(Angles[1]), 0, a1*math.cos(Angles[1])],
          [math.sin(Angles[1]),  math.cos(Angles[1]), 0, a1*math.sin(Angles[1])],
          [0                  , 0                   , 1, 0                     ],
          [0                  , 0                   , 0, 1                     ]]
    
    #A2 - Elbow Matrix
    A2 = [[math.cos(Angles[2]), -math.sin(Angles[2]), 0, a2*math.cos(Angles[2])],
          [math.sin(Angles[2]),  math.cos(Angles[2]), 0, a2*math.sin(Angles[2])],
          [0                  , 0                   , 1, 0                     ],
          [0                  , 0                   , 0, 1                     ]]
    
    #Transformation matrix from the origin to the hand
    T0h = np.matmul(A1,A2)
    
    return T0h
    
#Reverse Kinematic Calculations
    
    
"""Main Loop"""

numOfPoints = 2

#Define Servo Positions
servoAngles = [[0,90,45,0,90,90],
               [-90,0,135,90,0,0]]

#Define Time Delays
timeDelays = [2,2]


#Run Functions
ser = initLynx(COMPort)

i = 0
while (i <= numOfPoints-1):
    setServoPos(servoAngles[i])
    transformationMatrix = fwdKinematicCalc(servoAngles[i])
    endPt = transformationMatrix*np.matrix('0;0;0;1')
    
    time.sleep(timeDelays[i])
    i = i + 1



#Close the Program
ser.close()