# Robotic Arm Manipulator Program


#Import Statements
import serial
import numpy as np
import time
import matplotlib


#Define Variables
COMPort = 'COM3'


#Define Functions


#Connect to lynx motion controller
def initLynx(Port):
    connection = serial.Serial(Port, 9600, timeout=0.5)
    return connection        


#Set Servo Positions
def setServoPos(Positions):
    
    global ser
    
    
    
    baseCmd = '#0P%d\r' %(Positions[0])
    shoulderCmd = '#1P%d\r' %(Positions[1])
    elbowCmd = '#2P%d\r' %(Positions[2])
    wristCmd = '#3P%d\r' %(Positions[3])
    wristRotCmd = '#4P%d\r' %(Positions[4])
    gripperCmd = '#5P%d\r' %(Positions[5])
    
    ser.write(baseCmd.encode('utf-8'))
    ser.write(shoulderCmd.encode('utf-8'))
    ser.write(elbowCmd.encode('utf-8'))
    ser.write(wristCmd.encode('utf-8'))
    ser.write(wristRotCmd.encode('utf-8'))
    ser.write(gripperCmd.encode('utf-8'))

        
    
    
#Kinematic Calculations
    
    
#Reverse Kinematic Calculations
    
    
"""Main Loop"""

numOfPoints = 3

#Define Servo Positions
servoPositions = [[1500,1500,1500,1500,1500,1500],
                  [1000,1000,1000,1000,1000,1000],
                  [2200,2200,2200,2200,2200,2200]]

#Define Time Delays
timeDelays = [2,2,2]


#Run Functions
ser = initLynx(COMPort)

i = 0
while (i <= numOfPoints-1):
    setServoPos(servoPositions[i])
    print('Moved to Position: %d' %(i))
    time.sleep(timeDelays[i])
    i = i + 1



#Output Positions


#Close the Program
ser.close()