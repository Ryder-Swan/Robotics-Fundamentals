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
    
    
#Kinematic Calculations
    
    
#Reverse Kinematic Calculations
    
    
"""Main Loop"""


ser = initLynx(COMPort)


#Define Waypoints


#Run Functions


#Output Positions


#Close the Program
ser.close()