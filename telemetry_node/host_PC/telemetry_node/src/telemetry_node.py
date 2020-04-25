#!/usr/bin/env python
'''
***Code currently under development

Command to run:
rosrun telemetry_node telemetry_node.py [-m] [-p PORTNAME] [-b BAUD]
optional arguments:
	-m, --manual  
	Enable manual input of coordinates in place of
	subscribing to a mavros node
	
	-p PORTNAME, --portname PORTNAME
	The name of the serial port which the Arduino is
    connected to, i.e. COM1, /dev/ttyS0 (default:/dev/ttyS0)

	-b BAUD, --baud BAUD  
	The baud rate at which the Arduino is communicating;
    by default this is 9600 for the Tracker_Controller sketch

Without the manual flag, this node subscribes to 
/mavros/global_position/global and waits for telemetry
data to be published to that topic.
Upon recieveing a new set of coordinates, this node sends the
data to an arduino over serial. The arduino should be running
the code in the Tracker_Controller sketch.

Each set of coordinates is sent in the order Latitude->Longitude->Altitude,
with each value separated by a comma and the entire message surrounded in
parentheses to distinguish it as a complete set of coordinates
'''
import rospy
import serial
import time
from sensor_msgs.msg import NavSatFix
import argparse

#Parse Arguments
parser = argparse.ArgumentParser(description=("Sends sets of GPS coordinates "
												"to an Arduino."))
parser.add_argument('-m','--manual', action='store_true',
                    help=("Enable manual input of coordinates "
						"in place of subscribing to a mavros node "))
parser.add_argument('-p','--portname', type=str, default="/dev/ttyS0",
                    help=("The name of the serial port which the Arduino "
						"is connected to, i.e. COM1, /dev/ttyS0 "
						"(default: /dev/ttyS0)"))
parser.add_argument('-b','--baud', type=int, default="9600",
                    help=("The baud rate at which the Arduino is "
						"communicating; by default this is 9600 "
						"for the Tracker_Controller sketch"))
args = parser.parse_args()

# Establish serial connection
try:
    ser = serial.Serial(args.portname, args.baud)
except serial.serialutil.SerialException: 
    print("No connection on port " + arduinoPort)
    exit()
if ser.is_open:
  print("Serial connection established")

def sendCoordsToArduino(lat, lon, alt):
  stringToSend = "(" + str(lat) + "," + str(lon) + "," + str(alt) + ")"
  ser.write(stringToSend)

def telemetryCallback(data):
    sendCoordsToArduino(data.latitude, data.longitude, data.altitude)

# Try to prevent erroneous coords from being sent
def isValid(userInput):
  for x in userInput:
    if (x.isdigit() == False) and not (x == '.' or x == '-'):
      return False
  return True
  
def manualMode():
	coordStrings = ("latitude", "longitude", "altitude")
	userQuit = False

	while True:
	  print(("Enter a float value for each coordinate, or "
					"type \"exit\" to shutdown the node"))
	  userCoords = []
	  for i in range(3):
		userInput = " "
		while not isValid(userInput):
		  userInput = raw_input("Enter the UAV " + coordStrings[i] + ":")
		  if userInput == "exit":
			ser.close()
			exit()
		userCoords.append(userInput)
	  sendCoordsToArduino(userCoords[0],userCoords[1],userCoords[2])
	  
def listener():
    time.sleep(2)	# need this delay
    rospy.init_node('listener', anonymous=False)
    if not args.manual:
     rospy.loginfo("Connecting to active mavros node...")
     rospy.Subscriber("/mavros/global_position/global", NavSatFix, telemetryCallback)
    else:
     rospy.loginfo("Using manual input")
     manualMode()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

