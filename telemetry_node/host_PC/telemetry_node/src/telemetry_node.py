#!/usr/bin/env python
'''
***Code currently under development
Subscribes to: /mavros/global_position/global
Upon recieveing a new set of coordinates from the mavros,
sends the data to an arduino over serial. The arduino
should be running the code in the Tracker_Controller sketch.

Each set of coordinates is sent in the order Latitude->Longitude->Altitude,
with each value separated by a comma and the entire message surrounded in
parentheses to distinguish it as a complete set of coordinates
'''
import rospy
import serial
import time
from sensor_msgs.msg import NavSatFix

# Change this to reflect the serial port which the Arduino
# is connected on
arduinoPort = "/dev/ttyS0"
# Change this to reflect the baud rate which the Arduino
# is recieving on
baud = 9600
# Establish serial connection
try:
    ser = serial.Serial(arduinoPort, baud)
except serial.serialutil.SerialException: 
    rospy.loginfo("No connection on port " + arduinoPort)

def sendCoordsToArduino(lat, lon, alt):
  stringToSend = "(" + str(lat) + "," + str(lon) + "," + str(alt) + ")"
  ser.write(stringToSend)

def telemetryCallback(data):
    sendCoordsToArduino(data.latitude, data.longitude, data.altitude)
    
def listener():
    time.sleep(2)	# need this delay
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, telemetryCallback)
    if ser.is_open:
		rospy.loginfo("Connection established")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

