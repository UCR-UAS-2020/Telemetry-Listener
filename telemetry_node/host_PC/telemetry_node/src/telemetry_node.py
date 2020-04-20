#!/usr/bin/env python
'''
***Code currently under development
Subscribes to: /mavros/global_position/global
Upon recieveing a new set of coordinates from the mavros,
sends the data to an arduino over serial. The arduino
should be running the code in the Tracker_Controller_test sketch.

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
ser = serial.Serial(arduinoPort, baud)
#try:
#    ser = serial.Serial(arduinoPort, baud)
#    rospy.loginfo("Connection established")
#except serial.serialutil.SerialException: 
#    rospy.loginfo("No connection on port " + arduinoPort)

def sendCoordsToArduino(lat, lon, alt):
  stringToSend = "(" + str(lat) + "," + str(lon) + "," + str(alt) + ")"
  testData = []
  testData.append(bytearray(stringToSend))
  runTest(testData)
    #ser.write(bytearray(stringToSend))
    #rospy.loginfo("Sent from PC -- " + stringToSend)
ck = "---"
def recvFromArduino():  
  rospy.loginfo("entered recvFromArduino()")
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != 40: 
    x = ser.read()
  
  # save data until the end marker is found
  while ord(x) != 41:
    if ord(x) != 40:
      ck = ck + str(x).replace("b","")
      byteCount += 1
    x = ser.read()
  
  return(ck)

def runTest(td):
  numLoops = len(td)
  waitingForReply = False

  n = 0
  while n < numLoops:

    teststr = td[n]

    if waitingForReply == False:
      ser.write(teststr)
      rospy.loginfo("Sent from PC -- " + teststr)
      waitingForReply = True

    if waitingForReply == True:

      while ser.inWaiting() == 0:
        #rospy.loginfo("waiting for reply")
        pass
      
      #rospy.loginfo("serial data available")
      dataRecvd = recvFromArduino()
      rospy.loginfo("Reply Received  " + dataRecvd)
      n += 1
      waitingForReply = False

      print("===========")

    time.sleep(5)

def telemetryCallback(data):
    sendCoordsToArduino(data.latitude, data.longitude, data.altitude)
    #rospy.loginfo("Reply Received  " + dataRecvd)
    
def listener():
    try:
        ser = serial.Serial(arduinoPort, baud)
        rospy.loginfo("Connection established")
    except serial.serialutil.SerialException: 
        rospy.loginfo("No connection on port " + arduinoPort)
    time.sleep(2)	# need this delay
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, telemetryCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

