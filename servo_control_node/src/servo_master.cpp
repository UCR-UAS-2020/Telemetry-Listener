// ROS headers
#include "ros/ros.h"
// C library headers
#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// This node facilitated sending servo target commands to the maestro
// NOTE: INCOMPLETE
/*
  Sends a command to the servo controller to set a new target for a servo.
  The command is passed in as an array of bytes, each being a byte in the command.
  The port where the computer running this function connects to the servo controller is also needed.
*/
bool SetServoTarget(unsigned char command[], int port) {
  if (write(port, command, sizeof(command)) == -1) {
    ROS_INFO("Error writing");
    return false;
  }
  return true;
}

//Take the recieved message and send it. Message needs to specify a channel on the maestro and a target in microseconds
void callback(const <message type>& msg)
{
  int channel = msg->channel;
  unsigned char channel = ;
  unsigned char target = msg->target;
  unsigned char targetLower = target & 0x7F;
  unsigned char targetHigher = target >> 7 & 0x7F;
  unsigned char command[] = {0x84, channel, targetLower, targetHigher};

  bool success = SetServoTarget(command, serial_port);
  if (success) {
    ROS_INFO("Command sent");
  }
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "servo_master");
  
  //Open serial port for communication with maestro
  int serial_port = open("/dev/ttyS9", O_RDWR);
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  
  if (serial_port < 0) {
    ROS_INFO("Error %i from open: %s\n", errno, strerror(errno));
  }

  if(tcgetattr(serial_port, &tty) != 0) {
    ROS_INFO("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit
  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    ROS_INFO("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/whatever/topic/commands/are/published/on", 1000, callback);
  ros::spin();

 

/*
  char read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  if (num_bytes < 0) {
    ROS_INFO("Error reading: %s", strerror(errno));
  }
  ROS_INFO("Read %i bytes. Received message: %s", num_bytes, read_buf);
*/
  close(serial_port);
  return 0;
}