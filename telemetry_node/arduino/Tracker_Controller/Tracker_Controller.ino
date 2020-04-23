/*  Recieves a string containing a set of coordinates in the format (<Latitude>,<Longitude>,<Altitude>),
 *   then parses the string and updates a pair of angles that determine the pitch and yaw of the antenna
 *   tracker.
 */
#include "math.h"
const byte buffSize = 40; // Defines the maximum length of a message that can be recieved 
char inputBuffer[buffSize];
const char startMarker = '(';
const char endMarker = ')';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

// Store the gps coordinates as floats before calculating angles
float lat = 0;
float lon = 0;
float alt = 0;
// Store the target angles
float horizontal_angle_D;
float elevation_angle_D;
//=============

void setup() {
  Serial.begin(9600);
  //delay(500);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

//=============

void loop() {
  GetData();
  BlinkToAck();
  UpdateTargetAngles();
}

//=============

// Recieves data on the serial line that is enclosed in parentheses, and stores it
// in inputBuffer
void GetData() {
  if(Serial.available() > 0) {

    char x = Serial.read();
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
// Separates the data stored in inputBuffer and stores each value
void parseData() {
  // Use the commas to tell where to split the string
  char * strtokIndx;
  strtokIndx = strtok(inputBuffer,",");
  lat = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  lon = atof(strtokIndx);
  strtokIndx = strtok(NULL, ","); 
  alt = atof(strtokIndx);
}

//=============
// Blink the built-in LED to show that messages are being recieved
unsigned char ledState = LOW;
void BlinkToAck() {
  if (newDataFromPC) {
    newDataFromPC = false;
    if (ledState == LOW) {
      ledState = HIGH;
    }
    else {
      ledState = LOW;
    }
    digitalWrite(13,ledState);
  }
}

//============
void UpdateTargetAngles() {
  float Tlat_D = lat; //Target latitude in degrees
  float Tlon_D = lon; //Target longitude in degrees
  float Talt = alt; //Target altitude
  float GSlat_D = -35.363262;  //Coordinates of home point in SITL simulator, change later
  float GSlon_D = 149.165237;
  float GSalt = 0.0;
  
  float Tlat_R = Tlat_D*(M_PI/180); //Target latitude in radians
  float Tlon_R = Tlon_D*(M_PI/180); //Target longitude in radians
  float GSlat_R = GSlat_D*(M_PI/180); //Ground station latitude in radians
  float GSlon_R = GSlon_D*(M_PI/180); //Ground station longitude in radians
  float DLAT_D = Tlat_D - GSlat_D;  //Difference in latitude between target and ground station (Degrees)
  float DLON_D = Tlon_D - GSlon_D;  //Difference in laongitude between target and ground station (Degrees)
  float DLAT_R = Tlat_R - GSlat_R;  //Difference in latitude between target and ground station (Radians)
  float DLON_R = Tlon_R - GSlon_R;  //Difference in longitude between target and ground station (Radians)

  // Convert separation in lat/lon from degrees to meters
    //Average of latitudes (used as reference latitude for converting decimal degrees to distance in meters)
      float lat_ave = (Tlat_R + GSlat_R)/2.0;
    //Parameters for the shape of the Earth, in meters
      float a = 6378137; //Equatorial radius
      float e = 0.0818192; //eccentricity
    //Calculate distance in meters for 1 degree of latitude at this location (use chord length instead of arc length?)
      //Arc distance for 1 degree in meters
        float d1 = (M_PI * a * (1 - pow(e,2))) / (180 * pow((1-(pow(e,2)*pow(sin(lat_ave),2))),3/2));
    //Total distance in meters (y-direction)
      float dy = fabs(d1 * DLAT_D);
    //Calculate distance in meters for 1 degree of longitude at this location
      //Arc distance for 1 degree in meters
        float d2 = (M_PI * a * cos(lat_ave)) / (180 * sqrt(1 - (pow(e,2) * pow(sin(lat_ave),2))));
    //Total distance in meters (x-direction)
      float dx = fabs(d2 * DLON_D);

  //Calculate target angle for base servo, in radians
  float horizontal_angle_R = 0;
  if (DLAT_D > 0) {
    if (DLON_D > 0) {
      horizontal_angle_R = atan(dy / dx);
    }
    else {
       horizontal_angle_R = M_PI - atan(dy / dx);
    }
  }
  else {
    if (DLON_D > 0) {
      horizontal_angle_R = 2*M_PI - atan(dy / dx);
    }
    else {
       horizontal_angle_R = M_PI + atan(dy / dx);
    }
  }
  horizontal_angle_D = horizontal_angle_R * (180/M_PI); //Convert from radians to degrees

  //Calculate euclidean distance between ground station and target
  float E_distance_in_meters = sqrt(pow(dx, 2) + pow(dy, 2));
  //Calculate target angle for tilt servo, in radians
  float elevation_angle_R = atan((Talt - GSalt) / E_distance_in_meters);
  elevation_angle_D = elevation_angle_R * (180/M_PI); //Convert from radians to degrees
}
