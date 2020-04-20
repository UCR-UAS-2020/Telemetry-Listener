/*  Recieves a string containing a set of coordinates in the format (<Latitude>,<Longitude>,<Altitude>),
 *   then parses the string and reconstructs it to echo it back (meant for testing purposes)
 */
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '(';
const char endMarker = ')';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

unsigned long curMillis;
// Store the gps coordinates as (floats/long longs) before calculating angles
float lat = 0;
float lon = 0;
float alt = 0;
//=============

void setup() {
  Serial.begin(9600);
  //delay(500);
}

//=============

void loop() {
  getDataFromPC();
  replyToPC();
}

//=============

// Recieves data on the serial line that is enclosed in parentheses, and stores it
// in inputBuffer
void getDataFromPC() {
    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {
    char x = Serial.read();

      // the order of these IF clauses is significant
      
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
// For testing purposes, sends a response over serial that matches the data recieved
void replyToPC() {
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("(");
    Serial.print(lat,6);
    Serial.print(",");
    Serial.print(lon,6);
    Serial.print(",");
    Serial.print(alt,6);
    Serial.println(")");
  }
}

//============
