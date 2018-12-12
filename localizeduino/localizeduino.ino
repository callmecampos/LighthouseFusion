#include <sstream>

const unsigned int DATA_LEN = 200;
char data1[DATA_LEN];
char data2[DATA_LEN];
unsigned int ind1, ind2, check1, check2;
boolean reading1, reading2;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200); Serial1.begin(115200); Serial2.begin(115200);

  while (!Serial1 || !Serial2)  {
    ; // wait for serial to connect
  }

  Serial.println("Let's get this bread.");
  ind1 = 0; ind2 = 0; check1 = 0; check2 = 0;
  reading1 = false; reading2 = false;
}

void loop() {
  while (Serial1.available() || Serial2.available()) {
    if (Serial1.available()) {
      readSerial1();
    }
    
    if (Serial2.available()) {
      readSerial2();
    }
  }
}

void readSerial1() {
  if (ind1 < DATA_LEN-1) {
    char inChar = Serial1.read(); // Read a character
    if (inChar == '+') {
      reading1 = true;
      ind1 = 0;
    }
  
    if (reading1) {
      data1[ind1] = inChar;
      ind1 += 1;
      if (inChar == '\n') {
        reading1 = false;
        data1[ind1] = '\0'; // null terminate that bih
        
        String myString = String(data1);
        if (check1 == 6) { // check if tracking
          // read from IMU and send over 6DoF pose
          Serial.print("Pose 1: ");
          Serial.println(myString);
        }
        reading1 = false; ind1 = 0; check1 = 0;
      } else if (inChar == '\t') { // delimiter
        check1 += 1;
      }
    }
  } else {
    Serial.println("Error: Hit end of data array 1. Try extending capacity."); reading1 = false; ind1 = 0; check1 = 0;
  }
}

void readSerial2() {
  if (ind2 < DATA_LEN-1) {
    char inChar = Serial2.read(); // Read a character
    if (inChar == '+') {
      reading2 = true;
      ind2 = 0;
    }
  
    if (reading2) {
      data2[ind2] = inChar;
      ind2 += 1;
      if (inChar == '\n') {
        reading2 = false;
        data2[ind2] = '\0'; // null terminate that bih
        
        String myString = String(data2);
        if (check2 == 6) {
          // read from IMU and send over 6DoF pose
          Serial.print("Pose 2: ");
          Serial.println(myString);
        }
        reading2 = false; ind2 = 0; check2 = 0;
      } else if (inChar == '\t') { // delimiter
        check2 += 1;
      }
    }
  } else {
    Serial.println("Error: Hit end of data array 2. Try extending capacity."); reading2 = false; ind2 = 0; check2 = 0;
  }
}
