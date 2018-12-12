#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> lin;
imu::Quaternion quat;
const int ledPin=13; 
uint8_t sys, gyro=0, accel=0, mag = 0;

const unsigned int DATA_LEN = 200;
char data1[DATA_LEN];
char data2[DATA_LEN];
unsigned int ind1, ind2, check1, check2;
boolean reading1, reading2;

void setup() {
  /* setup serial, UART, and 9DoF IMU */
  
  Serial.begin(115200); Serial1.begin(115200); Serial2.begin(115200);

  pinMode(ledPin,OUTPUT);
  /* initialize the sensor */
  if(!bno.begin())
  {
    setupFlag();
    Serial.println("No IMU detected. Check connections.");
    while(1);
  }

  /* Set external Teensy crystal as clock reference for IMU. */
  bno.setExtCrystalUse(true);

  /* Calibrate IMU. */
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  if (gyro<3 || accel<3 || mag<3){
    digitalWrite(ledPin,LOW);
    while(gyro<3 || accel<3 || mag<3){
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      setupFlag();
      Serial.printf("%i\t%i\t%i\n",gyro,accel,mag);
      digitalWrite(ledPin,HIGH);
    }
    digitalWrite(ledPin, LOW);
    setupFlag();
    Serial.println("Sensor tentatively calibrated, waiting 5 seconds before continuing...");
    delay(5000);
  }

  setupFlag();
  Serial.println("Sensor fully calibrated, Waiting for UART communication to activate.");

  while (!Serial1 || !Serial2)  {
    ; // wait for serial to connect
  }

  setupFlag();
  Serial.println("UART connection established between Teensies. Setup complete. Now running.");
  ind1 = 0; ind2 = 0; check1 = 0; check2 = 0;
  reading1 = false; reading2 = false;
}

void loop() {
  // build up poses while UART ports are available
  while (Serial1.available() || Serial2.available()) {
    if (Serial1.available()) {
      readSerial1(); // back diode
    }
    
    if (Serial2.available()) {
      readSerial2(); // front diode
    }
  }
}

// MARK: UART comms

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
        ind1 -= 1; // remove endline
        data1[ind1] = '\0'; // null terminate that bih
        
        String myString = String(data1);
        if (check1 == 6) { // check if tracking
          // read from IMU and send over 6DoF pose
          Serial.print("Back:\t");
          Serial.print(myString);
          readIMU();
        }
        reading1 = false; ind1 = 0; check1 = 0; // reset params
      } else if (inChar == '\t') { // delimiter
        check1 += 1;
      }
    }
  } else {
    errorFlag();
    Serial.println("Hit end of data array 1. Try extending capacity."); reading1 = false; ind1 = 0; check1 = 0;
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
        ind2 -= 1; // remove endline
        data2[ind2] = '\0'; // null terminate that bih
        
        String myString = String(data2);
        if (check2 == 6) { // check if tracking
          // read from IMU and send over 6DoF pose
          Serial.print("Front:\t");
          Serial.println(myString);
          readIMU();
        }
        reading2 = false; ind2 = 0; check2 = 0; // reset params
      } else if (inChar == '\t') { // delimiter
        check2 += 1;
      }
    }
  } else {
    errorFlag();
    Serial.println("Hit end of data array 2. Try extending capacity."); reading2 = false; ind2 = 0; check2 = 0;
  }
}

// MARK: IMU interfacing

/**/
void readIMU() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat = bno.getQuat();
  
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.printf("%010.5f,%010.5f,%10.5f\t", lin.x(), lin.y(), lin.z()); // print linear acceleration values
  Serial.printf("%010.5f,%010.5f,%010.5f\t", (float) event.orientation.x, (float) event.orientation.y, (float) event.orientation.z); // print fused orientation values
  Serial.printf("%015.12f,%015.12f,%015.12f,%015.12f\t", quat.x(),quat.y(), quat.z(), quat.w()); // print quaternions
  Serial.printf("%i,%i,%i\n", gyro, accel, mag); // print calibration values
}

// MARK: Serial flags

void dataFlag() {
  Serial.print("Data:\n");
}

void setupFlag() {
  Serial.print("Setup:\t");
}

void errorFlag() {
  Serial.print("Error:\t");
}
