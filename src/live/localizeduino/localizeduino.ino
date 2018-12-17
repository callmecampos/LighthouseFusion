#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define INERTIAL true
#define DEBUG true

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
unsigned int ind1, check1;
boolean reading1;

void setup() {
  /* setup serial, UART, and 9DoF IMU */
  
  Serial.begin(115200); Serial1.begin(115200);

  if (INERTIAL) {
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
  }

  setupFlag();
  if (INERTIAL) {
    Serial.print("Sensor fully calibrated, ");
  }
  Serial.println("Waiting for UART communication to activate.");

  while (!Serial1)  {
    ; // wait for serial to connect
  }

  setupFlag();
  Serial.println("UART connection established between Teensies. Setup complete. Now running.");
  ind1 = 0; check1 = 0;
  reading1 = false;
}

void loop() {
  // build up poses while UART ports are available
  while (Serial1.available()) {
    readSerial1();
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
          Serial.print("6DOF:\t");
          Serial.print(myString);
          if (INERTIAL) {
            Serial.print("\t");
            readIMU(DEBUG);
          }
          Serial.println();
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

// MARK: IMU interfacing

/**/
void readIMU(boolean euler) {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  quat = bno.getQuat();
  
  sensors_event_t event;
  if (euler) {
    bno.getEvent(&event);
  }
  
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  if (euler) {
    Serial.printf("%010.5f\t%010.5f\t%010.5f\t", (float) event.orientation.x, (float) event.orientation.y, (float) event.orientation.z); // print fused orientation values
    // Serial.printf("%010.5f,%010.5f,%010.5f\t", (float) event.orientation.x - (float) (180.0/3.14159) * quat.toEuler().x(), (float) event.orientation.y - (float) (180.0/3.14159) * quat.toEuler().y(), (float) event.orientation.z - (float) (180.0/3.14159) * quat.toEuler().z());
  } else {
    Serial.printf("%015.12f\t%015.12f\t%015.12f\t%015.12f\t", quat.x(), quat.y(), quat.z(), quat.w()); // print quaternions
    Serial.printf("%i,%i,%i", gyro, accel, mag); // print calibration values
  }
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
