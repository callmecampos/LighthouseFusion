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

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (33)

Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> lin;
imu::Quaternion quat;
const int ledPin=13; 
uint8_t sys, gyro=0, accel=0, mag = 0;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  pinMode(ledPin,OUTPUT);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("No IMU detected. Check connections.");
    while(1);
  }


  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  displaySensorDetails();
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    /* Display calibration status for each sensor. */

  Serial.println("x,\t y,\t z,\t Ex,\t Ey,\t Ez,\t qx,\t qy,\t qz,\t qw,\t gyro,\t acce,\t mag\t ");


}


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void){
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  sensors_event_t event;
  bno.getEvent(&event);
  
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //Quaternion Data
  quat = bno.getQuat();
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  if (gyro<3 || accel<3 || mag<3){
    digitalWrite(ledPin,LOW);
    while(gyro<3 || accel<3 || mag<3){
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      Serial.printf("%i, %i, %i\n",gyro,accel,mag);
      digitalWrite(ledPin,HIGH);
    }
    Serial.printf("CALIBRATED Part wait 5 seconds");
    delay(5000);
  }

    
  
//  Serial.print(F("Orientation: "));
//  Serial.print((float)event.orientation.x);
//  Serial.print(F(" "));
//  Serial.print((float)event.orientation.y);
//  Serial.print(F(" "));
//  Serial.print((float)event.orientation.z);
//  Serial.println(F(""));

  Serial.printf("% 08.3f, % 08.3f, % 08.3f,\t% 010.5f, % 010.5f, % 010.5f,\t% 015.12f, % 015.12f, % 015.12f, % 015.12f,\t %i, %i, %i\n",
    lin.x(),lin.y(),lin.z(),(float)event.orientation.x,(float)event.orientation.y, (float)event.orientation.z, quat.x(),quat.y(),
    quat.z(),quat.w(),gyro,accel,mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
