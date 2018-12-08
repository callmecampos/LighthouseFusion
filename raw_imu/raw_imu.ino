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

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("No IMU detected. Check connections.");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  Serial.println("x, y, z, qx, qy, qz, qw, gyro, acce, mag ");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  uint8_t system, gyro, accel, mag = 0;
  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* Display the floating point data */
  //Serial.print("X: ");
  Serial.printf("% 08.3f, % 08.3f, % 08.3f\t% 015.12f, % 015.12f, % 015.12f, % 015.12f,\t %i, %i, %i\n",lin.x(),lin.y(),lin.z(),quat.x(),quat.y(),quat.z(),quat.w(),gyro,accel,mag);
//  Serial.print("X: ");
//  Serial.print(lin.x());
//  Serial.print(" Y: ");
//  Serial.print(lin.y());
//  Serial.print(" Z: ");
//  Serial.print(lin.z());
//  Serial.print("\t\t");


//  //Serial.print("qW: ");
//  Serial.print(quat.w(), 4);
// // Serial.print(" qX: ");
//  Serial.print(quat.y(), 4);
//  //Serial.print(" qY: ");
//  Serial.print(quat.x(), 4);
//  //Serial.print(" qZ: ");
//  Serial.print(quat.z(), 4);
//  Serial.print("\t\t");


 // bno.getCalibration(&system, &gyro, &accel, &mag);
  //Serial.print("CALIBRATION: Sys=");
 // Serial.print(system, DEC);
  //Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  //Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  //Serial.print(" Mag=");
//  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
