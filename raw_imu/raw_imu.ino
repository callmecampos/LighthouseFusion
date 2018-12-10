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
#define BNO055_SAMPLERATE_DELAY_MS (1)

Adafruit_BNO055 bno = Adafruit_BNO055();


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
const int ledPin=13; //
signed long sstatex, sstatey, sstatez;
signed int accelerationx[]={0,0}, accelerationy[]={0,0}, accelerationz[]={0,0};
signed long velocityx[]={0,0}, velocityy[]={0,0}, velocityz[]={0,0};
signed long positionX[]={0,0}, positionY[]={0,0}, positionZ[]={0,0};
unsigned char countx, county, countz;
void movement_end_check(void);
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

  Serial.println("x, y, z, qx, qy, qz, qw, gyro, acce, mag ");
  //Calibration for Earth's Gravity
  // NO MOVEMENT Required
   unsigned int count1;
   count1 = 0;

   do{
  //Linear Acceleration Data
   imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   sstatex = sstatex + 100*lin.x(); // Accumulate Samples
   sstatey = sstatey + 100*lin.y();
   sstatez = sstatez + 100*lin.z();
   count1++;
   }while(count1!=0x0400); // 1024 times
   sstatex=sstatex>>10; // division between 1024
   sstatey=sstatey>>10;
   sstatez=sstatez>>10;
   digitalWrite(ledPin,HIGH);
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
  float LPFrequency =.5;
  float HPFrequency =30;
  uint8_t system, gyro, accel, mag = 0;
  unsigned char count2 ;
  count2=0;

  imu::Vector<3> lin;
  imu::Quaternion quat;
  while(gyro==0 || accel==0 || mag==0){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.printf("%i, %i, %i\n",gyro,accel,mag);
  }
  
  do{
  lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
   // Quaternion Data
  quat = bno.getQuat();
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  accelerationx[1]=accelerationx[1] + lin.x(); //filtering routine for noise attenuation
  accelerationy[1]=accelerationy[1] + lin.y(); //64 samples are averaged. The resulting 
  accelerationz[1]=accelerationz[1] + lin.z(); //filtering routine for noise attenuation
  
  
  //average represents the acceleration of
  //an instant
  count2++;
  
  }while (count2!=0x40); // 64 sums of the acceleration sample
  
  
  accelerationx[1]= accelerationx[1]>>6; // division by 64
  accelerationy[1]= accelerationy[1]>>6;
  accelerationz[1]= accelerationz[1]>>6;
  
  accelerationx[1] = accelerationx[1] - (int)sstatex; //eliminating zero reference
   //offset of the acceleration data
  accelerationy[1] = accelerationy[1] - (int)sstatey; // to obtain positive and negative
   //acceleration
  accelerationz[1] = accelerationz[1] - (int)sstatez;
  
  if ((accelerationx[1] <=3)&&(accelerationx[1] >= -3)&&(accelerationz[1] >= -3)) //Discrimination window applied
   {accelerationx[1] = 0;} // to the X axis acceleration
   //variable
  if ((accelerationy[1] <=3)&&(accelerationy[1] >= -3)&&(accelerationz[1] >= -3))
    {accelerationy[1] = 0;}
  if ((accelerationz[1] <=3)&&(accelerationx[1] >= -3)&&(accelerationy[1] >= -3)) //Discrimination window applied
    {accelerationz[1] = 0;} // to the X axis acceleration
    
   //first X integration:
  velocityx[1]= velocityx[0]+ accelerationx[0]+ ((accelerationx[1] -accelerationx[0])>>1);
   //second X integration:
  positionX[1]= positionX[0] + velocityx[0] + ((velocityx[1] - velocityx[0])>>1);
   //first Y integration:
  velocityy[1] = velocityy[0] + accelerationy[0] + ((accelerationy[1] -accelerationy[0])>>1);
   //second Y integration:
  positionY[1] = positionY[0] + velocityy[0] + ((velocityy[1] - velocityy[0])>>1);
  
     //first Z integration:
  velocityz[1] = velocityz[0] + accelerationz[0] + ((accelerationz[1] -accelerationz[0])>>1);
   //second Z integration:
  positionZ[1] = positionZ[0] + velocityz[0] + ((velocityz[1] - velocityz[0])>>1);
  
  accelerationx[0] = accelerationx[1]; //The current acceleration value must be sent
  //to the previous acceleration
  accelerationy[0] = accelerationy[1]; //variable in order to introduce the new
  //acceleration value.
  accelerationz[0] = accelerationz[1];
  
  velocityx[0] = velocityx[1]; //Same done for the velocity variable
  velocityy[0] = velocityy[1];
  velocityz[0] = velocityz[1];
  
  Serial.printf("% 08ld, % 08ld, % 08ld,\t% 015.12f, % 015.12f, % 015.12f, % 015.12f,\t %i, %i, %i\n",positionX[1],positionY[1],positionZ[1],quat.x(),quat.y(),quat.z(),quat.w(),gyro,accel,mag);



//  positionX[1] = positionX[1]<<18; //The idea behind this shifting (multiplication)
//   //is a sensibility adjustment.
//  positionY[1] = positionY[1]<<18; //Some applications require adjustments to a
//  positionZ[1] = positionZ[1]<<18; //particular situation
//   //i.e. mouse application
//  data_transfer();
//  
//  positionX[1] = positionX[1]>>18; //once the variables are sent them must return to
//  positionY[1] = positionY[1]>>18; //their original state
  movement_end_check();

  positionX[0] = positionX[1]; //actual position data must be sent to the
  positionY[0] = positionY[1]; //previous position
  positionZ[0] = positionZ[1];
  //direction = 0; // data variable to direction variable reset 

  /* Display the floating point data */

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
void movement_end_check(void)
{
  if (accelerationx[1]==0) //we count the number of acceleration samples that equals cero
  { countx++;}
  else { countx =0;}

  if (countx>=25) //if this number exceeds 25, we can assume that velocity is cero
  {
    velocityx[1]=0;
    velocityx[0]=0;
  }

  if (accelerationy[1]==0) //we do the same for the Y axis
  { county++;}
  else { county =0;}

  if (county>=25)
  {
    velocityy[1]=0;
    velocityy[0]=0;
  }
 
  if (accelerationz[1]==0) //we do the same for the Y axis
  {countz++;}
  else { countz =0;}

  if (countz>=25)
  {
    velocityz[1]=0;
    velocityz[0]=0;
  }
}
