/*  *****************************************
 *  ADXL345_Calibration
 *  ADXL345 Hook Up Guide Calibration Example 
 *  
 *  Utilizing Sparkfun's ADXL345 Library
 *  Bildr ADXL345 source file modified to support 
 *  both I2C and SPI Communication
 *  
 *  E.Robert @ SparkFun Electronics
 *  Created: Jul 13, 2016
 *  Updated: Sep 13, 2016
 *  
 *  Development Environment Specifics:
 *  Arduino 1.6.11
 *    
 *  Hardware Specifications:
 *  SparkFun ADXL345
 *  Arduino Uno
 *  *****************************************/
 
#include <SparkFun_ADXL345.h>

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
//ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION
const float rawToG = 0.0039;
/****************** VARIABLES ******************/
/*                                             */
const float rawToG = 0.0039;
float xUp1g, xDown1g, yUp1g, yDown1g, zUp1g, zDown1g;
float xOffset, xGain, yOffset, yGain, zOffset, zGain;

// Values for one of the sensors/PCBs
const int xUp =  205;
const int xDown = -428;
const int yUp =  245;
const int yDown = -370;
const int zUp =  511;
const int zDown = 0;

/************** DEFINED VARIABLES **************/
/*                                             */
#define offsetX   -123       // OFFSET values
#define offsetY   -16
#define offsetZ   -10

#define gainX     133        // GAIN factors
#define gainY     261
#define gainZ     248 

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup()
{
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
  Serial.println();
  
  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(2);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity
                                      
  //adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
  Serial.print((int)sizeof(int));
  

xUp1g = xUp * rawToG;
xDown1g = xDown * rawToG;
yUp1g = yUp * rawToG;
yDown1g = yDown * rawToG;
zUp1g = zUp * rawToG;
zDown1g = zDown * rawToG;

xOffset = 0.5 * (xUp1g + xDown1g);
xGain = 0.5 * (xUp1g - xDown1g);
yOffset = 0.5 * (yUp1g + yDown1g);
yGain = 0.5 * (yUp1g - yDown1g);
zOffset = 0.5 * (zUp1g + zDown1g);
zGain = 0.5 * (zUp1g - zDown1g);
}

/****************** MAIN CODE ******************/
/*  Accelerometer Readings and Min/Max Values  */
void loop()
{
  Serial.println("Send any character to display values.");
  while (!Serial.available()){}       // Waiting for character to be sent to Serial
  Serial.println();
  
  // Get the Accelerometer Readings
  int x,y,z;                          // init variables hold results
  ReadAverage(&x,&y,&z);         // Read the accelerometer values and store in variables x,y,z

  float xCal = (((float)x * 0.0039 - xOffset) / xGain) * 256;
  float yCal = (((float)y * 0.0039 - yOffset) / yGain) * 256;
  float zCal = (((float)z * 0.0039 - zOffset) / zGain) * 256;

  double xyz[3];
  adxl.get_Gxyz(xyz);
  Serial.print("g: X = "); Serial.print(xyz[0]); Serial.print("  Y= ");Serial.print(xyz[1]); Serial.print("  Z= "); Serial.print(xyz[2]); Serial.println();
  /* Note: Must perform offset and gain calculations prior to seeing updated results
  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide */
  // Convert raw values to 'milli-Gs"


  Serial.print("New Calibrated Values: "); Serial.print(xCal); Serial.print("  "); Serial.print(yCal); Serial.print("  "); Serial.print(zCal);
  Serial.println(); 
  
  while (Serial.available())
  {
    Serial.read();                    // Clear buffer
  }
}

void ReadAverage(int* x,int* y,int* z)
{
  int X=0,Y=0,Z=0;
  int Xa=0,Ya=0,Za=0;
  for(int i=0;i<50;i++)
  {
    adxl.readAccel(&X, &Y, &Z);
    Xa = Xa+X;
    Ya = Ya +Y;
    Za = Za + Z;
    delay(1);
  }
  *x = Xa/50;
  *y = Ya/50;
  *z = Za/50;
  }