/*
    ADXL345 Triple Axis Accelerometer. Free Fall Detection
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-akcelerometr-adxl345.html
    GIT: https://github.com/jarzebski/Arduino-ADXL345
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <ADXL345.h>

ADXL345 accelerometer;

void setup(void) 
{
  Serial.begin(115200);

  // Initialize ADXL345
  Serial.println("Initialize L3G4200D");

  if (!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }

  // Values for Free Fall detection
  accelerometer.setFreeFallThreshold(0.35); // Recommended 0.3 -0.6 g
  accelerometer.setFreeFallDuration(0.1);  // Recommended 0.1 s

  // Select INT 1 for get activities
  accelerometer.useInterrupt(ADXL345_INT2);

  accelerometer.setRange(ADXL345_RANGE_8G);

  // Vector offst = {13,4,-229};
  // accelerometer.setxyzoffset(offst);

// Set tap detection on Z-Axis
  accelerometer.setTapDetectionX(0);       // Don't check tap on X-Axis
  accelerometer.setTapDetectionY(0);       // Don't check tap on Y-Axis
  accelerometer.setTapDetectionZ(1);       // Check tap on Z-Axis
  // or
  // accelerometer.setTapDetectionXYZ(1);  // Check tap on X,Y,Z-Axis

  accelerometer.setTapThreshold(2.5);      // Recommended 2.5 g
  accelerometer.setTapDuration(0.02);      // Recommended 0.02 s
  accelerometer.setDoubleTapLatency(0.10); // Recommended 0.10 s
  accelerometer.setDoubleTapWindow(0.30);  // Recommended 0.30 s
  
  // Check settings
  checkSetup();
}

void checkSetup()
{
  Serial.print("Free Fall Threshold = "); Serial.println(accelerometer.getFreeFallThreshold());
  Serial.print("Free Fall Duration = "); Serial.println(accelerometer.getFreeFallDuration());
}

void loop(void) 
{
  delay(15000);

  // Read values for activities
  Vector raw = accelerometer.readRaw();
  Serial.printf("raw values x = %f , y = %f , z = %f\n",raw.XAxis,raw.YAxis,raw.ZAxis);
  Vector norm = accelerometer.readNormalize();
  Serial.printf("norm values x = %f , y = %f , z = %f\n",norm.XAxis,norm.YAxis,norm.ZAxis);

  // Read activities
  Activites activ = accelerometer.readActivites();

  if (activ.isFreeFall)
  {
    Serial.println("Free Fall Detected!");
  }
  if (activ.isDoubleTap)
  {
    Serial.println("Double Tap Detected");
  } else
  if (activ.isTap)
  {
    Serial.println("Tap Detected");
  }

  Vector filtered = accelerometer.lowPassFilter(norm, 0.5);

  // Calculate Pitch & Roll
  int pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
  int roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;

  // Calculate Pitch & Roll (Low Pass Filter)
  int fpitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  int froll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;

  // Output
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);

  // Output (filtered)
  Serial.print(" (filter)Pitch = ");
  Serial.print(fpitch);
  Serial.print(" (filter)Roll = ");
  Serial.print(froll);
  Serial.println();

  if(pitch != -10 || roll !=0)
  {
    Serial.printf("Titl detected\n");
  }
}

