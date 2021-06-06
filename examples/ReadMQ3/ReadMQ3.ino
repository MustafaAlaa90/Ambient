#include <CMQ3.h>

CMQ3 mq3;
void setup()
{
   
    Serial.begin(115200);
    float R0 = 3000;
    uint8_t pin = 34;
    float m = -0.754896547;
    float b = 1.996459667;
    mq3.Initialize(pin,R0,b,m);
}
 
void loop()
{
    float v = mq3.GetSensorValue();
    Serial.print("Sensor volt = ");
    Serial.print(v);
    Serial.println();
    float sensor_volt = mq3.ReadSensorVolt();
    float  sensor_Ppm = mq3.ReadSensorPpm(sensor_volt);
    /*-----------------------------------------------*/
    Serial.print("Co Rs/R0 = ");
    Serial.print(sensor_volt);
    Serial.println();
    Serial.print("Co Ppm = ");
    Serial.print(sensor_Ppm);
    Serial.println();
    delay(1000);
}