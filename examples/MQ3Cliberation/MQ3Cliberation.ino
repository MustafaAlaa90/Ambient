#include <esp32-hal.h>
#define MQ3_A0 36
#define MAX_ADC_SAMPLING 4095
#define RL 10000
void setup()
{
    Serial.begin(115200);
}
 
void loop()
{
    float sensor_volt;
    float RS_air; //  Get the value of RS via in a clear air
    float R0;  // Get the value of R0 via in Alcohol
    float sensorValue = 0;
    /*--- Get a average data by testing 100 times ---*/
    for(int x = 0 ; x < 100 ; x++)
    {
        sensorValue = sensorValue + analogRead(MQ3_A0);
    }
    sensorValue = sensorValue/100.0;
    /*-----------------------------------------------*/
    Serial.print("sensorValue = ");
    Serial.print(sensorValue);
    Serial.println();
    sensor_volt = (sensorValue*3.3)/MAX_ADC_SAMPLING;
    RS_air = ((3.3*RL)/sensor_volt)-RL; // omit *RL
    R0 = RS_air/60.0; // The ratio of RS/R0 is 60 in a clear air from Graph (Found using WebPlotDigitizer)
    
    Serial.print("sensor_volt = ");
    Serial.print(sensor_volt);
    Serial.println("V");
    Serial.print("R0 = ");
    Serial.println(R0);
    delay(1000);
 
}