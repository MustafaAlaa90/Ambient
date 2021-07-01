#include <esp32-hal.h>
#include <math.h>

#define SoundSensorPin 36  //this pin read the analog voltage from the sound level meter
#define VREF  3.3  //voltage on AREF pin,default:operating voltage
#define MAX_AD_SAMPLES 4096

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    float voltageValue=0;
    float dbValue = 0;
    float voltage =0;
    voltageValue = analogRead(34);
    voltage = voltageValue / MAX_AD_SAMPLES * VREF;
    Serial.print("analog volt = ");
    Serial.println(voltage);
    dbValue = voltage * 50.0;  //convert voltage to decibel value
    Serial.print(dbValue);
    Serial.println(" dBA");
    delay(1000);
}