#ifndef MQ3_h
#define MQ3_h

#include <esp32-hal.h>
#include <math.h>

#define POWER_VOLT 3.3
#define MAX_ADC_SAMPLING 4095
class CMQ3
{
 public:
    CMQ3();
    ~CMQ3() = default;
    void Initialize(uint8_t pin, float R0, float b , float m);
    float ReadSensorVolt();
    float ReadSensorPpm(float R2_R0);
    void SetR0(float val);
    void SetRL(float val);
    void SetMAXADCSamples(uint16_t val);
    void SetAnalogPin(uint8_t pin);
    void Setm(float m);
    void Setb(float b);
    float GetR0();
    float GetRL();
    float GetSensorValue();

 private:
    uint8_t m_MQ3A0;
    float   m_R0;
    uint16_t m_MaxADCSamples;
    float   m_RL;
    float   m_sensor_volt;
    float m_b;
    float m_m;
};

#endif