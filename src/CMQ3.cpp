#include "CMQ3.h"
//------------------------------------------------
CMQ3::CMQ3():
 m_R0(3000)
,m_MQ3A0(36)
,m_MaxADCSamples(4095)
,m_RL(10000)
,m_b(1.996459667)
,m_m(-0.754896547)
{}
//------------------------------------------------
void CMQ3::Initialize(uint8_t pin, float R0, float b , float m)
{
    SetR0(R0);
    SetAnalogPin(pin);
    Setb(b);
    Setm(m);
}
//------------------------------------------------
float CMQ3::ReadSensorVolt()
{
    float sensorValue = 0;
    float sensor_volt = 0;
    float sensorReading = 0;
    float RS = 0;
    sensorValue = analogRead(m_MQ3A0);
    if(sensorValue !=0)
    {
       m_sensor_volt = (sensorValue*POWER_VOLT)/m_MaxADCSamples;
       RS = ((POWER_VOLT - m_sensor_volt)/m_sensor_volt) * m_RL;
       sensorReading = RS/m_R0;
    }
    else
    {
        sensorReading = 0;
    }
    return sensorReading;
}
float CMQ3::ReadSensorPpm(float RS_R0)
{
   float Ppm = pow( 10,  ( ( log10( RS_R0 ) - m_b ) / m_m )  ) ;
   return Ppm;
}
//-------------------------------------------------
void CMQ3::SetR0(float val)
{
    m_R0 = val;
}
//-------------------------------------------------
void CMQ3::SetRL(float val)
{
    m_RL = val;
}
//------------------------------------------------
void CMQ3::SetMAXADCSamples(uint16_t val)
{
    m_MaxADCSamples = val;
}
//-----------------------------------------------
void CMQ3::SetAnalogPin(uint8_t pin)
{
    m_MQ3A0 = pin;
}
//--------------------------------------------
float CMQ3::GetR0()
{
    return m_R0;
}
//-------------------------------------------
float CMQ3::GetRL()
{
    return m_RL;
}
//-----------------------------------------
void CMQ3::Setb(float b)
{
    m_b = b;
}
//-----------------------------------------
void CMQ3::Setm(float m)
{
    m_m = m;
}
//------------------------------------------
float CMQ3::GetSensorValue()
{
    return m_sensor_volt;
}