
#include "CUblox.h"


CUblox::CUblox(uint8_t nr): m_serial(nr)
{

}
//--------------------------------
void CUblox::UbloxInit( unsigned long baud, int8_t rxPin, int8_t txPin, uint32_t config)
{
    m_serial.begin(baud,config,rxPin,txPin);
}
//-----------------------------------------------
void CUblox::getInfo(double* lat,double* lng,double* meters)
{
  while(m_serial.available()>0)
  {
    m_gps.encode(m_serial.read());
  }
  if (m_gps.location.isValid())
  {
    Serial.println(m_gps.location.lat(), 6);
    *lat = m_gps.location.lat();
    Serial.print("Longitude: ");
    Serial.println(m_gps.location.lng(), 6);
    *lng = m_gps.location.lng();
    Serial.print("Altitude: ");
    *meters = m_gps.altitude.meters();
    Serial.println(m_gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (m_gps.date.isValid())
  {
    Serial.print(m_gps.date.month());
    Serial.print("/");
    Serial.print(m_gps.date.day());
    Serial.print("/");
    Serial.println(m_gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (m_gps.time.isValid())
  {
    if (m_gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(m_gps.time.hour());
    Serial.print(":");
    if (m_gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(m_gps.time.minute());
    Serial.print(":");
    if (m_gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(m_gps.time.second());
    Serial.print(".");
    if (m_gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(m_gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}