
#include "CUblox.h"


CUblox::CUblox(uint8_t nr): m_serial(nr)
{

}
//--------------------------------
void CUblox::UbloxInit( unsigned long baud )
{
    m_serial.begin(baud);
}
//-----------------------------------------------
void CUblox::getInfo()
{
  char buff[55]={0,};
  int offset=0;
  while(m_serial.available()>0)
  {
      //offset = sprintf(buff+offset,"%c",m_serial.read());
      //Serial.print(m_serial.read());
    m_gps.encode(m_serial.read());
  }
  if (m_gps.location.isValid())
  {
    Serial.println(m_gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(m_gps.location.lng(), 6);
    Serial.print("Altitude: ");
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