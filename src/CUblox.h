#ifndef _UBLOX_H
#define _UBLOX_H

#include <HardwareSerial.h>
#include <TinyGPS++.h>

class CUblox
{
    public:
        CUblox(uint8_t nr);
        ~CUblox() = default;
        void UbloxInit(unsigned long nr, int8_t rxPin=-1, int8_t txPin=-1,uint32_t config=SERIAL_8N1);
        void getInfo(double* lat,double* lng,double* meters);
        uint32_t getfixCount() {return m_gps.sentencesWithFix();}
    private:
    HardwareSerial m_serial;
    TinyGPSPlus    m_gps;
};










#endif