#ifndef _UBLOX_H
#define _UBLOX_H

#include <HardwareSerial.h>
#include <TinyGPS++.h>

class CUblox
{
    public:
        CUblox(uint8_t nr);
        ~CUblox() = default;
        void UbloxInit(unsigned long nr);
        void getInfo();
        uint32_t getfixCount() {return m_gps.sentencesWithFix();}
    private:
    HardwareSerial m_serial;
    TinyGPSPlus    m_gps;
};










#endif