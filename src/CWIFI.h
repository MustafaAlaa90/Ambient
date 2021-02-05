#ifndef CWiFi_h
#define CWiFi_h

#include "WiFi.h"

class CWIFI : public WiFiClass
{
 public:
    CWIFI() = default;
    ~CWIFI() = default;
    bool Initialize();
    bool IsWIFIConnected();
    bool IsWIFIConnected(const char* networkName);
    bool Connect(const char* networkName);
    bool Disconnect(const char* networkName);
    void GetMac(char* macAddress);
    void GetIP(char* ipAddress);
    void ScanNetworks(char** networkList);
    char* GetWIFIRRSI(char* networkName);

};

#endif