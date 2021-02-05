#include <CWIFI.h>

void setup()
{
    Serial.begin(115200);
    Serial.println("Testing New Library");
}

void loop()
{
    Serial.println("Inside Loop");
    CWIFI wifi;
    // WiFi.scanNetworks will return the number of networks found
    bool n = wifi.IsWIFIConnected();
    Serial.println("WIFI Status");
    if (n)
    Serial.println("Wifi is connected");
    else
    Serial.println("Wifi is disconnected");
    Serial.println("");

    // Wait a bit before scanning again
    delay(5000);
}