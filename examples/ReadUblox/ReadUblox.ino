#include <CUblox.h>

#define UBlox_UART 2
#define UBlox_baud 9600
CUblox gps(UBlox_UART);

void setup()
{
    Serial.begin(115200);
    gps.UbloxInit(UBlox_baud);
}
void loop()
{
    gps.getInfo();
    Serial.println(gps.getfixCount());
    delay(1000);
}