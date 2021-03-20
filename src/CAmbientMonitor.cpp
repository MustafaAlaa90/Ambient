#include "CAmbientMonitor.h"
//--------------------------------------------------------------------------
CAmbientMonitor::CAmbientMonitor():
 CO(BOARD,Voltage_Resolution,ADC_Bit_Resolution,COAnalogPIN,COTYPE)
,CH4(BOARD,Voltage_Resolution,ADC_Bit_Resolution,CH4AnalogPIN,CH4TYPE)
,CO2(CO2PIN,INERTIA,TRIES)
,GPS(UBlox_UART)
{}
//-------------------------------------------------------------
void CAmbientMonitor::COInit()
{
    CH4.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    CH4.setA(COVAL_A); CH4.setB(COVAL_B); // Configurate the ecuation values to get Benzene concentration
    CO.init();
    CO.setRL(CO_RL);
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      CO.update();
      calcR0 += CO.calibrate(RatioCleanAIRCO);
    }
    CO.setR0(calcR0/CO_RL);
}
//-------------------------------------------------------------
void CAmbientMonitor::COInit()
{
    CH4.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    CH4.setA(CH4VAL_A); CH4.setB(CH4VAL_B); // Configurate the ecuation values to get Benzene concentration
    CH4.init();
    CH4.setRL(CH4_RL);
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      CH4.update();
      calcR0 += CH4.calibrate(RatioCleanAIRCH4);
    }
    CH4.setR0(calcR0/CH4_RL);
}
//-------------------------------------------------------------
void CAmbientMonitor::CO2Init()
{
    CO2.calibrate();
}
//-------------------------------------------------------------
bool CAmbientMonitor::O3Init()
{
    bool Ret=true;
    uint8_t trials;
    while(!O3.begin(Ozone_IICAddress) || trials>3)
    {
        delay(1000);
        trials++;
    }
/*   Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
*/
    O3.SetModes(MEASURE_MODE_PASSIVE);
}
//------------------------------------------------------------
void CAmbientMonitor::GPSInit()
{
    GPS.UbloxInit(UBlox_baud);
}
//-------------------------------------------------------------
void CAmbientMonitor::ThinkSpeakInit()
{
    ThingSpeak.begin(client);
}
//-------------------------------------------------------------
void CAmbientMonitor::SetfieldMultiple(float* fieldNRArr,uint8_t ArrSize)
{
    for (uint8_t i=1;i<=ArrSize;i++)
    {
        ThingSpeak.setField(i,fieldNRArr[i-1]);
    }
}
//------------------------------------------------------------
void CAmbientMonitor::WriteGASSensorsChannel()
{
    ThingSpeak.writeFields(SECRET_GAS_SENSOR_ID,SECRET_GAS_SENSOR_WRITE_APIKEY);
}
//-------------------------------------------------------------

//-------------------------------------------------------------
float CAmbientMonitor::ReadCOPPM()
{
    CO.update();
    return CO.readSensor();
}
//------------------------------------------------------------
float CAmbientMonitor::ReadCH4PPM()
{
    CH4.update();
    return CH4.readSensor();
}
//-----------------------------------------------------------
double CAmbientMonitor::ReadCO2PPM()
{
    return CO2.read();
}
//-----------------------------------------------------------
int16_t CAmbientMonitor::ReadO3()
{
    return O3.ReadOzoneData(COLLECT_NUMBER);
}
//-----------------------------------------------------------
float CAmbientMonitor::ReadSoundLevel()
{
    float voltage =0;
    float voltageValue =0;
    for(int i=0;i<100;i++)
    {
        voltageValue=voltageValue+analogRead(SOUND_LEVEL_PIN);
    }
    voltage = (voltageValue / ADC_Bit_Resolution * Voltage_Resolution)/100;
    return voltage * 50.0;  //convert voltage to decibel value
}
//-------------------------------------------------------------
void CAmbientMonitor::ReadGPSInfo(double* lat,double* lng,double* meters)
{
    GPS.getInfo(lat,lng,meters);
}
//--------------------------------------------------------------
