#ifndef _AMBIENT_H
#define _AMBIENT_H

#include "WiFi.h"
#include "ADXL345.h"
#include "MQUnifiedsensor.h"
#include "CO2Sensor.h"
#include "DFRobot_OzoneSensor.h"
#include "ADXL345.h"
#include "bme680.h"
#include "ThingSpeak.h"
#include "CUblox.h"
#include "sps30.h"

 /* Common Defines */
#define BOARD               ("ESP32")
#define Voltage_Resolution  (3.3)
#define ADC_Bit_Resolution  (12)
#define _PPM                (1)

/* CO Defines */
#define COTYPE              ("MQ-3")
#define COAnalogPIN         (36)
#define RatioCleanAIRCO     (60)
#define COVAL_A             (521853)
#define COVAL_B             (-3.821) 
#define CO_RL               (10)

/* CH4 Defines */
#define CH4TYPE             ("MQ-5")
#define CH4AnalogPIN        (35)
#define RatioCleanAIRCH4    (6.5)
#define CH4VAL_A            (177.65)
#define CH4VAL_B            (-2.56)
#define CH4_RL              (47)

/* CO2 Defines */
#define CO2_LOW             600
#define CO2_HIGHT           1000
#define VREF                3300.0
#define Samples             4096.0
#define CO2PIN              34
#define INERTIA             0.99
#define TRIES               100           

/* Ozone Defines */
#define           ADDRESS_0                 0x70           // iic slave Address
#define           ADDRESS_1                 0x71
#define           ADDRESS_2                 0x72
#define           ADDRESS_3                 0x73
#define           MEASURE_MODE_AUTOMATIC    0x00           // active  mode
#define           MEASURE_MODE_PASSIVE      0x01           // passive mode
#define           AUTO_READ_DATA            0x00           // auto read ozone data
#define           PASSIVE_READ_DATA         0x01           // passive read ozone data
#define           MODE_REGISTER             0x03           // mode register
#define           SET_PASSIVE_REGISTER      0x04           // read ozone data register
#define           AUTO_DATA_HIGE_REGISTER   0x09           // AUTO data high eight bits
#define           AUTO_DATA_LOW_REGISTER    0x0A           // AUTO data Low  eight bits
#define           PASS_DATA_HIGE_REGISTER   0x07           // AUTO data high eight bits
#define           PASS_DATA_LOW_REGISTER    0x08           // AUTO data Low  eight bits
#define           OCOUNT                    100            // Ozone Count Value
#define           COLLECT_NUMBER            20              // collect number, the collection range is 1-100
#define           Ozone_IICAddress          ADDRESS_3

/* Sound Level Defines */
#define           SOUND_LEVEL_PIN           33

/* UBlox Defines */
#define UBlox_UART 2
#define UBlox_baud 9600

/* Think Speak Defines & enum */
    /* Channel 1 Defines */
    #define SECRET_GAS_SENSOR_ID            1326711			
    #define SECRET_GAS_SENSOR_WRITE_APIKEY  "9BYJUOIV17BZQKVC"
    typedef enum {
        Gas_Sensor_field_CO =1,
        Gas_Sensor_field_CO2,
        Gas_Sensor_field_CH4,
        Gas_Sensor_field_O3,
        Gas_Sensor_field_NO,
        Gas_Sensor_field_NO2,
        Gas_Sensor_field_SO2
    } gas_sensors_channel_fields;


class CAmbientMonitor
{
    public: 
        CAmbientMonitor();
        ~CAmbientMonitor();
        void                COInit();
        void                CH4Inti();
        void                CO2Init();
        bool                O3Init();
        void                GPSInit();
        void                ThinkSpeakInit();
        void                SetfieldMultiple(float* fieldNRArr,uint8_t ArrSize);
        void                WriteGASSensorsChannel();
        float               ReadCOPPM();
        float               ReadCH4PPM();
        double              ReadCO2PPM();
        int16_t             ReadO3();
        float               ReadSoundLevel();
        void                ReadGPSInfo(double* lat,double* lng,double* meters);
    private:
        MQUnifiedsensor     CO;
        MQUnifiedsensor     CH4;
        CO2Sensor           CO2;
        DFRobot_OzoneSensor O3;
        CUblox              GPS;
        WiFiClient          client;

};



#endif