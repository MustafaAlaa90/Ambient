#ifndef _AMBIENT_H
#define _AMBIENT_H


#include "WiFi.h"
#include "esp_wps.h"
#include "SparkFun_ADXL345.h"
#include "MQUnifiedsensor.h"
#include "CO2Sensor.h"
#include "DFRobot_OzoneSensor.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <DHT.h>
#include <DHT_U.h>
#include "ThingSpeak.h"
#include "CUblox.h"
#include "sps30.h"
#include "Adafruit_ADS1X15.h"


 /* Common Defines */
#define BOARD               ("ESP32")
#define Voltage_Resolution  (3.3)
#define ADC_Bit_Resolution  (16)      // external adc
#define _PPM                (1)



/* CO Defines */
#define COTYPE              ("MQ-3")
#define CO_ADC_PIN          (0)
#define RatioCleanAIRCO     (60)
#define COVAL_A             (521853)
#define COVAL_B             (-3.821) 
#define CO_RL               (10)

/* CH4 Defines */
#define CH4TYPE             ("MQ-5")
#define CH4_ADC_PIN         (2)
#define RatioCleanAIRCH4    (6.5)
#define CH4VAL_A            (177.65)
#define CH4VAL_B            (-2.56)
#define CH4_RL              (47)

/* CO2 Defines */
#define CO2_LOW             600
#define CO2_HIGHT           1000
#define CO2VREF                3300.0
#define CO2Samples          4096
#define CO2PIN              39
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
#define TX 17
#define RX 16

/* SPS Defines */
#define SP30_COMMS SERIALPORT1
#define TX_PIN 2
#define RX_PIN 4
#define DEBUG 0

/* BME Defines */
#define SEALEVELPRESSURE_HPA (1013.25)

/* DHT22 Defines */
#define DHTPIN  27
#define DHTTYPE DHT22

/* Think Speak Defines & enum */
    /* Channel 1 Defines */
    #define SECRET_GAS_SENSOR_ID            1326711			
    #define SECRET_GAS_SENSOR_WRITE_APIKEY  "9BYJUOIV17BZQKVC"
    #define GAS_SENSOR_READING_SIZE         8

    typedef enum {
        Gas_Sensor_field_CO =1,
        Gas_Sensor_field_CO2,
        Gas_Sensor_field_CH4,
        Gas_Sensor_field_O3,
        Gas_Sensor_field_NO,
        Gas_Sensor_field_NO2,
        Gas_Sensor_field_SO2
    } gas_sensors_channel_fields;

/* Channel 2 Defines */
    #define SECRET_AIR_QUALITY_ID            1377777			
    #define SECRET_AIR_QUALITY_WRITE_APIKEY  "WGTCPR59WO2C9Y4K"
    #define AIR_QUALITY_READING_SIZE         8

    typedef enum {
        Air_Quality_field_PM1 =1,
        Air_Quality_field_PM25,
        Air_Quality_field_PM4,
        Air_Quality_field_PM10,
        Air_Quality_field_TEMP,
        Air_Quality_field_PRESSURE,
        Air_Quality_field_HUMIDITY,
        Air_Quality_field_TVOC,
    } Air_Quality_channel_fields;

class CAmbientMonitor
{
    public: 
        CAmbientMonitor();
        ~CAmbientMonitor() = default;
        bool                InitGasSensorChannel();
        bool                InitAirQualityChannel();
        bool                InitMovementChannel();

        void                ReadGasSensorChannel();
        bool                ReadAirQualityChannel();
        bool                ReadMovementChannel();
        
        bool                WriteGASSensorsChannel();
        bool                WriteAirQualityChannel();
        bool                WriteMovementChannel();
        
        bool                ConnectWIFI(const char* ssid, const char* pass );
        bool                IsWiFiConnected();

        bool                InitSDcard();
        std::string         ReadFromScCard(std::string fileDir);
        bool                WriteToSDcrd(std::string fileDir,std::string val);
        void                COInit();
        void                CH4Init();
        void                CO2Init();
        bool                O3Init();
        void                GPSInit();
        bool                BMEInit();
        void                DHTInit();
        bool                SPSInit();
        void                WPSInit();
        void                ThinkSpeakInit();
        void                SetfieldMultiple(float* fieldNRArr,uint8_t ArrSize);
        
        float               ReadCOPPM();
        float               ReadCH4PPM();
        double              ReadCO2PPM();
        float               ReadO3();
        float               ReadSoundLevel();
        void                ReadGPSInfo(double* lat,double* lng,double* meters);
        bool                ReadBME(float* pressure,float* voc);
        bool                ReadSPS(float* pm1,float* pm2,float* pm4,float* pm10);
        bool                ReadDHT(float* temp,float* hum);

    private:
        String wpspin2string(uint8_t a[]);
        MQUnifiedsensor     CO;
        MQUnifiedsensor     CH4;
        CO2Sensor           CO2;
        DFRobot_OzoneSensor O3;
        CUblox              GPS;    // UART2
        Adafruit_BME680     bme;    // I2C
        SPS30               sps30;  // UART1
        DHT_Unified         DHT;
        Adafruit_ADS1115    ads;
        WiFiClient          client;
        float               m_GasSensorChReading[GAS_SENSOR_READING_SIZE];
        float               m_AirQualitySensorChReading[AIR_QUALITY_READING_SIZE];


};



#endif