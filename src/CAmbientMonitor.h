#ifndef _AMBIENT_H
#define _AMBIENT_H


#include "WiFi.h"
#include "esp_wps.h"
//#include "SparkFun_ADXL345.h"
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
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "ADXL345.h"
#include "ESP32Time.h"
#include "algorithm"
#include "ULP.h"



 /* Common Defines */
#define BOARD               ("ESP32")
#define Voltage_Resolution  3.3F
#define ADC_Bit_Resolution  32768.0F      // external adc
#define ADC_Bit_Resolution_ESP 4096  // of esp32
#define _PPM                1
#define PowrPIN             36
#define DATE_TIME_SIZE      30

/* Butons Defines */
#define Connect_WIFI_Pin       25
#define Start_Stop_Reading_Pin 35
#define Reset_Pin              26

/* LEDs Defines*/
#define WWIFI_LED              33
#define Start_Stop_LED         32

/* CO Defines */
#define COTYPE              ("MQ-3")
#define CO_ADC_PIN          (0)
#define RatioCleanAIRCO     60.0F
#define COVAL_A             521853.0F
#define COVAL_B             -3.821F 
#define CO_RL               100.0F
#define CO_R0               /*(87377.331250)*/ 1.823227F  // from calibration process

/* CH4 Defines */
#define CH4TYPE             ("MQ-5")
#define CH4_ADC_PIN         (2)
#define RatioCleanAIRCH4    6.5F
#define CH4VAL_A            177.65F
#define CH4VAL_B            -2.56F
#define CH4_RL              20.0F
#define CH4_R0              10.744135F // from calibration process

/* CO2 Defines */
#define CO2_LOW             600.0F
#define CO2_HIGHT           1000.0F
#define CO2VREF             3300.0F
#define CO2Samples          4096
#define CO2PIN              39
#define INERTIA             0.99
#define TRIES               3           

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

/* SO2 Sensor defines */
#define        SO2_ADC_PIN    3          // channel number of ads driver 
#define        SF             0.005      // snesitivity code 

/* Sound Level Defines */
#define           SOUND_LEVEL_PIN           34

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
        Gas_Sensor_field_SO2,
        Gas_Sensor_field_power_monitor
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

    /* Channel 3 Defines */
    #define SECRET_MOVEMENT_ID            1377787			
    #define SECRET_MOVEMENT_WRITE_APIKEY  "YQ7GJ56QS9BWYYD0"
    #define MOVEMENT_READING_SIZE         8

    typedef enum {
        Movement_field_Longitude =1,
        Movement_field_Latitude,
        Movement_field_Altitude,
        Movement_field_Tap,
        Movement_field_FreeFall,
        Movement_field_Titl,
        Movement_field_Sound_Level,
        Movement_field_wifi_signal
    } Movement_channel_fields;

class CAmbientMonitor
{
    public: 
        CAmbientMonitor();
        ~CAmbientMonitor() = default;
        bool                InitGasSensorChannel();
        bool                InitAirQualityChannel();
        void                InitMovementChannel();
        void                InitButtons();
        void                InitLEDs();

        void                ReadGasSensorChannel();
        bool                ReadAirQualityChannel();
        bool                ReadMovementChannel();
        
        bool                WriteGASSensorsChannel();
        bool                WriteAirQualityChannel();
        bool                WriteMovementChannel();
        
        bool                ConnectWIFI(const char* ssid, const char* pass );
        bool                DisconnectConnectWIFI();
        bool                IsWiFiConnected();

        bool                InitSDcard();
        bool                ReadFromSDCard(fs::FS &fs, const char * path);
        void                WriteToSDCard(fs::FS &fs, const char * path, const char * message);
        void                WriteLog();

        void                WriteGasSesnorsLog();
        void                WriteAirQualityLog();
        void                WriteMovementLog();
        void                COInit();
        void                CH4Init();
        void                CO2Init();
        bool                O3Init();
        void                SO2Init();
        void                GPSInit();
        bool                BMEInit();
        void                DHTInit();
        bool                SPSInit();
        void                WPSInit();

        void                InitADXL();
        void                ThinkSpeakInit();
        void                SetfieldMultiple(float* fieldNRArr,uint8_t ArrSize);
        void                SetfieldMultiple(double* fieldNRArr,uint8_t ArrSize);
        
        float               ReadCOPPM();
        float               ReadCH4PPM();
        int                 ReadCO2PPM();
        float               ReadO3();
        float               ReadSO2();
        float               ReadSoundLevel();
        void                ReadGPSInfo(double* lat,double* lng,double* meters);
        bool                ReadBME(float* pressure,float* voc);
        bool                ReadSPS(float* pm1,float* pm2,float* pm4,float* pm10);
        bool                ReadDHT(float* temp,float* hum);
        float               ReadPowerPin();
        float               ReadWIFISignal();
        void                ReadADXL(float* tap,float* tilt,float* freefall);
        void                ReadDateTime();
        void                CalibrateGasSensors();
        
        void                setNTPTime(bool val);

    private:
        String wpspin2string(uint8_t a[]);
        MQUnifiedsensor     CO;
        MQUnifiedsensor     CH4;
        CO2Sensor           CO2;
        DFRobot_OzoneSensor O3;
        SO2                 so2;
        CUblox              GPS;    // UART2
        Adafruit_BME680     bme;    // I2C
        SPS30               sps30;  // UART1
        DHT_Unified         DHT;
        Adafruit_ADS1115    ads;
        WiFiClient          client;
        ADXL345             accelerometer;
        float               m_GasSensorChReading[GAS_SENSOR_READING_SIZE];
        float               m_AirQualitySensorChReading[AIR_QUALITY_READING_SIZE];
        double              m_MovementSensorChReading[MOVEMENT_READING_SIZE];
        String              m_date;
        String              m_time;
        String              m_dateTimeRTC;
        uint32_t            m_size;
        bool                m_NTPtime;
        //ESP32Time           esptime;


};



#endif