#include "CAmbientMonitor.h"

CAmbientMonitor ambient;
/* WPS Varaibels */
static esp_wps_config_t config;
#define ESP_WPS_MODE      WPS_TYPE_PBC
#define ESP_MANUFACTURER  "ESPRESSIF"
#define ESP_MODEL_NUMBER  "ESP32"
#define ESP_MODEL_NAME    "ESPRESSIF IOT"
#define ESP_DEVICE_NAME   "ESP STATION"

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting AmbienMonitor");
    Serial.println("Init WPS ...");
    //-----------------------------------------------------------------------
    
    WiFi.onEvent(WiFiEvent);
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect(true);
    wpsInitConfig();
    esp_wifi_wps_enable(&config);
    WiFi.begin();
    esp_wifi_wps_start(0);
   //-------------------------------------------------------------------------
    // Serial.println("Init Gas Sensors .... ");
    // if(!IntiGasSensors())
    // {
    //     Serial.println("Couldn't initialize O3 Sensor");
    // }
    // Serial.println("Init GPS .... ");
    // ambient.GPSInit();
    // Serial.println("Init BME Sesnor .... ");
    // if(!ambient.BMEInit())
    // {
    //     Serial.println("Could not init BME Sensor ");
    // }
    // Serial.println("Init SPS Sesnor .... ");
    // if(!ambient.SPSInit())
    // {
    //     Serial.println("Could not init SPS Sensor ");
    // }
    ambient.DHTInit();
    
    


}

void loop()
{
  if(WiFi.status() != WL_CONNECTED)
  {
      esp_wifi_wps_start(0);
  }
  //-----------------------------------------
  float temp = 0,hum = 0;
  if(ambient.ReadDHT(&temp,&hum))
  {
    Serial.printf("Temp = %f , Humidity = %f\n",temp,hum);
  }
  delay(2000);
}
//-----------------------------------------------------------------------------
bool IntiGasSensors()
{
    bool Ret=true;
    ambient.CO2Init();
    ambient.COInit();
    Ret = ambient.O3Init();
    ambient.CH4Init();
    return Ret;
}
//---------------------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event, system_event_info_t info){
  switch(event){
    case SYSTEM_EVENT_STA_START:
      Serial.println("Station Mode Started");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to :" + String(WiFi.SSID()));
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from station, attempting reconnection");
      WiFi.reconnect();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WPS Successfull, stopping WPS and connecting to: " + String(WiFi.SSID()));
      esp_wifi_wps_disable();
      delay(10);
      WiFi.begin();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WPS Failed, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WPS Timedout, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WPS_PIN = " + wpspin2string(info.sta_er_pin.pin_code));
      break;
    default:
      break;
  }
}
String wpspin2string(uint8_t a[]){
  char wps_pin[9];
  for(int i=0;i<8;i++){
    wps_pin[i] = a[i];
  }
  wps_pin[8] = '\0';
  return (String)wps_pin;
}
void wpsInitConfig(){
  config.crypto_funcs = &g_wifi_default_wps_crypto_funcs;
  config.wps_type = ESP_WPS_MODE;
  strcpy(config.factory_info.manufacturer, ESP_MANUFACTURER);
  strcpy(config.factory_info.model_number, ESP_MODEL_NUMBER);
  strcpy(config.factory_info.model_name, ESP_MODEL_NAME);
  strcpy(config.factory_info.device_name, ESP_DEVICE_NAME);
}