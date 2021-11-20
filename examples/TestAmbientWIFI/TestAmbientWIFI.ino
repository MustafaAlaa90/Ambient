#include "CAmbientMonitor.h"

#define Calibrate false
static CAmbientMonitor ambient;
static bool connect_state = true;
static bool start_stop_state = true;
static int key_pessed_wifi_counter = 0;
static int key_pessed_start_stop_counter = 0;
bool wifi_connected = false;
/* WPS Varaibels */
static esp_wps_config_t config;
#define ESP_WPS_MODE      WPS_TYPE_PBC
#define ESP_MANUFACTURER  "ESPRESSIF"
#define ESP_MODEL_NUMBER  "ESP32"
#define ESP_MODEL_NAME    "ESPRESSIF IOT"
#define ESP_DEVICE_NAME   "ESP STATION"

 void isr_WIFI()
 {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 300) 
    {
      key_pessed_wifi_counter += 1;
      connect_state = !connect_state;
      Serial.printf("Button wifi has been pressed %u times, connect_state = %d\n", key_pessed_wifi_counter,connect_state);
    }
    last_interrupt_time = interrupt_time;
  }
  void isr_Start_Stop()
  {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 300) 
    {
      key_pessed_start_stop_counter += 1;
      start_stop_state = !start_stop_state;
      Serial.printf("Button start_stop has been pressed %u times, start_stop_state = %d\n", key_pessed_start_stop_counter,start_stop_state);
      digitalWrite(Start_Stop_LED,start_stop_state);
    }
    last_interrupt_time = interrupt_time;
  }
  void isr_Reset()
  {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 300) 
    {
      key_pessed_start_stop_counter += 1;
      //start_stop_state = !start_stop_state;
      Serial.printf("Button reset has been pressed %u times, reseting now\n", key_pessed_start_stop_counter,start_stop_state);
      ESP.restart();
    }
    last_interrupt_time = interrupt_time;
  }

void setup()
{
    delay(2000);
    Serial.begin(115200);
    Serial.println("Starting AmbienMonitor");
    //-----------------------------------------------------------------------
    ambient.InitButtons();
    ambient.InitLEDs();
    digitalWrite(WWIFI_LED,false);
    digitalWrite(Start_Stop_LED,start_stop_state);
    attachInterrupt(digitalPinToInterrupt(Connect_WIFI_Pin), isr_WIFI,  FALLING);
    attachInterrupt(digitalPinToInterrupt(Start_Stop_Reading_Pin), isr_Start_Stop, FALLING);
    attachInterrupt(digitalPinToInterrupt(Reset_Pin), isr_Reset, FALLING);
    //-----------------------------------------------------------------------
    Serial.println("Init WPS ...");
    WiFi.onEvent(WiFiEvent);
    WiFi.mode(WIFI_MODE_STA);
    wpsInitConfig();
    //esp_wifi_wps_start(0);
   //-------------------------------------------------------------------------
   Serial.printf("Init thingspeak client ...\n");
   ambient.ThinkSpeakInit();
   //-------------------------------------------------------------------------
   Serial.printf("Waiting 3 mintutes for Gas Sensor to heat up ....\n");
   bool LED = false;
   int i =0;
   while (i<180)
   {
      digitalWrite(Start_Stop_LED,LED);
      LED=!LED;
      Serial.printf("waiting for gas sensors heat up\n");
      delay(1000);
      i++;
   }
   digitalWrite(Start_Stop_LED,true);
   Serial.println("Init Gas Sensors channel .... \n");
   if(!ambient.InitGasSensorChannel())
   {
    Serial.println("Couldn't initialize O3 Sensor\n");
   }
   Serial.println("Init Air Quality channel .... \n");
   if(!ambient.InitAirQualityChannel())
   {
    Serial.println("Couldn't initialize Air Quality channel\n");
   }
   Serial.println("Init Movement channel .... \n");
   ambient.InitMovementChannel();
   Serial.printf("Init SD Card\n");
   ambient.InitSDcard();
  // //SD.remove("/log.txt");
}

void loop()
{
  // ambient.CalibrateGasSensors();
  // delay(1000);
  if(!wifi_connected && connect_state)
  {
    Serial.printf("inside condition of connect_state = true\n");
    WiFi.begin();
    int interval = 0;
    bool led=false;
    while(WiFi.status() != WL_CONNECTED && interval < 100)
    {
      digitalWrite(WWIFI_LED,led);
      led=!led;
      Serial.printf("waiting for connection through last AP\n");
      delay(100);
      interval++;
    }
    if( WiFi.status() != WL_CONNECTED )
    {
      //Serial.println("Init WPS ...");
      //WiFi.disconnect();
      WiFi.disconnect(false,true);
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      //bool led=false;
      while(WiFi.status() != WL_CONNECTED)
      {
        digitalWrite(WWIFI_LED,led);
        led=!led;
        Serial.printf("waiting for connection throgh WPS\n");
        delay(250);
      }
      wifi_connected = true;
      digitalWrite(WWIFI_LED,HIGH);
    }
    else
    {
      wifi_connected = true;
      digitalWrite(WWIFI_LED,HIGH);
    }
  }
  else if(wifi_connected && !connect_state)
  {
    Serial.printf("inside condition of connect state = false\n");
    if(esp_wifi_wps_disable() == ESP_OK && ambient.DisconnectConnectWIFI())
    {
      Serial.printf("Successfully disconnected from wifi\n");
      digitalWrite(WWIFI_LED,LOW);
      wifi_connected = false;
    }
    else
    {
      Serial.printf("Couldn't disconnect from wifi\n");
      digitalWrite(WWIFI_LED,HIGH);
    }
  }
  else if(!wifi_connected && !connect_state || !start_stop_state)
  {
    Serial.printf("WIFI Disconnected, readings will be wrote in sd card only or Reading Stoped\n");
    delay(1000);
  }  
  //-----------------------------------------
  if(start_stop_state)
  {
    ESP32Time.set_time() ? ambient.setNTPTime(true) : ambient.setNTPTime(false); 
    Serial.printf("Reading Gas Sensors values ... \n");
    ambient.ReadGasSensorChannel();
    Serial.printf("Write Gas Sensors values to ThingSpeak\n");
    ambient.WriteGASSensorsChannel();
    Serial.printf("Waiting 15 sec interval\n");
    delay(15000);
    Serial.printf("Reading Air Quality values ... \n");
    ambient.ReadAirQualityChannel();
    Serial.printf("Write Air Quality values to ThingSpeak\n");
    ambient.WriteAirQualityChannel();
    Serial.printf("Waiting 15 sec interval\n");
    delay(15000);
    Serial.printf("Reading Movement values ... \n");
    ambient.ReadMovementChannel();
    Serial.printf("Write Movement values to ThingSpeak\n");
    ambient.WriteMovementChannel();
    Serial.printf("Waiting 15 sec interval\n");
    delay(15000);
    //Serial.printf("Writing to SD Card\n");
    //ambient.WriteLog();
    //ambient.ReadFromSDCard(SD, "/log.txt");
  }
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