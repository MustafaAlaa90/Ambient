#include "CAmbientMonitor.h"
//--------------------------------------------------------------------------
CAmbientMonitor::CAmbientMonitor():
 CO(BOARD,Voltage_Resolution,ADC_Bit_Resolution,CO_ADC_PIN,COTYPE)
,CH4(BOARD,Voltage_Resolution,ADC_Bit_Resolution,CH4_ADC_PIN,CH4TYPE)
,CO2(CO2PIN,INERTIA,TRIES)
,GPS(UBlox_UART)
,DHT(DHTPIN, DHTTYPE)
{
}
//------------------------------------------------------------
void CAmbientMonitor::InitButtons()
{
  pinMode(Connect_WIFI_Pin, INPUT_PULLUP);
  pinMode(Start_Stop_Reading_Pin, INPUT_PULLUP);
  pinMode(Reset_Pin, INPUT_PULLUP);
}
//------------------------------------------------------------
void CAmbientMonitor::InitLEDs()
{
  pinMode(WWIFI_LED,OUTPUT);
  pinMode(Start_Stop_LED,OUTPUT);
}
//---------------------------------------------------------
void CAmbientMonitor::InitMovementChannel()
{
  GPSInit();
  memset(m_MovementSensorChReading,0,MOVEMENT_READING_SIZE);
}
//-----------------------------------------------------------
bool CAmbientMonitor::InitGasSensorChannel()
{
  bool Ret=true;
  Serial.printf("initialize CO2 ...\n");
  CO2Init();
  Serial.printf("initialize CO ...\n");
  COInit();
  Serial.printf("initialize O3 ...\n");
  Ret = O3Init();
  Serial.printf("initialize CH4 ...\n");
  CH4Init();
  memset(m_GasSensorChReading,0,GAS_SENSOR_READING_SIZE);
  return Ret;
}
//------------------------------------------------------------
bool CAmbientMonitor::InitAirQualityChannel()
{
  bool Ret=true;
  Serial.printf("initialize SPS ...\n");
  if(!SPSInit())
  {
    Serial.printf("Failed to initialize SPS\n");
    Ret = false;
  }
  Serial.printf("initialize BME ...\n");
  if(!BMEInit())
  {
    Serial.printf("Failed to initialize BME\n");
    Ret = false;
  }
  Serial.printf("initialize DHT ....\n");
  DHTInit();
  memset(m_AirQualitySensorChReading,0,AIR_QUALITY_READING_SIZE);
  return Ret;
}
//------------------------------------------------------------

//-------------------------------------------------------------
void CAmbientMonitor::COInit()
{
    Serial.printf("Begin ADC Driver\n");
    this->ads.begin(); // begin externa adc
    Serial.printf("set CO regression mode to %d\n",_PPM);
    this->CO.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    Serial.printf("Set CO point a = %f , CO point b = %f\n",COVAL_A,COVAL_B);
    this->CO.setA(COVAL_A); this->CO.setB(COVAL_B); // Configurate the ecuation values to get co concentration
    this->CO.setVoltResolution(Voltage_Resolution);
    Serial.printf("set voltResolution = %f\n",this->CO.getVoltResolution());
    //CO.init();
    this->CO.setRL(CO_RL);
    Serial.printf("set CO RL value %f\n",this->CO.getRL());
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      //CO.update();
      int16_t adc0 = this->ads.readADC_SingleEnded(CO_ADC_PIN);
      //float volts0 = this->ads.computeVolts(adc0);
      this->CO.setADC((float)adc0);
      calcR0 += this->CO.calibrate(RatioCleanAIRCO);
    }
    this->CO.setR0(calcR0/float(10.0));
    Serial.printf("RL = 100 , Final CO_R0 = %f\n",this->CO.getR0());
}
//-------------------------------------------------------------
void CAmbientMonitor::CH4Init()
{
    Serial.printf("Begin ADC Driver\n");
    this->ads.begin(); // begin externa adc
    Serial.printf("set CH4 regression mode to %d\n",_PPM);
    this->CH4.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    Serial.printf("Set CH4 point a = %f , CH4 point b = %f\n",CH4VAL_A,CH4VAL_B);
    this->CH4.setA(CH4VAL_A); this->CH4.setB(CH4VAL_B); // Configurate the ecuation values to get Benzene concentration
    this->CH4.setVoltResolution(Voltage_Resolution);
    Serial.printf("set voltResolution = %f\n",this->CH4.getVoltResolution());
    // //CH4.init();
    this->CH4.setRL(CH4_RL);
    Serial.printf("set CH4 RL value %f\n",CH4.getRL());
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      //CH4.update();
      int16_t adc2 = this->ads.readADC_SingleEnded(CH4_ADC_PIN);
      //float volts2 = this->ads.computeVolts(adc2);
      this->CH4.setADC((float)adc2);
      calcR0 += this->CH4.calibrate(RatioCleanAIRCH4);
    }
    this->CH4.setR0(calcR0/float(10.0));
    Serial.printf("RL = 100, Final CH4_R0 = %f\n",this->CH4.getR0());
}
//-------------------------------------------------------------
void CAmbientMonitor::CO2Init()
{
    Serial.printf("Setting co2 samples = %d, Vref = %f\n",CO2Samples,CO2VREF);
    CO2.SetVREF(CO2VREF);
    CO2.SetSamples(CO2Samples);
    Serial.printf("Caliberating Co2 sensor ... \n");
    CO2.calibrate();
}
//-------------------------------------------------------------
bool CAmbientMonitor::O3Init()
{
    bool Ret=true;
    uint8_t trials = 0;
    Serial.printf("Begin Ozone I2C bus on address %X\n",Ozone_IICAddress);
    while(!O3.begin(Ozone_IICAddress) || trials>3)
    {
        delay(1000);
        trials++;
    }
/*   Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
*/
    if(trials  < 3)
    {
      Serial.printf("Setting Ozone sensor to measure passive mode\n");
      O3.SetModes(MEASURE_MODE_PASSIVE);
    }
    else
    {
      Ret = false;
      Serial.printf("Failed to begin Ozone I2C bus on address %X\n",Ozone_IICAddress);
    }
    return Ret;
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
bool CAmbientMonitor::BMEInit()
{
    if (!bme.begin()) {
    Serial.printf("Could not find a valid BME680 sensor, check wiring!\n");
    return false;
  }
  else
  {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
  return true;
}
//-------------------------------------------------------------
bool CAmbientMonitor::SPSInit()
{
    sps30.EnableDebugging(DEBUG);
    // set pins to use for softserial and Serial1 on ESP32
    if (TX_PIN != 0 && RX_PIN != 0)
    {
        sps30.SetSerialPin(RX_PIN,TX_PIN);
    }
    if (! sps30.begin(SP30_COMMS))
    {
        return false;
    }
    // check for SPS30 connection
    if (! sps30.probe())
    {
        return false;
    }
    if (! sps30.reset())
    {
        return false;
    }
    return true;
}
//------------------------------------------------------------
void CAmbientMonitor::DHTInit()
{
    DHT.begin();
}
//-------------------------------------------------------------
void CAmbientMonitor::SetfieldMultiple(float* channelArr,uint8_t ArrSize)
{
    for (uint8_t i=1;i<=ArrSize;i++)
    {
        ThingSpeak.setField(i,channelArr[i-1]);
    }
}
//-------------------------------------------------------------
void CAmbientMonitor::SetfieldMultiple(double* channelArr,uint8_t ArrSize)
{
    for (uint8_t i=1;i<=ArrSize;i++)
    {
        ThingSpeak.setField(i,(float)channelArr[i-1]);
    }
}
//------------------------------------------------------------
bool CAmbientMonitor::WriteGASSensorsChannel()
{
    bool Ret = true;
    int status= ThingSpeak.writeFields(SECRET_GAS_SENSOR_ID,SECRET_GAS_SENSOR_WRITE_APIKEY);
    if( status == 200 )
    {
      Serial.printf("Gas Sensors value wrote successfully to ThingSpeak\n");
    }
    else
    {
      Serial.printf("Failed to Write Gas Sensors value to ThingSpeak, status = %d\n",status);
      Ret = false;
    }
    return Ret;
}
//--------------------------------------------------------------
bool CAmbientMonitor::WriteAirQualityChannel()
{
  bool Ret = true;
  if(ThingSpeak.writeFields(SECRET_AIR_QUALITY_ID,SECRET_AIR_QUALITY_WRITE_APIKEY) == 200 )
  {
    Serial.printf("Air Quality value wrote successfully to ThingSpeak\n");
  }
  else
  {
    Serial.printf("Failed to Write Air Quality value to ThingSpeak\n");
    Ret = false;
  }
  return Ret;
}
//--------------------------------------------------------------
bool CAmbientMonitor::WriteMovementChannel()
{
  bool Ret = true;
  if(ThingSpeak.writeFields(SECRET_MOVEMENT_ID,SECRET_MOVEMENT_WRITE_APIKEY) == 200 )
  {
    Serial.printf("Movement value wrote successfully to ThingSpeak\n");
  }
  else
  {
    Serial.printf("Failed to Write Movement value to ThingSpeak\n");
    Ret = false;
  }
  return Ret;
}
//-------------------------------------------------------------
void CAmbientMonitor::ReadGasSensorChannel()
{
  m_GasSensorChReading[Gas_Sensor_field_CO-1] =  ReadCOPPM();
  m_GasSensorChReading[Gas_Sensor_field_CO2-1] = (float) ReadCO2PPM();
  m_GasSensorChReading[Gas_Sensor_field_CH4-1] = ReadCH4PPM() /*+ float(200.0)*/;
  m_GasSensorChReading[Gas_Sensor_field_O3-1] =  ReadO3();
  SetfieldMultiple(m_GasSensorChReading,GAS_SENSOR_READING_SIZE);
  ThingSpeak.setLatitude(29.957441748034174);
  ThingSpeak.setLongitude(30.912880079820305);
}
//-----------------------------------------------------------
bool CAmbientMonitor::ReadAirQualityChannel()
{
  float pm1,pm2,pm4,pm10;
  if(ReadSPS(&pm1,&pm2,&pm4,&pm10))
  {
    m_AirQualitySensorChReading[Air_Quality_field_PM1-1] = pm1;
    m_AirQualitySensorChReading[Air_Quality_field_PM25-1] = pm2;
    m_AirQualitySensorChReading[Air_Quality_field_PM4-1] = pm4;
    m_AirQualitySensorChReading[Air_Quality_field_PM10-1] = pm10;
  }
  float temp,humd;
  if(ReadDHT(&temp,&humd))
  {
    m_AirQualitySensorChReading[Air_Quality_field_TEMP-1] = temp;
    m_AirQualitySensorChReading[Air_Quality_field_HUMIDITY-1] = humd;
  }
  float pressure,tvoc;
  if(ReadBME(&pressure,&tvoc))
  {
    // Serial.printf("pressure = %f, tvoc = %f\n",pressure,tvoc);
    m_AirQualitySensorChReading[Air_Quality_field_PRESSURE-1] = pressure;
    m_AirQualitySensorChReading[Air_Quality_field_TVOC-1] = tvoc;
  }
  // for(int i=0;i<AIR_QUALITY_READING_SIZE;i++)
  // {
  //   Serial.printf("Field = %f\n",m_AirQualitySensorChReading[i]);
  // }
  SetfieldMultiple(m_AirQualitySensorChReading,AIR_QUALITY_READING_SIZE);
  ThingSpeak.setLatitude(29.957441748034174);
  ThingSpeak.setLongitude(30.912880079820305);
}
//------------------------------------------------------------
bool CAmbientMonitor::ReadMovementChannel()
{
  m_MovementSensorChReading[Movement_field_Sound_Level-1]= (double)ReadSoundLevel();
  double lat=0.0,lng=0.0,alt=0.0;
  GPS.getInfo(&lat,&lng,&alt);
  Serial.printf("lat = %lf , lng = %lf , alt = %lf\n",lat,lng,alt);
  Serial.printf("Fix count = %d\n",GPS.getfixCount());
  // if(GPS.getfixCount()>0)
  // {
    m_MovementSensorChReading[Movement_field_Longitude-1]= lng;
    m_MovementSensorChReading[Movement_field_Latitude-1]= lat;
    m_MovementSensorChReading[Movement_field_Altitude-1]= alt;
  // }
  SetfieldMultiple(m_MovementSensorChReading,MOVEMENT_READING_SIZE);
  ThingSpeak.setLatitude((float)m_MovementSensorChReading[Movement_field_Latitude-1]);
  ThingSpeak.setLongitude((float)m_MovementSensorChReading[Movement_field_Longitude-1]);

}
//-------------------------------------------------------------
float CAmbientMonitor::ReadCOPPM()
{
    //CO.update();
    Serial.printf("Begin ADC Driver ...\n");
    ads.begin(); // begin externa adc
    Serial.printf("Read CO from ADC Driver\n");
    int16_t adc0 = ads.readADC_SingleEnded(CO_ADC_PIN);
    //float volts0 = ads.computeVolts(adc0);
    Serial.printf("CO samples = %f\n",(float)adc0);
    CO.setADC((float)adc0);
    float co = CO.readSensor();
    Serial.printf("CO PPM = %f\n",co);
    return co;
}
//------------------------------------------------------------
float CAmbientMonitor::ReadCH4PPM()
{
    Serial.printf("Begin ADC Driver ...\n");
    ads.begin(); // begin externa adc
    Serial.printf("Read CH4 from ADC Driver\n");
    int16_t adc2 = ads.readADC_SingleEnded(CH4_ADC_PIN);
    //float volts2 = ads.computeVolts(adc2);
    Serial.printf("CH4 samples = %f\n",(float)adc2);
    CH4.setADC((float)adc2);
    float ch4 = CH4.readSensor();
    Serial.printf("CH4 PPM = %f\n",ch4);
    return ch4;
}
//-----------------------------------------------------------
double CAmbientMonitor::ReadCO2PPM()
{
    Serial.printf("Read CO2 PPM ... \n");
    double co2 = CO2.read();
    Serial.printf("CO2 PPM = %f\n",co2);
    return co2;
}
//-----------------------------------------------------------
float CAmbientMonitor::ReadO3()
{
    Serial.printf("Reading OZONE ...\n");
    int16_t o3 = O3.ReadOzoneData(COLLECT_NUMBER);
    Serial.printf("O3 PPB = %d\n",o3);
    Serial.printf("O3 PPM = %f\n",o3/1000.0);
    return o3/1000.0;
}
//-----------------------------------------------------------
float CAmbientMonitor::ReadSoundLevel()
{
    float voltage =0;
    float voltageValue =0;
    for(int i=0;i<10;i++)
    {
        voltageValue=voltageValue+analogRead(SOUND_LEVEL_PIN);
        delay(10);
    }
    //Serial.printf("Sound level =%d \n",voltageValue);
    Serial.printf("Sound level =%f \n",voltageValue/10.0);
    voltage = (voltageValue / ADC_Bit_Resolution_ESP * Voltage_Resolution)/10.0;
    Serial.printf("Sound levl = %f V\n",voltage);
    Serial.printf("Sound levl = %f dbA\n",voltage*50.0);
    return voltage * 50.0;  //convert voltage to decibel value
}
//-------------------------------------------------------------
void CAmbientMonitor::ReadGPSInfo(double* lat,double* lng,double* meters)
{
    GPS.getInfo(lat,lng,meters);
}
//--------------------------------------------------------------
bool CAmbientMonitor::ReadBME(float* pressure, float* voc)
{
    // Tell BME680 to begin measurement.
    Serial.printf("Begin BME Readings .... \n");
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.printf("Failed to begin BME Readings\n");
    return false;
  }
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  Serial.printf("End BME Reading Period ... \n");
  if (!bme.endReading()) {
    Serial.printf("Failed to End BME Reading Period\n");
    return false;
  }
  *pressure = bme.pressure / 100.0;
  *voc      = bme.gas_resistance / 1000.0;

  Serial.printf("Pressure = %f hPa , TVOC = %f KOhms\n",*pressure,*voc);
  Serial.printf("Humidity from BME = %f%\n",bme.humidity);
  Serial.printf("Temp from BME = %f\n",bme.temperature);
  return true;
  //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
}
//--------------------------------------------------------------------------
bool CAmbientMonitor::ReadSPS(float* pm1,float* pm2,float* pm4,float* pm10)
{
  uint8_t ret, error_cnt = 0;
  struct sps_values val;
  
  // loop to get data
  do {
    Serial.printf("Getting SPS Vales .... Trial = %d \n",error_cnt+1);
    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          //ErrtoMess((char *) "Error during reading values: ",ret);
          return false;
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
     // ErrtoMess((char *) "Error during reading values: ",ret);
      Serial.printf("Unexpected erro while reading SPS Values\n");
      return false;
    }

  } while (ret != ERR_OK);

  // only print header first time
  *pm1  = val.MassPM1;
  *pm2  = val.MassPM2;
  *pm4  = val.MassPM4;
  *pm10 = val.MassPM10;
  Serial.printf("PM1 = %f , PM2.5 = %f , PM4 = %f, PM10 = %f\n",*pm1,*pm2,*pm4,*pm10);
//   Serial.print(val.MassPM1);
//   Serial.print(F("\t"));
//   Serial.print(val.MassPM2);
//   Serial.print(F("\t"));
//   Serial.print(val.MassPM4);
//   Serial.print(F("\t"));
//   Serial.print(val.MassPM10);
//   Serial.print(F("\t"));
//   Serial.print(val.NumPM0);
//   Serial.print(F("\t"));
//   Serial.print(val.NumPM1);
//   Serial.print(F("\t"));
//   Serial.print(val.NumPM2);
//   Serial.print(F("\t"));
//   Serial.print(val.NumPM4);
//   Serial.print(F("\t"));
//   Serial.print(val.NumPM10);
//   Serial.print(F("\t"));
//   Serial.print(val.PartSize);
//   Serial.print(F("\n"));

  return(true);
}
//------------------------------------------------------------
bool CAmbientMonitor::ReadDHT(float* temp,float* hum)
{
  bool ret = true;
  sensors_event_t event;
  DHT.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.printf("Error reading temperature!\n");
    ret= false;
  }
  else
  {
    *temp = event.temperature;
  }
  // Get humidity event and print its value.
  DHT.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.printf("Error reading humidity!\n");
    ret = false;
  }
  else
  {
    *hum = event.relative_humidity;
  }
  Serial.printf("Temp = %f °C, Humidity = %f%\n",*temp,*hum);
  return ret;
}
//------------------------------------------------------------------
bool CAmbientMonitor::ConnectWIFI(const char* ssid, const char* pass )
{
  int trials = 0;
  bool state=false;
  WiFi.disconnect(true);
  while(WiFi.begin(ssid,pass) == WL_CONNECT_FAILED)
  {
    Serial.printf("Connecting to wifi\n");
    digitalWrite(WWIFI_LED,state);
    state = !state;
    delay(500);
  }
  return WiFi.status()!=WL_CONNECT_FAILED;
}
bool CAmbientMonitor::DisconnectConnectWIFI()
{
  return WiFi.disconnect(false,true);
}
//------------------------------------------------------------------
bool CAmbientMonitor::IsWiFiConnected()
{
  return WiFi.status() == WL_CONNECTED;
}
//---------------------------------------------------------------
bool CAmbientMonitor::InitSDcard()
{
  char trials = 0;
  while(!SD.begin() && trials < 3)
  {
    trials++;
  }
  if(trials < 3)
  {
    Serial.printf("Memory Card Found\n");
    uint64_t totalcardBytes = SD.totalBytes() / (1024 * 1024);
    uint64_t usedcardBytes = SD.usedBytes() / (1024 * 1024);
    Serial.printf("SD Card Total Bytes: %lluMB\n", totalcardBytes);
    Serial.printf("SD Card Used Bytes: %lluMB\n", usedcardBytes);
  }
}
//----------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}
//-------------------------------------------------------------------
bool readFile(fs::FS &fs, const char * path, char* message){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file)
    {
        Serial.println("Failed to open file for reading");
        return false;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
    return true;
}