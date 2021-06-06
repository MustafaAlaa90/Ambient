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
//-------------------------------------------------------------
void CAmbientMonitor::COInit()
{
    Serial.printf("Begin ADC Driver\n");
    ads.begin(); // begin externa adc
    Serial.printf("set CO regression mode to %d\n",_PPM);
    CO.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    Serial.printf("Set CO point a = %d , CO point b = %d\n",COVAL_A,COVAL_B);
    CO.setA(COVAL_A); CO.setB(COVAL_B); // Configurate the ecuation values to get co concentration
    //CO.init();
    Serial.printf("set CO RL value %d\n",CO_RL);
    CO.setRL(CO_RL);
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      //CO.update();
      int16_t adc0 = ads.readADC_SingleEnded(CO_ADC_PIN);
      float volts0 = ads.computeVolts(adc0);
      CO.setADC(volts0);
      calcR0 += CO.calibrate(RatioCleanAIRCO);
    }
    CO.setR0(calcR0/10);
    Serial.printf("Final CO_R0 = %f\n",calcR0);
}
//-------------------------------------------------------------
void CAmbientMonitor::CH4Init()
{
    Serial.printf("Begin ADC Driver\n");
    ads.begin(); // begin externa adc
    Serial.printf("set CH4 regression mode to %d\n",_PPM);
    CH4.setRegressionMethod(_PPM); //_PPM =  a*ratio^b
    Serial.printf("Set CH4 point a = %d , CH4 point b = %d\n",CH4VAL_A,CH4VAL_B);
    CH4.setA(CH4VAL_A); CH4.setB(CH4VAL_B); // Configurate the ecuation values to get Benzene concentration
    CH4.init();
    CH4.setRL(CH4_RL);
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      //CH4.update();
      int16_t adc2 = ads.readADC_SingleEnded(CH4_ADC_PIN);
      float volts2 = ads.computeVolts(adc2);
      CH4.setADC(volts2);
      calcR0 += CH4.calibrate(RatioCleanAIRCH4);
    }
    CH4.setR0(calcR0/10);
    Serial.printf("Final CH4_R0 = %f\n",calcR0);
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
    GPS.UbloxInit(UBlox_baud,RX,TX);
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
    // Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
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
//------------------------------------------------------------
bool CAmbientMonitor::WriteGASSensorsChannel()
{
    bool Ret = true;
    if(ThingSpeak.writeFields(SECRET_GAS_SENSOR_ID,SECRET_GAS_SENSOR_WRITE_APIKEY) == 200 )
    {
      Serial.printf("Gas Sensors value wrote successfully to ThingSpeak\n");
    }
    else
    {
      Serial.printf("Failed to Write Gas Sensors value to ThingSpeak\n");
      Ret = false;
    }
    return Ret;
}
//-------------------------------------------------------------
void CAmbientMonitor::ReadGasSensorChannel()
{
  m_GasSensorChReading[Gas_Sensor_field_CO-1] =  ReadCOPPM();
  m_GasSensorChReading[Gas_Sensor_field_CO2-1] = (float) ReadCO2PPM();
  m_GasSensorChReading[Gas_Sensor_field_CH4-1] = ReadCH4PPM();
  m_GasSensorChReading[Gas_Sensor_field_O3-1] =  ReadO3();
  Serial.printf("O3 PPM after reading %f\n",m_GasSensorChReading[Gas_Sensor_field_O3-1]);
  SetfieldMultiple(m_GasSensorChReading,GAS_SENSOR_READING_SIZE);
  
}
//-------------------------------------------------------------
float CAmbientMonitor::ReadCOPPM()
{
    //CO.update();
    Serial.printf("Begin ADC Driver ...\n");
    ads.begin(); // begin externa adc
    Serial.printf("Read CO from ADC Driver\n");
    int16_t adc0 = ads.readADC_SingleEnded(0);
    float volts0 = ads.computeVolts(adc0);
    Serial.printf("CO volt = %f\n",volts0);
    CO.setADC(volts0);
    float co = CO.readSensor();
    Serial.printf("CO PPM = %f\n",co);
    return co;
}
//------------------------------------------------------------
float CAmbientMonitor::ReadCH4PPM()
{
    //CH4.update();
    //return CH4.readSensor();
    Serial.printf("Begin ADC Driver ...\n");
    ads.begin(); // begin externa adc
    Serial.printf("Read CH4 from ADC Driver\n");
    int16_t adc2 = ads.readADC_SingleEnded(2);
    float volts2 = ads.computeVolts(adc2);
    Serial.printf("CH4 volts = %f\n",volts2);
    CO.setADC(volts2);
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
    return o3/1000.0;
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
bool CAmbientMonitor::ReadBME(float* temp,uint32_t* pressure,float* humadity,uint32_t* voc)
{
    // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    return false;
  }
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    return false;
  }
  *temp     = bme.temperature;
  *pressure = bme.pressure / 100.0;
  *humadity = bme.humidity;
  *voc      = bme.gas_resistance / 1000.0;

  //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
}
//--------------------------------------------------------------------------
bool CAmbientMonitor::ReadSPS(float* pm1,float* pm2,float* pm10)
{
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

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
      return false;
    }

  } while (ret != ERR_OK);

  // only print header first time
  *pm1  = val.MassPM1;
  *pm2  = val.MassPM2;
  *pm10 = val.MassPM10;
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
    Serial.println(F("Error reading temperature!"));
    ret= false;
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    *temp = event.temperature;
  }
  // Get humidity event and print its value.
  DHT.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
    ret = false;
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    *hum = event.relative_humidity;
  }
}
//------------------------------------------------------------------
bool CAmbientMonitor::ConnectWIFI(const char* ssid, const char* pass )
{
  return WiFi.begin(ssid,pass) == WL_CONNECTED;
}
//------------------------------------------------------------------
bool CAmbientMonitor::IsWiFiConnected()
{
  return WiFi.status() == WL_CONNECTED;
}