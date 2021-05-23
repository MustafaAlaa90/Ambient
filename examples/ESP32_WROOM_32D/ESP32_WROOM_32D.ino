/*
 *  This sketch fixes the MQ5 measuring issue when using Wifi.h
 *  Wifi.h deactivates pin 25 analogical input on esp32.ino default sketch.
 *  This is because Wifi.h works only with ADC1 for analogical measurement
 *  Please check the esp32-wroom-32d.jpg image on ESP32 folder
 * 

 * ADC1 GPIO
 * ADC1_CH0 (GPIO 36) // only tested on this and it works as expected :)
 * ADC1_CH1 (GPIO 37)
 * ADC1_CH2 (GPIO 38)
 * ADC1_CH3 (GPIO 39)
 * ADC1_CH4 (GPIO 32)
 * ADC1_CH5 (GPIO 33)
 * ADC1_CH6 (GPIO 34)
 * ADC1_CH7 (GPIO 35)
 *
 * ADC2 GPIO
 * ADC2_CH0 (GPIO 4)
 * ADC2_CH1 (GPIO 0)
 * ADC2_CH2 (GPIO 2)
 * ADC2_CH3 (GPIO 15)
 * ADC2_CH4 (GPIO 13)
 * ADC2_CH5 (GPIO 12)
 * ADC2_CH6 (GPIO 14)
 * ADC2_CH7 (GPIO 27)
 * ADC2_CH8 (GPIO 25)
 * ADC2_CH9 (GPIO 26)
 *
 */


#include <WiFi.h>

//Include the library
#include <MQUnifiedsensor.h>
/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.

//https://www.amazon.com/HiLetgo-ESP-WROOM-32-Development-Microcontroller-Integrated/dp/B0718T232Z (Although Amazon shows ESP-WROOM-32 ESP32 ESP-32S, the board is the ESP-WROOM-32D)
#define         Pin                     (35) //check the esp32-wroom-32d.jpg image on ESP32 folder 

/***********************Software Related Macros************************************/
#define         Type                    ("MQ-5") //MQ5 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ5CleanAir        (6.5) //RS / R0 = 6.5 ppm
/*****************************Globals***********************************************/
MQUnifiedsensor MQ5(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

const char* ssid     = "Esraa";
const char* password = "Esraa+28121995";

void setup()
{

  //Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port
  delay(10);

  //Set math model to calculate the PPM concentration and the value of constants
  MQ5.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ5.setA(177.65); MQ5.setB(-2.56); // Configurate the ecuation values to get H2 concentration
  
/*
    Exponential regression:
  Gas    | a      | b
  H2     | 1163.8 | -3.874
  LPG    | 80.897 | -2.431
  CH4    | 177.65 | -2.56
  CO     | 491204 | -5.826
  Alcohol| 97124  | -4.918
  */

  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ5.init(); 
 
   
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ5.setRL(47);
 
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
  // and now is on clean air (Calibration conditions), and it will setup R0 value.
  // We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
  // This routine not need to execute to every restart, you can load your R0 if you know the value
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ5.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ5.calibrate(RatioMQ5CleanAir);
    Serial.print(".");
  }
  MQ5.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ5.serialDebug(true);// uncomment if you want to print the table on the serial port

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop()
{
  MQ5.update(); // Update data, the arduino will be read the voltage on the analog pin
  MQ5.serialDebug(); // Will print the table on the serial port
  Serial.print(MQ5.readSensor()); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  Serial.println(" PPM");
  delay(500); //Sampling frequency
}
