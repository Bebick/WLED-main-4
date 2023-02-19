#pragma once

#include <INA226.h>
#include <Wire.h>
#include "wled.h"
//This is an empty v2 usermod template. Please see the file usermod_v2_example.h in the EXAMPLE_v2 usermod folder for documentation on the functions you can use!

#ifndef I2C_PINS
  #ifdef ARDUINO_ARCH_ESP32
    #define SCL_PIN 5
    #define SDA_PIN 4
  #else //ESP8266 boards
    #define SCL_PIN 5
    #define SDA_PIN 4
  #endif
#endif

#ifndef USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL
#define USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL 6000
#endif


class CADSBITemperaturUsermod: public Usermod 
{
  private:
// I2C Configuration  
int8_t SDAPIN = SDA_PIN;
int8_t SCLPIN = SCL_PIN;
// Temperatursensor Configuration
  bool initDone = false;
  bool degC = true;
  unsigned long readingInterval = USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL;
  unsigned long lastTemperaturesRequest;
  float temperature = -100;
  bool waitingForConversion = false;
  bool sensorFound = false;
  bool enabled = true;
  bool enabled_ = true;
  #define Addr 0x48
  static const char _name[];
  static const char _enabled[];
  static const char _readInterval[];
  long lastTime = 0;
  int delayMs = 2000; 

// Currentmesurement Configuration
  INA226 ina;                     // Name Decl. Ina
  int check_ina = 0;              
  int Ina_nr = 0;
  byte Ina_addr = 0;
  float powertemp_ = 0.000;
  float shuntcurrentemp_ = 0.000;
  float power_1 = 0.000;
  float current_1 = 0.000;



    float readTemper() {
      unsigned int data[2];
      int16_t result;                         // raw data from sensor
      Wire.beginTransmission(Addr);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(Addr, 2);
      if (Wire.available() == 2)
      {
        data[0] = Wire.read();
        data[1] = Wire.read();
      }
      int temp = ((data[0] * 256) + data[1]) / 16;
      if(temp > 2047)
      {
        temp -= 4096; 
      }
     return result = temp * 0.0625;
      //return (float) result + ((cTemp * 1.8) + 32);
    }


    void requestTemperatures() {
      readTemper();
      lastTemperaturesRequest = millis();
      waitingForConversion = true;
      DEBUG_PRINTLN(F("Requested temperature."));
    }

    void readTemperature() {
      temperature = readTemper();
      lastMeasurement = millis();
      waitingForConversion = false;
      DEBUG_PRINTF("Read temperature %2.1f.\n", temperature);
    }


   //****************************************************************
//====================== Ina_Scan Begin =========================
  void Ina_Scan(int _ina)
  {
    Ina_nr = 1;
    Ina_(Ina_nr);
    check_ina = 1;
  }
//====================== Ina_Scan ENDE =========================
//******************************************************************  
//====================== Ina_Config Begin =========================
  void Ina_(int _Ina_Ch)
  {
    int _sensor_ina = 0;
    switch (_Ina_Ch)
    {
      case 1:
        Ina_addr = 0x40;             
        _sensor_ina = 1;
        Ina__(_Ina_Ch,_sensor_ina);
        power_1 = powertemp_;
        current_1 = shuntcurrentemp_;
      break;
    }
  }
//====================== Ina_Config ENDE =========================
//**************************************************************************
/*//====================== Ina__Check_Config Begin =========================
  void checkConfig()
  {
    Serial.print("Mode:                  ");
    switch (ina.getMode())
    {
      case INA226_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
      case INA226_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
      case INA226_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
      case INA226_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
      case INA226_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
      case INA226_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
      case INA226_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
      case INA226_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
      default: Serial.println("unknown");
    }
    Serial.print("Samples average:       ");
    switch (ina.getAverages())
    {
      case INA226_AVERAGES_1:           Serial.println("1 sample"); break;
      case INA226_AVERAGES_4:           Serial.println("4 samples"); break;
      case INA226_AVERAGES_16:          Serial.println("16 samples"); break;
      case INA226_AVERAGES_64:          Serial.println("64 samples"); break;
      case INA226_AVERAGES_128:         Serial.println("128 samples"); break;
      case INA226_AVERAGES_256:         Serial.println("256 samples"); break;
      case INA226_AVERAGES_512:         Serial.println("512 samples"); break;
      case INA226_AVERAGES_1024:        Serial.println("1024 samples"); break;
      default: Serial.println("unknown");
    }
    Serial.print("Bus conversion time:   ");
    switch (ina.getBusConversionTime())
    {
      case INA226_BUS_CONV_TIME_140US:  Serial.println("140uS"); break;
      case INA226_BUS_CONV_TIME_204US:  Serial.println("204uS"); break;
      case INA226_BUS_CONV_TIME_332US:  Serial.println("332uS"); break;
      case INA226_BUS_CONV_TIME_588US:  Serial.println("558uS"); break;
      case INA226_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
      case INA226_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
      case INA226_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
      case INA226_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
      default: Serial.println("unknown");
    }
    Serial.print("Shunt conversion time: ");
    switch (ina.getShuntConversionTime())
    {
      case INA226_SHUNT_CONV_TIME_140US:  Serial.println("140uS"); break;
      case INA226_SHUNT_CONV_TIME_204US:  Serial.println("204uS"); break;
      case INA226_SHUNT_CONV_TIME_332US:  Serial.println("332uS"); break;
      case INA226_SHUNT_CONV_TIME_588US:  Serial.println("558uS"); break;
      case INA226_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
      case INA226_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
      case INA226_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
      case INA226_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
      default: Serial.println("unknown");
    }
    Serial.print("Max possible current:  ");
    Serial.print(ina.getMaxPossibleCurrent());
    Serial.println(" A");
  
    Serial.print("Max current:           ");
    Serial.print(ina.getMaxCurrent());
    Serial.println(" A");
  
    Serial.print("Max shunt voltage:     ");
    Serial.print(ina.getMaxShuntVoltage());
    Serial.println(" V");
  
    Serial.print("Max power:             ");
    Serial.print(ina.getMaxPower());
    Serial.println(" W");
  }
//====================== Ina_Check_Config ENDE =========================
*/
//**************************************************************************
//====================== Ina_Out_Mqtt BEGIN =========================
  void Ina__(int channel_, int _sensor_ina_out)
  {
    ina.begin(Ina_addr); 
    ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
    ina.calibrate(0.01, 5); // Shuntresistor, max. Current
    //if (check_ina == 0)
    //{ 
    //  checkConfig();
    //}
   /* Serial.print("Bus voltage:   ");
    Serial.print(ina.readBusVoltage(), 5);
    Serial.println(" V");
    Serial.print("Bus power:     ");
    Serial.print((powertemp_), 5);
    Serial.println(" W");
    */
    powertemp_= ina.readBusPower();
    shuntcurrentemp_ = ina.readShuntCurrent();  
    String stringOne_ = " Bus power:   ";
    String stringTwo_ = "Channel: ";
    String stringFive_ = " W";
    char str_[20];
    sprintf(str_,"%02X",channel_);
    String stringThree_ =  stringTwo_ + str_+ stringOne_+ powertemp_ + stringFive_;
    char stringFour_[50];
    stringThree_.toCharArray(stringFour_,50);
    /*Serial.print("Shunt voltage: ");
    Serial.print(ina.readShuntVoltage(), 5);
    Serial.println(" V");
    Serial.print("Shunt current: ");
    Serial.print((shuntcurrentemp_), 5);
    Serial.println(" A");
    Serial.println("");
    */

  }
  //====================== Ina_Out_Mqtt ENDE =========================


  public:
  void setup() 
  {
     if (enabled) {
          Wire.begin(SDAPIN, SCLPIN);
        }
        if (enabled_) {
          Wire.begin(SDAPIN, SCLPIN);
        }
      lastMeasurement = millis() - readingInterval + 10000;
      initDone = true;
  }

  void loop()
  {
    if (!enabled || strip.isUpdating()) return;

      unsigned long now = millis();
        if (now - lastMeasurement < readingInterval) return;
      if (!waitingForConversion) {
        requestTemperatures();
        return;
      }
      if (now - lastTemperaturesRequest >= 100 /* 93.75ms per the datasheet but can be up to 750ms */) {
        readTemperature();
        if (!enabled_ || strip.isUpdating()) return;
        Ina_Scan(1);
        if (WLED_MQTT_CONNECTED) {
          char subuf[64];
          strcpy(subuf, mqttDeviceTopic);
          if (-100 <= temperature) {
            // dont publish super low temperature as the graph will get messed up
            // the DallasTemperature library returns -127C or -196.6F when problem
            // reading the sensor
            strcat_P(subuf, PSTR("/temperature"));
            mqtt->publish(subuf, 0, false, String(temperature).c_str());
            strcat_P(subuf, PSTR("_f"));
            mqtt->publish(subuf, 0, false, String((float)temperature * 1.8f + 32).c_str());
          } else {
            // publish something else to indicate status?
          }
        }
      }
    }
      inline float getTemperatureC() {
      return (float)temperature;
    }
    inline float getTemperatureF() {
    return (float)temperature * 1.8f + 32;
    }
//****************************************************************

    void addToJsonInfo(JsonObject& root) {
      // dont add temperature to info if we are disabled
      if ((!enabled) && (!enabled_)) return;
      if (enabled)
      {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray temp = user.createNestedArray(FPSTR(_name));
      //temp.add(F("Loaded."));
      if (temperature <= -100.0 || (!sensorFound && temperature == -1.0)) {
        temp.add(0);
        temp.add(F(" Sensor Error!"));
        return;
      }
      temp.add(degC ? temperature : (float)temperature * 1.8f + 32);
      if (degC) temp.add(F("°C"));
      else      temp.add(F("°F"));
    }
    else return;
   // JsonObject user_ = root["u"];
    //if (user.isNull()) user = root.createNestedObject("u");
    if (enabled_);
    else return;
    JsonObject user_ = root["u"];
    if (user_.isNull()) user_ = root.createNestedObject("u");
    JsonArray currentArr_ = user_.createNestedArray("Strom"); //name
    currentArr_.add(shuntcurrentemp_); //value
    currentArr_.add(" A"); //unit
  }

    void addToConfig(JsonObject &root) {
      // we add JSON object: {"Temperature": {"pin": 0, "degC": true}}
      JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
      top[FPSTR(_enabled)] = enabled;
      top["pinSDA"]  = SDAPIN ;     // usermodparam
      top["pinSCL"]  = SCLPIN ;
      top["degC"] = degC;  // usermodparam
      top[FPSTR(_readInterval)] = readingInterval / 1000;
      DEBUG_PRINTLN(F("Temperature config saved."));
      // we add JSON object: {"Currentmonitoring"}
    }

bool readFromConfig(JsonObject &root) {
      // we look for JSON object: {"Temperature": {"pin": 0, "degC": true}}
      int8_t newTEMPERATURE_SDA_PIN = SDAPIN;
      int8_t newTEMPERATURE_SCL_PIN = SCLPIN;

      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) {
        DEBUG_PRINT(FPSTR(_name));
        DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
        return false;
      }

      enabled                = top[FPSTR(_enabled)] | enabled;
      newTEMPERATURE_SDA_PIN = top["pinSDA"] | newTEMPERATURE_SDA_PIN;
      newTEMPERATURE_SCL_PIN = top["pinSCL"] | newTEMPERATURE_SCL_PIN;
//      newTemperaturePin = min(33,max(-1,(int)newTemperaturePin)); // bounds check
      degC              = top["degC"] | degC;
      readingInterval   = top[FPSTR(_readInterval)] | readingInterval/1000;
      readingInterval   = min(120,max(10,(int)readingInterval)) * 1000;  // convert to ms

      DEBUG_PRINT(FPSTR(_name));
      if (!initDone) {
        // first run: reading from cfg.json
        SDAPIN = newTEMPERATURE_SDA_PIN;
        SCLPIN = newTEMPERATURE_SCL_PIN;
        DEBUG_PRINTLN(F(" config loaded."));
      } else {
        DEBUG_PRINTLN(F(" config (re)loaded."));
        // changing paramters from settings page
        if ((newTEMPERATURE_SDA_PIN != SDAPIN) && (newTEMPERATURE_SCL_PIN != SCLPIN)) {
          DEBUG_PRINTLN(F("Re-init temperature."));
        SDAPIN = newTEMPERATURE_SDA_PIN;
        SCLPIN = newTEMPERATURE_SCL_PIN;
          // initialise
          setup();
        }
      }
    }
  uint16_t getId()
  {
    return USERMOD_ID_CADSBITemperatur;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char CADSBITemperaturUsermod::_name[]         PROGMEM = "Temperature";
const char CADSBITemperaturUsermod::_enabled[]      PROGMEM = "enabled";
const char CADSBITemperaturUsermod::_readInterval[] PROGMEM = "read-interval-s";
