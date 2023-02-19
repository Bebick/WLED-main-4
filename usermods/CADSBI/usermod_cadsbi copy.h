#pragma once


#include <Wire.h>
#include <INA226.h>
#include "wled.h"
INA226 ina(Wire);  
//This is an empty v2 usermod template. Please see the file usermod_v2_example.h in the EXAMPLE_v2 usermod folder for documentation on the functions you can use!
#ifndef USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL
#define USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL 6000
#endif

bool ESP32_i2c = true;


class CADSBIUsermod: public Usermod 
{
  private:
// Temperatursensor Configuration
  bool initDone = false;
  bool degC = true;
  bool checkwire=true;
  bool sensorFound_=false;
  unsigned long readingInterval = USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - USERMOD_CADSBItemperatur_MEASUREMENT_INTERVAL;
  unsigned long lastTemperaturesRequest;
  float temperature_s1 = -100;
  bool waitingForConversion = false;
  bool sensorFound = false;
  bool enabled = true;
  bool enabled_ = true;
  bool Found_Sensor_Temp1 = false;
  bool Found_Sensor_Temp2 = false;
  bool Found_Sensor_Ina = false;
  #define Addr_s1 0x49
  static const char _name_s1[];
  static const char _name_s3[];
  static const char _enabled[];
  static const char _readInterval[];
  long lastTime = 0;
  int delayMs = 2000; 

// Currentmesurement Configuration                   // Name Decl. Ina
  int check_ina = 0;              
  int Ina_nr = 0;
  byte Ina_addr = 0;
  float powertemp_ = 0.000;
  float shuntcurrentemp_ = 0.000;
  float power_1 = 0.000;
  float current_1 = 0.000;
  int Wireavailable_=0;

bool findSensor() {
Serial.println("Searching for sensor...");
  Serial.println("checkwire vor dem Scan");
  Serial.println(checkwire);
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
   nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
     if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
      Serial.print("0");
      Serial.println(address,HEX);
      Serial.print(address);
      Serial.println(" address");
      if (address== 73){Found_Sensor_Temp1 = true; Serial.print ("Sensor Found Temperatur 1");}
      Serial.println("  !");
       nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found\n");
    checkwire = false; 
    Serial.println("Sensor nach dem Scan");
    Serial.println(sensorFound);
    return false;
    }
  else
   Serial.println("done\n");
    DEBUG_PRINTLN(F("Sensor found."));
    checkwire = false; 
    return true;
}
    float readTemper() {
      unsigned int data[2];
      int16_t result;                         // raw data from sensor
      Wire.beginTransmission(Addr_s1);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(Addr_s1, 2);
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
      temperature_s1 = readTemper();
      lastMeasurement = millis();
      waitingForConversion = false;
      DEBUG_PRINTF("Read temperature %2.1f.\n", temperature);
    }


  
  public:
  void setup() 
  {
    Wire.begin(42,2);
    int retries = 5;
    if ((checkwire == 1) && sensorFound == false) {
    while ((sensorFound=findSensor() && retries--)){
    if (sensorFound ==1) {(sensorFound_ = sensorFound);}
    delay(25); // try to find sensor
    }
    if (sensorFound == true) {
     if (enabled) {;
           }
        if (enabled_) {
        ;
        }
    }
      lastMeasurement = millis() - readingInterval + 10000;
      initDone = true;
  }
}
  void loop()
  {
    if (sensorFound_==false || !enabled || strip.isUpdating()) return;

      unsigned long now = millis();
        if (now - lastMeasurement < readingInterval) return;
      if (!waitingForConversion) {
        if (Found_Sensor_Temp1 == true) {
        requestTemperatures();
        Serial.print("Temperatur Sensor 1 = ");
        Serial.print(temperature_s1);
        Serial.print(" °C");
        Serial.println(" ");
        }
        return;
      }
      if (now - lastTemperaturesRequest >= 100 /* 93.75ms per the datasheet but can be up to 750ms */) {
        if (Found_Sensor_Temp1 == true) {
        readTemperature();
    /*  Serial.print("Temperatur Sensor 1 = ");
        Serial.print(temperature_s1);
        Serial.print(" °C");
        Serial.println(" ");
    */
        }
        if (sensorFound_==false || !enabled_ || strip.isUpdating()) return;
        if (WLED_MQTT_CONNECTED) {
          char subuf[64];
          strcpy(subuf, mqttDeviceTopic);
          if (-100 <= temperature_s1) {
            // dont publish super low temperature as the graph will get messed up
            // the DallasTemperature library returns -127C or -196.6F when problem
            // reading the sensor
            strcat_P(subuf, PSTR("/temperature_s1"));
            mqtt->publish(subuf, 0, false, String(temperature_s1).c_str());
            strcat_P(subuf, PSTR("_f"));
            mqtt->publish(subuf, 0, false, String((float)temperature_s1 * 1.8f + 32).c_str());
          } else {
            // publish something else to indicate status?
          }
        }
      }
    }
      inline float getTemperatureC() {
      return (float)temperature_s1;
    }
    inline float getTemperatureF() {
    return (float)temperature_s1 * 1.8f + 32;
    }
//****************************************************************


    void addToJsonInfo(JsonObject& root) {
      // dont add temperature to info if we are disabled
      if ((!enabled) && (!enabled_)) return;
      if (enabled)
      {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray temp_s1 = user.createNestedArray(FPSTR(_name_s1));
      //temp.add(F("Loaded."));
      if (temperature_s1 <= -100.0 || (sensorFound_==false && temperature_s1 == -1.0)) {
        temp_s1.add(0);
        temp_s1.add(F(" Sensor Error!"));
        return;
      }
      temp_s1.add(degC ? temperature_s1 : (float)temperature_s1 * 1.8f + 32);
      if (degC) temp_s1.add(F("°C"));
      else      temp_s1.add(F("°F"));
    }
    else return;    
  }

    void addToConfig(JsonObject &root) {
      // we add JSON object: {"Temperature": {"pin": 0, "degC": true}}
      JsonObject top = root.createNestedObject(FPSTR(_name_s3)); // usermodname
      top[FPSTR(_enabled)] = enabled;
      top["degC"] = degC;  // usermodparam
      top[FPSTR(_readInterval)] = readingInterval / 1000;
      DEBUG_PRINTLN(F("Temperature config saved."));
      // we add JSON object: {"Currentmonitoring"}
    }

bool readFromConfig(JsonObject &root) {
      // we look for JSON object: {"Temperature": {"pin": 0, "degC": true}}
      JsonObject top = root[FPSTR(_name_s3)];
      if (top.isNull()) {
        DEBUG_PRINT(FPSTR(_name_s3));
        DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
        return false;
      }

      enabled                = top[FPSTR(_enabled)] | enabled;
//      newTemperaturePin = min(33,max(-1,(int)newTemperaturePin)); // bounds check
      degC              = top["degC"] | degC;
      readingInterval   = top[FPSTR(_readInterval)] | readingInterval/1000;
      readingInterval   = min(120,max(10,(int)readingInterval)) * 1000;  // convert to ms

      DEBUG_PRINT(FPSTR(_name));
      if (!initDone) {
        // first run: reading from cfg.json
        DEBUG_PRINTLN(F(" config loaded."));
      } else {
        DEBUG_PRINTLN(F(" config (re)loaded."));
        // changing paramters from settings page
      }
    }
  uint16_t getId()
  {
   return USERMOD_ID_CADSBI;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char CADSBIUsermod::_name_s1[]          PROGMEM = "Temperature S1";
const char CADSBIUsermod::_name_s3[]          PROGMEM = "Temperature";
const char CADSBIUsermod::_enabled[]          PROGMEM = "enabled";
const char CADSBIUsermod::_readInterval[]     PROGMEM = "read-interval (sec.)";

   