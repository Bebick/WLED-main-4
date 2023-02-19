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
  bool Found_Sensor_Ina = false;
  static const int Addr_s1 = 0x49;
  static const int Ina_addr = 0x40;
  static const char _name_temp[];
  static const char _name_ina[];
  static const char _name[];
  static const char _enabled[];
  static const char _readInterval[];
  long lastTime = 0;
  int delayMs = 2000; 

// Currentmesurement Configuration                   // Name Decl. Ina
  int check_ina = 0;              
  int Ina_nr = 0;
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
      Serial.print("Bin im Usermod 108");
      Serial.print("I2C device found at address 0x");
      if (address<16)
      Serial.print("0");
      Serial.println(address,HEX);
      Serial.print(address);
      Serial.println(" address");
      if (address== 73){Found_Sensor_Temp1 = true; Serial.print ("Sensor Found Temperatur 1");}
      if (address== 64){Found_Sensor_Ina = true; Serial.print ("Sensor Found Strom");}
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
        _sensor_ina = 1;
        Ina__(_Ina_Ch,_sensor_ina);
        power_1 = powertemp_;
        current_1 = shuntcurrentemp_;
      break;
    }
  }

//**************************************************************************
//====================== Ina__Check_Config Begin =========================
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
//====================== Ina_Out_Mqtt BEGIN =========================
  void Ina__(int channel_, int _sensor_ina_out)
  {
    ina.begin(Ina_addr);
    ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
    ina.calibrate(0.01, 5); // Shuntresistor, max. Current
    if (check_ina == 0)
    { 
      checkConfig();
    }
    Serial.print("Bus voltage:   ");
    Serial.print(ina.readBusVoltage(), 5);
    Serial.println(" V");
    Serial.print("Bus power:     ");
    Serial.print((powertemp_), 5);
    Serial.println(" W");
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
    Serial.print("Shunt voltage: ");
    Serial.print(ina.readShuntVoltage(), 5);
    Serial.println(" V");
    Serial.print("Shunt current: ");
    Serial.print((shuntcurrentemp_), 5);
    Serial.println(" A");
    Serial.println("");
      }
  //====================== Ina_Out_Mqtt ENDE =========================


  public:
  void setup() 
  {
    Wire.begin(42,2);
    int retries = 2;
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
        Serial.print("Temperatur = ");
        Serial.print(temperature_s1);
        Serial.print(" °C");
        Serial.println(" ");
        }
        return;
      }
      if (now - lastTemperaturesRequest >= 100 /* 93.75ms per the datasheet but can be up to 750ms */) {
        if (Found_Sensor_Temp1 == true) {
        readTemperature();
     }
          if (Found_Sensor_Ina == true) {
          //Serial.print("Ina gefunden");
          Ina_Scan(1);
        }
    }
  }
      inline float getTemperatureC() {
      return (float)temperature_s1;
    }
    inline float getTemperatureF() {
    return (float)temperature_s1 * 1.8f + 32;
    }


    void addToJsonInfo(JsonObject& root) {
      // dont add temperature to info if we are disabled
      if ((!enabled) && (!enabled_)) return;
      if (enabled)
      {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray temp_s1 = user.createNestedArray(FPSTR(_name_temp));
      //temp.add(F("Loaded."));
      if (temperature_s1 <= -100.0 || (sensorFound_==false && temperature_s1 == -1.0)) {
        temp_s1.add(0);
        temp_s1.add(F(" Sensor Error!"));
        return;
      }
      temp_s1.add(degC ? temperature_s1 : (float)temperature_s1 * 1.8f + 32);
      if (degC) temp_s1.add(F("°C"));
      else      temp_s1.add(F("°F"));
    JsonObject user_ = root["u"];
    if (user_.isNull()) user_ = root.createNestedObject("u");
    JsonArray currentArr_ = user_.createNestedArray("Strom"); //name
    currentArr_.add(shuntcurrentemp_); //value
    currentArr_.add(" A"); //unit
    }
    else return;    
  }

    void addToConfig(JsonObject &root) {
      // we add JSON object: {"Temperature": {"pin": 0, "degC": true}}
      JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
      top[FPSTR(_enabled)] = enabled;
      top["degC"] = degC;  // usermodparam
      top[FPSTR(_readInterval)] = readingInterval / 1000;
      DEBUG_PRINTLN(F("Temperature config saved."));
      // we add JSON object: {"Currentmonitoring"}
    }

bool readFromConfig(JsonObject &root) 
{
      bool oldEnabled = enabled;
      // we look for JSON object: {"Temperature": {"pin": 0, "degC": true}}
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();

    DEBUG_PRINT(FPSTR(_name));
    if (top.isNull()) {
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }
      enabled           = top[FPSTR(_enabled)] | enabled;
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
        return configComplete;

    }
  




//****************************************************************
  uint16_t getId()
  {
   return USERMOD_ID_CADSBI;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char CADSBIUsermod::_name_temp[]        PROGMEM = "Temperature Sensor";
const char CADSBIUsermod::_name_ina[]         PROGMEM = "Strom Sensor";
const char CADSBIUsermod::_name[]             PROGMEM = "CADSBI Usermod";
const char CADSBIUsermod::_enabled[]          PROGMEM = "enabled";
const char CADSBIUsermod::_readInterval[]     PROGMEM = "read-interval (sec.)";

   