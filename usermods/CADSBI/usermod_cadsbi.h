  #pragma once

  #include <Wire.h>
  #include <INA226.h>
  #include "wled.h"
  INA226 ina(Wire);  
  //This is an empty v2 usermod template. Please see the file usermod_v2_example.h in the EXAMPLE_v2 usermod folder for documentation on the functions you can use!

  class CADSBIUsermod: public Usermod 
  {
    private:
  // Temperatursensor Configuration
    bool degC = true;
    bool checkwire=true;
    bool sensorFound_=false;
    unsigned long readingInterval = 6000;
    unsigned long lastMeasurement = UINT32_MAX - 6000;
    unsigned long lastRequest;
    float temperature_s1 = -100;
    bool waitingForConversion = false;
    bool sensorFound = false;
    bool enabled = true;
    const int TEMP_SENSOR_ADDR = 73;
    const int INA_SENSOR_ADDR = 64;
    bool Found_Temperatursensor = false;
    bool Found_Stromsensor = false;
    static const int Addr_s1 = 0x49;
    static const int Ina_addr = 0x40;
    static const char _name_temp[];
    static const char _name_ina[];
    static const char _name[];
    static const char _enabled[];
    static const char _readInterval[];

  // Currentmesurement Configuration                   // Name Decl. Ina
    float powertemp_ = 0.000;
    float shuntcurrentemp_ = 0.000;

  // Variablen für INA226-Kalibrierung
  const float INA226_SHUNT_RESISTOR = 0.01;
  const float INA226_MAX_CURRENT = 5;


  bool findSensors() {
    bool success = true;
    Wire.beginTransmission(TEMP_SENSOR_ADDR);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("Temperature Sensor 1 found!");
      Found_Temperatursensor = true;
    } else {
      Serial.println("Error: Temperature Sensor 1 not found.");
      success = false;
    }
    Wire.beginTransmission(INA_SENSOR_ADDR);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("INA Sensor found!");
      Found_Stromsensor = true;
    } else {
      Serial.println("Error: INA Sensor not found.");
      success = false;
    }
  return success;
}

//hier wird die Temperatur ausgelesen
float readTemperatureFromSensor() {
  uint16_t data;
  Wire.beginTransmission(Addr_s1);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(Addr_s1, 2);
  if (Wire.available() == 2) {
    data = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Error: Failed to read data from sensor!");
    return -999.0f; // Rückgabe eines ungültigen Wertes
  }
  int temp = data >> 4;
  return temp * 0.0625f;
}

//temperatur empfangen?
void requestTemperatures() {
  (void)readTemperatureFromSensor();
  
  // store timestamp of the last request
  lastRequest = millis();
  
  // set flag to indicate that conversion is pending
  waitingForConversion = true;

  // print debug message
  DEBUG_PRINTLN(F("Temperature request completed."));
}

void readTemperature() {
  float temp = readTemperatureFromSensor();
  temperature_s1 = temp;
  lastMeasurement = millis();
  waitingForConversion = false;
  DEBUG_PRINTF("Read temperature_s1 %2.1f.\n", temperature_s1);

  // print temperature to serial monitor
  printf("Temperatur:   %.2f °C\n", temperature_s1);
}

void Ina_Scan() {
  if (Ina_Configuration()) {
  } else {
    Serial.println("INA226 configuration failed!");
  }
}

bool Ina_Configuration() {
  ina.begin(Ina_addr);
  if (!ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT)) {
    Serial.println("INA226 configuration failed!");
    return false;
  }
  ina.calibrate(INA226_SHUNT_RESISTOR, INA226_MAX_CURRENT);
  float busVoltage = ina.readBusVoltage();
  float busPower = ina.readBusPower();
  shuntcurrentemp_ = ina.readShuntCurrent();
  printf("Bus voltage:   %.2f V\n", busVoltage);
  printf("Bus power:     %.2f W\n", busPower);
  printf("Shunt current: %.2f A\n\n", shuntcurrentemp_);
  return true;
  }
  
public:
  void setup() 
  {
  Wire.begin(42,2);
    if (checkwire == 1 && !sensorFound) {
      for (int retries = 0; retries < 2 && !sensorFound; ++retries) {
        sensorFound = findSensors();
        if (sensorFound) {
          sensorFound_ = sensorFound;
          }
      // Wait for 25 ms before trying again
      unsigned long start = millis();
      while (millis() - start < 25) {}
      }
    }
  lastMeasurement = millis() - readingInterval + 10000;
  }

  void loop()
  {       
  if (!sensorFound_ || !enabled || strip.isUpdating()) return;
  unsigned long now = millis();
  if (now - lastMeasurement < readingInterval) return;
  if (now - lastRequest >= 100 /* 93.75ms per the datasheet but can be up to 750ms */) {
    if (Found_Temperatursensor) {
      readTemperature();
    }
    if (Found_Stromsensor) {
      Ina_Scan();
    }
    }
  }

 //Umwandlung von °C in F 
  inline float getTemperatureC() {
  return (float)temperature_s1;
  }
  inline float getTemperatureF() {
  return (float)temperature_s1 * 1.8f + 32;
  }

void addToJsonInfo(JsonObject& root) {
  // Add temperature to root
  JsonObject user = root.createNestedObject("u");
  if (sensorFound_ && enabled) {
    JsonArray temperatureArray = user.createNestedArray(FPSTR("Temperatur"));
    if (temperature_s1 <= -100.0) {
      temperatureArray.add(0);
      temperatureArray.add(FPSTR("Sensor Error!"));
    } else {
      float temperatureValue = temperature_s1 * (degC ? 1.0f : 1.8f) + (degC ? 0 : 32);
      char tempStr[10];
      dtostrf(temperatureValue, 4, 2, tempStr);
      temperatureArray.add(tempStr);
      temperatureArray.add(degC ? FPSTR("°C") : FPSTR("°F"));
    }
  }
  
  // Add current to root
  JsonArray currentObj = user.createNestedArray("Strom");
  currentObj.add(shuntcurrentemp_);
  currentObj.add(FPSTR("A"));
}

  void addToConfig(JsonObject &root) 
  {
    // we add JSON object: {"Temperature": {"pin": 0, "degC": true}}
    JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
    top[FPSTR(_enabled)] = enabled;
    top["degC"] = degC;  // usermodparam
    top[FPSTR(_readInterval)] = readingInterval / 1000;
    DEBUG_PRINTLN(F("Temperature config saved."));
    // we add JSON object: {"Currentmonitoring"}
  }

bool readFromConfig(JsonObject &root) {
  // Store old enabled state
  bool oldEnabled = enabled;

  // Look for JSON object: {"Temperature": {"pin": 0, "degC": true}}
  JsonObject top = root[FPSTR(_name)];
  
  DEBUG_PRINT(FPSTR(_name));
    if (top.isNull()) {
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

  // Set configuration parameters
  enabled = top[FPSTR(_enabled)] || enabled;
  degC = top["degC"] || degC;
  readingInterval = min(120, max(10, top[FPSTR(_readInterval)].as<int>())) * 1000;
  
  DEBUG_PRINT(FPSTR(_name));
  return !top.isNull();
}

  //****************************************************************
    uint16_t getId()
    {
    return USERMOD_ID_CADSBI;
    }
  };

  // strings to reduce flash memory usage (used more than twice)
  const char CADSBIUsermod::_name_temp[]        PROGMEM = "Temperatur Sensor";
  const char CADSBIUsermod::_name_ina[]         PROGMEM = "Strom Sensor";
  const char CADSBIUsermod::_name[]             PROGMEM = "CADSBI Usermod";
  const char CADSBIUsermod::_enabled[]          PROGMEM = "Aktiv";
  const char CADSBIUsermod::_readInterval[]     PROGMEM = "Aktualisierungsrate (sec.)";

    