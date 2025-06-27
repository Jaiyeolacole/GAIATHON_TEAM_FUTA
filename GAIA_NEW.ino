#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Sensor Configuration
Adafruit_BME280 bme;
RTC_DS3231 rtc;

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define GPS_RX 16
#define GPS_TX 17
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

#define MQ2_PIN 34
#define MQ7_PIN 35
#define SUN_VOLTAGE_PIN 33
#define BATTERY_PIN 25
#define SOIL_MOISTURE_PIN 26
#define RAIN_DIGITAL_PIN 32

#define SD_CS 5
File dataFile;

#define SEALEVELPRESSURE_HPA (1013.25)
#define ADC_REF 3.3f
#define BATTERY_MAX_VOLTAGE 4.2f
#define BATTERY_MIN_VOLTAGE 3.0f

// Voltage Divider for Sunlight Sensor
#define R1 10000.0f  
#define R2 10000.0f  

#define INTERNET_LED 14
#define DATA_SENT_LED 27

// MQ Sensor Configuration
#define MQ_SAMPLES 10
#define MQ_BASELINE_TIME 5000
#define MQ2_RO_CLEAN_AIR 9.83
#define MQ7_RO_CLEAR_AIR 27.0
#define MQ2_RL 10.0
#define MQ7_RL 10.0

float temperature = 0, humidity = 0;
float mq2Baseline = 0.0, mq7Baseline = 0.0;
unsigned long lastSendTime = 0;
unsigned long lastLEDTime = 0;

const char* ssid = "Ceejay";
const char* password = "nopassword";

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);

  pinMode(INTERNET_LED, OUTPUT);
  pinMode(DATA_SENT_LED, OUTPUT);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  initializeSensors();
  initializeSDCard();
  calibrateMQSensors();

  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(INTERNET_LED, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected to WiFi.");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
    gps.encode(c);
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
  }

  DateTime now = rtc.now();
  
  // Read and average sensor values
  float bmeTemp = bme.readTemperature();
  float dhtTemp = dht.readTemperature();
  float bmeHum = bme.readHumidity();
  float dhtHum = dht.readHumidity();

  temperature = (bmeTemp + dhtTemp) / 2.0;
  humidity = (bmeHum + dhtHum) / 2.0;
  float atm_pressure = bme.readPressure() / 100.0F;

  // Calculate derived values
  float specificHumidity = computeSpecificHumidity(temperature, humidity, atm_pressure);
  float heat_index = dht.computeHeatIndex(temperature, humidity, false);
  float enthalpy = (1.006 * temperature) + (specificHumidity * (2501.0 + 1.86 * temperature));
  float dew_point = computeDewPoint(temperature, humidity);

  // Read environmental sensors
  int soilVal = analogRead(SOIL_MOISTURE_PIN);
  float soil_moisture = map(soilVal, 0, 4095, 0, 100);

  // Sun intensity 
  int rawSun = analogRead(SUN_VOLTAGE_PIN);
  float vin = (rawSun * ADC_REF) / 4095.0;
  float vout = vin * ((R1 + R2) / R2);
  float sunlight_intensity = (vout / 7.0) * 100.0;

  // Read and calculate battery percentage
  int adcValue = 0;
  for(int i=0; i<10; i++) {
    adcValue += analogRead(BATTERY_PIN);
    delay(2);
  }
  adcValue /= 10;
  float voltage = (adcValue * 3.3 / 4095.0) * 2 * 1.02; 
  float batteryPercent = getBatteryPercent(voltage); 

  // Read and process MQ sensors
  float mq2Raw = smoothAnalogRead(MQ2_PIN, 5);
  float mq7Raw = smoothAnalogRead(MQ7_PIN, 5);
  float mq2PPM = mq2ToPPM(mq2Raw);
  float mq7PPM = mq7ToPPM(mq7Raw);

  bool raining = digitalRead(RAIN_DIGITAL_PIN) == LOW;

  // Simulated wind data
  float wind_speed = random(0, 10) / 2.5;
  int wind_direction = random(0, 360);

  // Timestamp
  char timestamp[16];
  sprintf(timestamp, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 5000) {
    printSensorData(timestamp, temperature, humidity, atm_pressure, specificHumidity,
                   heat_index, enthalpy, dew_point, soil_moisture, sunlight_intensity,
                   batteryPercent, mq2PPM, mq7PPM, raining, gps.location.lat(), gps.location.lng(),
                   wind_speed, wind_direction);
    lastDebug = millis();
  }

  // LED heartbeat
  unsigned long currentMillis = millis();
  if (currentMillis - lastLEDTime >= 2000) {
    digitalWrite(INTERNET_LED, HIGH);
    delay(100);
    digitalWrite(INTERNET_LED, LOW);
    lastLEDTime = currentMillis;
  }

  // Data transmission
  if (currentMillis - lastSendTime >= 10000) {
    saveToSD(timestamp, temperature, humidity, atm_pressure, heat_index, enthalpy,
             dew_point, soil_moisture, sunlight_intensity, mq2PPM, mq7PPM,
             raining, gps.location.lat(), gps.location.lng(), wind_speed, wind_direction, batteryPercent);

    if (sendDataToServer(timestamp, temperature, humidity, atm_pressure, heat_index,
                        enthalpy, dew_point, soil_moisture, sunlight_intensity,
                        mq2PPM, mq7PPM, raining, gps.location.lat(), gps.location.lng(), wind_speed,
                        wind_direction, batteryPercent)) {
      blinkLED(DATA_SENT_LED, 3, 100);
    }
    lastSendTime = currentMillis;
  }
}

// ====================== BATTERY FUNCTIONS ======================
float getBatteryPercent(float voltage) {
  // Li-ion battery range (3.0V-4.2V)
  voltage = constrain(voltage, 3.0, 4.2);
  return (voltage - 3.0) / (4.2 - 3.0) * 100.0;
}

// ====================== SENSOR FUNCTIONS ======================
void initializeSensors() {
  if (!bme.begin(0x76)) Serial.println("BME280 failed!");
  dht.begin();
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(RAIN_DIGITAL_PIN, INPUT);
}

void calibrateMQSensors() {
  Serial.println("Calibrating MQ sensors...");
  float mq2Sum = 0, mq7Sum = 0;
  for (int i = 0; i < MQ_SAMPLES; i++) {
    mq2Sum += smoothAnalogRead(MQ2_PIN, 5);
    mq7Sum += smoothAnalogRead(MQ7_PIN, 5);
    delay(100);
  }
  mq2Baseline = mq2Sum / MQ_SAMPLES;
  mq7Baseline = mq7Sum / MQ_SAMPLES;
  Serial.printf("MQ-2 Baseline: %.1f | MQ-7 Baseline: %.1f\n", mq2Baseline, mq7Baseline);
}

float smoothAnalogRead(int pin, int samples) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return sum / samples;
}

float mq2ToPPM(float rawValue) {
  float sensorVoltage = (rawValue / 4095.0) * ADC_REF;
  float rs = (5.0 - sensorVoltage) / sensorVoltage * MQ2_RL;
  float ratio = rs / MQ2_RO_CLEAN_AIR;
  return 1000 * pow(10, (log10(ratio) - 0.48) / -0.28); // LPG PPM
}

float mq7ToPPM(float rawValue) {
  float sensorVoltage = (rawValue / 4095.0) * ADC_REF;
  float rs = (5.0 - sensorVoltage) / sensorVoltage * MQ7_RL;
  float ratio = rs / MQ7_RO_CLEAR_AIR;
  return 1000 * pow(10, (log10(ratio) - 0.72) / -0.34); // CO PPM
}

// ====================== ENVIRONMENTAL CALCULATIONS ======================
float computeSpecificHumidity(float tempC, float relHumidity, float pressure) {
  float Psat = 0.61094 * exp((17.625 * tempC) / (tempC + 243.04));
  float Pv = (relHumidity / 100.0) * Psat;
  return 0.622 * Pv / (pressure - Pv);
}

float computeDewPoint(float tempC, float humidity) {
  const float a = 17.62, b = 243.12;
  float gamma = (a * tempC) / (b + tempC) + log(humidity / 100.0);
  return (b * gamma) / (a - gamma);
}

// ====================== DATA LOGGING & TRANSMISSION ======================
void initializeSDCard() {
  if (!SD.begin(SD_CS)) Serial.println("SD Card failed!");
  else {
    if (!SD.exists("data.csv")) {
      dataFile = SD.open("data.csv", FILE_WRITE);
      dataFile.println("Time,Temp,Humidity,Pressure,SpecificHumidity,HeatIndex,Enthalpy,DewPoint,Soil%,Sunlight%,Battery%,MQ2_PPM,MQ7_PPM,Rain,Lat,Long,WindSpd,WindDir");
      dataFile.close();
    }
  }
}

void saveToSD(String time, float temp, float hum, float pressure, float heatIndex, 
              float enthalpy, float dewPoint, float soilMoisture, float sunlight,
              float mq2PPM, float mq7PPM, bool rainStatus, float lat, float lng,
              float windSpd, int windDir, float batteryPct) {
  dataFile = SD.open("data.csv", FILE_APPEND);
  if (dataFile) {
    dataFile.printf("%s,%.2f,%.2f,%.2f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.6f,%.6f,%.2f,%d\n",
                    time.c_str(), temp, hum, pressure, computeSpecificHumidity(temp, hum, pressure),
                    heatIndex, enthalpy, dewPoint, soilMoisture, sunlight, batteryPct,
                    mq2PPM, mq7PPM, rainStatus ? "Rain" : "Dry", lat, lng, windSpd, windDir);
    dataFile.close();
  }
}

bool sendDataToServer(String time, float temp, float hum, float pressure, float heatIndex,
                      float enthalpy, float dewPoint, float soilMoisture, float sunlight,
                      float mq2PPM, float mq7PPM, bool rainStatus, float lat, float lng,
                      float windSpd, int windDir, float batteryPct) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("https://api.gaiathonfuta.online/model/modeldata/create/");
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"Time\":\"" + time + "\",";
    payload += "\"temperature\":" + String(temp, 2) + ",";
    payload += "\"humidity\":" + String(hum, 2) + ",";
    payload += "\"atm_pressure\":" + String(pressure, 2) + ",";
    payload += "\"specific_humidity\":" + String(computeSpecificHumidity(temp, hum, pressure), 6) + ",";
    payload += "\"heat_index\":" + String(heatIndex, 2) + ",";
    payload += "\"enthalpy\":" + String(enthalpy, 2) + ",";
    payload += "\"dew_point\":" + String(dewPoint, 2) + ",";
    payload += "\"soil_moisture\":" + String(soilMoisture, 2) + ",";
    payload += "\"sunlight_intensity\":" + String(sunlight, 2) + ",";
    payload += "\"battery_percentage\":" + String(batteryPct, 1) + ","; // Changed to percentage
    payload += "\"pollution_level\":" + String(mq2PPM, 2) + ",";
    payload += "\"co_level\":" + String(mq7PPM, 2) + ",";
    payload += "\"raining\":\"" + String(rainStatus ? "Rain" : "Dry") + "\",";
    payload += "\"latitude\":" + String(lat, 5) + ",";
    payload += "\"longitude\":" + String(lng, 5) + ",";
    payload += "\"wind_speed\":" + String(windSpd, 2) + ",";
    payload += "\"wind_direction\":" + String(windDir);
    payload += "}";

    int responseCode = http.POST(payload);
    http.end();
    return responseCode == 200 || responseCode == 201;
  }
  return false;
}

// ====================== UTILITIES ======================
void printSensorData(String time, float temp, float hum, float pressure, float specHum,
                    float heatIdx, float enthalpy, float dewPt, float soil, float sun,
                    float batt, float mq2, float mq7, bool rain, float lat, float lng,
                    float windSpd, int windDir) {
  Serial.println("\n===== SENSOR DATA =====");
  Serial.printf("Time: %s\n", time.c_str());
  Serial.printf("Temperature: %.2f C\n", temp);
  Serial.printf("Humidity: %.2f %%\n", hum);
  Serial.printf("Pressure: %.2f hPa\n", pressure);
  Serial.printf("Specific Humidity: %.6f kg/kg\n", specHum);
  Serial.printf("Heat Index: %.2f C\n", heatIdx);
  Serial.printf("Enthalpy: %.2f kJ/kg\n", enthalpy);
  Serial.printf("Dew Point: %.2f C\n", dewPt);
  Serial.printf("Soil Moisture: %.2f %%\n", soil);
  Serial.printf("Sunlight: %.2f %%\n", sun);
  Serial.printf("Battery: %.1f %%\n", batt); // Now shows percentage
  Serial.printf("MQ-2 (Pollution): %.2f PPM\n", mq2);
  Serial.printf("MQ-7 (CO): %.2f PPM\n", mq7);
  Serial.printf("Rain: %s\n", rain ? "Yes" : "No");
  Serial.printf("GPS: %.6f, %.6f\n", lat, lng);
  Serial.printf("Wind: %.2f km/h @ %d°\n", windSpd, windDir);
  Serial.println("========================");
}

void blinkLED(int pin, int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}
