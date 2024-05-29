#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CScd4x.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <lorawan.h>
#include "esp_sleep.h"

// OTAA credentials
const char *devEui = "0123456789ABCDEF";
const char *appEui = "0102030405060708";
const char *appKey = "00000000000000000000000000000000";

char outStr[255];
byte recvStatus = 0;

// LoRaWAN Pin Mapping
const sRFM_pins RFM_pins = {
  .CS = 14,
  .RST = 32,
  .DIO0 = 26,
  .DIO1 = 25,
};

#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

SensirionI2CSen5x sen5x;
SensirionI2CScd4x scd4x;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

const int mq4SensorPin = 34;
const int xPin = 39;
const int yPin = 36;
const int zPin = 35;
const int sd_cs = 5;
const float zeroG = 1.65;
const float scale = 0.330;

// Data File
File dataFile;

// Sensor Data Variables
float massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0;
float ambientHumidity, ambientTemperature, vocIndex, noxIndex;
uint16_t co2;
float temperature, humidity;
int methaneSensorValue;
float xAccel, yAccel, zAccel;

// Shake to Wake
unsigned long lastDataCollectionTime = 0; // Last time the data was collected
unsigned long dataCollectionInterval = 30000; // Default to 30 seconds when not moving
unsigned long motionDetectedStartTime = 0;
bool isMotionPeriodActive = false;

// Flags for sensor data readiness
bool co2DataReady = false;
bool gpsDataReady = false;
static bool gpsDataProcessed = false;
bool isJoinedToNetwork = false; // To manage the LoRa connection state
static bool slow;
static bool deep_sleep = false;

// Deep Sleep
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for microseconds to seconds
#define SLEEP_15_MIN (15 * 60)     // Time ESP32 will wake up to send data (in seconds)
#define OPERATIONAL_6_HOUR (6 * 60 * 60) // Operational time before deep sleep (in seconds)
RTC_DATA_ATTR static int bootCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    ++bootCount;
    print_wakeup_reason();

    if (bootCount == 1) {
      // Only the first boot we start counting for 6 hours
      esp_sleep_enable_timer_wakeup(OPERATIONAL_6_HOUR * uS_TO_S_FACTOR);
    } else {
      // After waking up from 15 minutes sleep
      esp_sleep_enable_timer_wakeup(SLEEP_15_MIN * uS_TO_S_FACTOR);
    }

    Wire.begin();
    initializeSensors();
    initializeGPS();
    initializeSDCard();
    initializeAnalogPins();
    initializeLoRa();  // Initialize LoRa
    Serial.println("Initializations done...");
    delay(5000);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void initializeLoRa() {
    if (!lora.init()) {
        Serial.println("ERROR: RFM95 not detected");
        return;
    }

    lora.setDeviceClass(CLASS_A);
    lora.setDataRate(SF9BW125);
    lora.setChannel(MULTI);
    // Set OTAA credentials
    lora.setDevEUI(devEui);
    lora.setAppEUI(appEui);
    lora.setAppKey(appKey);
    Serial.println("LoRa initialization done, waiting for network...");
}

void initializeSensors() {
    setupSen5x();
    setupScd4x();
    pinMode(mq4SensorPin, INPUT);
}

void setupSen5x() {
    sen5x.begin(Wire);
    uint16_t sen5xError = sen5x.deviceReset();
    checkSensorErrors(sen5xError, "SEN5x deviceReset");
    sen5xError = sen5x.startMeasurement();
    checkSensorErrors(sen5xError, "SEN5x startMeasurement");
}

void setupScd4x() {
    scd4x.begin(Wire);
    uint16_t scd4xError = scd4x.stopPeriodicMeasurement();
    checkSensorErrors(scd4xError, "SCD4x stopPeriodicMeasurement");
    scd4xError = scd4x.startPeriodicMeasurement();
    checkSensorErrors(scd4xError, "SCD4x startPeriodicMeasurement");
}

void checkSensorErrors(uint16_t error, const char* message) {
    if (error) {
        char errorMessage[256];
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.print(message); Serial.print(": "); Serial.println(errorMessage);
    }
}

void initializeGPS() {
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    Serial.println("GPS Module Test");
}

void initializeSDCard() {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, sd_cs);
    delay(100);
    if (!SD.begin(sd_cs)) {
        Serial.println("Card Mount Failed");
        return;
    }
    Serial.println("Card Mount Success");
    if (!SD.mkdir("/logs")) {
        Serial.println("Directory creation failed");
    } else {
        Serial.println("Directory created");
    }
}

void initializeAnalogPins() {
    analogReadResolution(12);
    Serial.println("Sensor setup complete...");
}

void logDataToSDCard(String dataLine) {
    dataFile = SD.open("/logs/dataFile.csv", FILE_APPEND);
    Serial.print("SD CARD: ");
    if (dataFile) {
        dataFile.println(dataLine);
        dataFile.close();
        Serial.println("DATA LOGGED!\n");
    } else {
        Serial.println("ERROR: Unable to open dataFile.csv");
    }
}

void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastDeepSleepStart = millis();

  if (millis() - lastDeepSleepStart >= (OPERATIONAL_6_HOUR * 1000)) {
    // Time to go to deep sleep after 6 hours of operation
    Serial.println("END OF OPERATIONAL TIME: DEEP SLEEP");
    enterDeepSleep(SLEEP_15_MIN);
    deep_sleep = true;
  }

  readADXL335();

  handleDataCollectionTiming(currentTime);

  // Collect data based on the defined interval
  if (deep_sleep) {
    Serial.println("_____DEEP SLEEP MODE ACTIVE_____");

    attemptJoinLoRa();
    if (isJoinedToNetwork) {
      gpsDataProcessed = false;
      readGPS();
      sendSensorDataOverLoRa();
    }
  }
  else if (currentTime - lastDataCollectionTime >= dataCollectionInterval) {
    if (slow) {
      Serial.println("_____SLOW COLLECTION ACTIVE_____");
      slow = false;
    }
      gpsDataProcessed = false;
      readSEN5x();
      readSCD4x();
      readMQ4();
      readGPS();
      attemptJoinLoRa();
      if (isJoinedToNetwork) {
        sendSensorDataOverLoRa();
        // Resend data when docked 
        if (slow) {
          //resendStoredData();
        }
      }
      else {
        writeToSDCard();
      }
      lastDataCollectionTime = currentTime; // Update last data collection time
  }

  // Ensure accelerometer is sampled every 100 ms
  // delay(100 - (millis() - currentTime));
}

void resendStoredData() {
    if (!SD.exists("/logs/dataFile.csv")) {
        Serial.println("No stored data to send.");
        return;
    }

    File originalFile = SD.open("/logs/dataFile.csv", FILE_READ);
    if (!originalFile) {
        Serial.println("Failed to open stored data file for reading.");
        return;
    }

    File tempFile = SD.open("/logs/temp.csv", FILE_WRITE);
    if (!tempFile) {
        Serial.println("Failed to open temporary file for writing.");
        originalFile.close();
        return;
    }

    while (originalFile.available()) {
        String dataLine = originalFile.readStringUntil('\n');
        if (dataLine.length() > 1) {
            char buffer[256];
            dataLine.toCharArray(buffer, sizeof(buffer));
            lora.sendUplink(buffer, strlen(buffer), 0, 1);
            Serial.print("Resending stored data: ");
            Serial.println(buffer);
            delay(100);
            tempFile.println(dataLine);
        }
    }

    originalFile.close();
    tempFile.close();

    // Replace the original file with the temporary file
    SD.remove("/logs/dataFile.csv");
    SD.rename("/logs/temp.csv", "/logs/dataFile.csv");
}


void enterDeepSleep(unsigned long sleepTimeSeconds) {
  esp_sleep_enable_timer_wakeup(sleepTimeSeconds * uS_TO_S_FACTOR);
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void handleDataCollectionTiming(unsigned long currentTime) {
    bool moving = isDeviceMoving();

    if (moving) {
        if (!isMotionPeriodActive) {
            isMotionPeriodActive = true; 
            motionDetectedStartTime = currentTime;
        }
    }

    // Determine if we are still within the 10 minutes window
    if (isMotionPeriodActive) {
        if (currentTime - motionDetectedStartTime < 60000) { // Set to 1 min for testing
            dataCollectionInterval = 5000;
            Serial.println("_____FAST COLLECTION ACTIVE_____");
        } else {
            isMotionPeriodActive = false;
            dataCollectionInterval = 30000;
        }
    } else {
        dataCollectionInterval = 30000;
        slow = true;
    }
}

void attemptJoinLoRa() {
    Serial.print("LORA NETWORK: ");
    isJoinedToNetwork = lora.join();
    if (isJoinedToNetwork) {
        Serial.println("JOINED!");
    } else {
        Serial.println("UNAVAILABLE...");
    }
}

void sendSensorDataOverLoRa() {
    char data[256];
    createDataString(data, sizeof(data));

    lora.sendUplink(data, strlen(data), 0, 1);
    Serial.print("LoRa Data Sent\n");
    Serial.print(data);
    Serial.print("\n");

    // Receiving
    recvStatus = lora.readData(outStr);
    if(recvStatus) {
      Serial.println(outStr);
    }

    // Check Lora RX
    lora.update();
}

void createDataString(char* buffer, size_t bufSize) {
    snprintf(buffer, bufSize, "%04d-%02d-%02d,%02d.%02d.%02d,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.location.lat(), gps.location.lng(),
            massConcentrationPm1p0, massConcentrationPm2p5,
            massConcentrationPm4p0, massConcentrationPm10p0,
            ambientHumidity, ambientTemperature, vocIndex, noxIndex,
            co2, temperature, humidity, methaneSensorValue);
}

bool isDeviceMoving() {
  float magnitude = sqrt(xAccel * xAccel + yAccel * yAccel + zAccel * zAccel);

  if (magnitude > 1.7 || magnitude < 0.8) {
    return true;
  }
  return false;
}


void readADXL335() {
    int xRaw = analogRead(xPin);
    int yRaw = analogRead(yPin);
    int zRaw = analogRead(zPin);
    float xVoltage = (xRaw / 4095.0) * 3.3;
    float yVoltage = (yRaw / 4095.0) * 3.3;
    float zVoltage = (zRaw / 4095.0) * 3.3;
    xAccel = (xVoltage - zeroG) / scale;
    yAccel = (yVoltage - zeroG) / scale;
    zAccel = (zVoltage - zeroG) / scale;
    // Serial.printf("______ADXL335______\nX: %.3f g, Y: %.3f g, Z: %.3f g\n", xAccel, yAccel, zAccel);
}

void readSEN5x() {
    uint16_t sen5xError = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if (!sen5xError) {
        Serial.println("_____SEN55______");
        Serial.printf("PM1.0: %.2f\n", massConcentrationPm1p0);
        Serial.printf("PM2.5: %.2f\n", massConcentrationPm2p5);
        Serial.printf("PM4.0: %.2f\n", massConcentrationPm4p0);
        Serial.printf("PM10: %.2f\n", massConcentrationPm10p0);
        Serial.printf("Humidity: %.2f\n", ambientHumidity);
        Serial.printf("Temperature: %.2f\n", ambientTemperature);
        Serial.printf("VOC: %.2f\n", vocIndex);
        Serial.printf("NOx: %.2f\n", noxIndex);
    } else {
        checkSensorErrors(sen5xError, "SEN5x readMeasuredValues");
    }
}

void readSCD4x() {
    bool isDataReady = false;
    uint16_t scd4xError = scd4x.getDataReadyFlag(isDataReady);
    if (!scd4xError && isDataReady) {
        scd4xError = scd4x.readMeasurement(co2, temperature, humidity);
        if (!scd4xError) {
            Serial.println("______SCD4x_____");
            Serial.printf("CO2: %u ppm\n", co2);
            Serial.printf("Temperature: %.2f °C\n", temperature);
            Serial.printf("Humidity: %.2f%%\n", humidity);
            co2DataReady = true;  // Set the flag indicating data is ready
        } else {
            checkSensorErrors(scd4xError, "SCD4x readMeasurement");
        }
    } else if (!isDataReady) {
        Serial.println("SCD4x data not ready.");
    }
}

void readMQ4() {
    methaneSensorValue = analogRead(mq4SensorPin);
    Serial.printf("______MQ4_____\nCH4: %d\n", methaneSensorValue);
}

void readGPS() {
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        if (gps.encode(c) && !gpsDataProcessed) {
          Serial.print("_____GPS_____\n");
            if (gps.location.isValid()) {
                Serial.print("Latitude: ");
                Serial.println(gps.location.lat(), 6);
                Serial.print("Longitude: ");
                Serial.println(gps.location.lng(), 6);
            }
            if (gps.date.isValid()) {
                Serial.print("Date: ");
                Serial.print(gps.date.day());
                Serial.print("/");
                Serial.print(gps.date.month());
                Serial.print("/");
                Serial.println(gps.date.year());
            }
            if (gps.time.isValid()) {
                Serial.print("Time: ");
                Serial.print(gps.time.hour());
                Serial.print(":");
                Serial.print(gps.time.minute());
                Serial.print(":");
                Serial.println(gps.time.second());
            }
            // Serial.print("\n");
            gpsDataProcessed = true;
        }
    }
}

void writeToSDCard() {
    String dataLine = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + "," +
                      String(gps.time.hour()) + "." + String(gps.time.minute()) + "." + String(gps.time.second()) + "," +
                      String(gps.location.lat(), 6) + "," +
                      String(gps.location.lng(), 6) + "," +
                      String(massConcentrationPm1p0) + "," + 
                      String(massConcentrationPm2p5) + "," + 
                      String(massConcentrationPm4p0) + "," + 
                      String(massConcentrationPm10p0) + "," + 
                      String(ambientHumidity) + "," + 
                      String(ambientTemperature) + "," + 
                      String(vocIndex) + "," + 
                      String(noxIndex) + "," + 
                      String(co2) + "," + 
                      String(temperature) + "," + 
                      String(humidity) + "," + 
                      String(methaneSensorValue) + "," + 
                      String(xAccel, 3) + "," + 
                      String(yAccel, 3) + "," + 
                      String(zAccel, 3);
    logDataToSDCard(dataLine);
}
