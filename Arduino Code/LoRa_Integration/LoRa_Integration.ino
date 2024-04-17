#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CScd4x.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <lorawan.h>

// OTAA credentials
const char *devEui = "0123456789ABCDEF";
const char *appEui = "0123456789ABCDEF";
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

// Flags for sensor data readiness
bool co2DataReady = false;
bool gpsDataReady = false;
static bool gpsDataProcessed = false;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    Wire.begin();
    initializeSensors();
    initializeGPS();
    initializeSDCard();
    initializeAnalogPins();
    initializeLoRa();  // Initialize LoRa
    Serial.println("Initializations done...");
    delay(5000);
}

void initializeLoRa() {

    if (!lora.init()) {
        Serial.println("ERROR: RFM95 not detected");
        return;
    }

    // Set LoRaWAN Class change CLASS_A or CLASS_C
    lora.setDeviceClass(CLASS_A);

    // Set Data Rate
    lora.setDataRate(SF9BW125);

    // set channel to random
    lora.setChannel(MULTI);
    
    // Put OTAA Key and DevAddress here
    lora.setDevEUI(devEui);
    lora.setAppEUI(appEui);
    lora.setAppKey(appKey);

    // Join procedure
    bool isJoined;
    do {
      Serial.println("Joining...");
      isJoined = lora.join();
      
      //wait for 10s to try again
      delay(10000);
    }while(!isJoined);
    Serial.println("Joined to network");
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
    Serial.print("_____SD Card_____\n");
    if (dataFile) {
        dataFile.println(dataLine);
        dataFile.close();
        Serial.println("Data Logged\n\n\n");
    } else {
        Serial.println("ERROR: Unable to open dataFile.csv\n\n\n");
    }
}

void loop() {
    gpsDataProcessed = false;
    readADXL335();
    readSEN5x();
    readSCD4x();
    readMQ4();
    readGPS();
    writeToSDCard();
    sendSensorDataOverLoRa();
    delay(5000);
}

void sendSensorDataOverLoRa() {
    char data[256];  // Ensure the array is large enough for your data
    createDataString(data);  // Pass the array to be populated

    lora.sendUplink(data, strlen(data), 0, 1);  // Use the populated array
    Serial.print("LoRa Data Sent\n");
    Serial.print(data);

    // Receiving
    recvStatus = lora.readData(outStr);
    if(recvStatus) {
      Serial.println(outStr);
    }
    
    // Check Lora RX
    lora.update();
}

void createDataString(char* buffer) {
    sprintf(buffer, "Temp: %.2f, Hum: %.2f, CO2: %u, PM2.5: %.2f, GPS Lat: %.6f, GPS Long: %.6f",
            ambientTemperature, ambientHumidity, co2, massConcentrationPm2p5, 
            gps.location.lat(), gps.location.lng());
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
    Serial.printf("______ADXL335______\nX: %.3f g, Y: %.3f g, Z: %.3f g\n\n\n", xAccel, yAccel, zAccel);
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
        Serial.printf("NOx: %.2f\n\n\n", noxIndex);
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
            Serial.printf("Humidity: %.2f%%\n\n\n", humidity);
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
    Serial.printf("______MQ4_____\nCH4: %d\n\n\n", methaneSensorValue);
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
            Serial.print("\n\n\n");
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
