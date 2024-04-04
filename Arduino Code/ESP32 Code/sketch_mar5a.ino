#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CScd4x.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>

SensirionI2CSen5x sen5x;
SensirionI2CScd4x scd4x;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // UART1 for GPS
const int mq4SensorPin = 34; //MQ4 Pin

// ADXL335 analog pins connected to ESP32
const int xPin = 39;
const int yPin = 36;
const int zPin = 35;

// ADXL335 setup values
const float zeroG = 1.65;
const float scale = 0.330;


void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    // Initialize SEN5x
    sen5x.begin(Wire);
    uint16_t sen5xError = sen5x.deviceReset();
    if (sen5xError) {
        char errorMessage[256];
        errorToString(sen5xError, errorMessage, sizeof(errorMessage));
        Serial.print("SEN5x Error trying to execute deviceReset(): ");
        Serial.println(errorMessage);
    }

    // Initialize SCD4x
    scd4x.begin(Wire);
    uint16_t scd4xError = scd4x.stopPeriodicMeasurement();
    if (scd4xError) {
        char errorMessage[256];
        errorToString(scd4xError, errorMessage, sizeof(errorMessage));
        Serial.print("SCD4x Error trying to execute stopPeriodicMeasurement(): ");
        Serial.println(errorMessage);
    }

    // Start measurements for SEN5x
    sen5xError = sen5x.startMeasurement();
    if (sen5xError) {
        char errorMessage[256];
        errorToString(sen5xError, errorMessage, sizeof(errorMessage));
        Serial.print("SEN5x Error trying to execute startMeasurement(): ");
        Serial.println(errorMessage);
    }

    // Start measurements for SCD4x
    scd4xError = scd4x.startPeriodicMeasurement();
    if (scd4xError) {
        char errorMessage[256];
        errorToString(scd4xError, errorMessage, sizeof(errorMessage));
        Serial.print("SCD4x Error trying to execute startPeriodicMeasurement(): ");
        Serial.println(errorMessage);
    }

    // Initialize MQ4
    pinMode(mq4SensorPin, INPUT);

    // Initialize GPS
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Start GPS serial communication
    Serial.println("GPS Module Test");

    // Initialize analog pins
    analogReadResolution(12); // ESP32 uses 12 bits

    Serial.println("Setup complete, starting measurements...");
}

void loop() {

    // Read raw values
    int xRaw = analogRead(xPin);
    int yRaw = analogRead(yPin);
    int zRaw = analogRead(zPin);

    // Convert raw readings to voltage
    float xVoltage = (xRaw / 4095.0) * 3.3; // Convert analog reading to voltage
    float yVoltage = (yRaw / 4095.0) * 3.3;
    float zVoltage = (zRaw / 4095.0) * 3.3;

    // Convert voltage to acceleration
    float xAccel = (xVoltage - zeroG) / scale;
    float yAccel = (yVoltage - zeroG) / scale;
    float zAccel = (zVoltage - zeroG) / scale;

    // Output the acceleration values to the serial monitor
    Serial.print("X: ");
    Serial.print(xAccel, 3); // 3 decimal places for accuracy
    Serial.print(" g, Y: ");
    Serial.print(yAccel, 3);
    Serial.print(" g, Z: ");
    Serial.print(zAccel, 3);
    Serial.println(" g");

    static bool gpsDataProcessed = false;
    gpsDataProcessed = false;

    // Read and print SEN5x measurements
    float massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0;
    float ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t sen5xError = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if (!sen5xError) {
        // Print SEN5x measurements
        Serial.println("SEN5x Measurements:");
        Serial.print("MassConcentrationPm1p0:");
        Serial.print(massConcentrationPm1p0);
        Serial.print("\n");
        Serial.print("MassConcentrationPm2p5:");
        Serial.print(massConcentrationPm2p5);
        Serial.print("\n");
        Serial.print("MassConcentrationPm4p0:");
        Serial.print(massConcentrationPm4p0);
        Serial.print("\n");
        Serial.print("MassConcentrationPm10p0:");
        Serial.print(massConcentrationPm10p0);
        Serial.print("\n");
        Serial.print("AmbientHumidity:");
        if (isnan(ambientHumidity)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientHumidity);
        }
        Serial.print("\n");
        Serial.print("AmbientTemperature:");
        if (isnan(ambientTemperature)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientTemperature);
        }
        Serial.print("\n");
        Serial.print("VocIndex:");
        if (isnan(vocIndex)) {
            Serial.print("n/a");
        } else {
            Serial.print(vocIndex);
        }
        Serial.print("\n");
        Serial.print("NoxIndex:");
        if (isnan(noxIndex)) {
            Serial.println("n/a");
        } else {
            Serial.println(noxIndex);
        }
    } else {
        char errorMessage[256];
        errorToString(sen5xError, errorMessage, sizeof(errorMessage));
        Serial.print("SEN5x readMeasuredValues error: ");
        Serial.println(errorMessage);
    }

    // Read and print SCD4x measurements
    uint16_t co2;
    float temperature, humidity;
    bool isDataReady = false;
    uint16_t scd4xError = scd4x.getDataReadyFlag(isDataReady);
    if (!scd4xError && isDataReady) {
        scd4xError = scd4x.readMeasurement(co2, temperature, humidity);
        if (!scd4xError) {
            // Print SCD4x measurements
            Serial.println("SCD4x Measurements:");
            Serial.print("Co2:");
            Serial.print(co2);
            Serial.print("\n");
            Serial.print("Temperature:");
            Serial.print(temperature);
            Serial.print("\n");
            Serial.print("Humidity:");
            Serial.println(humidity);
        } else {
            char errorMessage[256];
            errorToString(scd4xError, errorMessage, sizeof(errorMessage));
            Serial.print("SCD4x readMeasurement error: ");
            Serial.println(errorMessage);
        }
    }

    // Read and Print MQ4 Measurements
    int methaneSensorValue = analogRead(mq4SensorPin);
    Serial.print("Methane Gas Sensor Value: ");
    Serial.println(methaneSensorValue);

    // Read and Print GPS Data
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read(); // Read a character from the GPS serial
        if (gps.encode(c) && !gpsDataProcessed) { // If new GPS data is decoded and not yet processed
            // It's important to keep decoding all available data to keep the GPS data stream fluent
            if (gps.location.isValid()) {
                // Print latitude and longitude once per loop after the delay
                Serial.print("Latitude: ");
                Serial.println(gps.location.lat(), 6); // 6 decimal places
                Serial.print("Longitude: ");
                Serial.println(gps.location.lng(), 6); // 6 decimal places
            }

            if (gps.date.isValid()) {
                // Print date
                Serial.print("Date: ");
                Serial.print(gps.date.day());
                Serial.print("/");
                Serial.print(gps.date.month());
                Serial.print("/");
                Serial.println(gps.date.year());
            }

            if (gps.time.isValid()) {
                // Print time
                Serial.print("Time: ");
                Serial.print(gps.time.hour());
                Serial.print(":");
                Serial.print(gps.time.minute());
                Serial.print(":");
                Serial.println(gps.time.second());
            }

            // After printing, set flag to true to avoid printing repeatedly in this loop iteration
            gpsDataProcessed = true;
        }
    }
    for (int i = 0; i < 5; i++){
    Serial.print("\n");
    }

    delay(5000);
}
