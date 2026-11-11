#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
    Serial.begin(9600);

    qtr.setTypeAnalog();

    // ✅ Use all 8 analog pins on Nano
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

    delay(1000);

    Serial.println("Starting Calibration... Move sensor over line!");

    for (uint16_t i = 0; i < 300; i++) {
        qtr.calibrate();
        delay(20);
    }

    Serial.println("Calibration Complete!");
    delay(1500);
}

void loop() {
    uint16_t position = qtr.readLineBlack(sensorValues);

    int center = ((SensorCount - 1) * 1000) / 2;
    int error = position - center;

    for (uint8_t i = 0; i < SensorCount; i++) {
        // Adjust threshold if needed
        if (sensorValues[i] < 500)
            Serial.print("W | ");
        else
            Serial.print("B | ");

        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }

    Serial.print("   POS: ");
    Serial.print(position);
    Serial.print("   ERR: ");
    Serial.print(error);

    Serial.println();
    delay(50);
}