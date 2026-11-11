#include <QTRSensors.h>
#include "configuration.h"

QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// ================= PID VARIABLES =================
float prevError = 0.0f;
bool started = false;

// ================= DEBUG TELEMETRY =================
#define DEBUG_INTERVAL_MS 100
unsigned long lastDebugTime = 0;

// ================= MOTOR CONTROL =================
void setMotorRaw(int leftSpeed, int rightSpeed)
{
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0)
    {
        analogWrite(IN1, leftSpeed);
        analogWrite(IN2, 0);
    }
    else
    {
        analogWrite(IN1, 0);
        analogWrite(IN2, -leftSpeed);
    }

    if (rightSpeed >= 0)
    {
        analogWrite(IN3, rightSpeed);
        analogWrite(IN4, 0);
    }
    else
    {
        analogWrite(IN3, 0);
        analogWrite(IN4, -rightSpeed);
    }
}

// ================= SENSOR BAR VISUALIZER =================
void printSensorBar()
{
    qtr.read(sensorValues);
    // Serial.print(" SENS [");
    // for (uint8_t i = 0; i < SensorCount; i++)
    // {
    //     if (sensorValues[i] < 900)
    //         Serial.print("█");
    //     else
    //         Serial.print("·");
    // }
    // Serial.print("] ");
    Serial.print("RAW:");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(" ");
        Serial.print(sensorValues[i]);
    }
    Serial.println();
}

// ================= DEBUG TELEMETRY PRINTER =================
void printDebug(float position, float error, float P, float correction, int leftMotorSpeed, int rightMotorSpeed, float ir_error)
{

    if (millis() - lastDebugTime < DEBUG_INTERVAL_MS)
        return;
    lastDebugTime = millis();

    printSensorBar();
    Serial.println();

    // Serial.print("POS:");
    // Serial.print((int)position);
    Serial.print("  ERR:");
    Serial.print(error, 2);

    // Serial.print("  P:");
    // Serial.print(P, 3);
    // Serial.print("  COR:");
    // Serial.print(correction, 2);
    // Serial.print("  IR_Rrror:");
    // Serial.print(ir_error, 2);
}

// ================= PID LOGIC =================
unsigned long lastTime = 0;

void followLinePID()
{
    float error = 0;

    // Read calibrated position: 0 (far left) → 5000 (far right) for 6 sensors
    qtr.read(sensorValues);

    // Dynamic threshold using calibration
    int weights[SensorCount] = {-3, -2, -1, 0, 0, 1, 2, 3};
    float position = 0;
    int activeSensors = 0;

    for (int i = 0; i < SensorCount; i++)
    {
        int binary = (sensorValues[i] < 900) ? 1 : 0;

        if (binary) activeSensors++;

        position += binary * weights[i];
    }

    // If no sensor detects line → lost line protection
    if (activeSensors == 0) {
        position = (prevError > 0) ? 12 : -12;
    }

    float ir_error = (float)position / 2;

    // ir_error > 0 ? left : right;
    error += ir_error;

    // --- P ---
    float P = Kp * error;

    // --- Correction ---
    float correction = P;

    // --- Motor speeds ---
    int leftMotorSpeed = (int)(baseSpeed - correction);
    int rightMotorSpeed = (int)(baseSpeed + correction);

    setMotorRaw(leftMotorSpeed, rightMotorSpeed);

    // --- Debug output ---
    printDebug(position, error, P, correction, leftMotorSpeed, rightMotorSpeed, ir_error);
}

// ================= SETUP =================
void setup()
{
    Serial.begin(115200);

    // Motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    setMotorRaw(0, 0);

    // QTR Sensor setup
    qtr.setTypeAnalog();
    qtr.setSensorPins(SensorPins, SensorCount);

    delay(500);
    // ---- CALIBRATION ----
    // runAutoCalibration();

    Serial.println();
    Serial.println("Commands:");
    Serial.println("  START        → Begin line following");
    Serial.println("  STOP         → Stop motors");
    Serial.println("  KP:<value>   → Set Kp  (e.g. KP:0.08)");
    Serial.println("  SPEED:<val>  → Set base speed (e.g. SPEED:150)");
    Serial.println("  TUNE         → Print current PID values");
    Serial.println("===========================================");
}

// ================= MAIN LOOP =================
void loop()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd == "START")
        {
            prevError = 0.0f;
            started = true;
            Serial.println("🚀 GOING!");
        }
        else if (cmd == "STOP")
        {
            started = false;
            setMotorRaw(0, 0);
            Serial.println("⛔ STOPPED");
        }
        else if (cmd.startsWith("KP:"))
        {
            Kp = cmd.substring(3).toFloat();
            Serial.print("✅ Kp = ");
            Serial.println(Kp, 6);
        }
        else if (cmd.startsWith("SPEED:"))
        {
            baseSpeed = cmd.substring(6).toInt();
            Serial.print("✅ Base Speed = ");
            Serial.println(baseSpeed);
        }
        else if (cmd == "TUNE")
        {
            Serial.println("--- Current PID Values ---");
            Serial.print("  Kp = ");
            Serial.println(Kp, 6);
            Serial.print("  Speed = ");
            Serial.println(baseSpeed);
            Serial.println("--------------------------");
        }
        else
        {
            Serial.print("❓ Unknown command: ");
            Serial.println(cmd);
        }
    }

    if (started)
    {
        followLinePID();
    }
}
