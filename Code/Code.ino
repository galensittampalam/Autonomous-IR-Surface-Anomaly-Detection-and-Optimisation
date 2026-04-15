#include <Arduino.h>

#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "LineSensors.h"
#include "PID.h"

// ======================================================
// =================== CONFIGURATION ====================
// ======================================================

// ------------------- Sensors -------------------
const int sensorPins[NUM_SENSORS] = {A11, A0, A2, A3, A4};

// EMA filter
const float ALPHA = 0.6;
const float DETECT_THRESHOLD = 0.2;

// ------------------- PID -------------------
const float KP = 1.2;
const float KI = 0.0;
const float KD = 0.4;

// ------------------- Drive -------------------
const float BASE_PWM = 30.0;
const float BACKTRACK_SCALE = 0.8;
const float SCAN2_PWM = 12.0;
const float MAX_TURN_PWM = 20.0;

// ------------------- Timing -------------------
const unsigned long CALIB_TIME = 5000;
const unsigned long WAIT_5S = 5000;
const unsigned long WAIT_1S = 500;
const unsigned long EXIT_DELAY = 150;

// ======================================================
// ===================== OBJECTS ========================
// ======================================================
Motors_c motors;
Kinematics_c kinematics;
PID_c headingPID;
LineSensors_c sensors(sensorPins);

// ======================================================
// ===================== FSM ============================
// ======================================================
enum State {
    STATE_CALIBRATION,
    STATE_WAIT_5S,
    STATE_SCAN_1,
    STATE_WAIT_1S_A,
    STATE_BACKTRACK,
    STATE_WAIT_1S_B,
    STATE_SCAN_2,
    STATE_REPORT
};

State state = STATE_CALIBRATION;
State prevState = STATE_REPORT;

// ======================================================
// ===================== TIMERS =========================
// ======================================================
unsigned long stateTimer = 0;

// scan arming
bool scan1Armed = false;
unsigned long lastDetectionTime = 0;

bool scan2Armed = false;
unsigned long lastDetectionTime2 = 0;

// ======================================================
// ================= SENSOR DATA ========================
// ======================================================
float values[NUM_SENSORS];
float filtered[NUM_SENSORS];

// ======================================================
// ==================== STORAGE =========================
// ======================================================
float X_1_Start[NUM_SENSORS];
float X_1_End[NUM_SENSORS];
float maxSensor1[NUM_SENSORS];
bool inTarget1[NUM_SENSORS];

float X_2_Start[NUM_SENSORS];
float X_2_End[NUM_SENSORS];
float maxSensor2[NUM_SENSORS];
bool inTarget2[NUM_SENSORS];

float THRESH2[NUM_SENSORS];

// ======================================================
// ===================== SETUP ==========================
// ======================================================
void setup()
{
    Serial.begin(9600);

    motors.initialise();

    setupEncoder0();
    setupEncoder1();

    kinematics.initialise(0, 0, 0);

    sensors.begin();

    headingPID.initialise(KP, KI, KD);

    stateTimer = millis();

    sensors.readCalibrated();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        filtered[i] = sensors.getCalibrated(i);
    }

    lastDetectionTime = millis();
    lastDetectionTime2 = millis();
}

// ======================================================
// ================= SENSOR PIPELINE ====================
// ======================================================
void updateSensors()
{
    sensors.readCalibrated();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float input = sensors.getCalibrated(i);
        filtered[i] = ALPHA * input + (1.0 - ALPHA) * filtered[i];
        values[i] = filtered[i];
    }
}

// ======================================================
// ================== DRIVE STRAIGHT ====================
// ======================================================
void driveStraight(float speed)
{
    kinematics.update();

    float error = -kinematics.theta;

    float turn = headingPID.update(0, error) * 180.0 / PI;
    turn = constrain(turn, -MAX_TURN_PWM, MAX_TURN_PWM);

    motors.setPWM(speed + turn, speed - turn);
}

// ======================================================
// ============== STATE CHANGE RESET ====================
// ======================================================
void onStateChange()
{
    if (state == prevState) return;

    kinematics.initialise(0, 0, 0);

    if (state == STATE_SCAN_1)
    {
        scan1Armed = false;
        lastDetectionTime = millis();

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            inTarget1[i] = false;
            maxSensor1[i] = 0;
            X_1_Start[i] = 0;
            X_1_End[i] = 0;
        }
    }

    if (state == STATE_SCAN_2)
    {
        scan2Armed = false;
        lastDetectionTime2 = millis();

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            inTarget2[i] = false;
            maxSensor2[i] = 0;
            X_2_Start[i] = 0;
            X_2_End[i] = 0;
        }
    }

    prevState = state;
}

// ======================================================
// =================== STATES ===========================
// ======================================================

void stateCalibration()
{
    motors.turnLeft(30);
    sensors.updateCalibration();

    if (millis() - stateTimer > CALIB_TIME)
    {
        motors.stop();
        state = STATE_WAIT_5S;
        stateTimer = millis();
    }
}

void stateWait5s()
{
    motors.stop();

    if (millis() - stateTimer > WAIT_5S)
    {
        state = STATE_SCAN_1;
        stateTimer = millis();
    }
}

void stateScan1()
{
    driveStraight(BASE_PWM);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float v = values[i];
        bool detected = (v > DETECT_THRESHOLD);

        if (v > maxSensor1[i])
            maxSensor1[i] = v;

        if (detected)
        {
            scan1Armed = true;
            lastDetectionTime = millis();

            if (!inTarget1[i])
            {
                inTarget1[i] = true;
                X_1_Start[i] = kinematics.x;
            }
        }

        if (!detected && inTarget1[i])
        {
            inTarget1[i] = false;
            X_1_End[i] = kinematics.x;
        }
    }

    if (scan1Armed && millis() - lastDetectionTime > EXIT_DELAY)
    {
        motors.stop();
        state = STATE_WAIT_1S_A;
        stateTimer = millis();
    }
}

void stateWait1A()
{
    motors.stop();

    if (millis() - stateTimer > WAIT_1S)
    {
        state = STATE_BACKTRACK;
        stateTimer = millis();
    }
}

void stateBacktrack()
{
    driveStraight(-BACKTRACK_SCALE * BASE_PWM);

    static bool seen = false;
    static unsigned long lastSeen = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (values[i] > DETECT_THRESHOLD)
        {
            seen = true;
            lastSeen = millis();
        }
    }

    if (seen && millis() - lastSeen > 100)
    {
        motors.stop();
        state = STATE_WAIT_1S_B;
        stateTimer = millis();
        seen = false;
    }
}

void stateWait1B()
{
    motors.stop();

    if (millis() - stateTimer > WAIT_1S)
    {
        state = STATE_SCAN_2;
        stateTimer = millis();
    }
}

void stateScan2()
{
    for (int i = 0; i < NUM_SENSORS; i++)
        THRESH2[i] = 0.5 * maxSensor1[i];

    driveStraight(SCAN2_PWM);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float v = values[i];
        bool detected = (v > THRESH2[i]);

        if (v > maxSensor2[i])
            maxSensor2[i] = v;

        if (detected)
        {
            scan2Armed = true;
            lastDetectionTime2 = millis();

            if (!inTarget2[i])
            {
                inTarget2[i] = true;
                X_2_Start[i] = kinematics.x;
            }
        }

        if (!detected && inTarget2[i])
        {
            inTarget2[i] = false;
            X_2_End[i] = kinematics.x;
        }
    }

    if (scan2Armed && millis() - lastDetectionTime2 > EXIT_DELAY)
    {
        motors.stop();
        state = STATE_REPORT;
    }
}

void stateReport()
{
    motors.stop();

    float sumPeakP2 = 0;
    float sumWidthP2 = 0;
    int validCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float width2 = X_2_End[i] - X_2_Start[i];

        // only count valid detections
        if (maxSensor2[i] > 0)
        {
            sumPeakP2 += maxSensor2[i];
            sumWidthP2 += width2;
            validCount++;
        }
    }

    float avgPeakP2 = 0;
    float avgWidthP2 = 0;

    if (validCount > 0)
    {
        avgPeakP2 = sumPeakP2 / validCount;
        avgWidthP2 = sumWidthP2 / validCount;
    }

    Serial.println("===== P2 AVERAGES =====");
    Serial.print("Average Peak (P2): ");
    Serial.println(avgPeakP2);

    Serial.print("Average Width (P2): ");
    Serial.println(avgWidthP2);

    Serial.println("===== FINAL RESULTS =====");

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i);

        Serial.print(" | P1 Start: ");
        Serial.print(X_1_Start[i]);

        Serial.print(" | P1 End: ");
        Serial.print(X_1_End[i]);

        Serial.print(" | P1 Peak: ");
        Serial.print(maxSensor1[i]);

        Serial.print(" | Width1: ");
        Serial.print(X_1_End[i] - X_1_Start[i]);

        Serial.print(" || P2 Start: ");
        Serial.print(X_2_Start[i]);

        Serial.print(" | P2 End: ");
        Serial.print(X_2_End[i]);

        Serial.print(" | P2 Peak: ");
        Serial.print(maxSensor2[i]);

        Serial.print(" | Width2: ");
        Serial.println(X_2_End[i] - X_2_Start[i]);
    }
}

// ======================================================
// ===================== LOOP ===========================
// ======================================================
void loop()
{
    updateSensors();
    onStateChange();

    switch (state)
    {
        case STATE_CALIBRATION: stateCalibration(); break;
        case STATE_WAIT_5S:     stateWait5s(); break;
        case STATE_SCAN_1:       stateScan1(); break;
        case STATE_WAIT_1S_A:    stateWait1A(); break;
        case STATE_BACKTRACK:    stateBacktrack(); break;
        case STATE_WAIT_1S_B:    stateWait1B(); break;
        case STATE_SCAN_2:       stateScan2(); break;
        case STATE_REPORT:       stateReport(); break;
    }
}
