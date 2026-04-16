#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include <math.h>

// ================== OBJECTS ==================
Kinematics_c pose;
Motors_c motors;
PID_c headingPID;

// ================== NAVIGATION PARAMETERS ==================
const float DIST_TOLERANCE = 1.0;
const float BASE_PWM = 40.0;
const float MAX_TURN_PWM = 20.0;

// ================== NAVIGATION GLOBALS ==================
float goalX = 0;
bool gotoActive = false;
bool motionComplete = false;

// ================== SENSOR DEFINITIONS ==================
#define NUM_SENSORS 1
#define MAX_RESULTS 250
#define VARIABLES 2

const int sensorPins[NUM_SENSORS] = {A2}; //{A11, A0, A2, A3, A4}
LineSensors_c line(sensorPins);

// ================== RESULTS STORAGE ==================
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;

unsigned long record_results_ts = 0;
unsigned long results_interval_ms = 20;

// ================== FSM ==================
enum Stage {
  STATE_READING,
  STATE_REPORT
};

Stage currentStage = STATE_READING;

// ================== NAVIGATION ==================
void GOTO(float x) {
    goalX = x;
    gotoActive = true;
}

void updateMotionStraight() {

    if (!gotoActive) {
        motors.setPWM(0,0);
        motionComplete = true;
        return;
    }

    motionComplete = false;

    pose.update();

    float dx = goalX - pose.x;

    if (fabs(dx) < DIST_TOLERANCE) {
        motors.setPWM(0,0);
        gotoActive = false;
        motionComplete = true;
        headingPID.reset();
        return;
    }

    // Robot should face straight (0 radians)
    float heading_error = 0 - pose.theta;

    float turnPWM = constrain(
        headingPID.update(0, heading_error) * 180.0 / PI,
        -MAX_TURN_PWM,
        MAX_TURN_PWM
    );

    float leftPWM  = BASE_PWM + turnPWM;
    float rightPWM = BASE_PWM - turnPWM;

    motors.setPWM(leftPWM, rightPWM);
}

// ================== STATE: READING ==================
void handleReading() {

    GOTO(200);

    updateMotionStraight();

    line.readRawData();

    if (millis() - record_results_ts > results_interval_ms) {
        record_results_ts = millis();

        if (results_index < MAX_RESULTS) {
            results[results_index][0] = pose.x;
            results[results_index][1] = line.getRaw(0);
            results_index++;
        }
    }

    if (motionComplete) {
        motors.setPWM(0,0);
        currentStage = STATE_REPORT;
    }
}

// ================== STATE: REPORT ==================
void handleReport() {

    motors.setPWM(0,0);

    Serial.println("X,S3");

    for (int r=0; r<results_index; r++) {
        Serial.print(results[r][0]);
        Serial.print(",");
        Serial.print(results[r][1]);
        Serial.println();
    }

    delay(1000);
}

// ================== SETUP ==================
void setup() {

    Serial.begin(115200);
    delay(2000);

    setupEncoder0();
    setupEncoder1();

    motors.initialise();
    pose.initialise(0,0,0);

    line.begin();

    headingPID.initialise(1.6, 0.0, 0.4);

    record_results_ts = millis();
}

// ================== LOOP ==================
void loop() {

    switch(currentStage) {

        case STATE_READING:
            handleReading();
            break;

        case STATE_REPORT:
            handleReport();
            break;
    }
}
