#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#include <Arduino.h>

#define NUM_SENSORS 5
#define EMIT_PIN 11

class LineSensors_c {

private:

    int sensorPins[NUM_SENSORS];

    float raw[NUM_SENSORS];
    float calibrated[NUM_SENSORS];

    float minimum[NUM_SENSORS];
    float maximum[NUM_SENSORS];

public:

    // Constructor
    LineSensors_c(const int pins[NUM_SENSORS]) {
        for(int i = 0; i < NUM_SENSORS; i++){
            sensorPins[i] = pins[i];
            minimum[i] = 1023.0;
            maximum[i] = 0.0;
        }
    }

    // Initialize pins
    void begin() {
        pinMode(EMIT_PIN, OUTPUT);
        digitalWrite(EMIT_PIN, HIGH);

        for(int i = 0; i < NUM_SENSORS; i++){
            pinMode(sensorPins[i], INPUT_PULLUP);
        }
    }

    // -------------------------
    // RAW DATA
    // -------------------------
    void readRawData() {
        for(int i = 0; i < NUM_SENSORS; i++){
            raw[i] = analogRead(sensorPins[i]);
        }
    }

    float getRaw(int i) {
        return raw[i];
    }

    // -------------------------
    // CALIBRATION
    // -------------------------
    void startCalibration() {
        for(int i = 0; i < NUM_SENSORS; i++){
            minimum[i] = 1023.0;
            maximum[i] = 0.0;
        }
    }

    void updateCalibration() {
        readRawData();

        for(int i = 0; i < NUM_SENSORS; i++){
            if(raw[i] < minimum[i]) minimum[i] = raw[i];
            if(raw[i] > maximum[i]) maximum[i] = raw[i];
        }
    }

    void finishCalibration() {}

    // -------------------------
    // NORMALIZED VALUES
    // -------------------------
    void readCalibrated() {

        readRawData();

        for(int i = 0; i < NUM_SENSORS; i++){

            float range = maximum[i] - minimum[i];
            if(range < 1.0) range = 1.0;

            calibrated[i] = (raw[i] - minimum[i]) / range;

            if(calibrated[i] < 0.0) calibrated[i] = 0.0;
            if(calibrated[i] > 1.0) calibrated[i] = 1.0;
        }
    }

    float getCalibrated(int i) {
        return calibrated[i];
    }

};

#endif
