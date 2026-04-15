#ifndef _MOTORS_H
#define _MOTORS_H

#include <Arduino.h>

// ================= CONFIG =================
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

#define MOTORS_MAX_PWM 180
#define FWD LOW
#define REV HIGH

// ================= MOTOR CLASS =================
class Motors_c {

  public:

    Motors_c() {}

    // -------------------------
    // INITIALISE MOTORS
    // -------------------------
    void initialise() {
      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      setPWM(0, 0);
    }

    // -------------------------
    // CORE MOTOR CONTROL
    // -------------------------
    void setPWM(float left_pwr, float right_pwr) {

      // Left motor direction
      if (left_pwr < 0) {
        digitalWrite(L_DIR, REV);
        left_pwr = -left_pwr;
      } else {
        digitalWrite(L_DIR, FWD);
      }

      // Right motor direction
      if (right_pwr < 0) {
        digitalWrite(R_DIR, REV);
        right_pwr = -right_pwr;
      } else {
        digitalWrite(R_DIR, FWD);
      }

      // Clamp PWM
      left_pwr  = constrain(left_pwr, 0, MOTORS_MAX_PWM);
      right_pwr = constrain(right_pwr, 0, MOTORS_MAX_PWM);

      analogWrite(L_PWM, (int)left_pwr);
      analogWrite(R_PWM, (int)right_pwr);
    }

    // -------------------------
    // HIGH LEVEL HELPERS (OPTIONAL BUT CLEAN)
    // -------------------------

    void stop() {
      setPWM(0, 0);
    }

    void forward(float speed) {
      setPWM(speed, speed);
    }

    void backward(float speed) {
      setPWM(-speed, -speed);
    }

    void turnLeft(float speed) {
      setPWM(-speed, speed);
    }

    void turnRight(float speed) {
      setPWM(speed, -speed);
    }
};

#endif
