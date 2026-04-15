#ifndef _ENCODERS_H
#define _ENCODERS_H

/*
 * Encoder interface code (DO NOT MODIFY LOGIC)
 */

#define ENCODER_0_A_PIN  7
#define ENCODER_0_B_PIN  23
#define ENCODER_1_A_PIN  26
//#define ENCODER_1_B_PIN Non-standard pin!

// ======================================================
// Global encoder variables (used in ISR + kinematics)
// ======================================================
volatile long count_e0;
volatile byte state_e0;

volatile long count_e1;
volatile byte state_e1;

// ======================================================
// ISR: Encoder 0
// ======================================================
ISR(INT6_vect)
{
    boolean e0_B = digitalRead(ENCODER_0_B_PIN);
    boolean e0_A = digitalRead(ENCODER_0_A_PIN);

    e0_A = e0_A ^ e0_B;

    state_e0 = state_e0 | (e0_B << 3);
    state_e0 = state_e0 | (e0_A << 2);

    if (state_e0 == 1)      count_e0--;
    else if (state_e0 == 2) count_e0++;
    else if (state_e0 == 4) count_e0++;
    else if (state_e0 == 7) count_e0--;
    else if (state_e0 == 8) count_e0--;
    else if (state_e0 == 11) count_e0++;
    else if (state_e0 == 13) count_e0++;
    else if (state_e0 == 14) count_e0--;

    state_e0 = state_e0 >> 2;
}

// ======================================================
// ISR: Encoder 1
// ======================================================
ISR(PCINT0_vect)
{
    boolean e1_B = PINE & (1 << PINE2);
    boolean e1_A = digitalRead(ENCODER_1_A_PIN);

    e1_A = e1_A ^ e1_B;

    state_e1 = state_e1 | (e1_B << 3);
    state_e1 = state_e1 | (e1_A << 2);

    if (state_e1 == 1)      count_e1--;
    else if (state_e1 == 2) count_e1++;
    else if (state_e1 == 4) count_e1++;
    else if (state_e1 == 7) count_e1--;
    else if (state_e1 == 8) count_e1--;
    else if (state_e1 == 11) count_e1++;
    else if (state_e1 == 13) count_e1++;
    else if (state_e1 == 14) count_e1--;

    state_e1 = state_e1 >> 2;
}

// ======================================================
// Setup Encoder 0
// ======================================================
void setupEncoder0()
{
    count_e0 = 0;

    pinMode(ENCODER_0_A_PIN, INPUT);
    pinMode(ENCODER_0_B_PIN, INPUT);

    state_e0 = 0;

    boolean e0_A = digitalRead(ENCODER_0_A_PIN);
    boolean e0_B = digitalRead(ENCODER_0_B_PIN);

    e0_A = e0_A ^ e0_B;

    state_e0 |= (e0_B << 1);
    state_e0 |= (e0_A << 0);

    // Disable INT6
    EIMSK &= ~(1 << INT6);

    // Configure INT6
    EICRB |= (1 << ISC60);

    // Clear interrupt flag
    EIFR |= (1 << INTF6);

    // Enable INT6
    EIMSK |= (1 << INT6);
}

// ======================================================
// Setup Encoder 1
// ======================================================
void setupEncoder1()
{
    count_e1 = 0;

    DDRE &= ~(1 << DDE6);
    PORTE |= (1 << PORTE2);

    pinMode(ENCODER_1_A_PIN, INPUT);
    digitalWrite(ENCODER_1_A_PIN, HIGH);

    state_e1 = 0;

    boolean e1_B = PINE & (1 << PINE2);
    boolean e1_A = digitalRead(ENCODER_1_A_PIN);

    e1_A = e1_A ^ e1_B;

    state_e1 |= (e1_B << 1);
    state_e1 |= (e1_A << 0);

    PCICR &= ~(1 << PCIE0);
    PCMSK0 |= (1 << PCINT4);
    PCIFR |= (1 << PCIF0);
    PCICR |= (1 << PCIE0);
}

#endif
