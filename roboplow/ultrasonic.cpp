// Ultrasonic sensor module implementation

#include "include/sensors.hpp"
#include "Arduino.h"

//Left Sensor
const int TRIG_PIN0 = 6;   // Trigger pin
const int ECHO_PIN0 = 7;   // Echo pin
//RIght Sensor
const int TRIG_PIN1 = 42;   // Trigger pin
const int ECHO_PIN1 = 43;   // Echo pin

/**************************************************************
 * Read a single LEFT ultrasonic distance value (cm) 
 **************************************************************/
float ultrasonic_singleRead_Left() {
    // Send trigger pulse
    digitalWrite(TRIG_PIN0, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN0, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN0, LOW);

    // Measure echo pulse (timeout = 50 ms)
    unsigned long pulse = pulseIn(ECHO_PIN0, HIGH, 50000);

    // No echo received
    if (pulse == 0) return -1;

    // Convert pulse length to distance (cm)
    return (pulse * 0.0343f) / 2.0f;
}
/**************************************************************
 * Read a single RIGHT ultrasonic distance value (cm) 
 **************************************************************/
float ultrasonic_singleRead_Right() {
    // Send trigger pulse
    digitalWrite(TRIG_PIN1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN1, LOW);

    // Measure echo pulse (timeout = 50 ms)
    unsigned long pulse = pulseIn(ECHO_PIN1, HIGH, 50000);

    // No echo received
    if (pulse == 0) {
        Serial.print("no echo");
        return -1; 
    }

    // Convert pulse length to distance (cm)
    return (pulse * 0.0343f) / 2.0f;
}

/**************************************************************
 * Average ultrasonic distance over 200 ms, sampling every 50 ms
 **************************************************************/
float ultrasonic_readDistance_Left() {
    unsigned long start = millis();
    float sum = 0;
    int count = 0;

    while (millis() - start < 100) {
        float d = ultrasonic_singleRead_Left();

        if (d > 0) {  // Only add valid readings
            sum += d;
            count++;
        }

        delay(10);
    }

    if (count == 0) return -1; // No valid readings
    return sum / count;        // Return average distance
}

float ultrasonic_readDistance_Right() {
    unsigned long start = millis();
    float sum = 0;
    int count = 0;

    while (millis() - start < 100) {
        float d = ultrasonic_singleRead_Right();

        if (d > 0) {  // Only add valid readings
            sum += d;
            count++;
        }

        delay(10);
    }

    if (count == 0) return -1; // No valid readings
    return sum / count;        // Return average distance
}
