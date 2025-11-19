// Ultrasonic sensor module implementation

#include "include/sensors.hpp"
#include "Arduino.h"

const int TRIG_PIN = 43;   // Trigger pin
const int ECHO_PIN = 42;   // Echo pin
float distanceCm;

/**************************************************************
 * Read a single ultrasonic distance value (cm)
 **************************************************************/
float ultrasonic_singleRead() {
    // Send trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo pulse (timeout = 50 ms)
    unsigned long pulse = pulseIn(ECHO_PIN, HIGH, 50000);

    // No echo received
    if (pulse == 0) return -1;

    // Convert pulse length to distance (cm)
    return (pulse * 0.0343f) / 2.0f;
}

/**************************************************************
 * Average ultrasonic distance over 200 ms, sampling every 50 ms
 **************************************************************/
float ultrasonic_readDistance() {
    unsigned long start = millis();
    float sum = 0;
    int count = 0;

    while (millis() - start < 100) {
        float d = ultrasonic_singleRead();

        if (d > 0) {  // Only add valid readings
            sum += d;
            count++;
        }

        delay(10);
    }

    if (count == 0) return -1; // No valid readings
    return sum / count;        // Return average distance
}
