#include "include/sensor_manager.hpp"
#include "Arduino.h"

// --- IR Obstacle Detection Sensor Pin ---
const int IR_SENSOR_FR = 40;    // Digital input from VMA330 OUT pin
const int IR_SENSOR_FL = 41;    // Digital input from VMA330 OUT pin

void ir_init() {
    pinMode(IR_SENSOR_FL, INPUT);
    pinMode(IR_SENSOR_FR, INPUT);
}

// IR sensor behavior:
// HIGH = No Obstacle
// LOW  = Obstacle Detected

bool ir_obstacleDetected(int IR_SENSOR_PIN) {
    int reading = digitalRead(IR_SENSOR_PIN);
    if (reading == LOW) {
        Serial.println("IR Sensor: Obstacle Detected (reading = 0)");
        return true;
    } else {
        Serial.println("IR Sensor: No Obstacle (reading = 1)");
        return false;
    }
}
