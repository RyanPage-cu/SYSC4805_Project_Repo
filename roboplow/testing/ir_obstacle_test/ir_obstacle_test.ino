// IR Obstacle Detection unit test
// Embedded IR sensor init and detection copied from
// `roboplow/ir_obstacle_detection_sensors.cpp` so the test runs standalone.
// Patterns: Front-Left, Front-Right, Both, None

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

void waitForPattern(const char* patternName, bool (*patternCheck)()) {
    Serial.print("Waiting for pattern: "); Serial.println(patternName);
    while (true) {
        if (patternCheck()) {
            Serial.print("Pattern detected: "); Serial.println(patternName);
            delay(1000);
            break;
        }
        delay(200);
    }
}

bool frontLeftDetected() {
    return ir_obstacleDetected(IR_SENSOR_FL);
}

bool frontRightDetected() {
    return ir_obstacleDetected(IR_SENSOR_FR);
}

bool bothFrontDetected() {
    return ir_obstacleDetected(IR_SENSOR_FL) && ir_obstacleDetected(IR_SENSOR_FR);
}

bool noneDetected() {
    return !ir_obstacleDetected(IR_SENSOR_FL) && !ir_obstacleDetected(IR_SENSOR_FR);
}

void setup() {
    Serial.begin(115200);
    Serial.println("IR Obstacle Sensor Unit Test Start");

    ir_init();
    Serial.println("IR sensors initialized (from ir_obstacle_detection_sensors.cpp).\nAdjust pins if needed.");

    waitForPattern("Front-Left Obstacle", frontLeftDetected);
    waitForPattern("Front-Right Obstacle", frontRightDetected);
    waitForPattern("Both Front Obstacles", bothFrontDetected);
    waitForPattern("No Front Obstacles", noneDetected);

    Serial.println("IR Obstacle Sensor Unit Test Complete");
}

void loop() {
}
