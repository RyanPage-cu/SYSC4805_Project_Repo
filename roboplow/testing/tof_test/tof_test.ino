#include "include/sensor_manager.hpp"

// Simple VL53L1X (ToF) sensor unit test
// Patterns: Close (<30cm), Medium (30-100cm), Far (>100cm)

const int CLOSE_THRESHOLD = 30;    // cm
const int MEDIUM_THRESHOLD = 100;  // cm

float readToFDistance() {
    return tof_readDistance();
}

void waitForPattern(const char* patternName, bool (*patternCheck)(float)) {
    Serial.print("Waiting for pattern: "); Serial.println(patternName);
    while (true) {
        float d = readToFDistance();
        if (patternCheck(d)) {
            Serial.print("Pattern detected: "); Serial.println(patternName);
            Serial.print("Distance: "); Serial.println(d);
            delay(1000);
            break;
        }
        delay(200);
    }
}

bool objectClose(float d) {
    return (d > 0 && d < CLOSE_THRESHOLD);
}

bool objectMedium(float d) {
    return (d >= CLOSE_THRESHOLD && d <= MEDIUM_THRESHOLD);
}

bool objectFar(float d) {
    return (d > MEDIUM_THRESHOLD || d == 0); // 0 may indicate no reading
}

void setup() {
    Serial.begin(115200);
    Serial.println("ToF Sensor Unit Test Start");

    if (!tof_init()) {
        Serial.println("ToF init failed!");
    } else {
        Serial.println("ToF initialized.");
    }

    waitForPattern("Object Close (<30cm)", objectClose);
    waitForPattern("Object Medium (30-100cm)", objectMedium);
    waitForPattern("Object Far (>100cm)", objectFar);

    Serial.println("ToF Sensor Unit Test Complete");
}

void loop() {
}
