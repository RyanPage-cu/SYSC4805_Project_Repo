// Magnetometer (compass) unit test
// Embedded magnetometer init/read implementation copied from
// `roboplow/magnetometer.cpp` so this test runs standalone.

#include <Wire.h>
#include <LIS3MDL.h>
#include "Arduino.h"
#include <math.h>

const float TOLERANCE = 15.0; // degrees

// Magnetometer globals (copied/adapted from magnetometer.cpp)
static LIS3MDL mag;
static LIS3MDL::vector<float> mag_min = {10000.0f, 10000.0f, 10000.0f};
static LIS3MDL::vector<float> mag_max = {-10000.0f, -10000.0f, -10000.0f};
static LIS3MDL::vector<float> current;

void magnetometer_init() {
    Wire.begin();
    if (!mag.init()) {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }
    mag.enableDefault();
    Serial.println("Calibrating Magnetometer for 1 Second. Move robot around!");
    unsigned long startTime = millis();
    while (millis() < startTime + 1000) {
        mag.read();
        mag_min.x = min(mag_min.x, mag.m.x);
        mag_min.y = min(mag_min.y, mag.m.y);
        mag_min.z = min(mag_min.z, mag.m.z);
        mag_max.x = max(mag_max.x, mag.m.x);
        mag_max.y = max(mag_max.y, mag.m.y);
        mag_max.z = max(mag_max.z, mag.m.z);
        delay(100);
    }
}

float read_heading() {
    mag.read();
    mag_min.x = min(mag_min.x, mag.m.x);
    mag_min.y = min(mag_min.y, mag.m.y);
    mag_min.z = min(mag_min.z, mag.m.z);
    mag_max.x = max(mag_max.x, mag.m.x);
    mag_max.y = max(mag_max.y, mag.m.y);
    mag_max.z = max(mag_max.z, mag.m.z);
    current.x = 2 * ((mag.m.x - mag_min.x) / (mag_max.x - mag_min.x)) - 1;
    current.y = 2 * ((mag.m.y - mag_min.y) / (mag_max.y - mag_min.y)) - 1;
    current.z = 2 * ((mag.m.z - mag_min.z) / (mag_max.z - mag_min.z)) - 1;
    float heading = atan2(current.y, current.x);
    heading *= 180.0 / PI;
    if (heading < 0) heading += 360;
    return heading;
}

float readHeading() {
    return read_heading();
}

// Normalize difference to [0,180]
float headingDiff(float a, float b) {
    float diff = fabs(a - b);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

void waitForPattern(const char* patternName, bool (*patternCheck)(float)) {
    Serial.print("Waiting for pattern: "); Serial.println(patternName);
    while (true) {
        float h = readHeading();
        if (patternCheck(h)) {
            Serial.print("Pattern detected: "); Serial.println(patternName);
            Serial.print("Heading: "); Serial.println(h);
            delay(1000);
            break;
        }
        delay(200);
    }
}

bool headingNorth(float h) {
    return headingDiff(h, 0.0) <= TOLERANCE || headingDiff(h, 360.0) <= TOLERANCE;
}

bool headingEast(float h) {
    return headingDiff(h, 90.0) <= TOLERANCE;
}

bool headingSouth(float h) {
    return headingDiff(h, 180.0) <= TOLERANCE;
}

bool headingWest(float h) {
    return headingDiff(h, 270.0) <= TOLERANCE;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Magnetometer Unit Test Start");

    // Initialize magnetometer
    magnetometer_init();
    Serial.println("Magnetometer initialized.");

    waitForPattern("North (0°)", headingNorth);
    waitForPattern("East (90°)", headingEast);
    waitForPattern("South (180°)", headingSouth);
    waitForPattern("West (270°)", headingWest);

    Serial.println("Magnetometer Unit Test Complete");
}

void loop() {
}
