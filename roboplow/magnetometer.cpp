#include <Wire.h>
#include <LIS3MDL.h>
#include "Arduino.h"
#include "include/sensor_manager.hpp"

// Magnetometer globals
static LIS3MDL mag;
//LIS3MDL::vector<float> mag_min = {10000, 10000, 10000};
//LIS3MDL::vector<float> mag_max = {-10000, -10000, -10000};
//LIS3MDL::vector<float> current;
static LIS3MDL::vector<float> mag_min, mag_max, current;


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

char read_heading() {
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
    
    if (heading >= 315 || heading < 45) {
        return 'N';
    } else if (heading >= 45 && heading < 135) {
        return 'E';
    } else if (heading >= 135 && heading < 225) {
        return 'S';
    } else {
        return 'W';
    }
}
