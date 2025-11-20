// Time of flight sensor module implementation
#include <Wire.h>
#include <VL53L1X.h>
#include "include/sensors.hpp"
#include "Arduino.h"

static VL53L1X tofSensor;

bool tof_init() {
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C

    tofSensor.setTimeout(500);
    if (!tofSensor.init()) {
        return false;
    }

    // configure for long range and set timing budget
    tofSensor.setDistanceMode(VL53L1X::Long);
    tofSensor.setMeasurementTimingBudget(50000); // 50 ms

    // don't start continuous here unless caller requests it
    return true;
}

void tof_startContinuous(uint32_t period_ms) {
    // ensure sensor is initialized; caller should check tof_init()
    tofSensor.startContinuous(period_ms);
}

float tof_readDistance() {
    // returns centimeters, or -1.0f on timeout/error
    int mm = tofSensor.read();
    if (tofSensor.timeoutOccurred()) {
        return -1.0f;
    }
    return mm / 10.0f;
}

bool tof_timeoutOccurred() {
    return tofSensor.timeoutOccurred();
}