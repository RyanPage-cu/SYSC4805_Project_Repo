// Time of Flight sensor module — Two Sensors (Single Shot Mode)
#include <Wire.h>
#include <VL53L1X.h>
#include "include/sensors.hpp"
#include "Arduino.h"

// Create two separate sensor objects
static VL53L1X tofSensor0;   // uses Wire  (SDA/SCL)
static VL53L1X tofSensor1;   // uses Wire1 (SDA1/SCL1)

// ------------------------------------------------------------
// Initialize both VL53L1X sensors
// ------------------------------------------------------------
bool tof_init() 
{
    // Init first I2C bus
    Wire.begin();
    Wire.setClock(400000);

    // Init second I2C bus
    Wire1.begin();
    Wire1.setClock(400000);

    // Tell each sensor which bus to use
    tofSensor0.setBus(&Wire);
    tofSensor1.setBus(&Wire1);

    tofSensor0.setTimeout(500);
    tofSensor1.setTimeout(500);
    if (!tofSensor1.init()) {
        Serial.println("ToF sensor 1 init failed!");
        return false;
    }
    if (!tofSensor0.init()) {
        Serial.println("ToF sensor 0 init failed!");
        return false;
    }


    Serial.println("ToF sensors initialized successfully.");

    tofSensor0.setDistanceMode(VL53L1X::Medium);
    tofSensor1.setDistanceMode(VL53L1X::Medium);

    tofSensor0.setMeasurementTimingBudget(50000);
    tofSensor1.setMeasurementTimingBudget(50000);

    return true;
}

// ------------------------------------------------------------
// Read distance from Sensor 0 (SDA/SCL)
// ------------------------------------------------------------
float tof_readDistance()
{
    uint16_t mm_0 = tofSensor0.readRangeSingleMillimeters();
    if (tofSensor0.timeoutOccurred()) return -1.0f;
    uint16_t mm_1 = tofSensor0.readRangeSingleMillimeters();
    if (tofSensor1.timeoutOccurred()) return -1.0f;
  
    return ((mm_0 + mm_1)/2.0f) / 10.0f;
}


