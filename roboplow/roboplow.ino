#include "include/sensors.hpp"
#include <watchdog.h>

/************************************************************
 * Watchdog requirement
 ************************************************************/
void watchdogSetup(void) {}

/************************************************************
 * Arduino Setup
 ************************************************************/
void setup() {
    watchdogEnable(4000);

    Serial.begin(115200);
    Serial.println("Robot Initializing...");

    /***** Motor Pins *****/
    pinMode(motor1_dir, OUTPUT);
    pinMode(motor2_dir, OUTPUT);
    pinMode(motor3_dir, OUTPUT);
    pinMode(motor4_dir, OUTPUT);

    /***** Ultrasonic Sensor Pins *****/
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    /***** Line Follower Sensor Pins *****/
    pinMode(LFS_L1, INPUT);
    pinMode(LFS_M1, INPUT);
    pinMode(LFS_R1, INPUT);

    pinMode(LFS_L2, INPUT);
    pinMode(LFS_M2, INPUT);
    pinMode(LFS_R2, INPUT);

    Serial.println("Robot Initialized using Analog Line Detectors.");
}

/************************************************************
 * Get All Line Sensor Values
 ************************************************************/
void readAllLineSensors(
    int &L1, int &M1, int &R1,
    int &L2, int &M2, int &R2
) {
    readLineSensors(L1, M1, R1, L2, M2, R2);
}

/************************************************************
 * Arduino Loop
 ************************************************************/
void loop() {
    watchdogReset();

    /***** Read Ultrasonic Distance Value *****/
    float distanceCm = ultrasonic_readDistance();
    Serial.print("Ultrasonic: ");
    Serial.println(distanceCm);

    /***** Read Sharp IR Distance Sensors *****/
    float distanceLeft, distanceRight;
    readSharpDistances(distanceLeft, distanceRight);

    //Serial.print("IR Left: ");
    //Serial.print(distanceLeft);
    //Serial.print(" cm | IR Right: ");
    //Serial.print(distanceRight);
    //Serial.println(" cm");

    /***** Read Line Sensors *****/
    int L1, M1, R1, L2, M2, R2;
    readAllLineSensors(L1, M1, R1, L2, M2, R2);

    bool lineDetected = (L1 == HIGH || M1 == HIGH || R1 == HIGH ||
                         L2 == HIGH || M2 == HIGH || R2 == HIGH);

    /********************************************************
     * DETECTION LOGIC
     * Stop immediately if ANY sensor is triggered
     ********************************************************/

    bool ultrasonicBlocked = (distanceCm > 0 && distanceCm < 30);
    bool IRblocked = (distanceLeft < 10 || distanceRight < 10); // adjust if needed

    if (ultrasonicBlocked) {
        Serial.println("[STOP] Ultrasonic detected obstacle!");
        stopAll();
        pivotTurn(RIGHT,180);
        return;
    }
    /*
    if (IRblocked) {
        Serial.println("[STOP] Sharp distance sensor triggered!");
        stopAll();
        return;
    }
    

    if (lineDetected) {
        Serial.println("[STOP] Line detected!");
        stopAll();
        return;
    }

    */

    /********************************************************
     * NO DETECTION → MOVE FORWARD
     ********************************************************/

    Serial.println("[CLEAR] Moving forward...");
    moveStraight(180);

    delay(20);
}

#ifdef TEST_MAIN
// Simple main for testing functions outside the Arduino loop.
// Define TEST_MAIN to use this (e.g. add `#define TEST_MAIN` at top or pass -DTEST_MAIN).
extern "C" int main(void) {
#if defined(ARDUINO)
    init(); // initialize Arduino core if available
#endif
    setup();

    Serial.println("=== TEST MAIN START ===");

    // Example: test ultrasonic
    float d = ultrasonic_readDistance();
    Serial.print("ultrasonic_readDistance() -> ");
    Serial.println(d);

    // Example: test Sharp IR readings
    float left, right;
    readSharpDistances(left, right);
    Serial.print("readSharpDistances() -> ");
    Serial.print(left);
    Serial.print(", ");
    Serial.println(right);

    // Example: test line sensors
    int L1, M1, R1, L2, M2, R2;
    readAllLineSensors(L1, M1, R1, L2, M2, R2);
    Serial.print("Line sensors -> ");
    Serial.print(L1); Serial.print(' ');
    Serial.print(M1); Serial.print(' ');
    Serial.print(R1); Serial.print(' ');
    Serial.print(L2); Serial.print(' ');
    Serial.print(M2); Serial.print(' ');
    Serial.println(R2);

    // Example: test motors briefly
    Serial.println("moveStraight(150) for 500ms");
    moveStraight(150);
    delay(500);
    stopAll();

    Serial.println("=== TEST MAIN END ===");

    // Keep running so Serial output remains available
    while (true) {
        delay(1000);
    }
    return 0;
}
#endif






