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

    // Stop after this first cycle
    noLoop();

    /***** Read Ultrasonic Distance Value *****/
    float distanceCm = ultrasonic_readDistance();
    Serial.print("Ultrasonic: ");
    Serial.println(distanceCm);

    /***** Read Sharp IR Distance Sensors *****/
    float distanceLeft, distanceRight;
    readSharpDistances(distanceLeft, distanceRight);


    /***** Read Line Sensors *****/

    /*
    int L1, M1, R1, L2, M2, R2;
    readAllLineSensors(L1, M1, R1, L2, M2, R2);

    bool lineDetected = (L1 == HIGH || M1 == HIGH || R1 == HIGH ||
                         L2 == HIGH || M2 == HIGH || R2 == HIGH);

    bool ultrasonicBlocked = (distanceCm > 0 && distanceCm < 30);
    bool IRblocked = (distanceLeft < 10 || distanceRight < 10); // adjust if needed

    if (ultrasonicBlocked) {
        Serial.println("[STOP] Ultrasonic detected obstacle!");
        stopAll();
        pivotTurn(RIGHT,180);
        return;
    }
    
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

    Serial.println("[CLEAR] Moving forward...");
    moveStraight(180);
    */

    pivotTurn(RIGHT,180);
    delay(20);
}




