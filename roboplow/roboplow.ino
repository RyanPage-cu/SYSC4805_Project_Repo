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
 * Motor Helper: Step Forward for Fixed Time
 ************************************************************/
void stepForward(int speed, unsigned long duration_ms) {
    moveStraight(speed);
    delay(duration_ms);
    stopAll();
}

/************************************************************
 * Get All Line Sensor Values
 ************************************************************/
void readAllLineSensors(
    int &L1, int &M1, int &R1,
    int &L2, int &M2, int &R2
) {
    readLineSensors(L1, M1, R1, L2, M2, R2);

    Serial.print("Front L/M/R: ");
    Serial.print(L1); Serial.print(" ");
    Serial.print(M1); Serial.print(" ");
    Serial.println(R1);

    Serial.print("Back  L/M/R: ");
    Serial.print(L2); Serial.print(" ");
    Serial.print(M2); Serial.print(" ");
    Serial.println(R2);
}

/************************************************************
 * Arduino Loop
 ************************************************************/
void loop() {
    watchdogReset();

    /***** Read Ultrasonic Distance Value *****/
    ultrasonic_readDistance();
    float distanceCm = distanceCm;

    // Example: Detect obstacle ahead
    if (distanceCm < 10.0) {
        stopAll();
        Serial.println("[WARN] Obstacle detected! <15 cm");
        delay(200);
		pivotTurn(RIGHT, 180);
        return;
    }

    /***** Read Line Sensors *****/
    int L1, M1, R1, L2, M2, R2;
    readAllLineSensors(L1, M1, R1, L2, M2, R2);

	/***** Read Distance Sensors *****/
	float distanceLeft, distanceRight;
	readSharpDistances(distanceLeft, distanceRight);


     
    if (L1 == HIGH) {
        Serial.println("Line Left → turn RIGHT");
        stopAll();
        pivotTurn(RIGHT, 180);
    }
    else if (R1 == HIGH) {
        Serial.println("Line Right → turn LEFT");
        stopAll();
        pivotTurn(LEFT, 180);
    }
    else {
        Serial.println("No deviation → Straight");
        moveStraight(180);
    }

    /***** Forward Step (Temporary Movement Command) *****/
    // stepForward(180, 1000);

    delay(20);
}



