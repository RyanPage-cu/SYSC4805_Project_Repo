#include "include/sensor_manager.hpp"
#include <watchdog.h>

float heading;
static float initialHeading;
static char startingSide;
bool snakingDirection = RIGHT;

/************************************************************
 * Watchdog requirement
 ************************************************************/
void watchdogSetup(void) {}

void turn(bool direction) {
    int offset = direction ? 90 : -90;
    float initial = read_heading();
    pivotTurn(direction, 1400);
    delay(200);
    float head = read_heading();
    float current_offset = (int(initial + offset) % 360) - head;
    Serial.println(initial, head);
    while(abs(current_offset) > 15.0){
        Serial.println("---");
        Serial.println(initial);
        Serial.println("-");
        Serial.println(head);
        Serial.println("-");
        Serial.println(current_offset);
        Serial.println("---");

        delay(200);
        if(current_offset < 0){
            pivotTurn(RIGHT, 200);
        }
        else{
            pivotTurn(LEFT, 200);
        }
        head = read_heading();
        current_offset = (int(initial + offset) % 360) - head;
    }
}

/************************************************************
 * Arduino Setup
 ************************************************************/
void setup() {
    watchdogEnable(16000);

    Serial.begin(115200);
    Serial.println("Robot Initializing...");

    /***** Motor Pins *****/
    motors_init();

    /***** Ultrasonic Sensor Pins *****/
    ultrasonic_init();

    /***** Line Follower Sensor Pins *****/
    line_detection_init();

    // --- Initialize Magnetometer ---
    magnetometer_init();

    // --- Initialize VL53L1X Time-of-Flight sensor ---
    if (!tof_init()) {
        Serial.println("ToF init failed!");
    }

    // --- Initialize IR Obstacle Detection Sensors ---
    ir_init();



}

/************************************************************
 * Arduino Loop
 ************************************************************/
void loop() {
    watchdogReset();

    // --- FSM State Definitions ---
    enum RoboState {
        INITIAL_STARTUP = 0,
        VERIFY_LOCATION,
        PRE_MOVEMENT_CHECK,
        MOVEMENT,
        ODDM,
        LDDM,
        SAFE_SHUTDOWN
    };

    static RoboState currentState = INITIAL_STARTUP;

    static unsigned long startupTime = 0;
    switch (currentState) {
        case INITIAL_STARTUP:
            // Initial Start-Up: remain stationary, allow sensors to stabilize
            if (startupTime == 0) {
                startupTime = millis();
                stopAll(); // Ensure motors are stopped
                Serial.println("FSM: Initial Start-Up. Stabilizing...");
            }
            if (millis() - startupTime >= 1000) {
                Serial.println("FSM: Startup complete. Transitioning to Verify Location.");
                currentState = VERIFY_LOCATION;
                startupTime = 0; // Reset for future use
            }
            break;
        case VERIFY_LOCATION: {
            // Read line sensors
            int FL, FM, FR, BL, BM, BR;
            readLineSensors(FL, FM, FR, BL, BM, BR);

            // Read compass (magnetometer)
            heading = read_heading();

            // Determine starting corner (example logic, adjust as needed)
            //bool inCorner = (FL == HIGH && BL == HIGH) || (FR == HIGH && BR == HIGH);
            bool inCorner = true;
            if (inCorner) {
                Serial.println("FSM: Robot is in a valid starting corner.");
                initialHeading = heading;
                Serial.print("FSM: Initial heading set to: ");
                Serial.println(initialHeading);
                // Orientation is always valid at startup; future turns will use initialHeading as reference
                currentState = PRE_MOVEMENT_CHECK;
            } else {
                Serial.println("FSM: Robot not in a valid corner. Waiting...");
                // Remain in this state until placed correctly
            }
            break;
        }
        case PRE_MOVEMENT_CHECK: {
            Serial.println("FSM: Transitioning to Movement state.");
            currentState = MOVEMENT;

            // Determine starting side based on line sensors and heading then moving off line
            /*
            if(heading == 'N' && FL == 0 && BL == 0){
                startingSide = 'W';
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
            }else if(heading == 'N' && FR == 0 && BR == 0){
                startingSide = 'E';
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
            }else if(heading == 'S' && FR == 0 && BR == 0){
                startingSide = 'W';
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
            }else if(heading == 'S' && FL == 0 && BL == 0){
                startingSide = 'E';
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
            }else if(heading == 'E' && FR == 0 && BR == 0){
                startingSide = 'S';
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
            }else if(heading == 'E' && FL == 0 && BL == 0){
                startingSide = 'N';
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
            }else if(heading == 'W' && FR == 0 && BR == 0){
                startingSide = 'N';
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
            }else if(heading == 'W' && FL == 0 && BL == 0){
                startingSide = 'S';
                pivotTurn(RIGHT, 150);
                stepForward(180, 150);
                pivotTurn(LEFT, 150);
                stepForward(180, 150);
            }*/
            break;
        }
        case MOVEMENT: {
            // 1. Move forward about 36 cm
            Serial.println("FSM: Movement state. Moving forward 36cm.");
            stepForward(180, 500); // Example: adjust parameters for ~36cm
            stopAll();

            // 2. Check distance sensors for obstacles
            float front_DistanceCm = tof_readDistance();
            float right_DistanceCm = ultrasonic_singleRead_Right();
            float left_DistanceCm = ultrasonic_singleRead_Left();
            bool obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            bool obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);

            bool obstacleDetected = false;
            if ((front_DistanceCm > 0 && front_DistanceCm < 30) ||
                obstacleFrontLeft || obstacleFrontRight) {
                obstacleDetected = true;
            }
            if (obstacleDetected) {
                Serial.println("FSM: Obstacle detected during movement. Transitioning to ODDM.");
                currentState = ODDM;
                break;
            }

            // 3. Check line detection sensors
            int FL, FM, FR, BL, BM, BR;
            readLineSensors(FL, FM, FR, BL, BM, BR);
            if (FL == HIGH || FM == HIGH || FR == HIGH|| BL == HIGH || BM == HIGH || BR == HIGH) {
                Serial.println("FSM: Line detected. Transitioning to LDDM.");
                currentState = LDDM;
                break;
            }

            // No obstacle or line detected, repeat movement loop
            break;
        }
        case ODDM: {
            // Re-check sensors
            float front_DistanceCm = tof_readDistance();
            float right_DistanceCm = ultrasonic_singleRead_Right();
            float left_DistanceCm = ultrasonic_singleRead_Left();
            bool obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            bool obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);

            // If no obstacle detected, return to Movement
            bool obstacleDetected = false;
            if ((front_DistanceCm > 0 && front_DistanceCm < 30) ||
                obstacleFrontLeft || obstacleFrontRight) {
                obstacleDetected = true;
            }
            if (!obstacleDetected) {
                Serial.println("FSM: No obstacle detected. Returning to Movement state.");
                currentState = MOVEMENT;
                break;
            }

            Serial.println("FSM: Executing obstacle avoidance maneuver.");

            stepForward(180, 200);
            // 1. Turn 90° right
            pivotTurn(RIGHT, 1400);
            delay(500);
            // 2. Move forward until side sensor no longer detects obstacle
            int forwardSteps = 0;
            Serial.println("LEFT DISTANCE CM");
            left_DistanceCm = 10.0;
            while ((left_DistanceCm > 0 && left_DistanceCm < 20) || obstacleFrontLeft) {
                Serial.println("STILL IN VIEW L");
                stepForward(180, 400); // Example: adjust for step size
                forwardSteps++;
                left_DistanceCm = ultrasonic_singleRead_Left();
                obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            }
            stepForward(180, 1000);
            stopAll();
            // 3. Turn 90° left
            pivotTurn(LEFT, 1400);
            stepForward(180, 600);
            delay(500);
            // 4. Move forward until opposite side sensor no longer detects obstacle
            left_DistanceCm = 10.0;
            while ((left_DistanceCm > 0 && left_DistanceCm < 20) || obstacleFrontLeft) {
                Serial.println("STILL IN VIEW R");
                stepForward(180, 400); // Example: adjust for step size
                left_DistanceCm = ultrasonic_singleRead_Left();
                obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            }
            stepForward(180, 600);
            stopAll();
            // 5. Turn 90° left again
            pivotTurn(LEFT, 1400);
            delay(500);
            // 6. Move forward same number of steps to realign
            for (int i = 0; i < forwardSteps; i++) {
                stepForward(180, 500);
            }
            stepForward(180, 1000);
            stopAll();
            // 7. Turn 90° right to restore orientation
            pivotTurn(LEFT, 1400);
            delay(500);
            
            Serial.println("FSM: Obstacle avoided. Returning to Movement state.");
            currentState = MOVEMENT;
            break;
        }
        case LDDM: {
            // Stop all motion
            stopAll();
            Serial.println("FSM: Line detected. Executing boundary correction maneuver.");

            // Read line sensors
            int FL, FM, FR, BL, BM, BR;
            float front_DistanceCm;
            bool obstacleFrontLeft;
            bool obstacleFrontRight;


            readLineSensors(FL, FM, FR, BL, BM, BR);

            // Example: front line sensor triggers correction
            if (FL == HIGH || FM == HIGH || FR == HIGH) {
                // 1. Back up slightly
                stepForward(-180, 500);
                delay(500);
                // 2. Turn 90° left
                pivotTurn(snakingDirection, 1400);
                delay(500);
                // 3. Move forward a short distance
                front_DistanceCm = tof_readDistance();
                obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
                obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);
                if ((front_DistanceCm > 0 && front_DistanceCm < 30) ||
                obstacleFrontLeft || obstacleFrontRight) {
                    stepForward(180, 100);
                } else {
                    stepForward(180, 600);
                }
                delay(500);
                // 4. Turn 90° left again
                pivotTurn(snakingDirection, 1400);
                delay(500);
                // 5. Back up slightly to center
                stepForward(-180, 500);
                delay(500);
                snakingDirection = !snakingDirection;
            }

            if ((FL == HIGH  && FR == LOW) || (FL == LOW && FR == HIGH)) {
                pivotTurn(snakingDirection, 400);
                delay(200);
                pivotTurn(!snakingDirection, 400);
            } 
            // Add additional logic for back line sensors if needed

            Serial.println("FSM: Boundary correction complete. Returning to Movement state.");
            currentState = MOVEMENT;
            break;
        }
        case SAFE_SHUTDOWN:
            // Safe Shutdown: stop all motors and remain stationary
            stopAll();
            Serial.println("FSM: Critical fault detected. Entering Safe Shutdown state.");
            // Optionally, power down components or signal error
            // Remain in this state until manual reset
            break;
        default:
            // Should not reach here
            break;
    }
}




