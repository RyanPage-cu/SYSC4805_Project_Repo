#include "include/sensor_manager.hpp"
#include <watchdog.h>


// ...existing code...

/************************************************************
 * Watchdog requirement
 ************************************************************/
void watchdogSetup(void) {}

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

    // --- Initialize VL53L1X Time-of-Flight sensor ---
    if (!tof_init()) {
        Serial.println("ToF init failed!");
    }

    // --- Initialize IR Obstacle Detection Sensors ---
    ir_init();

    // --- Initialize Magnetometer ---
    magnetometer_init();

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

    static RoboState currentState = MOVEMENT;

    static unsigned long startupTime = 0;
    switch (currentState) {
        case INITIAL_STARTUP:
            // Initial Start-Up: remain stationary, allow sensors to stabilize
            if (startupTime == 0) {
                startupTime = millis();
                stopAll(); // Ensure motors are stopped
                Serial.println("FSM: Initial Start-Up. Stabilizing...");
            }
            if (millis() - startupTime >= 5000) {
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
            bool inCorner = (FL == 0 && BL == 0) || (FR == 0 && BR == 0);
            if (inCorner) {
                Serial.println("FSM: Robot is in a valid starting corner.");
                if (initialHeading < 0) {
                    initialHeading = heading;
                    Serial.print("FSM: Initial heading set to: ");
                    Serial.println(initialHeading);

                    if(heading == 'N' && FL == 0 && BL == 0){
                        startingSide = "W";
                    }else if(heading == 'N' && FR == 0 && BR == 0){
                        startingSide = "E";
                    }else if(heading == 'S' && FR == 0 && BR == 0){
                        startingSide = "W";
                    }else if(heading == 'S' && FL == 0 && BL == 0){
                        startingSide = "E";
                    }else if(heading == 'E' && FR == 0 && BR == 0){
                        startingSide = "S";
                    }else if(heading == 'E' && FL == 0 && BL == 0){
                        startingSide = "N";
                    }else if(heading == 'W' && FR == 0 && BR == 0){
                        startingSide = "N";
                    }else if(heading == 'W' && FL == 0 && BL == 0){
                        startingSide = "S";
                    }
                }
                // Orientation is always valid at startup; future turns will use initialHeading as reference
                currentState = PRE_MOVEMENT_CHECK;
            } else {
                Serial.println("FSM: Robot not in a valid corner. Waiting...");
                // Remain in this state until placed correctly
            }
            break;
        }
        case PRE_MOVEMENT_CHECK: {
            // Read distance sensors
            float front_DistanceCm = tof_readDistance();
            float right_DistanceCm = ultrasonic_singleRead_Right();
            float left_DistanceCm = ultrasonic_singleRead_Left();

            // Analog/IR sensors can be added here if needed
            bool obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            bool obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);

            // Check for obstacles (example thresholds, adjust as needed)
            bool obstacleDetected = false;
            if ((front_DistanceCm > 0 && front_DistanceCm < 30) ||
                (right_DistanceCm > 0 && right_DistanceCm < 20) ||
                (left_DistanceCm > 0 && left_DistanceCm < 20) ||
                obstacleFrontLeft || obstacleFrontRight) {
                obstacleDetected = true;
            }

            if (obstacleDetected) {
                Serial.println("FSM: Obstacle detected in Pre-Movement Check. Transitioning to ODDM.");
                currentState = ODDM;
            } else {
                Serial.println("FSM: No obstacle detected. Transitioning to Movement state.");
                currentState = MOVEMENT;
            }
            break;
        }
        case MOVEMENT: {
            // 1. Move forward about 36 cm
            Serial.println("FSM: Movement state. Moving forward 36cm.");
            stepForward(180, 1000); // Example: adjust parameters for ~36cm
            stopAll();

            // 2. Check distance sensors for obstacles
            float front_DistanceCm = tof_readDistance();
            float right_DistanceCm = ultrasonic_singleRead_Right();
            float left_DistanceCm = ultrasonic_singleRead_Left();
            bool obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            bool obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);

            bool obstacleDetected = false;
            if ((front_DistanceCm > 0 && front_DistanceCm < 30) ||
                (right_DistanceCm > 0 && right_DistanceCm < 20) ||
                (left_DistanceCm > 0 && left_DistanceCm < 20) ||
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
            if (FL == 0 || FM == 0 || FR == 0 || BL == 0 || BM == 0 || BR == 0) {
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
                (right_DistanceCm > 0 && right_DistanceCm < 20) ||
                (left_DistanceCm > 0 && left_DistanceCm < 20) ||
                obstacleFrontLeft || obstacleFrontRight) {
                obstacleDetected = true;
            }
            if (!obstacleDetected) {
                Serial.println("FSM: No obstacle detected. Returning to Movement state.");
                currentState = MOVEMENT;
                break;
            }

            Serial.println("FSM: Executing obstacle avoidance maneuver.");
            if(initialHeading == "E" && startingside == "S"){
                
           
            // 1. Turn 90° right
            pivotTurn(RIGHT, 150); // Example: adjust for 90°
            delay(500);
            // 2. Move forward until side sensor no longer detects obstacle
            int forwardSteps = 0;
            while ((right_DistanceCm > 0 && right_DistanceCm < 20) || obstacleFrontRight) {
                stepForward(90, 500); // Example: adjust for step size
                forwardSteps++;
                right_DistanceCm = ultrasonic_singleRead_Right();
                obstacleFrontRight = ir_obstacleDetected(IR_SENSOR_FR);
            }
            stopAll();
            // 3. Turn 90° left
            pivotTurn(LEFT, 150);
            delay(500);
            // 4. Move forward until opposite side sensor no longer detects obstacle
            while ((left_DistanceCm > 0 && left_DistanceCm < 20) || obstacleFrontLeft) {
                stepForward(90, 500);
                left_DistanceCm = ultrasonic_singleRead_Left();
                obstacleFrontLeft = ir_obstacleDetected(IR_SENSOR_FL);
            }
            stopAll();
            // 5. Turn 90° left again
            pivotTurn(LEFT, 150);
            delay(500);
            // 6. Move forward same number of steps to realign
            for (int i = 0; i < forwardSteps; i++) {
                stepForward(90, 500);
            }
            stopAll();
            // 7. Turn 90° right to restore orientation
            pivotTurn(RIGHT, 150);
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
            readLineSensors(FL, FM, FR, BL, BM, BR);

            // Example: front line sensor triggers correction
            if (FL == 0 || FM == 0 || FR == 0) {
                // 1. Back up slightly
                stepForward(-90, 500);
                delay(500);
                // 2. Turn 90° left
                pivotTurn(LEFT, 150);
                delay(500);
                // 3. Move forward a short distance
                stepForward(90, 500);
                delay(500);
                // 4. Turn 90° left again
                pivotTurn(LEFT, 150);
                delay(500);
                // 5. Back up slightly to center
                stepForward(-90, 500);
                delay(500);
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




