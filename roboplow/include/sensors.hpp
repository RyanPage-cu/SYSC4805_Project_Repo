// === sensors.h ===
// This header defines pin mappings, thresholds, and function prototypes for all sensors.
// Each sensor module should include its own implementation file.

// --- Motor Control (to be moved to motors.cpp) ---
extern const int motor1_dir;
extern const int motor1_pwm;
extern const int motor2_dir;
extern const int motor2_pwm;
extern const int motor3_dir;
extern const int motor3_pwm;
extern const int motor4_dir;
extern const int motor4_pwm;
extern const bool RIGHT;
extern const bool LEFT;
void setMotor(int dirPin, int pwmPin, bool forward, int speed); // Implementation in motors.cpp
void moveStraight(int speed = 180); // Implementation in motors.cpp
void stopAll(); // Implementation in motors.cpp
void pivotTurn(bool direction, int speed = 180); // Implementation in motors.cpp
void stepForward(int speed, unsigned long duration_ms); // Implementation in motors.cpp

// --- Ultrasonic Sensor (to be moved to ultrasonic.cpp) ---
extern const int TRIG_PIN;
extern const int ECHO_PIN;
extern float distanceCm;
extern float ultrasonic_readDistance(); // Implementation in ultrasonic.cpp

// --- Line Follower Sensors (to be moved to line_follower.cpp) ---
extern const int LFS_R1;
extern const int LFS_M1;
extern const int LFS_L1;
extern const int LFS_R2;
extern const int LFS_M2;
extern const int LFS_L2;
extern const int LINE_THRESHOLD;
void readLineSensors(int &leftState1, int &middleState1, int &rightState1, int &leftState2, int &middleState2, int &rightState2); // Implementation in line_follower.cpp

// --- Distance Sensor (to be moved to distance_sensor.cpp) ---
extern const int SENSOR_LEFT;    // Left Sharp sensor VOUT
extern const int SENSOR_RIGHT;   // Right Sharp sensor VOUT
extern const float THRESHOLD_V;  // ~8 cm boundary voltage
float readVoltage(int pin);      // Implementation in distance_sensor.cpp
void readSharpDistances(float &distLeft, float &distRight) ; // Implementation in distance_sensor.cpp

// --- Time of Flight Sensor (VL53L1X) ---
bool tof_init();                             // initialize sensor, returns true on success
void tof_startContinuous(uint32_t period_ms = 50); // start continuous readings (ms)
float tof_readDistance();                    // read distance in cm, returns -1.0 on timeout/error
bool tof_timeoutOccurred();                  // check for timeout
