// === Line Follower Sensor Pin Definitions ===
const int LFS_R1 = A2;   // Left sensor 1
const int LFS_M1 = A0;   // Middle sensor 1
const int LFS_L1 = A1;   // Right sensor 1
const int LFS_R2 = A3;   // Left sensor 2
const int LFS_M2 = A4;   // Middle sensor 2
const int LFS_L2 = A5;   // Right sensor 2
const int LINE_THRESHOLD = 900;

// Function to read line sensors and convert to states
void readLineSensors(int &leftState1, int &middleState1, int &rightState1, int &leftState2, int &middleState2, int &rightState2) {
	int leftVal1 = analogRead(LFS_L1);
	int middleVal1 = analogRead(LFS_M1);
	int rightVal1 = analogRead(LFS_R1);

	int leftVal2 = analogRead(LFS_L2);
	int middleVal2 = analogRead(LFS_M2);
	int rightVal2 = analogRead(LFS_R2);

	// Convert to HIGH if black detected, LOW if white
	leftState1 = (leftVal1 > LINE_THRESHOLD) ? HIGH : LOW;
	middleState1 = (middleVal1 > LINE_THRESHOLD) ? HIGH : LOW;
	rightState1 = (rightVal1 > LINE_THRESHOLD) ? HIGH : LOW;

	leftState2 = (leftVal2 > LINE_THRESHOLD) ? HIGH : LOW;
	middleState2 = (middleVal2 > LINE_THRESHOLD) ? HIGH : LOW;
	rightState2 = (rightVal2 > LINE_THRESHOLD) ? HIGH : LOW;

	// Debug log
	Serial.print("[LineSensorTask] S1 L:"); Serial.print(leftState1);
	Serial.print(" M:"); Serial.print(middleState1);
	Serial.print(" R:"); Serial.print(rightState1);
	Serial.print(" | S2 L:"); Serial.print(leftState2);
	Serial.print(" M:"); Serial.print(middleState2);
	Serial.print(" R:"); Serial.println(rightState2);
}
// Autonomous Snow Plow Robot System Boilerplate
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

// --- Sensor Pin Definitions (placeholders) ---
#define ULTRASONIC_PIN 2
#define LINE_SENSOR1_LEFT A0
#define LINE_SENSOR1_MID A1
#define LINE_SENSOR1_RIGHT A2
#define LINE_SENSOR2_LEFT A3
#define LINE_SENSOR2_MID A4
#define LINE_SENSOR2_RIGHT A5
#define ANALOG_DIST1 A6
#define ANALOG_DIST2 A7
#define MOTOR_PWM_PIN 8

// === Analog Distance Sensor Pin Definitions ===
const int SENSOR_LEFT  = A6; // Left sensor VOUT
const int SENSOR_RIGHT = A7; // Right sensor VOUT
const float THRESHOLD_V = 0.72; // ~8 cm boundary voltage

// Function to read average voltage from analog pin
float readVoltage(int pin) {
	const int SAMPLES = 10;
	long sum = 0;
	for (int i = 0; i < SAMPLES; i++) {
		sum += analogRead(pin);
		vTaskDelay(1000);
	}
	return (sum / (float)SAMPLES) * (5.0 / 1023.0);
}
// === Motor Pin Definitions ===
const int motor1_dir = 53;
const int motor1_pwm = 52;
const int motor2_dir = 51;
const int motor2_pwm = 50;
const int motor3_dir = 49;
const int motor3_pwm = 48;
const int motor4_dir = 47;
const int motor4_pwm = 46;

const bool RIGHT = true;
const bool LEFT = false;

// === Motor Helper Functions ===
void setMotor(int dirPin, int pwmPin, bool forward, int speed) {
	digitalWrite(dirPin, forward ? HIGH : LOW);
	analogWrite(pwmPin, speed);
}

void moveStraight(int speed = 180) {
	setMotor(motor1_dir, motor1_pwm, true, speed);
	setMotor(motor2_dir, motor2_pwm, true, speed);
	setMotor(motor3_dir, motor3_pwm, false, speed);
	setMotor(motor4_dir, motor4_pwm, false, speed);
}

void stopAll() {
	analogWrite(motor1_pwm, 0);
	analogWrite(motor2_pwm, 0);
	analogWrite(motor3_pwm, 0);
	analogWrite(motor4_pwm, 0);
}

void pivotTurn(bool direction, int speed = 180) {
	if (direction == RIGHT) {
		setMotor(motor1_dir, motor1_pwm, false, speed);
		setMotor(motor2_dir, motor2_pwm, false, speed);
		setMotor(motor3_dir, motor3_pwm, false, speed);
		setMotor(motor4_dir, motor4_pwm, false, speed);
	} else {
		setMotor(motor1_dir, motor1_pwm, true, speed);
		setMotor(motor2_dir, motor2_pwm, true, speed);
		setMotor(motor3_dir, motor3_pwm, true, speed);
		setMotor(motor4_dir, motor4_pwm, true, speed);
	}
	vTaskDelay(1000);
	stopAll();
}
// Autonomous Snow Plow Robot System Boilerplate



// --- FreeRTOS Queue Handle ---
QueueHandle_t motorCommandQueue;

// --- Motor Commands ---
typedef enum {
	MOTOR_GO_STRAIGHT,
	MOTOR_STOP,
	MOTOR_TURN_LEFT,
	MOTOR_TURN_RIGHT
} MotorCommand_t;

// --- Sensor Initialization Placeholders ---
void initUltrasonicSensor() {
	// TODO: Configure ultrasonic sensor
}

void initLineSensors() {
	// TODO: Configure line follower sensors
}

void initAnalogDistanceSensors() {
	// TODO: Configure analog distance sensors
}

void initMotors() {
	// TODO: Configure motor PWM
}

// --- FreeRTOS Tasks ---
void LineSensorTask(void *pvParameters) {
	Serial.println("[LineSensorTask] Started");
	for (;;) {
		int leftState1, middleState1, rightState1, leftState2, middleState2, rightState2;
		readLineSensors(leftState1, middleState1, rightState1, leftState2, middleState2, rightState2);

		MotorCommand_t cmd = MOTOR_GO_STRAIGHT;
		// Simple line-following logic (based on Due_code.ino)
		if (leftState1 == HIGH) {
			Serial.println("[LineSensorTask] Black detected on left sensor, sending MOTOR_TURN_RIGHT");
			cmd = MOTOR_STOP;
		} else if (rightState1 == HIGH) {
			Serial.println("[LineSensorTask] Black detected on right sensor, sending MOTOR_TURN_LEFT");
			cmd = MOTOR_STOP;
		}
		xQueueSend(motorCommandQueue, &cmd, 0);
		vTaskDelay(1000);
	}
}

void UltrasonicTask(void *pvParameters) {
	Serial.println("[UltrasonicTask] Started");
	for (;;) {
		// TODO: Read ultrasonic sensor, send command to motorCommandQueue if obstacle detected
		// Serial.println("[UltrasonicTask] Distance: ...");
		vTaskDelay(1000);
	}
}

void AnalogDistanceTask(void *pvParameters) {
	Serial.println("[AnalogDistanceTask] Started");
	for (;;) {
		float vLeft  = readVoltage(SENSOR_LEFT);
		float vRight = readVoltage(SENSOR_RIGHT);
		Serial.print("[AnalogDistanceTask] Left: ");
		Serial.print(vLeft, 2);
		Serial.print(" V | Right: ");
		Serial.print(vRight, 2);
		Serial.print(" V");

		// If either sensor detects object closer than ~8 cm, send stop command
		if (vLeft > THRESHOLD_V || vRight > THRESHOLD_V) {
			Serial.println(" | Object detected! Sending MOTOR_STOP");
			MotorCommand_t cmd = MOTOR_STOP;
			xQueueSend(motorCommandQueue, &cmd, 0);
		} else {
			Serial.println(" | No object detected");
		}
		vTaskDelay(1000);
	}
}

void MotorTask(void *pvParameters) {
	Serial.println("[MotorTask] Started");
	MotorCommand_t cmd = MOTOR_GO_STRAIGHT;
	moveStraight(180); // Start by moving straight
	Serial.println("[MotorTask] MOTOR_GO_STRAIGHT (initial)");
	for (;;) {
		if (xQueueReceive(motorCommandQueue, &cmd, portMAX_DELAY) == pdPASS) {
			switch (cmd) {
				case MOTOR_GO_STRAIGHT:
					Serial.println("[MotorTask] MOTOR_GO_STRAIGHT");
					moveStraight(180);
					break;
				case MOTOR_STOP:
					Serial.println("[MotorTask] MOTOR_STOP");
					stopAll();
					break;
				case MOTOR_TURN_LEFT:
					Serial.println("[MotorTask] MOTOR_TURN_LEFT");
					stopAll();
					pivotTurn(LEFT, 180);
					break;
				case MOTOR_TURN_RIGHT:
					Serial.println("[MotorTask] MOTOR_TURN_RIGHT");
					stopAll();
					pivotTurn(RIGHT, 180);
					break;
			}
		}
	}
}

void setup() {
	Serial.begin(115200);
	Serial.println("[setup] Initializing hardware...");
	// Motor Pins
	pinMode(motor1_dir, OUTPUT);
	pinMode(motor2_dir, OUTPUT);
	pinMode(motor3_dir, OUTPUT);
	pinMode(motor4_dir, OUTPUT);
	// Motor PWM pins are set by analogWrite

	initUltrasonicSensor();
	initLineSensors();
	initAnalogDistanceSensors();
	Serial.println("[setup] Hardware initialized.");

	// --- Create motor command queue ---
	motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t));
	if (motorCommandQueue == NULL) {
		Serial.println("[setup] ERROR: motorCommandQueue creation failed!");
	} else {
		Serial.println("[setup] motorCommandQueue created.");
	}

	// --- Create FreeRTOS tasks ---
	if (xTaskCreate(LineSensorTask, "LineSensor", 256, NULL, 2, NULL) != pdPASS)
		Serial.println("[setup] ERROR: LineSensorTask creation failed!");
	if (xTaskCreate(UltrasonicTask, "Ultrasonic", 256, NULL, 2, NULL) != pdPASS)
		Serial.println("[setup] ERROR: UltrasonicTask creation failed!");
	if (xTaskCreate(AnalogDistanceTask, "AnalogDist", 256, NULL, 2, NULL) != pdPASS)
		Serial.println("[setup] ERROR: AnalogDistanceTask creation failed!");
	if (xTaskCreate(MotorTask, "Motor", 256, NULL, 3, NULL) != pdPASS)
		Serial.println("[setup] ERROR: MotorTask creation failed!");
// === Line Follower Sensor Pin Definitions ===
const int LFS_R1 = A2;   // Left sensor 1
const int LFS_M1 = A0;   // Middle sensor 1
const int LFS_L1 = A1;   // Right sensor 1
const int LFS_R2 = A3;   // Left sensor 2
const int LFS_M2 = A4;   // Middle sensor 2
const int LFS_L2 = A5;   // Right sensor 2
const int LINE_THRESHOLD = 900;


	Serial.println("[setup] Starting FreeRTOS scheduler...");
	// --- Start FreeRTOS scheduler ---
	vTaskStartScheduler();
}

void loop() {
	// Not used with FreeRTOS
}


