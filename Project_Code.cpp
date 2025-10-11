//currently implemented is some motor contols, ultrasonic sensor and line follower sensors

// === Motor Pin Definitions ===
const int motor1_dir = 2;
const int motor1_pwm = 8;
const int motor2_dir = 3;
const int motor2_pwm = 9;
const int motor3_dir = 4;
const int motor3_pwm = 10;
const int motor4_dir = 5;
const int motor4_pwm = 11;

const bool RIGHT = true;
const bool LEFT = false;

// Ultrasonic Sensor Pins
const int TRIG_PIN = 6;   // Trigger pin
const int ECHO_PIN = A6;  // Echo pin
float distanceCm;

// Line Follower Sensor #1 Pins (analog)
const int LFS_R = A0;   // Left sensor
const int LFS_M = A1;   // Middle sensor
const int LFS_L = A2;   // Right sensor
// Line Follower Sensor #1 Pins (analog)
const int LFS_R = A3;   // Left sensor
const int LFS_M = A4;   // Middle sensor
const int LFS_L = A5;   // Right sensor

// Threshold for analog line sensors
const int LINE_THRESHOLD = 900;

// === Helper Functions ===
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
  delay(1300);
  stopAll();
}

void ultrasonic() {
  long duration;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distanceCm = (duration * 0.034) / 2;
}

// === New Analog Line Detector Function ===
void readLineSensors(int &leftState, int &middleState, int &rightState) {
  int leftVal = analogRead(LFS_L);
  int middleVal = analogRead(LFS_M);
  int rightVal = analogRead(LFS_R);

  // Convert to HIGH if black detected, LOW if white
  leftState = (leftVal > LINE_THRESHOLD) ? HIGH : LOW;
  middleState = (middleVal > LINE_THRESHOLD) ? HIGH : LOW;
  rightState = (rightVal > LINE_THRESHOLD) ? HIGH : LOW;

  char leftColor = (leftState == HIGH) ? 'B' : 'W';
  char middleColor = (middleState == HIGH) ? 'B' : 'W';
  char rightColor = (rightState == HIGH) ? 'B' : 'W';

  Serial.print("Line Sensors >> Left: "); Serial.print(leftColor);
  Serial.print(" - Middle: "); Serial.print(middleColor);
  Serial.print(" - Right: "); Serial.println(rightColor);

  Serial.print("Raw Analog >> L: "); Serial.print(leftVal);
  Serial.print(" M: "); Serial.print(middleVal);
  Serial.print(" R: "); Serial.println(rightVal);
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_dir, OUTPUT);
  pinMode(motor4_dir, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LFS_L, INPUT);
  pinMode(LFS_M, INPUT);
  pinMode(LFS_R, INPUT);

  Serial.println("Robot Initialized with Analog Line Detectors");
}

// === Main Loop ===
void loop() {
  ultrasonic(); // Check obstacle

  int leftState, middleState, rightState;
  readLineSensors(leftState, middleState, rightState);

  // Simple line-following logic
  if(leftState == HIGH) {  // Black detected on left sensor
    stopAll();
    pivotTurn(LEFT, 180);
    Serial.println("Turned Left due to line");
  } 
  else if(rightState == HIGH) { // Black detected on right sensor
    stopAll();
    pivotTurn(RIGHT, 180);
    Serial.println("Turned Right due to line");
  } 
  else {
    moveStraight(180);
  }

  delay(20); // Adjust loop speed
}
