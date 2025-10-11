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
const int LFS_R1 = A0;   // Left sensor
const int LFS_M1 = A1;   // Middle sensor
const int LFS_L1 = A2;   // Right sensor
// Line Follower Sensor #2 Pins (analog)
const int LFS_R2 = A3;   // Left sensor
const int LFS_M2 = A4;   // Middle sensor
const int LFS_L2 = A5;   // Right sensor

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

  //Uncomment below for Debugging
  /*
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  */
}

// === New Analog Line Detector Function ===
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

  char leftColor1 = (leftState1 == HIGH) ? 'B' : 'W';
  char middleColor1 = (middleState1 == HIGH) ? 'B' : 'W';
  char rightColor1 = (rightState1 == HIGH) ? 'B' : 'W';

  char leftColor2 = (leftState2 == HIGH) ? 'B' : 'W';
  char middleColor2 = (middleState2 == HIGH) ? 'B' : 'W';
  char rightColor2 = (rightState2 == HIGH) ? 'B' : 'W';

  //Umcomment below for Debugging
  /*
  Serial.print("Line Sensors >> Left: "); Serial.print(leftColor1);
  Serial.print(" - Middle: "); Serial.print(middleColor1);
  Serial.print(" - Right: "); Serial.println(rightColor1);

  Serial.print("Raw Analog >> L: "); Serial.print(leftVal1);
  Serial.print(" M: "); Serial.print(middleVal1);
  Serial.print(" R: "); Serial.println(rightVal1);

  Serial.print("Line Sensors >> Left: "); Serial.print(leftColor2);
  Serial.print(" - Middle: "); Serial.print(middleColor2);
  Serial.print(" - Right: "); Serial.println(rightColor2);

  Serial.print("Raw Analog >> L: "); Serial.print(leftVal2);
  Serial.print(" M: "); Serial.print(middleVal2);
  Serial.print(" R: "); Serial.println(rightVal2);
  */
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_dir, OUTPUT);
  pinMode(motor4_dir, OUTPUT);

  // Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Line Follower Sensor Pins
  pinMode(LFS_L1, INPUT);
  pinMode(LFS_M1, INPUT);
  pinMode(LFS_R1, INPUT);

  pinMode(LFS_L2, INPUT);
  pinMode(LFS_M2, INPUT);
  pinMode(LFS_R2, INPUT);

  Serial.println("Robot Initialized with Analog Line Detectors");
}

// === Main Loop ===
void loop() {
  ultrasonic(); // Check obstacle

  int leftState1, middleState1, rightState1, leftState2, middleState2, rightState2;
  readLineSensors(leftState1, middleState1, rightState1, leftState2, middleState2, rightState2);

  // Simple line-following logic
  if(leftState1 == HIGH) {  // Black detected on left sensor, turn away to the right
    stopAll();
    pivotTurn(RIGHT, 180);
  } 
  else if(rightState1 == HIGH) { // Black detected on right sensor, turn away to the left
    stopAll();
    pivotTurn(LEFT, 180);
  } 
  else {
    moveStraight(180);
  }

  delay(20); // Adjust loop speed
}
