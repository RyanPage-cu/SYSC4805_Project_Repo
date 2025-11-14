// Motor pin definitions (copied from motors.cpp)
const int motor1_dir = 53;
const int motor1_pwm = 52;
const int motor2_dir = 51;
const int motor2_pwm = 50;
const int motor3_dir = 49;
const int motor3_pwm = 48;
const int motor4_dir = 47;
const int motor4_pwm = 46;

void setMotor(int dirPin, int pwmPin, bool forward, int speed) {
    digitalWrite(dirPin, forward ? HIGH : LOW);
    analogWrite(pwmPin, speed);
}

void setup() {
    Serial.begin(115200);
    pinMode(motor1_dir, OUTPUT);
    pinMode(motor1_pwm, OUTPUT);
    pinMode(motor2_dir, OUTPUT);
    pinMode(motor2_pwm, OUTPUT);
    pinMode(motor3_dir, OUTPUT);
    pinMode(motor3_pwm, OUTPUT);
    pinMode(motor4_dir, OUTPUT);
    pinMode(motor4_pwm, OUTPUT);
    
    Serial.println("Motor Unit Test Start");
    int speeds[3] = {0, 180, 2};
    
    // Test PWM speed control
    for (int i = 0; i < 3; i++) {
        Serial.print("Testing speed "); Serial.println(speeds[i]);
        setMotor(motor1_dir, motor1_pwm, true, speeds[i]);
        setMotor(motor2_dir, motor2_pwm, true, speeds[i]);
        setMotor(motor3_dir, motor3_pwm, false, speeds[i]);
        setMotor(motor4_dir, motor4_pwm, false, speeds[i]);
        delay(2000);
    }
    // Stop motors
    setMotor(motor1_dir, motor1_pwm, true, 0);
    setMotor(motor2_dir, motor2_pwm, true, 0);
    setMotor(motor3_dir, motor3_pwm, false, 0);
    setMotor(motor4_dir, motor4_pwm, false, 0);
    delay(500);
    
    // Test direction switching
    Serial.println("Testing direction: FORWARD");
    setMotor(motor1_dir, motor1_pwm, true, 150);
    setMotor(motor2_dir, motor2_pwm, true, 150);
    setMotor(motor3_dir, motor3_pwm, false, 150);
    setMotor(motor4_dir, motor4_pwm, false, 150);
    delay(1000);
    Serial.println("Testing direction: REVERSE");
    setMotor(motor1_dir, motor1_pwm, false, 150);
    setMotor(motor2_dir, motor2_pwm, false, 150);
    setMotor(motor3_dir, motor3_pwm, true, 150);
    setMotor(motor4_dir, motor4_pwm, true, 150);
    delay(1000);
    // Stop motors
    setMotor(motor1_dir, motor1_pwm, false, 0);
    setMotor(motor2_dir, motor2_pwm, false, 0);
    setMotor(motor3_dir, motor3_pwm, true, 0);
    setMotor(motor4_dir, motor4_pwm, true, 0);
    Serial.println("Motor Unit Test Complete");
}

void loop() {
}
