// Line Follower Sensor #1 Pins (analog)
const int LFS_R1 = A2;   // Left sensor
const int LFS_M1 = A0;   // Middle sensor
const int LFS_L1 = A1;   // Right sensor

const int LINE_THRESHOLD = 900;

void readLineSensors(int &leftState1, int &middleState1, int &rightState1) {
    int leftVal1 = analogRead(LFS_L1);
    int middleVal1 = analogRead(LFS_M1);
    int rightVal1 = analogRead(LFS_R1);
    leftState1 = (leftVal1 > LINE_THRESHOLD) ? HIGH : LOW;
    middleState1 = (middleVal1 > LINE_THRESHOLD) ? HIGH : LOW;
    rightState1 = (rightVal1 > LINE_THRESHOLD) ? HIGH : LOW;
    Serial.print("L:"); Serial.print(leftVal1);
    Serial.print(" M:"); Serial.print(middleVal1);
    Serial.print(" R:"); Serial.println(rightVal1);
    delay(500);
}

void waitForPattern(const char* patternName, bool (*patternCheck)(int,int,int)) {
    int l1, m1, r1;
    Serial.print("Waiting for pattern: "); Serial.println(patternName);
    while (true) {
        readLineSensors(l1, m1, r1);
        if (patternCheck(l1, m1, r1)) {
            Serial.print("Pattern detected: "); Serial.println(patternName);
            delay(1000);
            break;
        }
        delay(100);
    }
}

// Pattern checks
bool verticalLeft(int l1, int m1, int r1) {
    // Left sensor HIGH, others LOW
    return (l1 == HIGH && m1 == LOW && r1 == LOW);
}
bool verticalRight(int l1, int m1, int r1) {
    // Right sensor HIGH, others LOW
    return (r1 == HIGH && l1 == LOW && m1 == LOW);
}
bool horizontalFront(int l1, int m1, int r1) {
    // Middle sensor HIGH, others LOW
    return (m1 == HIGH && l1 == HIGH && r1 == HIGH);
}
bool angle45(int l1, int m1, int r1) {
    // Left and middle sensors HIGH, right LOW (simulate 45°)
    return (l1 == HIGH && m1 == LOW && r1 == HIGH);
}
bool intersectTopRight(int l1, int m1, int r1) {
    // Middle and right sensors HIGH (simulate top right corner)
    return (m1 == HIGH && r1 == HIGH);
}
bool intersectTopLeft(int l1, int m1, int r1) {
    // Middle and left sensors HIGH (simulate top left corner)
    return (m1 == HIGH && l1 == HIGH);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Line Follower Sensor Unit Test Start");
    waitForPattern("Vertical Line Left", verticalLeft);
    waitForPattern("Vertical Line Right", verticalRight);
    waitForPattern("Horizontal Line Front", horizontalFront);
    waitForPattern("45 Degree Line", angle45);
    waitForPattern("Intersect Top Right", intersectTopRight);
    waitForPattern("Intersect Top Left", intersectTopLeft);
    Serial.println("Line Follower Sensor Unit Test Complete");
}

void loop() {
    // No repeated tests
}
