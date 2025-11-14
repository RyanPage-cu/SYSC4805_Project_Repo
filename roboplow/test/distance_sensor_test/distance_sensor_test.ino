const int SENSOR_LEFT  = A8;       // Left sensor VOUT
const int SENSOR_RIGHT = A9;       // Right sensor VOUT
const float THRESHOLD_V = 0.72;    // ~8 cm boundary voltage

float readVoltage(int pin) {
    const int SAMPLES = 10;
    long sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);
        delay(2);
    }
    return (sum / (float)SAMPLES) * (5.0 / 1023.0);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Distance Sensor Unit Test Start");
}

void loop() {
    float vLeft  = readVoltage(SENSOR_LEFT);
    float vRight = readVoltage(SENSOR_RIGHT);
    Serial.print("Left Voltage: "); Serial.print(vLeft, 3);
    Serial.print(" V | Right Voltage: "); Serial.print(vRight, 3); Serial.println(" V");
    if (vLeft > THRESHOLD_V) {
        Serial.println("Object detected by LEFT sensor (<8cm)");
    }
    if (vRight > THRESHOLD_V) {
        Serial.println("Object detected by RIGHT sensor (<8cm)");
    }
    delay(500);
}
