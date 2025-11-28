const int TRIG_PIN = 32;   // Trigger pin
const int ECHO_PIN = 33;   // Echo pin
float distanceCm;

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

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("Ultrasonic Sensor Unit Test Start");
}

void loop() {
    ultrasonic();
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
    if (distanceCm > 0 && distanceCm < 20) {
        Serial.println("Object detected within 20cm");
        delay(2000); // Wait before next detection
    }
    delay(500);
}
