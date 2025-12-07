// Motor Shield Rev3 – Channel A pins
const int dirA   = 12;  // direction
const int brakeA = 9;   // brake
const int pwmA   = 3;   // speed (PWM)

void setup() {
  pinMode(dirA, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(pwmA, OUTPUT);

  // Start with brake released
  digitalWrite(brakeA, LOW);
}

void loop() {
  // 1) Forward, full speed, 3 seconds
  digitalWrite(brakeA, LOW);      // release brake
  digitalWrite(dirA, HIGH);       // set direction
  analogWrite(pwmA, 255);         // 0–255, full speed
  delay(3000);

  // 2) Brake for 1 second
  digitalWrite(brakeA, HIGH);     // active brake
  delay(1000);

  // 3) Backward, half speed, 3 seconds
  digitalWrite(brakeA, LOW);      // release brake
  digitalWrite(dirA, LOW);        // opposite direction
  analogWrite(pwmA, 128);         // half speed
  delay(3000);

  // 4) Coast stop (no brake) for 2 seconds
  digitalWrite(brakeA, LOW);      // brake off
  analogWrite(pwmA, 0);           // no drive
  delay(2000);
}