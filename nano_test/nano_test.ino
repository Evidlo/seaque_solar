#define ISENSE A0
#define VSENSE A1
#define PWM 11

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM, OUTPUT);
  pinMode(VSENSE, INPUT);
  pinMode(ISENSE, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0;i<255; i++) {
    analogWrite(PWM, i);
    measure();
    delay(10);
  }
  for (int i=255;i>=0; i--) {
    analogWrite(PWM, i);
    measure();
    delay(10);
  }
}

void measure() {
  Serial.print(analogRead(VSENSE));
  Serial.print('\t');
  Serial.print(analogRead(ISENSE));
  Serial.print('\n');
}
