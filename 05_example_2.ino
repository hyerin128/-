#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  int i = 1;
  while(i <= 5 ) {
      digitalWrite(PIN_LED, 1);
      delay(100);
      digitalWrite(PIN_LED, 0);
      delay(100);
      i += 1;
  }
  digitalWrite(PIN_LED, 1);
  while(1) {}
}
