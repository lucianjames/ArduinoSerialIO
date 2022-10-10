// C++ code
//

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
}

void loop() {
  Serial.print(".32.23.21.sex");
  delay(1000);
}
