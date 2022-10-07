void setup() {
  Serial.begin(57600); // opens serial port, sets data rate to 57600 bps
  //Serial.setTimeout(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  
  if (Serial.available() > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    // read the incoming string:
    String incomingString = Serial.readString();
    // prints the received data
    Serial.println(incomingString);
  }
}
