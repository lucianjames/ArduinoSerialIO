void setup() {
  Serial.begin(57600); // opens serial port, sets data rate to 57600 bps
  Serial.setTimeout(10);
}

void loop() {
  // read the incoming string:
  String incomingString = Serial.readString();
  // prints the received data
  Serial.println(incomingString);
}
