void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // read the incoming string:
  String incomingString = Serial.readString();
  // prints the received data
  Serial.print("I received: ");
  Serial.println(incomingString);
}
