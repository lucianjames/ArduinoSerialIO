// C++ code
//
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);
  delay(100);
  Serial.println("Arduino ready"); // For some reason if i dont println outside of the serial.available loop, shit just doesnt work
  Serial.setTimeout(10);
}

void loop(){
  while(Serial.available() > 0){
    digitalWrite(LED_BUILTIN, HIGH);
    String inStr = Serial.readString();
    Serial.println(inStr);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
