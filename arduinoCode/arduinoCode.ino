// C++ code
//
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Arduino ready"); // For some reason if i dont println outside of the serial.available loop, shit just doesnt work
}

void loop(){
  delay(100);
  Serial.println("Hello world");
}
