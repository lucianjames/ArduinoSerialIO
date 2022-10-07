void setup() {
  Serial.begin (9600);
}

void sendSomeData(){
  for(int i=0; i<100; i++){
    Serial.print(i);
  }
  Serial.println();
}

void loop() {
  delay(100);
  sendSomeData();
}
