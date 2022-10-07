// C++ code
//
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop(){

  for(int i=0; i<100000; i++){
    Serial.print(i);
    Serial.print('\n');
  }
}
