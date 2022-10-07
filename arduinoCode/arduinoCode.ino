// C++ code
//
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop(){
  Serial.print("Hello world\n");
}
