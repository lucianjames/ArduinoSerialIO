#include "arduinoSerialIO.hpp"

int main(){
    ArduinoSerialIO arduino("/dev/ttyACM0");
    
    while(1){
        arduino.writeString("HELLO WORLD");
        arduino.readUntilNewline();
        std::cout << arduino.getDataString();
        arduino.clearDataVect();
        //sleep(1);
    }
    return 0;
}