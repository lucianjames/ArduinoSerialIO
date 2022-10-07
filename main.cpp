#include "arduinoSerialIO.hpp"

int main(){
    ArduinoSerialIO arduino("/dev/ttyACM0");
    while(1){
        arduino.readUntilNewline();
        std::cout << arduino.getDataString();
        arduino.clearDataVect();
    }
    return 0;
}