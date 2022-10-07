#include <chrono>

#include "arduinoSerialIO.hpp"



int main(){
    ArduinoSerialIO arduino("/dev/ttyACM0");

    while(1){
        arduino.readUntilNewline();
        std::cout << "Data read: " << arduino.getDataInt() << '\n';
        arduino.clearDataVect();
    }
    
    return 0;
}