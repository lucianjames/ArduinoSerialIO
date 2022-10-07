#include <chrono>

#include "arduinoSerialIO.hpp"



int main(){
    ArduinoSerialIO arduino("/dev/ttyACM0");
    
    while(1){
        auto t1 = std::chrono::high_resolution_clock::now();
        arduino.writeString("HELLO WORLD");
        arduino.readUntilNewline();
        std::cout << arduino.getDataString();
        arduino.clearDataVect();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Time taken: " << ms_int.count() << "ms\n";
    }
    return 0;
}