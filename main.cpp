#include <chrono>
#include <thread>

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", false);
    serial.begin(B9600);
    serial.setTimeout(1000);
    while(true){
        std::cout << serial.find("gaming") << std::endl;
        //std::cout << serial.readString() << std::endl;
    }
    return 0;
}
