#include <chrono>
#include <thread>

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", false);
    serial.begin(B9600);
    while(true){
        std::cout << serial.readStringUntil('e') << std::endl;
    }
    return 0;
}
