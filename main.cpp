#include <chrono>
#include <thread>

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", false);
    serial.begin(B9600);
    while(true){
        if(serial.available()){
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait for the data to come in, otherwise it will be incomplete
            std::cout << serial.parseFloat() << std::endl;
            serial.flush();
        }
    }
    return 0;
}
