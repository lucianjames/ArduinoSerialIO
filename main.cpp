#include <chrono>
#include <thread>

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", true);
    serial.begin(B9600);
    serial.setTimeout(100000);
    while(true){
        if(serial.available() > 0){
            std::cout << serial.readStringUntil('\n') << " <--- was read" << std::endl;
            serial.flush();
        }
        // Wait for a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
