#include <chrono>
#include <thread>

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", false);
    serial.begin(B9600);
    serial.setTimeout(100000);
    while(true){
        if(serial.available() > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait for all the data to come in (small numbers shouldnt take more than like 10ms probably)
            std::cout << serial.parseInt() << std::endl;
            serial.flush();
        }
    }
    return 0;
}
