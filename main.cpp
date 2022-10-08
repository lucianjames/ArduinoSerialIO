#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0");
    serial.begin(9600);
    while(1){
        int byte = serial.read_s();
        if(byte != -1){
            std::cout << (char)byte;
        }
    }
    return 0;
}
