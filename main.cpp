#include<unistd.h> // Just for sleep()

#include <iostream>
#include <string>

#include "arduinoSerial.h"

#define DEBUGMODE true

int main(){
    arduinoSerial serial("/dev/ttyACM0", DEBUGMODE);
    serial.begin(9600);
    while(1){
        std::string str = serial.readString();
        std::cout << str;
        std::cout << "Sleeping for 2.3 seconds\n";
        sleep(2.3);
    }
    return 0;
}
