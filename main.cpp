#include<unistd.h> // Just for sleep()

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", true);
    serial.begin(B9600);
    // ParseFloat test
    while(true){
        std::cout << serial.parseFloat() << std::endl;
        sleep(1);
    }
    return 0;
}
