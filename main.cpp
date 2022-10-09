#include<unistd.h> // Just for sleep()

#include <iostream>
#include <string>

#include "arduinoSerial.h"

int main(){
    arduinoSerial serial("/dev/ttyACM0", true);
    serial.begin(9600);
    while(1){
        // Send a message to the Arduino
        serial.print("Hello Arduino!");
        // Wait for the Arduino to respond
        sleep(1);
        std::string response = serial.readString();
        std::cout << "Arduino says: " << response << "\n";
    }
    return 0;
}
