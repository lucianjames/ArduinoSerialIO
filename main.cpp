#include <chrono>

#include "arduinoSerialIO.hpp"


int main(){
    ArduinoSerialIO arduino("/dev/ttyACM0", B115200, true);

    while(1){
        // Time readUntilNewline() in microseconds
        auto start = std::chrono::high_resolution_clock::now();
        arduino.readUntilNewline();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end - start;
        std::cout << "readUntilNewline() took " << elapsed.count() << " microseconds\n";
        std::cout << "Data read: " << arduino.getDataString() << '\n';
        arduino.clearDataVect();
    }
    
    return 0;
}
