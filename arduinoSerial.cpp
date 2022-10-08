#include "arduinoSerial.h"

#include <stdio.h> 
#include <string.h>
#include <unistd.h>  
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <iostream>
#include <vector>
#include <string>

arduinoSerial::arduinoSerial(std::string port, bool debug=false) {
    this->debug = debug;
    this->ttyName = port;
    // Setting up the file descriptor will be done by the begin() function.
}

arduinoSerial::~arduinoSerial() {
    close(this->fd); // Close the file descriptor
}

unsigned int arduinoSerial::available() {
    return -1; // Function not yet implemented
}

unsigned int arduinoSerial::availableForWrite() {
    return -1; // Function not yet implemented
}

void arduinoSerial::begin(unsigned long baudRate){
    this->fd = open(this->ttyName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // Open the file descriptor
    if(this->fd == -1){
        throw std::runtime_error("Unable to start the serial port " + this->ttyName);
    }
    if(this->debug){ std::cout << "Serial port " << this->ttyName << " opened\n"; }
    fcntl(this->fd, F_SETFL, 0); // Set the file descriptor to blocking mode
    struct termios options;
    tcgetattr(this->fd, &options); // Get the current options for the port
    cfsetispeed(&options, baudRate); // Set the baud rates
    cfsetospeed(&options, baudRate);
    options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    tcsetattr(this->fd, TCSANOW, &options); // Set the new options for the port
    options.c_cflag &= ~CSIZE; // Mask the character size bits
    options.c_cflag |= CS8; // Select 8 data bits
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 Stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    if(this->debug){ std::cout << "Serial port " << this->ttyName << " configured\n"; }
}

void arduinoSerial::end(){
    close(this->fd); // Close the file descriptor
    if(this->debug){ std::cout << "Serial port " << this->ttyName << " closed\n"; }
}

bool arduinoSerial::find(char *target){
    return false; // Function not yet implemented
}

bool arduinoSerial::findUntil(char *target, char *terminator){
    return false; // Function not yet implemented
}

void arduinoSerial::flush(){
    // Function not yet implemented
}

float arduinoSerial::parseFloat(){
    return -1; // Function not yet implemented
}

int arduinoSerial::parseInt(){
    return -1; // Function not yet implemented
}

int arduinoSerial::peek(){
    return -1; // Function not yet implemented
}

void arduinoSerial::print(char *str){
    // Function not yet implemented
}

void arduinoSerial::println(char *str){
    // Function not yet implemented
}

int arduinoSerial::read(){
    return -1; // Function not yet implemented
}

size_t arduinoSerial::readBytes(char *buffer, size_t length){
    return -1; // Function not yet implemented
}

size_t arduinoSerial::readBytesUntil(char terminator, char *buffer, size_t length){
    return -1; // Function not yet implemented
}

std::string arduinoSerial::readString(){
    return ""; // Function not yet implemented
}

std::string arduinoSerial::readStringUntil(char terminator){
    return ""; // Function not yet implemented
}

void arduinoSerial::setTimeout(unsigned long timeout){
    // Function not yet implemented
}

size_t arduinoSerial::write(unsigned char byte){
    return -1; // Function not yet implemented
}

size_t arduinoSerial::write(char *buffer, size_t size){
    return -1; // Function not yet implemented
}
