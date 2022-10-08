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

#define BUFFERSIZE 1024 // Used by functions that are not provided a buffer by the user (e.g. readString()). This could be as small as 1, but i dont think a single kb is too much to ask for.

arduinoSerial::arduinoSerial(std::string port, bool debug){
    this->debug = debug;
    this->ttyName = port;
    // Setting up the file descriptor will be done by the begin() function.
}

arduinoSerial::~arduinoSerial(){
    close(this->fd); // Close the file descriptor
}

unsigned int arduinoSerial::available(){
    return -1; // Function not yet implemented
}

unsigned int arduinoSerial::availableForWrite(){
    return -1; // Function not yet implemented
}

void arduinoSerial::begin(unsigned long baudRate){
    this->fd = open(this->ttyName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // Open the file descriptor
    if(this->fd == -1){
        throw std::runtime_error("Unable to start the serial port " + this->ttyName);
    }
    if(this->debug){ std::cout << "Serial port " << this->ttyName << " opened\n"; }
    fcntl(this->fd, F_SETFL, O_NONBLOCK); // Set the file descriptor to nonblocking mode
    // !!! The following termios options may or may not be correct:
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

/*
    * This function reads a single byte from the serial port.
    * It returns -1 if no data is available.
    * Function is called read_s() because read() is already taken by the C library.
*/
int arduinoSerial::read_s(){
    unsigned char byte;
    int bytesRead = read(this->fd, &byte, 1);
    if(bytesRead == -1){
        if(debug){ std::cout << "Error reading from serial port " << this->ttyName << " (Returned -1) - Buffer is likely empty\n"; }
        return -1;
    }
    if(bytesRead == 0){
        if(debug){ std::cout << "Did not read from serial port " << this->ttyName << " (Returned 0, EOF)\n"; }
        return -1;
    }
    return byte;
}

/*
    * Reads characters from the serial port into a buffer.
    * The function terminates if the terminator character is read, or if it times out.
    * Returns the number of bytes placed in the buffer (0 means no valid data found).
*/
size_t arduinoSerial::readBytes(char *buffer, size_t length){
    size_t bytesRead = 0;
    while(bytesRead < length){
        int byte = this->read_s();
        if(byte == -1){
            if(this->debug){ std::cout << "readBytes(): Error reading from serial port " << this->ttyName << " (this->read_s() returned either -1 or 0) - Buffer is likely empty\n"; }
            break;
        }
        buffer[bytesRead] = byte;
        bytesRead++;
    }
    if(debug){ std::cout << "Read " << bytesRead << " bytes from serial port " << this->ttyName << "\n"; }
    return bytesRead;
}

/*
    * Reads characters from the serial port into a buffer.
    * The function terminates if the terminator character is read, or if it times out.
    * Returns the number of bytes placed in the buffer (0 means no valid data found).
*/
size_t arduinoSerial::readBytesUntil(char terminator, char *buffer, size_t length){
    size_t bytesRead = 0;
    while(bytesRead < length){ // Read until the desired number of bytes have been read
        int byte = this->read_s(); // Read the next byte in the serial port using the read_s() function from above
        if(byte == -1){ // -1 Means some error occurred (Such as no data available)
            if(this->debug){ std::cout << "readBytesUntil(): Error reading from serial port " << this->ttyName << " (this->read_s() returned either -1 or 0) - Buffer is likely empty\n"; }
            break;
        }
        buffer[bytesRead] = byte;
        bytesRead++;
        if(byte == terminator){
            if(this->debug){ std::cout << "readBytesUntil(): Terminator character found, stopping read\n"; }
            break;
        }
    }
    if(debug){ std::cout << "Read " << bytesRead << " bytes from serial port " << this->ttyName << "\n"; }
    return bytesRead;
}

/*
    * Reads characters from the serial port into a std::string.
    * The function terminates if it times out. (Not implemented yet, for now it just reads until /dev/ttyACM0 is empty)
*/
std::string arduinoSerial::readString(){
    char buffer[BUFFERSIZE]; // Create a buffer to store the data
    std::string str = ""; // Create a string to return
    while(1){
        size_t bytesRead = this->readBytes(buffer, BUFFERSIZE); // Read the data into the buffer local to this function
        for(size_t i = 0; i < bytesRead; i++){
            str += buffer[i]; // Add the data to the string
        }
        if(bytesRead < BUFFERSIZE){ // If the buffer is not full, then we have reached the end of the data
            break;
        }
    }
    if(debug){ std::cout << "readString(): Read std::string from serial port " << this->ttyName << ", bytes read: " << str.length() << "\n"; }
    return str;
}

/*
    * Reads characters from the serial port into a std::string.
    * The function terminates if the terminator character is read, or if it times out.
*/
std::string arduinoSerial::readStringUntil(char terminator){
    char buffer[BUFFERSIZE]; // Create a buffer to store the data
    std::string str = ""; // Create a string to return
    while(1){
        size_t bytesRead = this->readBytesUntil(terminator, buffer, BUFFERSIZE); // Read the data into the buffer local to this function
        for(size_t i = 0; i < bytesRead; i++){
            str += buffer[i]; // Add the data to the string
        }
        if(bytesRead < BUFFERSIZE){ // If the buffer is not full, then we have reached the end of the data
            break;
        }
    }
    if(debug){ std::cout << "readStringUntil(): Read std::string from serial port " << this->ttyName << ", bytes read: " << str.length() << "\n"; }
    return str;
}

void arduinoSerial::setTimeout(unsigned long timeout){
    // Function not yet implemented
    // Print something because this function makes a huge difference to the program if it is not implemented
    std::cout << "setTimeout() not yet implemented, read arduinoSerial.h for more information.\n";
}

size_t arduinoSerial::write(unsigned char byte){
    return -1; // Function not yet implemented
}

size_t arduinoSerial::write(char *buffer, size_t size){
    return -1; // Function not yet implemented
}
