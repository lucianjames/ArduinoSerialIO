#include <stdio.h> 
#include <string.h>
#include <unistd.h>  
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <iostream>
#include <vector>

/*

This class uses C-style file I/O to read from the serial port, this is because the C++ style file I/O is not as easily compatible with reading TTY devices.

*/

#define BUFFERSIZE 16

class ArduinoSerialIO{
private:
    int fd; // File descriptor for the serial port (/dev/ttyACM0 usually)
    unsigned char buffer[BUFFERSIZE]; // Input buffer array, this array has to exist because its not possible to read into a vector
    std::vector<unsigned char> dataVect; // Vector to store the data from multiple reads into the buffer
    std::string ttyName; // Path to the serial port
    bool debugLog = false; // Debug log flag
public:
    // ===== Constructor/Destructor =====
    ArduinoSerialIO(std::string tty, int baudRate, bool debug=false){
        this->ttyName = tty;
        this->debugLog = debug;
        this->fd = open(this->ttyName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if(this->fd == -1){
            throw std::runtime_error("Failed to open port " + this->ttyName);
        }
        // The following options should probably be configurable somehow
        // Honestly I have no fucking clue what most of these options do
        fcntl(this->fd, F_SETFL, 0); // Blocking mode - waits for data in input buffer
        struct termios options; // Port options
        tcgetattr(this->fd, &options); // Get the current options for the port
        cfsetispeed(&options, baudRate); // Set baud rates
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
        //sleep(1); // Give arduino a second to start up
        tcflush(this->fd, TCIFLUSH); // Flush the input buffer
        std::cout << "Serial port " << this->ttyName << " opened\n";
    }
    ~ArduinoSerialIO(){
        close(this->fd);
    }


    // ===== Input operations =====
    void readUntilNewline(){ // Reads from this->fd into this->dataVect
        int bytes_read_total = 0;
        while(1){
            int bytes_read = read(this->fd, this->buffer, BUFFERSIZE);
            bytes_read_total += bytes_read;
            if(bytes_read == -1){
                throw std::runtime_error("Failed to read from port " + this->ttyName);
                return;
            }
            for(int i = 0; i < bytes_read; i++){
                this->dataVect.push_back(this->buffer[i]); // Reading buffer[i] into the vector before checking if it is a newline will cause the vector to contain the newline character
                if(this->buffer[i] == '\n'){
                    if(debugLog){std::cout << "Read " << bytes_read_total << " bytes from " + this->ttyName + "\n";}
                    return;
                }
            }
        }
    }
    void readChar(){ // Reads a single character from this->fd into this->dataVect
        int bytes_read = read(this->fd, this->buffer, 1);
        if(bytes_read == -1){
            throw std::runtime_error("Failed to read from port " + this->ttyName);
            return;
        }
        if(debugLog){std::cout << "Read " << bytes_read << " bytes from " + this->ttyName << " (Should be 1, readChar() called)\n";}
        this->dataVect.push_back(this->buffer[0]);
    }


    // ===== Output operations =====
    void writeString(std::string str){ // Writes a string to the serial port
        int bytes_written = write(this->fd, str.c_str(), str.length());
        if(bytes_written == -1){
            throw std::runtime_error("Failed to write to port " + this->ttyName);
            return;
        }
        if(debugLog){std::cout << "Wrote " << bytes_written << " bytes to " << this->ttyName << '\n';}
    }
    void writeChar(char c){ // Writes a single character to the serial port
        int bytes_written = write(this->fd, &c, 1);
        if(bytes_written == -1){
            throw std::runtime_error("Failed to write to port " + this->ttyName);
            return;
        }
        if(debugLog){std::cout << "Wrote " << bytes_written << " bytes to " << this->ttyName << " (Should be 1, writeChar() called)\n";}
    }


    // ===== This->dataVect operations =====
    void clearDataVect(){ // Clears this->dataVect
        this->dataVect.clear();
        if(debugLog){std::cout << "Cleared this->dataVect\n";}
    }
    std::vector<unsigned char> getDataVect(){ // Returns this->dataVect
        if(debugLog){std::cout << "Returned this->dataVect\n";}
        return this->dataVect;
    }
    std::string getDataString(){ // Returns this->dataVect as a string
        std::string str;
        for(int i = 0; i < this->dataVect.size(); i++){
            str += this->dataVect[i];
        }
        if(debugLog){std::cout << "Returned this->dataVect as a string of size " << str.length() << '\n';}
        return str;
    }
    int getDataInt(){ // Returns this->dataVect as an int, -1 if stoi() fails
        // Remove last character (assumed to be newline)
        std::string str = this->getDataString();
        if(str.length() < 2){
            if(debugLog){std::cout << "getDataInt() failed, string too short (Assumed single newline)\n";}
            return -1;
        }
        str = str.substr(0, str.length() - 1);
        try{
            if(debugLog){std::cout << "Returned this->dataVect as an int\n";}
            return std::stoi(str);
        }catch(...){
            if(debugLog){std::cout << "Failed to convert this->dataVect to an int\n";}
            return -1;
        }
    }


    // ===== Misc =====
    void tcflushFD(){ // Flushes /dev/ttyACM_ buffer
        tcflush(this->fd, TCIFLUSH);
        if(debugLog){std::cout << "tcflushed " << this->ttyName << "\n";}
    }
};