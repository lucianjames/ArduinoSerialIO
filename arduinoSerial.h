#include <string>
#include <vector>

#define BUFFER_SIZE 1024 // 1KB is probably enough for most cases, but it wont matter since this code will handle any size using a std::vector

class arduinoSerial {
private:
    int fd; // The file descriptor for the serial port (Usually /dev/ttyACM0)
    unsigned char buffer[BUFFER_SIZE]; // Need to use an array since using C functions to read from the serial port
    std::vector<unsigned char> data; // Bytes are read into the buffer then into this vector
    std::string ttyName; // The name of the serial port (Usually /dev/ttyACM0). This string exists mainly for debugging purposes
    bool debug; // If true, debug messages will be printed to the console
public:
    arduinoSerial(std::string port, bool debug=false);
    ~arduinoSerial();
    unsigned int available();
    unsigned int availableForWrite();
    void begin(unsigned long baudRate);
    void end();
    bool find(char *target);
    bool findUntil(char *target, char *terminator);
    void flush();
    float parseFloat();
    int parseInt();
    int peek();
    void print(char *str);
    void println(char *str);
    int read();
    size_t readBytes(char *buffer, size_t length);
    size_t readBytesUntil(char terminator, char *buffer, size_t length);
    std::string readString();
    std::string readStringUntil(char terminator);
    void setTimeout(unsigned long timeout);
    size_t write(unsigned char byte);
    size_t write(char *buffer, size_t size);
};