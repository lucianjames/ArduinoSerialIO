#include <string>
#include <vector>
#include <array>
#include <termios.h>

/*
    * ArduinoSerial
    * 
    * This class enables communication with an Arduino over a serial port.
    * It is based on the Arduino Serial class, but is not a direct copy (At least not yet).
    * 
    * Notes:
    *  - Timeout is not yet implemented, so functions may block indefinitely. Once ive implemented everything without timeout, I'll add it (probably in a separate branch).
    *  - The configuration of the serial port may not be correct. 
*/


class arduinoSerial {
private:
    int fd; // The file descriptor for the serial port (Usually /dev/ttyACM0)
    std::string ttyName; // The name of the serial port (Usually /dev/ttyACM0). This string exists mainly for debugging purposes
    bool debug; // If true, debug messages will be printed to the console
    // Every single baud rate defined in termios.h or termios-baud.h:
    const std::array<unsigned long, 31> acceptableBaudRates = {B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000};

public:
    arduinoSerial(std::string port, bool debug=false);
    ~arduinoSerial();
    unsigned int available();
    unsigned int availableForWrite();
    void begin(unsigned long baudRate);
    void end();
    bool find(char target);
    bool find(std::string targetStr);
    bool findUntil(char *target, char *terminator);
    void flush();
    float parseFloat();
    long parseInt();
    int peek();
    void print(std::string str);
    void print(char c);
    void print(int num);
    void print(float num);
    void println(std::string str);
    void println(char c);
    void println(int num);
    void println(float num);
    int read_s(); // read_s() is used instead of read() because read() is already taken by the C library. >:(
    size_t readBytes(char *buffer, size_t length);
    size_t readBytesUntil(char terminator, char *buffer, size_t length);
    std::string readString();
    std::string readStringUntil(char terminator);
    void setTimeout(unsigned long timeout);
    size_t write_s(char byte);
    size_t write_s(char *buffer, size_t size);
};