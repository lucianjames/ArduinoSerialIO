# ArduinoSerialIO
## Header file for communicating easily with serial devices via /dev/tty____  written in C++. (Linux only)

This header file is designed to act as close as possible to the arduino "Serial" library, there are a couple missing functions, and I cant guarantee that everything does indeed work *exactly* as it should, but its pretty close.


`read()` and `write()` are actually named `read_s()` and `write_s()`

## Public functions
```
unsigned int available();
void begin(unsigned long baudRate);
void end();
bool find(char target);
bool find(std::string targetStr);
bool findUntil(char target, char terminator);
bool findUntil(std::string targetStr, char terminator);
void flush();
float parseFloat();
long parseInt();
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
size_t write_s(char byte); // Named like read_s for the same reason
size_t write_s(char *buffer, size_t size);
```
If youve done some serial stuff on arduinos before, this should hopefully be very familiar.
