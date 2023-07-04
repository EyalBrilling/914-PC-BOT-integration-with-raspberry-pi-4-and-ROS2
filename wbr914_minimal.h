#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <cstring>

#define DEFAULT_WBR914_PORT "/dev/ttyUSB0"

class wbr914_minimal{
    public:
    int _fd;
    const char* _serial_port;

    bool open_serial(const char* _serial_port);
};