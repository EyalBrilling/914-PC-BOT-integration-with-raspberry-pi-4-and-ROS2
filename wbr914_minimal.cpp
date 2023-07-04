#include "wbr914_minimal.h"

bool wbr914_minimal::open_serial(const char* _serial_port){

    if (_serial_port == NULL){
        _serial_port = DEFAULT_WBR914_PORT;
    }

    printf( "Initializing White Box Robotics Controller on %s...\n", _serial_port);
    fflush(stdout);
    
    // open it.  non-blocking at first, in case there's no robot
    if((this->_fd = open(_serial_port, O_RDWR | O_NOCTTY, S_IRUSR | S_IWUSR )) < 0 )
    {
      printf("open() failed: %s\n", strerror(errno));
      return(false);
    }
    return true;
}