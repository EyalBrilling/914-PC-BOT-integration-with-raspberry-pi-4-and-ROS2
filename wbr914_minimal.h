#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cassert>
#include <termios.h>

/* Conenction */
#define DEFAULT_WBR914_PORT "/dev/ttyUSB0" // Default robot port via USB
#define DELAY_US 10000

/* Math const */
#define M_PI            3.14159265358979323846

/* PMD3410 command codes */
#define GETSAMPLETIME       0x61
#define SETOUTPUTMODE       0xE0
#define RESET               0x39
#define SETVEL              0x11
#define UPDATE              0x1A
#define SETMOTORMODE        0xDC
#define SETMOTORCMD         0x77

#define MOTOR_0             ((unsigned char)0x00)
#define MOTOR_1             ((unsigned char)0x01)

/* Robot configuration */
#define LEFT_MOTOR          MOTOR_1
#define RIGHT_MOTOR         MOTOR_0

/* Robot-specific info */
#define GEAR_RATIO              (4.8)
#define MOTOR_TICKS_PER_STEP    (64.0)
#define MOTOR_TICKS_PER_REV     (200.0*MOTOR_TICKS_PER_STEP)
#define WHEEL_DIAMETER          (0.125)
#define WHEEL_CIRC              (M_PI*WHEEL_DIAMETER)

#define _BAUD                   416666 // Baud rate
/* For safety */
#define MAX_WHEELSPEED      8000

#define FULL_STOP           0
#define STOP                1

#define DEFAULT_PERCENT_TORQUE  75


class wbr914_minimal{


    private:
    // Send commands
    int  sendCmd0( unsigned char address, unsigned char c,int ret_num, unsigned char * ret );
    int  sendCmd16( unsigned char address, unsigned char c,int16_t arg, int ret_num, unsigned char * ret );
    int  sendCmd32( unsigned char address, unsigned char c,int32_t arg, int ret_num, unsigned char * ret );
    int  sendCmdCom( unsigned char address, unsigned char c,int cmd_num, unsigned char* arg,int ret_num, unsigned char * ret );
    
    // Write and read: buffer(s)<->_fd
    int  ReadBuf(unsigned char* s, size_t len);
    bool RecvBytes( unsigned char*s, int len );
    int  WriteBuf(unsigned char* s, size_t len);


    public:
    int _fd;
    const char* _serial_port;

    bool open_serial(const char* _serial_port);

    // Robot commands
    void init_robot();
    bool EnableMotors( bool enable );
    void UpdateM3();
    void SetVelocity( float mpsL, float mpsR );
    const char* GetPMDErrorString( int rc );

    // Format casting functions
    int16_t BytesToInt16(unsigned char *ptr);

    // Speed units coversion MPS<->Velocity
    int32_t MPS2Vel( float mps );
    float Vel2MPS( int32_t vel );

    // State of robot
    double  _velocityK;
    int     _usCycleTime;
    bool    _motorsEnabled;
    int     _percentTorque;
};