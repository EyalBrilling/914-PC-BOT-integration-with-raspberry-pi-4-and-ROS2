#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cassert>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

// Default max speeds
#define ACCELERATION_DEFAULT 100
#define DECELERATION_DEFAULT 250
#define MOTOR_DEF_MAX_SPEED 0.3

/* Conenction */
#define DEFAULT_WBR914_PORT "/dev/serial/by-id/usb-FTDI_USB__-__Serial-if00-port0" // Default robot port via USB
#define DELAY_US 10000

/* Math const */
#define M_PI            3.14159265358979323846

/* PMD3410 command codes */
#define GETSAMPLETIME       0x61
#define SETOUTPUTMODE       0xE0
#define RESET               0x39
#define SETVEL              0x11 // Set velocity
#define UPDATE              0x1A
#define SETMOTORMODE        0xDC
#define SETMOTORCMD         0x77
#define RESETEVENTSTATUS    0x34 // Reseting the driver configurations in Stop function
#define SETSTOPMODE         0xD0
#define SETACCEL            0x90 // Acceleration 
#define SETDECEL            0x91 // Deacceleration
#define SETPROFILEMODE      0xA0
#define GETVERSION          0x8F
#define GETCMDPOS           0x1D // Get position
#define GETCMDVEL           0x1E // Get velocity(of 2 wheels)
#define SETPHASECOUNTS      0x75
#define SETACTUALPOS        0x4D // Odometry
#define READANALOG          0xEF
#define WRITEDIGITAL        0x82
#define READDIGITAL         0x83

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
#define DEFAULT_AXLE_LENGTH     (0.301)
#define NUM_IR_SENSORS          8  //1-5 on base  6-8 on beak

#define _BAUD                   416666 // Baud rate
/* For safety */
#define MAX_WHEELSPEED      8000

#define FULL_STOP           0
#define STOP                1

#define DEFAULT_PERCENT_TORQUE  75

// Read more here: https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article
typedef enum {
  TrapezoidalProfile = 0, // Decelerate faster than accelerating
  VelocityContouringProfile,
  SCurveProfile,
} ProfileMode_t;

// Different stop modes the driver can enter into
typedef enum {
  NoStopMode = 0,
  AbruptStopMode,
  SmoothStopMode
} StopMode;

class wbr914_m3_io
{

    private:
    // Comm info for connection to M3 controller
    struct termios     _old_tio;
    bool               _tioChanged;
    bool               _fd_blocking;

    // Send commands
    int  sendCmd0( unsigned char address, unsigned char c,int ret_num, unsigned char * ret );
    int  sendCmd16( unsigned char address, unsigned char c,int16_t arg, int ret_num, unsigned char * ret );
    int  sendCmd32( unsigned char address, unsigned char c,int32_t arg, int ret_num, unsigned char * ret );
    int  sendCmdCom( unsigned char address, unsigned char c,int cmd_num, unsigned char* arg,int ret_num, unsigned char * ret );
    
    // Write and read: buffer(s)<->_fd
    int  ReadBuf(unsigned char* s, size_t len);
    bool RecvBytes( unsigned char*s, int len );
    int  WriteBuf(unsigned char* s, size_t len);

    //Read A/D's
    int GetAnalogSensor(int s, short * val );
    int GetDigitalIn( uint16_t* d );
    int SetDigitalOut( unsigned short digOut );

    public:
    wbr914_m3_io();
    virtual ~wbr914_m3_io();
    bool open_serial(const char* _serial_port);

    /*
     Robot commands
    */
    int MainSetup(); // Initiate robot driver comm and set needed variables
    void MainQuit(); // Quit the program safly by stopping motors and robot comm
    int  InitRobot(); 
    void init_robot();
    void Stop(int StopMode= FULL_STOP);
    bool EnableMotors( bool enable );
    void UpdateM3();
    const char* GetPMDErrorString( int rc );

    // Setters of robot state via the USB 
    void SetVelocity( float mpsL, float mpsR );
    void SetVelocityInTicks( int32_t left, int32_t right );
    int SetDio(uint16_t data);
    
    void SetContourMode( ProfileMode_t prof );
    void SetMicrosteps();
    void SetAccelerationProfile();
    void SetActualPositionInTicks( int32_t left, int32_t right ); // Odometry
    int ResetRawPositions();  // Reset Odometry
    
    // Getters of robot state via the USB
    int GetPositionInTicks( int32_t* left, int32_t* right );
    int GetVelocityInTicks( int32_t* left, int32_t* right );
    int GetIRData(float* voltages, float* ranges);
    int GetDio(uint16_t* data);

    int _fd;
    const char* _serial_port;

    // Format casting functions
    int16_t BytesToInt16(unsigned char *ptr);
    int32_t BytesToInt32( unsigned char *ptr );

    // Speed units coversion MPS<->Velocity
    int32_t MPS2Vel( float mps );
    float Vel2MPS( int32_t vel );
    float Ticks2Meters( int32_t ticks );

    // Odometry stuff
    int32_t last_lpos;
    int32_t last_rpos;
    double  _x;
    double  _y;
    double  _yaw;

    // State of robot
    bool    _stopped;
    double  _velocityK;
    int     _usCycleTime;
    bool    _motorsEnabled;
    int     _percentTorque;
    int     _debug;
};