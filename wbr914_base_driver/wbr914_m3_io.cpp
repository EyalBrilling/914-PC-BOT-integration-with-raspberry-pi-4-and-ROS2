#include "wbr914_m3_io.h"

wbr914_m3_io::wbr914_m3_io()
  : _tioChanged( false ),
    _stopped( true ), _motorsEnabled( false )
{
  _fd = -1;

    // Decide torque precentage
   _percentTorque = DEFAULT_PERCENT_TORQUE;
   // Constrain torque (power to motor phases) between 0 and 100.
   // Smaller numbers mean less torque, but less power used and less
   // heat generated. Not much use reducing the torque setting below
   // 20%.
   if ( _percentTorque > 100 )
   {
     _percentTorque = 100;
   }
   else if ( _percentTorque < 20 )
   {
     _percentTorque = 20;
   }
}

int wbr914_m3_io::MainSetup()
{
  struct termios term;
  int flags;
  //int ltics,rtics,lvel,rvel;

  if(open_serial(NULL) == 0)
  {
    printf("open_serial() failed: %s\n", strerror(errno));
  }
  // Save the serial port attributes to be reseted when program terminates
  if(tcgetattr(this->_fd, &term) < 0 )
  {
    printf("tcgetattr() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  tcgetattr( this->_fd, &_old_tio);

  cfmakeraw( &term );
  cfsetispeed( &term, B38400 ); // Set communication speed(baud)
  cfsetospeed( &term, B38400 );

  // 2 stop bits
  term.c_cflag |= CSTOPB | CLOCAL | CREAD;
  term.c_iflag |= IGNPAR;

  // Set timeout to .1 sec
  term.c_cc[ VTIME ] = 1;
  term.c_cc[ VMIN ]  = 0;

  if(tcsetattr(this->_fd, TCSADRAIN, &term) < 0 )
  {
    printf("tcsetattr() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  _tioChanged = true;

  {
    struct serial_struct serial_info;

    // get serial information using ioctl function

    if ( ioctl( _fd, TIOCGSERIAL, &serial_info ) < 0)
    {
      // get the serial info
      perror("config_serial_port: ioctl TIOCGSERIAL");
      return(-1);
    }

    // Custom baud rate of 416666 baud, the max the
    // motor controller will handle.
    // round off to get the closest divisor.
    serial_info.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
    serial_info.custom_divisor = (int)((float)24000000.0/(float)_BAUD + 0.5);
    if ( _debug )
    {
      printf( "Custom divisor = %d\n", serial_info.custom_divisor );
    }
    if ( ioctl( _fd, TIOCSSERIAL, &serial_info ) < 0)
    {
      perror("config_serial_port: ioctl TIOCSSERIAL");
      return(-1);
    }
  }

  _fd_blocking = false;

  if ( _debug )
    printf( "InitRobot\n" );
  fflush(stdout);
  if(InitRobot() < 0)
  {
    printf("failed to initialize robot\n");
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  /* ok, we got data, so now set NONBLOCK, and continue */
  if((flags = fcntl(this->_fd, F_GETFL)) < 0)
  {
    printf("fcntl() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  if(fcntl(this->_fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
  {
    printf("fcntl() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }
  _fd_blocking = true;

  _usCycleTime = 154;
  // Get cycle time
  unsigned char ret[4];
  if( sendCmd0( LEFT_MOTOR, GETSAMPLETIME, 4,ret ) < 0)
  {
    printf("failed to get cycle time\n");
    return -1;
  }
  _usCycleTime = BytesToInt16( &(ret[2]) );

  _velocityK = (GEAR_RATIO * MOTOR_TICKS_PER_REV * _usCycleTime * 65536)/(WHEEL_CIRC * 1000000);

  SetMicrosteps();

  // PWM sign magnitude mode
  if ( (sendCmd16( LEFT_MOTOR, SETOUTPUTMODE, 1, 2, ret ) < 0 ) ||
       (sendCmd16( RIGHT_MOTOR, SETOUTPUTMODE, 1, 2, ret ) < 0 ))
  {
    printf( "Error setting sign-magnitude mode\n" );
  }

  /*  This might be a good time to reset the odometry values */
  if ( _debug )
    printf( "ResetRawPositions\n" );
  fflush( stdout );
  ResetRawPositions();

  if ( _debug )
    printf( "SetAccelerationProfile\n" );
  SetAccelerationProfile();
  UpdateM3();

  return(0);
}

void wbr914_m3_io::init_robot()
{
    struct termios term;
    int flags;
    
    // Open the serial port of the robot
    if(!this->open_serial(NULL))
    {
        printf("open() failed: %s\n", strerror(errno));
    }

    // Get cycle time
     _usCycleTime = 154;
     unsigned char ret[4];
     if( sendCmd0( LEFT_MOTOR, GETSAMPLETIME, 4,ret ) < 0)
     {
       printf("failed to get cycle time\n");
       return;
     }
     _usCycleTime = BytesToInt16( &(ret[2]) );

     _velocityK = (GEAR_RATIO * MOTOR_TICKS_PER_REV * _usCycleTime * 65536)/(WHEEL_CIRC * 1000000);

    // PWM sign magnitude mode
     if ( (sendCmd16( LEFT_MOTOR, SETOUTPUTMODE, 1, 2, ret ) < 0 ) ||
          (sendCmd16( RIGHT_MOTOR, SETOUTPUTMODE, 1, 2, ret ) < 0 ))
     {
       printf( "Error setting sign-magnitude mode\n" );
     }

     // Decide torque precentage
     _percentTorque = DEFAULT_PERCENT_TORQUE;
     // Constrain torque (power to motor phases) between 0 and 100.
     // Smaller numbers mean less torque, but less power used and less
     // heat generated. Not much use reducing the torque setting below
     // 20%.
     if ( _percentTorque > 100 )
     {
       _percentTorque = 100;
     }
     else if ( _percentTorque < 20 )
     {
       _percentTorque = 20;
     }

    // Reset the motors
     unsigned char buf[6];
     usleep( DELAY_US);

     if ( (sendCmd0( LEFT_MOTOR, RESET, 2, buf )<0 ) ||
          (sendCmd0( RIGHT_MOTOR, RESET, 2, buf )<0 ))
     {
       printf( "Error Resetting motors\n" );
     }
}


bool wbr914_m3_io::open_serial(const char* _serial_port)
{
    if (_serial_port == NULL)
    {
        _serial_port = DEFAULT_WBR914_PORT;
    }

    printf( "Initializing White Box Robotics Controller on %s...\n", _serial_port);
    fflush(stdout);

    // open it.  non-blocking at first, in case there's no robot
    if((this->_fd = open(_serial_port, O_RDWR | O_NOCTTY, S_IRUSR | S_IWUSR )) < 0 )
    {
      printf("open() failed: %s\n", strerror(errno));
      return false;
    }
    return true;
}

void wbr914_m3_io::UpdateM3()
{
  unsigned char ret[2];

  if ( (sendCmd0( LEFT_MOTOR,  UPDATE, 2, ret )<0 ) ||
       (sendCmd0( RIGHT_MOTOR, UPDATE, 2, ret )<0 ))
  {
    printf( "Error updating M3\n" );
  }
}

bool wbr914_m3_io::EnableMotors( bool enable )
{
  unsigned char ret[2];
  long torque = 0;

  if ( enable )
  {
    torque = _percentTorque*0x8000/100;
    if ( torque > 0x8000 )
      torque = 0x8000;
  }

  // Need to turn off motors to change the torque
  if ( ( sendCmd16( LEFT_MOTOR,  SETMOTORMODE, 0, 2, ret )<0)||
       ( sendCmd16( RIGHT_MOTOR, SETMOTORMODE, 0, 2, ret )<0))
  {
    printf( "Error resetting motor mode\n" );
  }

  if ((sendCmd16( LEFT_MOTOR, SETMOTORCMD, (short)torque, 2, ret )<0)||
      (sendCmd16( RIGHT_MOTOR, SETMOTORCMD, (short)torque, 2, ret )<0))
  {
    printf( "Error setting motor mode\n" );
  }

  // Update the torque setting
  UpdateM3();

  if ( enable )
  {
    if ((sendCmd16( LEFT_MOTOR,  SETMOTORMODE, 1, 2, ret )<0)||
	(sendCmd16( RIGHT_MOTOR, SETMOTORMODE, 1, 2, ret )<0))
    {
      printf( "Error setting motor mode\n" );
    }
  }

  _motorsEnabled = enable;

  return ( true );
}

void wbr914_m3_io::SetVelocity( float mpsL, float mpsR )
{
  uint8_t ret[2];

  if ( (sendCmd32( LEFT_MOTOR,  SETVEL, -MPS2Vel( mpsL ), 2, ret )<0)||
       (sendCmd32( RIGHT_MOTOR, SETVEL, MPS2Vel( mpsR ), 2, ret )<0))
  {
    printf( "Error setting L/R velocity\n" );
  }
}

void wbr914_m3_io::SetVelocityInTicks( int32_t left, int32_t right )
{
  uint8_t ret[2];
  
  if ( (sendCmd32( LEFT_MOTOR,  SETVEL, -left, 2, ret )<0)||
       (sendCmd32( RIGHT_MOTOR, SETVEL, right, 2, ret )<0))
  {
    printf( "Error setting velocity in ticks\n" );
  }
}

int wbr914_m3_io::SetDio(uint16_t data)
{
  // Byte flip the data to make the Output to from the
  // optional I/O board show up in the upper byte.
  data = ( (data>>8) | (data<<8) );

  // Always set 16 bits of data
  return (SetDigitalOut( data ));
}

/*
  Update the Digital input part of the client data

  We cannot reliably detect whether there is an I/O
  board attached to the M3 so blindly return the data.
 */
int wbr914_m3_io::GetDio(uint16_t* data)
{
  // Read the 16 digital inputs
  uint16_t din;
  int ret;

    ret = GetDigitalIn( &din );

  // Byte flip the data to make the Input from the
  // optional I/O board show up in the upper byte.
  *data = ( (din>>8) | (din<<8));

  return ret;
}

int wbr914_m3_io::GetIRData(float* voltages, float* ranges)
{
  // At 80cm Vmin=0.25V Vtyp=0.4V Vmax=0.55V
  // At 10cm delta increase in voltage Vmin=1.75V Vtyp=2.0V Vmax=2.25V
  // Therefore lets choose V=0.25V at 80cm and V=2.8V (2.25+0.55) at 10cm
  // Assume the formula for mm = 270 * (voltage)^-1.1 for 80cm to 10cm
  // Assume ADC input of 5.0V gives max value of 1023

  float adcLo = 0.0;
  float adcHi = 5.0;
  float vPerCount = (adcHi-adcLo)/1023.0;
  //  float v80 = 0.25;
  //  float deltaV = 2.25;
  //  float v10 = v80+deltaV;
  //  float mmPerVolt = (800.0-100.0)/(v80-v10);

  int ranges_count = NUM_IR_SENSORS;  //assign number of ranges from header delclaration of number of IR sensors

  for (int i=0; i < ranges_count; i++) //ranges_count is set from NUM_IR_SENSORS
  {
    int16_t val = 0;

    if (GetAnalogSensor( i+8, &val )==0)
    {
      float voltage = (float)val*vPerCount;
      voltages[ i ] = voltage;

      // Range values are useless further out than 80-90 cm
      // with the Sharp sensors, so truncate them accordingly
      if ( val < 80 )
      {
        val = 80;
      }

      // Convert 10 bit value to a distance in meters
      float meters;

      // Formula for range conversion is different for long range
      // sensors than short range ones. Use appropriate formula.
      if ( i == 5 || i == 7 )
      {
        // Beak side firing sensors are longer range sensors
        // Sharp GP2Y0A02 sensors 20-150cm
        meters = ((16933.0/((float)val - 8.0)) - 13.0)/100.0;
      }
      else
      {
        // Sharp GP2Y0A21 sensors 10-80cm
        meters = ((6787.0/((float)val - 3.0)) - 4.0)/100.0;
      }
      ranges[ i ] = meters;
    }
    else
    {
      return -1;
    }
  }
  return 0;
}

int wbr914_m3_io::GetAnalogSensor(int s, short * val )
{
  unsigned char ret[6];

  if ( sendCmd16( s / 8, READANALOG, s % 8, 4, ret )<0 )
  {
    printf( "Error reading Analog values\n" );
    return -1;
  }

  // Analog sensor values are 10 bit values that have been left shifted
  // 6 bits, so right-shift them back
  uint16_t v = ( (uint16_t)BytesToInt16(  &(ret[2]) ) >> 6) & 0x03ff;

#ifdef DEBUG
    printf( "sensor %d value: 0x%hx\n", s, v );
#endif

  *val = (uint16_t)v;

  return 0;
}

int32_t wbr914_m3_io::MPS2Vel( float mps )
{
  return (int32_t)( (double)mps * _velocityK );
}

float wbr914_m3_io::Vel2MPS( int32_t count )
{
  return (float)( (double)count/_velocityK );
}

float wbr914_m3_io::Ticks2Meters( int32_t ticks )
{
  return (float)( WHEEL_CIRC*((double)ticks/GEAR_RATIO)/ MOTOR_TICKS_PER_REV );
}

int wbr914_m3_io::SetDigitalOut( uint16_t d )
{
  unsigned char ret[2];

  if ( sendCmd32( 0, WRITEDIGITAL, d, 2, ret ) < 0 )
  {
    printf( "Error setting Digital output values\n" );
    return -1;
  }
  return 0;
}

int wbr914_m3_io::GetDigitalIn( uint16_t* d )
{
  unsigned char ret[6];

  if ( sendCmd16( 0, READDIGITAL, 0, 4, ret )<0)
  {
    printf( "Error reading Digital input values\n" );
    return -1;
  }

  *d = (uint16_t)BytesToInt16(  &(ret[2]) );
  return 1;
}

int wbr914_m3_io::sendCmdCom( unsigned char address, unsigned char c,
			int cmd_num, unsigned char* arg,
			int ret_num, unsigned char * ret )
{
  assert( cmd_num<=4 );

  unsigned char cmd[8];
  //bool retry = true;
  unsigned char savecs;

  cmd[0] = address;
  cmd[1] = 0;          // checksum. to be overwritten
  cmd[2] = 0x00;
  cmd[3] = c;

  // Add arguments to command
  int i;
  for ( i=0; i<cmd_num; i++ )
  {
    cmd[4+i] = arg[i];
  }

  // compute checksum. Ignore cmd[1] since it is the checksum location
  int chk = 0;
  for ( i=0; i<cmd_num+4; i++ )
  {
    chk += cmd[i];
  }

  // Set the checksum
  int cs = -chk;
  cmd[1] = (unsigned char) (cs & 0xff);
  savecs = cmd[1];

  int result;

  // Flush the receive buffer. It should be empty so lets make it be so.
  tcflush( _fd, TCIFLUSH );

  result = WriteBuf( cmd, 4+cmd_num );

  if( result != 4+cmd_num )
  {
    printf( "failed to send command %d\n", (int)c );
    return( -1 );
  }


  if( ret != NULL && ret_num > 0 )
  {
    //    if ( _fd_blocking == false )
      usleep( DELAY_US );

    int rc;
    if( (rc = ReadBuf( ret, ret_num )) < 0 )
    {
      printf( "failed to read response\n" );
      result = -1;
    }
  }

//      PLAYER_WARN1( "cmd: 0x%4x", *((int *) cmd) );
//      PLAYER_WARN1( "cmd: 0x%4x", *((int *) &(cmd[4])) );
  return result;
}

int wbr914_m3_io::sendCmd0( unsigned char address, unsigned char c,
		      int ret_num, unsigned char * ret )
{
  return sendCmdCom( address, c, 0, NULL, ret_num, ret );
}

int wbr914_m3_io::sendCmd16( unsigned char address, unsigned char c,
		       int16_t arg, int ret_num, unsigned char * ret )
{

  unsigned char args[2];
  uint16_t a = htons( arg );

  args[1] = (a >> 8) & 0xFF;
  args[0] = (a >> 0) & 0xFF;

  return sendCmdCom( address, c, 2, args, ret_num, ret );
}

int wbr914_m3_io::sendCmd32( unsigned char address, unsigned char c,
		       int32_t arg, int ret_num, unsigned char * ret )
{
  unsigned char args[4];
  uint32_t a = htonl( arg );
  args[3] = (a >> 24) & 0xFF;
  args[2] = (a >> 16) & 0xFF;
  args[1] = (a >> 8 ) & 0xFF;
  args[0] = (a >> 0 ) & 0xFF;

  return sendCmdCom( address, c, 4, args, ret_num, ret );
}

bool wbr914_m3_io::RecvBytes( unsigned char*s, int len )
{
  int nbytes;
  int bytesRead = 0;

  // max 10 retries
  for ( int i=0; i<10; i++ )
  {
    nbytes = read( _fd, s+bytesRead, len-bytesRead );
    if ( nbytes < 0 )
    {
      if ( errno != EAGAIN )
      {
	printf( "M3 Read error: %s\n", strerror( errno ));
	return false;
      }
      else
      {
	nbytes = 0;
      }
    }

    bytesRead += nbytes;
    if ( bytesRead == len )
    {
      return true;
    }

    // wait for the rest of the byte
    // calc time based on number of bytes left to read,
    // baud rate and num bits/byte
    int t = ( len-bytesRead )*11*1000/_BAUD;
    usleep( t*1000 );
  }

  printf( "Read timeout; Got %d bytes, expected %d\n",
	  bytesRead, len );

  return false;
}

int wbr914_m3_io::ReadBuf(unsigned char* s, size_t len)
{
  // Get error code
  bool rc = RecvBytes( s, 1 );
  if ( !rc )
  {
    return -1;
  }

  // PMD 3410 error code in first byte
  if ( *s != 0 && *s != 1 )
  {
    const char* err = GetPMDErrorString( *s );
    printf( "Cmd error: %s\n", err );
    return( -(*s) );
  }

  // Read the rest of the response
  rc = RecvBytes( s+1, len-1 );
  if ( !rc )
  {
    return -1;
  }

  uint8_t chksum = 0;

  // Verify the checksum is correct
  for ( unsigned i=0; i<len; i++ )
  {
    chksum += *(s+i);
  }
  if ( chksum != 0 )
  {
    printf( "Read checksum error\n" );
    return -1;
  }

#ifdef DEBUG
  printf("read: %d of %d bytes - ", numread, len );
  ssize_t i;
  for( i=0; i<numread; i++)
  {
    printf( "0x%02x ", s[i] );
  }
  printf( "\n" );
#endif

return len;

}

int wbr914_m3_io::WriteBuf(unsigned char* s, size_t len)
{
  size_t numwritten;
  int thisnumwritten;
  numwritten=0;

#ifdef DEBUG
  printf("write: %d bytes - ", len );
  size_t i;
  for( i=0; i<len; i++)
  {
    printf( "0x%02x ", s[i] );
  }
  printf( "\n" );
#endif

  for ( int i=0; i<10; i++ )
  {
    thisnumwritten = write( _fd, s+numwritten, len-numwritten);
    numwritten += thisnumwritten;

    if ( numwritten == len )
    {
      return numwritten;
    }
    else if ( thisnumwritten < 0 )
    {
      printf( "Write error; %s\n", strerror( errno ));
      return -1;
    }
  }

  printf( "Write timeout; wrote %ld bytes, tried to write %ld\n",
	  numwritten, len );

  return numwritten;
}

const char* wbr914_m3_io::GetPMDErrorString( int rc )
{
  static const char* errorStrings[] =
  {
    "Success",
    "Reset",
    "Invalid Instruction",
    "Invalid Axis",
    "Invalid Parameter",
    "Trace Running",
    "Flash",
    "Block Out of Bounds",
    "Trace buffer zero",
    "Bad checksum",
    "Not primary port",
    "Invalid negative value",
    "Invalid parameter change",
    "Limit event pending",
    "Invalid move into limit"
  };
  static char bogusRC[ 80 ];

  if ( rc < (int)sizeof( errorStrings ) )
  {
    return errorStrings[ rc ];
  }

  snprintf( bogusRC,sizeof(bogusRC), "Unknown error %d", rc );
  return bogusRC;
}

int32_t wbr914_m3_io::BytesToInt32(unsigned char *ptr)
{
  unsigned char char0,char1,char2,char3;
  int32_t data = 0;

  char0 = ptr[3];
  char1 = ptr[2];
  char2 = ptr[1];
  char3 = ptr[0];

  data |=  ((int)char0)        & 0x000000FF;
  data |= (((int)char1) << 8)  & 0x0000FF00;
  data |= (((int)char2) << 16) & 0x00FF0000;
  data |= (((int)char3) << 24) & 0xFF000000;

  //this could just be ntohl

  return data;
}

int16_t wbr914_m3_io::BytesToInt16(unsigned char *ptr)
{
  unsigned char char0,char1;
  int16_t data = 0;

  char0 = ptr[1];
  char1 = ptr[0];

  data |=  ((int)char0)        & 0x000000FF;
  data |= (((int)char1) << 8)  & 0x0000FF00;

  return data;
}

wbr914_m3_io::~wbr914_m3_io()
{
  if ( _tioChanged )
    tcsetattr( this->_fd, TCSADRAIN, &_old_tio);
  MainQuit();
}

void wbr914_m3_io::Stop( int StopMode ) 
{
  unsigned char ret[8];

  if ( _debug )
    printf( "Stop\n" );

  /* Start with motor 0*/
  _stopped = true;

  if( StopMode == FULL_STOP )
  {
    if (sendCmd16( LEFT_MOTOR, RESETEVENTSTATUS, 0x0000, 2, ret )<0 )
    {
      printf( "Error resetting event status\n" );
    }
    if ( sendCmd16( LEFT_MOTOR, SETSTOPMODE, AbruptStopMode, 2, ret )<0 )
    {
      printf( "Error setting stop mode\n" );
    }
    if ( sendCmd32( LEFT_MOTOR, SETVEL, 0, 2, ret )<0)
    {
      printf( "Error resetting motor velocity\n" );
    }
    if ( sendCmd32( LEFT_MOTOR, SETACCEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting acceleration\n" );
    }
    if ( sendCmd32( LEFT_MOTOR, SETDECEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting deceleration\n" );
    }


    if ( sendCmd16( RIGHT_MOTOR, RESETEVENTSTATUS, 0x0000, 2, ret )<0 )
    {
      printf( "Error resetting event status\n" );
    }
    if ( sendCmd16( RIGHT_MOTOR, SETSTOPMODE, AbruptStopMode, 2, ret )<0 )
    {
      printf( "Error setting stop mode\n" );
    }
    if ( sendCmd32( RIGHT_MOTOR, SETVEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting motor velocity\n" );
    }
    if ( sendCmd32( RIGHT_MOTOR, SETACCEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting acceleration\n" );
    }
    if ( sendCmd32( RIGHT_MOTOR, SETDECEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting deceleration\n" );
    }

    SetContourMode( VelocityContouringProfile );

    EnableMotors( false );
    UpdateM3();

    if ((sendCmd0( LEFT_MOTOR, RESET, 2, ret )<0)||
	(sendCmd0( RIGHT_MOTOR, RESET, 2, ret )<0))
    {
      printf( "Error resetting motor\n" );
    }

  }
  else
  {
    if ( sendCmd16( LEFT_MOTOR, RESETEVENTSTATUS, 0x0700, 2, ret )<0 )
    {
      printf( "Error resetting event status\n" );
    }
    if ( sendCmd32( LEFT_MOTOR, SETVEL, 0, 2, ret )<0 )
    {
      printf( "Error resetting motor velocity\n" );
    }

    if ( sendCmd16( RIGHT_MOTOR, RESETEVENTSTATUS, 0x0700, 2, ret )<0 )
    {
      printf( "Error resetting event status\n" );
    }
    if ( sendCmd32( RIGHT_MOTOR, SETVEL, 0, 2, ret )<0)
    {
      printf( "Error resetting motor velocity\n" );
    }

    UpdateM3();

    if ((sendCmd0( LEFT_MOTOR, RESET, 2, ret )<0)||
	(sendCmd0( RIGHT_MOTOR, RESET, 2, ret )<0))
    {
      printf( "Error resetting motor\n" );
    }
  }
}

int wbr914_m3_io::InitRobot()
{

  // initialize the robot
  unsigned char buf[6];
  usleep( DELAY_US);

  if ( (sendCmd0( LEFT_MOTOR, RESET, 2, buf )<0 ) ||
       (sendCmd0( RIGHT_MOTOR, RESET, 2, buf )<0 ))
  {
    printf( "Error Resetting motors\n" );
  }

  if ( _debug )
    printf( "GetVersion\n" );
  if( (sendCmd0( LEFT_MOTOR, GETVERSION, 6, buf ) < 0)||
      (sendCmd0( RIGHT_MOTOR, GETVERSION, 6, buf )<0 ))
  {
    printf("Cannot get version\n");
    return -1;
  }

  _stopped = true;
  return(0);
}

void wbr914_m3_io::SetMicrosteps()
{
  uint8_t ret[2];

  if ( (sendCmd16( LEFT_MOTOR, SETPHASECOUNTS, (short)MOTOR_TICKS_PER_STEP*4, 2, ret)<0)||
       (sendCmd16( RIGHT_MOTOR, SETPHASECOUNTS, (short)MOTOR_TICKS_PER_STEP*4, 2, ret)<0))
  {
    printf( "Error setting phase counts\n" );
  }
}

void wbr914_m3_io::SetActualPositionInTicks( int32_t left, int32_t right )
{
  uint8_t ret[6];
  if ( (sendCmd32( LEFT_MOTOR, SETACTUALPOS, -left, 2, ret )<0)||
       (sendCmd32( RIGHT_MOTOR, SETACTUALPOS, right, 2, ret )<0))
  {
    printf( "Error in SetActualPositionInTicks\n" );
  }
}

int wbr914_m3_io::ResetRawPositions()
{
  if ( _debug )
    printf("Reset Odometry\n");
  int Values[2];
  Values[0] = 0;
  Values[1] = 0;

  if ( _debug )
    printf( "SetActualPositionInTicks\n" );
  SetActualPositionInTicks( 0, 0 );
  UpdateM3();

  last_lpos = 0;
  last_rpos = 0;
  /*
  player_position2d_data_t data;
  memset(&data,0,sizeof(player_position2d_data_t));
  Publish( position_id, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, &data, sizeof(data),NULL);
  */
  _x   = 0;
  _y   = 0;
  _yaw = 0;
  return 0;
}

// How fast or slow De/Acceleration should be
void wbr914_m3_io::SetAccelerationProfile()
{
  uint8_t ret[2];
  //  int32_t accel = (int32_t)MOTOR_TICKS_PER_STEP*2;

  // Decelerate faster than accelerating.
  if ( (sendCmd32( LEFT_MOTOR,  SETACCEL, ACCELERATION_DEFAULT, 2, ret )<0)||
       (sendCmd32( RIGHT_MOTOR, SETACCEL, ACCELERATION_DEFAULT, 2, ret )<0))
  {
    printf( "Error setting Accelleration profile\n" );
  }
  if ((sendCmd32( LEFT_MOTOR,  SETDECEL, DECELERATION_DEFAULT, 2, ret )<0)||
      (sendCmd32( RIGHT_MOTOR, SETDECEL, DECELERATION_DEFAULT, 2, ret )<0))
  {
    printf( "Error setting Decelleration profile\n" );
  }
  SetContourMode( TrapezoidalProfile );
}

int wbr914_m3_io::GetPositionInTicks( int32_t* left, int32_t* right )
{
  uint8_t ret[6];
  if ( sendCmd0( LEFT_MOTOR, GETCMDPOS, 6, ret )<0)
  {
    printf( "Error in Left GetPositionInTicks\n" );
    return -1;
  }
  *left = -BytesToInt32( &ret[2] );
  if ( sendCmd0( RIGHT_MOTOR, GETCMDPOS, 6, ret )<0 )
  {
    printf( "Error in Right GetPositionInTicks\n" );
    return -1;
  }
  *right = BytesToInt32( &ret[2] );
  return 0;
}

int wbr914_m3_io::GetVelocityInTicks( int32_t* left, int32_t* right )
{
  uint8_t ret[6];
  if ( sendCmd0( LEFT_MOTOR, GETCMDVEL, 6, ret )<0 )
  {
    printf( "Error in Left GetVelocityInTicks\n" );
    return -1;
  }
  *left = -BytesToInt32( &ret[2] );
  if ( sendCmd0( RIGHT_MOTOR, GETCMDVEL, 6, ret )<0 )
  {
    printf( "Error in Left GetVelocityInTicks\n" );
    return -1;
  }
  *right = BytesToInt32( &ret[2] );
  return 0;
}

void wbr914_m3_io::SetContourMode( ProfileMode_t prof )
{
  uint8_t ret[2];

  if ( (sendCmd16( LEFT_MOTOR, SETPROFILEMODE, prof, 2, ret)<0)||
       (sendCmd16( RIGHT_MOTOR, SETPROFILEMODE, prof, 2, ret)<0))
  {
    printf( "Error setting profile mode\n" );
  }
}

void wbr914_m3_io::MainQuit()
{
  if( this->_fd == -1 )
    return;

  // Stop the robot
  Stop();

  EnableMotors( false );

  // Close the connection to the M3
  int fd = _fd;
  this->_fd = -1;
  close( fd );

  puts( "914 has been shut down" );
}