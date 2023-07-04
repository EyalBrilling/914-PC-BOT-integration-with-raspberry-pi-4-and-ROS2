#include "wbr914_minimal.h"


void wbr914_minimal::init_robot(){

    // Open the serial port of the robot
    if(!this->open_serial(NULL)){
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


bool wbr914_minimal::open_serial(const char* _serial_port){

    if (_serial_port == NULL){
        _serial_port = DEFAULT_WBR914_PORT;
    }

    printf( "Initializing White Box Robotics Controller on %s...\n", _serial_port);
    fflush(stdout);

    // open it.  non-blocking at first, in case there's no robot
    if((this->_fd = open(_serial_port, O_RDWR | O_NOCTTY, S_IRUSR | S_IWUSR )) < 0 )
    {
      return false;
    }
    return true;
}

void wbr914_minimal::UpdateM3()
{
  unsigned char ret[2];

  if ( (sendCmd0( LEFT_MOTOR,  UPDATE, 2, ret )<0 ) ||
       (sendCmd0( RIGHT_MOTOR, UPDATE, 2, ret )<0 ))
  {
    printf( "Error updating M3\n" );
  }
}

bool wbr914_minimal::EnableMotors( bool enable )
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

void wbr914_minimal::SetVelocity( float mpsL, float mpsR )
{
  uint8_t ret[2];

  if ( (sendCmd32( LEFT_MOTOR,  SETVEL, -MPS2Vel( mpsL ), 2, ret )<0)||
       (sendCmd32( RIGHT_MOTOR, SETVEL, MPS2Vel( mpsR ), 2, ret )<0))
  {
    printf( "Error setting L/R velocity\n" );
  }
}



int32_t wbr914_minimal::MPS2Vel( float mps )
{
  return (int32_t)( (double)mps * _velocityK );

}

float wbr914_minimal::Vel2MPS( int32_t count )
{
  return (float)( (double)count/_velocityK );

}

int wbr914_minimal::sendCmdCom( unsigned char address, unsigned char c,
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
      //      printf( "failed to read response\n" );
      result = -1;
    }
  }

//      PLAYER_WARN1( "cmd: 0x%4x", *((int *) cmd) );
//      PLAYER_WARN1( "cmd: 0x%4x", *((int *) &(cmd[4])) );
  return result;
}





int wbr914_minimal::sendCmd0( unsigned char address, unsigned char c,
		      int ret_num, unsigned char * ret )
{
  return sendCmdCom( address, c, 0, NULL, ret_num, ret );
}

int wbr914_minimal::sendCmd16( unsigned char address, unsigned char c,
		       int16_t arg, int ret_num, unsigned char * ret )
{

  unsigned char args[2];
  uint16_t a = htons( arg );

  args[1] = (a >> 8) & 0xFF;
  args[0] = (a >> 0) & 0xFF;

  return sendCmdCom( address, c, 2, args, ret_num, ret );
}

int wbr914_minimal::sendCmd32( unsigned char address, unsigned char c,
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

bool wbr914_minimal::RecvBytes( unsigned char*s, int len )
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

int wbr914_minimal::ReadBuf(unsigned char* s, size_t len)
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

int wbr914_minimal::WriteBuf(unsigned char* s, size_t len)
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

const char* wbr914_minimal::GetPMDErrorString( int rc )
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

int16_t wbr914_minimal::BytesToInt16(unsigned char *ptr)
{
  unsigned char char0,char1;
  int16_t data = 0;

  char0 = ptr[1];
  char1 = ptr[0];

  data |=  ((int)char0)        & 0x000000FF;
  data |= (((int)char1) << 8)  & 0x0000FF00;

  return data;
}