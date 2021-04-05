#ifndef CSERIAL_H
#define CSERIAL_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include "serial/serial.h"
#include <string.h>
#include "cstdlib"
#include "math.h"
#include "stdio.h"
#include "CommTurret_pkg/DataType.h"

class cserial {
  // Define ////////////////////////////////////////////////////////
  public:

  protected:

    // Method ////////////////////////////////////////////////////////
  public:
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: CComm
    // Input	: None
    // Output	: None
    // Summury	: Standard constructor
    ////////////////////////////////////////////////////////////////////////////////////////////
    cserial();



    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: ~CComm
    // Input	: None
    // Output	: None
    // Summury	: Standard destructor
    ////////////////////////////////////////////////////////////////////////////////////////////
    ~cserial();



    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: Open
    // Input	: port name(std::stirng), baudrate(int)
    // Output	: Result(bool)
    // Summury	: Open port handler.
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool Open(std::string port, int baudrate);



    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: Close
    // Input	: None
    // Output	: Result(bool)
    // Summury	: Close port handler.
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool Close();

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: _execute()
    // Input	: None
    // Output	: None
    // Summury	: Execute Serial Communication work.
    ////////////////////////////////////////////////////////////////////////////////////////////
    void Execute();

  protected:

  private:


  // Member ////////////////////////////////////////////////////////
  public:
    bool m_Open;
    unsigned char m_recvBuf[4096];
    unsigned char m_writeBuf[4096];

    Packet_t m_sendPacket;
    Packet_t m_packet;

    ControlData_t m_target, m_current;

  protected:

  private:
    serial::Serial m_ser;

};



#endif // CSERIAL_H
