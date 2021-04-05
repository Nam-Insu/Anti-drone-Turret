#include "CommTurret_pkg/serial_turret.hpp"
#include "CommTurret_pkg/cserial.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "Comm_turret");
  ros::NodeHandle nh;
  std::string port = "/dev/ttyUSB0";
//  //nh.param("CommTurret/Port",port,std::string("/dev/ttyUSB0"));
  serial_turret ser(port,115200);
//  ser_turret.m_Cserial.Open(port,115200);
  ser.process();

  return 0;
}
