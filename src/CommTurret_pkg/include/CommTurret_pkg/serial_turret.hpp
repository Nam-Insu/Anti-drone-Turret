#ifndef SERIAL_TURRET_H
#define SERIAL_TURRET_H



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "cstdlib"
#include "math.h"
#include "stdio.h"
#include <string>
#include "string.h"
#include "sensor_msgs/JointState.h"
#include "ros_tutorial_msgs/msgData.h"
#include "CommTurret_pkg/cserial.h"
#define PROCESS_HZ 10

class serial_turret {
  /////////////////////////////////////////////
  //define/////////////////////////////////////
  /////////////////////////////////////////////
  public :
  private :
  protected:

  /////////////////////////////////////////////
  //method/////////////////////////////////////
  /////////////////////////////////////////////
  public :
  serial_turret(std::string port, int baudrate);
  serial_turret();
  ~serial_turret();
  void process();

  //callback
  void joint_callback(const sensor_msgs::JointStateConstPtr& msg);


  private :
  protected:

  /////////////////////////////////////////////
  //member/////////////////////////////////////
  /////////////////////////////////////////////
  public :

  sensor_msgs::JointState m_joint_state;
  cserial m_Cserial;
  ros_tutorial_msgs::msgData m_Mode;

  private :
  ros::NodeHandle nh;

  ros::Subscriber joint_sub;

  ros::Publisher Turret_sub = nh.advertise<ros_tutorial_msgs::msgData>("Turret_Mode",10);

  protected:

};



#endif // SERIAL_TURRET_H
