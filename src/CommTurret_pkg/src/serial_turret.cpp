#include "CommTurret_pkg/serial_turret.hpp"

serial_turret::serial_turret(std::string port, int baudrate) {
    m_Cserial.Open(port, baudrate);
}

serial_turret::serial_turret(){}

serial_turret::~serial_turret(){

}

void serial_turret::process(){
    ros::Rate loop_rate(PROCESS_HZ);

    while(ros::ok()){
        ros::spinOnce();
        //ROS_INFO("SpinWorking....\n");
        m_Cserial.Execute();
        loop_rate.sleep();
    }
}

void serial_turret::joint_callback(const sensor_msgs::JointStateConstPtr& msg){
    m_Cserial.m_target.position_Y = msg->position.at(0);
    m_Cserial.m_target.position_P = msg->position.at(1);
    m_Cserial.m_target.velocity_Y = msg->velocity.at(0);
    m_Cserial.m_target.velocity_P = msg->velocity.at(1);
}
