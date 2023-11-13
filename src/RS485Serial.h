#ifndef RS485_Serial_H
#define RS485_Serial_H


#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <liancheng_socket/MotorOrder.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <string.h>
#include <iostream>

using namespace std;

class serialpublisher{

public:
    serialpublisher(ros::NodeHandle & nh);
    ~serialpublisher();

    void prepare();
    void vel_form(uint8_t station_num,int16_t vel=100, uint16_t vel_ac=0 ,uint16_t vel_de=0);
    void pos_form(uint8_t station_num,bool pos_mode=true,int32_t pos=0, uint16_t pos_thr=0 ,uint16_t vel=100,uint16_t vel_ac=0);
    void enable_off(uint8_t station_num);
    void enable_on(uint8_t station_num);
    void read_data(uint8_t station_num);


private:
    serial::Serial ser;
    ros::NodeHandle nh_;
    ros::Duration sleeptime=ros::Duration(3);
    ros::Duration sleeptime_code=ros::Duration(0.01);
    serial::Timeout to;
    uint32_t baud;
    ros::Subscriber sub;
    std::string rl;
    std::string port;
    int8_t form=-1;
    char *p;

    vector<vector<uint8_t>> code_list{vector<vector<uint8_t>>(23)};
    
    
    
    
    

    void callback(const liancheng_socket::MotorOrder & order);
    vector<uint8_t> CRC16_MudBus(vector<uint8_t> puchMsg, uint8_t usDataLen,uint8_t station_num);
    bool set_form(uint8_t station_num,int8_t form_input);//mode select:0=pos;1=vel/模式选择:0=位移模式；1=速度模式。
};
#endif