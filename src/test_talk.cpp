#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <liancheng_socket/MotorOrder.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <string.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
    ros::init( argc, argv, "talker" );
    ros::NodeHandle nh;
    ros::Rate rate(1.0);

    ros::Publisher pub=nh.advertise<liancheng_socket::MotorOrder>("Controller_motor_order",5);
    rate.sleep();

    liancheng_socket::MotorOrder msg;

    string rec;
    cin>>rec;

    if (rec=="1")
			{
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(3);
				msg.form.push_back(0);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(20000);
				msg.pos_thr.push_back(10);
			}
			else if (rec=="2")
			{
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(3);
				msg.form.push_back(0);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
			}
            else if(rec=="3")
            {
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(1);
				msg.form.push_back(0);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(20000);
				msg.pos_thr.push_back(10);
			}
			else if (rec=="4")
			{
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(1);
				msg.form.push_back(0);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
			}
			else if (rec=="99")
			{
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(3);
				msg.form.push_back(99);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
                msg.station_num.push_back(1);
				msg.form.push_back(99);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
			}
			else if (rec=="100")
			{
				msg.header.stamp=ros::Time::now();
				msg.station_num.push_back(3);
				msg.form.push_back(100);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
                msg.station_num.push_back(1);
				msg.form.push_back(100);
				msg.vel.push_back(1000);
				msg.vel_ac.push_back(0);
				msg.vel_de.push_back(0);
				msg.pos_mode.push_back(true);
				msg.pos.push_back(0);
				msg.pos_thr.push_back(10);
			}
			

			pub.publish(msg);
    cout<<"222"<<endl;
    
    ros::spinOnce();
    return 0;
}