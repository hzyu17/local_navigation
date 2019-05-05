//
// Created by lucasyu on 18-11-29.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
 
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include <numeric>

using namespace std;

ofstream outFile_route;
// ofstream outFile_command;
// ofstream outFile_control;

float arr_route[2] = {0.0, 0.0}; // x; y
float arr_command = 0; // 0:idle 1:fwd 2:bckwd 3:lftwd 4:rightwd
// float arr_control[2] = {0.0, 0.0}; // linear vel; angular vel


void callBackOdom(const nav_msgs::OdometryConstPtr& odom)
{
    arr_route[0] = odom->pose.pose.position.x;
    arr_route[1] = odom->pose.pose.position.y;
    
    char ch[100];
    for (int i=0; i<2; i++)
    {
        sprintf(ch, "%f", arr_route[i]);
        outFile_route << ch << ",";
    }

    sprintf(ch, "%f", arr_command);
    outFile_route << ch << endl;

    // for (int i=0; i<2; i++)
    // {
    //     sprintf(ch, "%f", arr_control[i]);
    //     outFile_control << ch << ",";
    // }
    // outFile_control << endl;

    

}

void callBackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->axes[7] == 1)
    {
        arr_command = 1;
    }
    else if (joy->axes[7] == -1)
    {
        arr_command = 2;
    }
    else if (joy->axes[6] == 1)
    {
        arr_command = 3;
    }
    else if (joy->axes[6] == -1)
    {
        arr_command = 4;
    }
}

// void callBackCmdMobileBase(const geometry_msgs::Twist::ConstPtr& data)
// {
//     arr_control[0] = data->linear.x;
//     arr_control[1] = data->angular.z;
// }

int main(int argc, char** argv)
{
    time_t t = time(0);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y_%m_%d_%X",localtime(&t) );

    char c_route_data[100];
    sprintf(c_route_data,"routeCommand_%s.csv",tmp);
    cout<<"----- file routeCommand_ data: "<< c_route_data<<endl;

    // char c_command_data[100];
    // sprintf(c_command_data,"command_data_%s.csv",tmp);
    // cout<<"----- file command data: "<< c_command_data<<endl;

    // char c_control_data[100];
    // sprintf(c_control_data,"control_data_%s.csv",tmp);
    // cout<<"----- file control data: "<< c_control_data<<endl;
    
    outFile_route.open(c_route_data, ios::out);
    // outFile_command.open(c_command_data, ios::out);
    // outFile_control.open(c_control_data, ios::out);


    ros::init(argc, argv, "recordRoute");
    ros::NodeHandle nh;
    ros::Subscriber Odom_sub = nh.subscribe("/odom", 2, callBackOdom);
    ros::Subscriber Joy_sub = nh.subscribe("/joy", 2, callBackJoy);
    // ros::Subscriber Cmd_sub = nh.subscribe("/mobile_base/commands/velocity", 2, callBackCmdMobileBase);

    ros::spin();

    outFile_route.close();
    // outFile_control.close();
    // outFile_command.close();

    return 0;
}

