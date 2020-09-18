/*

本节点为串口指令执行机构，接收决策而节点传过来的消息，然后发送给串口设备

*/


#include <ros/ros.h>
#include "edog_robot/command.h"
#include "serialport.h"
#include "command_builder.h"


static int serialport_fd = -1;//串口设备描述符

//注意这里的回调函数参数类型与消息类型一致
void chatterCallback(const edog_robot::command& msg)
{
    /*将ROS消息转为要发送的命令包*/
    Command cmd;
    for(int i=0;i<msg.length;i++)
    {
        cmd.data[i] = msg.data[i];
    }
    cmd.length = msg.length;

    /*发送命令*/
    SendData(serialport_fd,cmd.data,cmd.length);
    usleep(50000);
}

int main(int argc, char **argv)
{
    //创建节点，名字为listener
    ros::init(argc, argv, "command_listener");

    if(argc != 2)
    {
        ROS_INFO("Usage: command_listener <serialport name>");
        ROS_INFO("eg.: command_listener /dev/ttyUSB0");
        return 1;
    }
    /*打开串口设备*/
    char *device = argv[1];
    serialport_fd = openSerialPort(device);
    if(serialport_fd < 0)
    {
        ROS_INFO("open %s error!",device);
        return 1;
    }

    //节点句柄
    ros::NodeHandle n;
    //订阅chatter话题，环形缓冲队列长度为3，回调函数
    ros::Subscriber sub = n.subscribe("command", 3, chatterCallback);

    //进入循环(不会浪费CPU),ros::ok返回false时，spin将会退出，可以尽快的响应回调函数
    ros::spin();

    /*关闭串口设备*/
    closeSerialPort(serialport_fd);
    return 0;
}

